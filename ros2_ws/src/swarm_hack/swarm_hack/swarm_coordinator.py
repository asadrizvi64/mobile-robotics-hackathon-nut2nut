"""Swarm coordinator: the robot <-> drone "brain".

Robot -> drones:  the TurtleBot's SLAM pose (TF map->robot_frame) defines a
                  formation; each drone gets a slot in the robot's frame.
                  Objects mapped by the robot (object_mapper) can be handed
                  to a drone, which flies over and hovers above them.
Drones -> robot:  /swarm/events reports inspections, including an
                  EXPERIMENTAL object-height estimate from the drone's
                  downward ToF (step in range.zrange when it crosses the object).

Mode / commands (std_msgs/String on /swarm/mode):
  follow                     drones hold formation slots around the robot (default)
  auto                       follow + automatically send a free drone to every new object
  hold                       freeze all drones where they are
  inspect <id>               send the nearest free drone to object <id>
  goto <x> <y> [z]           send the nearest free drone to a map point and hover there
  goto <drone> <x> <y> [z]   send a specific drone (cf1 / cf2)
  fly <waypoint>             send the nearest free drone to a waypoint saved by nav_manager
  home                       all drones back to their formation slots

Also: RViz "Publish Point" tool (/clicked_point) -> nearest free drone flies there.
Coordinates are in the robot's SLAM map frame (metres; origin = where SLAM started).
"""
import json
import math
import os
import statistics
import time
from collections import deque

import rclpy
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .geom import quat_from_yaw, se2_apply, wrap, yaw_from_quat


class SwarmCoordinator(Node):
    def __init__(self):
        super().__init__('swarm_coordinator')
        dp = self.declare_parameter
        self.names = dp('drones', ['cf1', 'cf2']).value
        self.map_frame = dp('map_frame', 'map').value
        self.robot_frame = dp('robot_frame', 'base_footprint').value
        rate = dp('rate_hz', 20.0).value
        self.alpha = dp('robot_filter_alpha', 0.25).value   # EMA on SLAM pose (it jumps)
        self.min_sep = dp('min_separation', 0.6).value       # drone-drone, metres
        self.inspect_z = dp('inspect_z', 0.6).value
        self.inspect_time = dp('inspect_time', 5.0).value
        self.arrive_r = dp('arrive_radius', 0.15).value
        self.max_inspect_dist = dp('max_inspect_dist', 2.5).value
        self.inspect_classes = list(dp('inspect_classes', ['']).value)
        self.inspect_classes = [c for c in self.inspect_classes if c]  # [] = any class
        self.point_z = dp('point_z', 0.8).value
        self.wp_file = os.path.expanduser(dp('waypoints_file', '~/tb_waypoints.yaml').value)
        self.mode = dp('mode', 'follow').value
        self.auto = self.mode == 'auto'

        self.slots = {}
        for i, n in enumerate(self.names):
            side = 1.0 if i % 2 == 0 else -1.0
            self.slots[n] = dp(f'slot.{n}', [-0.6, 0.6 * side, 0.8]).value  # (x, y, z) in robot frame

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.robot = None
        self.objects = {}
        self.inspected = set()
        self.tasks = {n: {'kind': 'slot'} for n in self.names}
        self.pose = {n: None for n in self.names}
        self.status = {n: 'UNKNOWN' for n in self.names}
        self.zr = {n: deque(maxlen=60) for n in self.names}  # (t, zrange, dist_to_target)
        self.hold = {}

        self.goal_pub = {n: self.create_publisher(PoseStamped, f'/{n}/goal', 10) for n in self.names}
        for n in self.names:
            self.create_subscription(PoseStamped, f'/{n}/pose', lambda m, n=n: self.pose.__setitem__(n, m), 10)
            self.create_subscription(String, f'/{n}/status', lambda m, n=n: self.status.__setitem__(n, m.data), 10)
            self.create_subscription(Float32, f'/{n}/zrange', lambda m, n=n: self._on_zrange(n, m), 10)
        self.create_subscription(String, '/swarm/mode', self._on_mode, 10)
        self.create_subscription(String, '/objects/json', self._on_objects, 10)
        self.create_subscription(PointStamped, '/clicked_point', self._on_clicked, 10)
        self.event_pub = self.create_publisher(String, '/swarm/events', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/swarm/targets', 10)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f'coordinator up: drones={self.names} mode={self.mode}')

    # ---------------- inputs ----------------
    def _on_mode(self, m):
        parts = m.data.strip().split()
        if not parts:
            return
        cmd = parts[0].lower()
        if cmd in ('follow', 'auto'):
            self.mode, self.auto = 'follow', cmd == 'auto'
            self.hold.clear()
        elif cmd == 'hold':
            self.mode = 'hold'
            self.hold.clear()
        elif cmd == 'inspect' and len(parts) == 2:
            self._assign(parts[1], manual=True)
        elif cmd == 'goto' and len(parts) >= 3:
            try:
                if parts[1] in self.names:
                    nums = [float(v) for v in parts[2:]]
                    self._send_point(nums[0], nums[1], nums[2] if len(nums) > 2 else None, drone=parts[1])
                else:
                    nums = [float(v) for v in parts[1:]]
                    self._send_point(nums[0], nums[1], nums[2] if len(nums) > 2 else None)
            except (ValueError, IndexError):
                self.get_logger().warn('usage: goto [cf1|cf2] x y [z]')
                return
        elif cmd == 'fly' and len(parts) == 2:
            wp = self._load_waypoints().get(parts[1])
            if wp is None:
                self.get_logger().warn(f'no waypoint "{parts[1]}" in {self.wp_file}')
                return
            self._send_point(wp[0], wp[1], None, label=parts[1])
        elif cmd == 'home':
            for n in self.names:
                self.tasks[n] = {'kind': 'slot'}
            self.mode = 'follow'
            self.hold.clear()
        else:
            self.get_logger().warn(f'unknown mode "{m.data}"')
            return
        self.get_logger().info(f'mode -> {m.data} (auto_inspect={self.auto})')

    def _on_objects(self, m):
        try:
            self.objects = {str(o['id']): o for o in json.loads(m.data)['objects']}
        except (ValueError, KeyError) as e:
            self.get_logger().warn(f'bad /objects/json: {e}', throttle_duration_sec=5.0)

    def _on_zrange(self, n, m):
        task = self.tasks[n]
        if task['kind'] != 'inspect' or self.pose[n] is None or math.isnan(m.data):
            return
        obj = self.objects.get(task['obj'])
        if obj is None:
            return
        p = self.pose[n].pose.position
        self.zr[n].append((time.monotonic(), m.data, math.hypot(p.x - obj['x'], p.y - obj['y'])))

    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, Time())
        except TransformException as e:
            self.get_logger().warn(f'no robot TF: {e}', throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        x, y, yaw = t.x, t.y, yaw_from_quat(tf.transform.rotation)
        if self.robot is None:
            self.robot = [x, y, yaw]
        else:
            a = self.alpha
            self.robot[0] += a * (x - self.robot[0])
            self.robot[1] += a * (y - self.robot[1])
            self.robot[2] = wrap(self.robot[2] + a * wrap(yaw - self.robot[2]))
        return tuple(self.robot)

    # ---------------- task allocation ----------------
    def _free_flying(self):
        return [n for n in self.names if self.status[n] == 'FLYING'
                and self.tasks[n]['kind'] == 'slot' and self.pose[n] is not None]

    def _assign(self, obj_id, manual=False):
        obj = self.objects.get(obj_id)
        if obj is None:
            if manual:
                self.get_logger().warn(f'object {obj_id} not known')
            return False
        free = self._free_flying()
        if not free:
            if manual:
                self.get_logger().warn('no free flying drone')
            return False
        def dist(n):
            p = self.pose[n].pose.position
            return math.hypot(p.x - obj['x'], p.y - obj['y'])
        n = min(free, key=dist)
        self.tasks[n] = {'kind': 'inspect', 'obj': obj_id, 't_arrive': None}
        self.zr[n].clear()
        self._event(n, 'inspect_start', obj)
        return True

    def _load_waypoints(self):
        try:
            with open(self.wp_file) as f:
                return yaml.safe_load(f) or {}
        except (OSError, yaml.YAMLError):
            return {}

    def _on_clicked(self, m):
        if m.header.frame_id not in ('', self.map_frame):
            self.get_logger().warn(f'clicked point is in {m.header.frame_id}, set RViz Fixed Frame to {self.map_frame}')
            return
        self._send_point(m.point.x, m.point.y, None, label='clicked')

    def _send_point(self, x, y, z, drone=None, label=None):
        """Send a drone to hover at map point (x, y, z). Nearest free flying drone if drone is None."""
        z = self.point_z if z is None else z
        if drone is None:
            cands = [n for n in self.names if self.status[n] == 'FLYING' and self.pose[n] is not None
                     and self.tasks[n]['kind'] != 'inspect']
            if not cands:
                self.get_logger().warn('no flying drone available')
                return False
            def dist(n):
                p = self.pose[n].pose.position
                return math.hypot(p.x - x, p.y - y)
            drone = min(cands, key=dist)
        if self.mode == 'hold':
            self.mode = 'follow'
            self.hold.clear()
        label = label or f'({x:.2f},{y:.2f})'
        self.tasks[drone] = {'kind': 'point', 'x': x, 'y': y, 'z': z, 'label': label, 't_arrive': None}
        self._event(drone, 'goto', {'id': label, 'cls': 'point', 'x': x, 'y': y}, z=z)
        return True

    def _auto_assign(self):
        if self.robot is None:
            return
        busy = {t.get('obj') for t in self.tasks.values()}
        for oid, o in self.objects.items():
            if oid in self.inspected or oid in busy:
                continue
            if self.inspect_classes and o['cls'] not in self.inspect_classes:
                continue
            if math.hypot(o['x'] - self.robot[0], o['y'] - self.robot[1]) > self.max_inspect_dist:
                continue
            if self._assign(oid):
                return  # one new assignment per tick

    def _tof_height(self, n):
        """EXPERIMENTAL. Floor->ToF range before the object minus the minimum range over it."""
        samples = list(self.zr[n])
        before = [z for _, z, d in samples if d > 0.35]
        over = [z for _, z, d in samples if d < 0.12]
        if len(before) < 3 or not over:
            return None
        h = statistics.median(before) - min(over)
        return round(h, 3) if h > 0.04 else 0.0

    def _event(self, n, kind, obj, **extra):
        ev = {'t': time.time(), 'drone': n, 'event': kind, 'object': obj.get('id'), 'cls': obj.get('cls')}
        ev.update(extra)
        self.event_pub.publish(String(data=json.dumps(ev)))
        self.get_logger().info(f'event: {ev}')

    # ---------------- main loop ----------------
    def _tick(self):
        robot = self._robot_pose()
        if robot is None:
            return  # bridge will hold, then land after land_timeout
        if self.auto and self.mode == 'follow':
            self._auto_assign()

        now = time.monotonic()
        targets = {}
        for n in self.names:
            task = self.tasks[n]
            if self.mode == 'hold':
                if n not in self.hold and self.pose[n] is not None:
                    p = self.pose[n].pose
                    self.hold[n] = (p.position.x, p.position.y, p.position.z, yaw_from_quat(p.orientation))
                if n in self.hold:
                    targets[n] = self.hold[n]
                continue
            if task['kind'] == 'point':
                if self.status[n] != 'FLYING':
                    self.tasks[n] = {'kind': 'slot'}
                else:
                    targets[n] = (task['x'], task['y'], task['z'], robot[2])
                    if self.pose[n] is not None and task['t_arrive'] is None:
                        p = self.pose[n].pose.position
                        if math.hypot(p.x - task['x'], p.y - task['y']) < self.arrive_r:
                            task['t_arrive'] = now
                            self._event(n, 'arrived', {'id': task['label'], 'cls': 'point'},
                                        pose=[round(p.x, 2), round(p.y, 2), round(p.z, 2)])
                    continue
            if task['kind'] == 'inspect':
                obj = self.objects.get(task['obj'])
                if obj is None or self.status[n] != 'FLYING':
                    self.tasks[n] = {'kind': 'slot'}
                else:
                    targets[n] = (obj['x'], obj['y'], self.inspect_z, robot[2])
                    if self.pose[n] is not None:
                        p = self.pose[n].pose.position
                        if math.hypot(p.x - obj['x'], p.y - obj['y']) < self.arrive_r:
                            if task['t_arrive'] is None:
                                task['t_arrive'] = now
                                self._event(n, 'arrived', obj)
                            elif now - task['t_arrive'] > self.inspect_time:
                                self.inspected.add(task['obj'])
                                self._event(n, 'inspect_done', obj, tof_height_m=self._tof_height(n),
                                            lidar_height_m=obj.get('h'))
                                self.tasks[n] = {'kind': 'slot'}
                    continue
            sx, sy, sz = self.slots[n]
            x, y = se2_apply(robot, (sx, sy))
            targets[n] = (x, y, sz, robot[2])

        self._separate(targets)
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        for i, (n, (x, y, z, yaw)) in enumerate(targets.items()):
            g = PoseStamped()
            g.header.stamp, g.header.frame_id = stamp, self.map_frame
            g.pose.position.x, g.pose.position.y, g.pose.position.z = x, y, z
            g.pose.orientation = quat_from_yaw(yaw)
            self.goal_pub[n].publish(g)
            mk = Marker()
            mk.header = g.header
            mk.ns, mk.id, mk.type, mk.action = 'drone_targets', i, Marker.SPHERE, Marker.ADD
            mk.pose = g.pose
            mk.scale.x = mk.scale.y = mk.scale.z = 0.15
            inspecting = self.tasks[n]['kind'] in ('inspect', 'point')
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = (1.0, 0.4, 0.0, 0.8) if inspecting else (0.0, 0.6, 1.0, 0.8)
            markers.markers.append(mk)
        self.marker_pub.publish(markers)

    def _separate(self, targets):
        """Push goals apart in xy so no two drones are commanded closer than min_sep."""
        names = list(targets)
        for _ in range(3):
            for i in range(len(names)):
                for j in range(i + 1, len(names)):
                    a, b = list(targets[names[i]]), list(targets[names[j]])
                    dx, dy = b[0] - a[0], b[1] - a[1]
                    d = math.hypot(dx, dy)
                    if d >= self.min_sep:
                        continue
                    ux, uy = (dx / d, dy / d) if d > 1e-3 else (0.0, 1.0)
                    gap = self.min_sep - d
                    # an inspecting drone keeps its spot; the other one yields
                    wa = 0.0 if self.tasks[names[i]]['kind'] in ('inspect', 'point') else 1.0
                    wb = 0.0 if self.tasks[names[j]]['kind'] in ('inspect', 'point') else 1.0
                    if wa + wb == 0.0:
                        wa = wb = 1.0
                    a[0] -= ux * gap * wa / (wa + wb)
                    a[1] -= uy * gap * wa / (wa + wb)
                    b[0] += ux * gap * wb / (wa + wb)
                    b[1] += uy * gap * wb / (wa + wb)
                    targets[names[i]], targets[names[j]] = tuple(a), tuple(b)


def main():
    rclpy.init()
    node = SwarmCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
