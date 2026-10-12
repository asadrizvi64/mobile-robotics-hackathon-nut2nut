"""nav_manager: one node that drives the TurtleBot through Nav2.

Works in both modes:
  slam          slam_toolbox builds the map and localizes; Nav2 navigates on the live map
  localization  map_server + AMCL on a saved map; Nav2 navigates on it

Commands (std_msgs/String on /nav/command):
  goto <x> <y> [yaw_deg]     drive to a map coordinate
  goto <name>                drive to a saved waypoint
  save <name>                save the robot's current pose as a waypoint (written to waypoints_file)
  delete <name>              remove a waypoint
  list                       print waypoints
  patrol [loops]             visit all waypoints in order (loops=0 -> forever)
  object <id>                drive to an object from /objects/json, stop `standoff` m in front, face it
  init <x> <y> [yaw_deg]     set AMCL initial pose (localization mode)
  relocalize                 AMCL global localization + spin in place
  cancel                     stop

Outputs:
  /nav/status   std_msgs/String JSON: state, goal, distance_remaining, eta_s, recoveries, localized, pose
  /nav/events   std_msgs/String JSON: arrived / failed / patrol_done ...

Direct action client (no nav2_simple_commander), so it does not block waiting for
AMCL lifecycle states that don't exist in SLAM mode.
"""
import json
import math
import os
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Empty as EmptySrv
from tf2_ros import Buffer, TransformException, TransformListener

STATUS_NAMES = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_ABORTED: 'aborted',
                GoalStatus.STATUS_CANCELED: 'canceled'}


# ---------------- pure helpers (unit-testable without ROS) ----------------
def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def standoff_goal(robot_xy, obj_xy, standoff):
    """Goal `standoff` metres short of the object on the line robot->object, facing the object."""
    dx, dy = obj_xy[0] - robot_xy[0], obj_xy[1] - robot_xy[1]
    d = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx) if d > 1e-6 else 0.0
    if d <= standoff:
        return robot_xy[0], robot_xy[1], yaw  # already close: just turn to face it
    k = (d - standoff) / d
    return robot_xy[0] + k * dx, robot_xy[1] + k * dy, yaw


def localized_from_cov(cov, xy_std_max, yaw_std_max):
    xy_std = math.sqrt(max(cov[0], 0.0) + max(cov[7], 0.0))
    yaw_std = math.sqrt(max(cov[35], 0.0))
    return xy_std < xy_std_max and yaw_std < yaw_std_max, xy_std, yaw_std


class NavManager(Node):
    def __init__(self):
        super().__init__('nav_manager')
        dp = self.declare_parameter
        self.mode = dp('mode', 'slam').value  # slam | localization
        self.map_frame = dp('map_frame', 'map').value
        self.robot_frame = dp('robot_frame', 'base_footprint').value
        self.wp_file = os.path.expanduser(dp('waypoints_file', '~/tb_waypoints.yaml').value)
        self.standoff = dp('object_standoff', 0.5).value
        self.require_localized = dp('require_localized', True).value
        self.xy_std_max = dp('localized_xy_std', 0.3).value
        self.yaw_std_max = dp('localized_yaw_std', 0.35).value
        self.retry = dp('retries_per_goal', 1).value
        init = dp('initial_pose', [0.0, 0.0, 0.0]).value       # x, y, yaw_deg
        self.auto_init = dp('set_initial_pose', False).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.global_loc = self.create_client(EmptySrv, 'reinitialize_global_localization')

        self.status_pub = self.create_publisher(String, '/nav/status', 10)
        self.event_pub = self.create_publisher(String, '/nav/events', 10)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/nav/command', self._on_command, 10)
        self.create_subscription(String, '/objects/json', self._on_objects, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, latched)

        self.waypoints = self._load_waypoints()
        self.objects = {}
        self.localized = self.mode == 'slam'  # slam_toolbox localizes itself
        self.loc_std = (0.0, 0.0)

        self.state = 'IDLE'      # IDLE | NAVIGATING | PATROL
        self.seq = 0             # bumps on every new goal; stale callbacks are ignored
        self.goal_handle = None
        self.current = None      # dict(label, x, y, yaw, tries)
        self.queue = []          # pending (label, x, y, yaw) for patrol
        self.patrol_loops = None
        self.fb = {}

        self.pending_init = tuple(init) if (self.auto_init and self.mode == 'localization') else None
        self.create_timer(0.5, self._tick)
        self.get_logger().info(f'nav_manager up: mode={self.mode}, {len(self.waypoints)} waypoints from {self.wp_file}')

    # ---------------- waypoints ----------------
    def _load_waypoints(self):
        try:
            with open(self.wp_file) as f:
                data = yaml.safe_load(f) or {}
            return {k: tuple(v) for k, v in data.items()}
        except FileNotFoundError:
            return {}
        except (OSError, yaml.YAMLError) as e:
            self.get_logger().error(f'cannot read {self.wp_file}: {e}')
            return {}

    def _save_waypoints(self):
        with open(self.wp_file, 'w') as f:
            yaml.safe_dump({k: [round(v, 3) for v in p] for k, p in self.waypoints.items()}, f, sort_keys=False)

    # ---------------- inputs ----------------
    def _on_objects(self, m):
        try:
            self.objects = {str(o['id']): o for o in json.loads(m.data)['objects']}
        except (ValueError, KeyError):
            pass

    def _on_amcl(self, m):
        ok, xy, yaw = localized_from_cov(m.pose.covariance, self.xy_std_max, self.yaw_std_max)
        if ok and not self.localized:
            self.get_logger().info(f'localized (xy std {xy:.2f} m, yaw std {math.degrees(yaw):.1f} deg)')
        self.localized, self.loc_std = ok, (xy, yaw)

    def robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, Time())
        except TransformException:
            return None
        t = tf.transform.translation
        return t.x, t.y, quat_to_yaw(tf.transform.rotation)

    def _on_command(self, m):
        parts = m.data.strip().split()
        if not parts:
            return
        cmd, args = parts[0].lower(), parts[1:]
        try:
            if cmd == 'goto' and len(args) == 1:
                if args[0] not in self.waypoints:
                    return self._warn(f'unknown waypoint "{args[0]}" (have: {list(self.waypoints)})')
                self._start([(args[0], *self.waypoints[args[0]])], 'NAVIGATING')
            elif cmd == 'goto' and len(args) in (2, 3):
                x, y = float(args[0]), float(args[1])
                yaw = math.radians(float(args[2])) if len(args) == 3 else self._yaw_towards(x, y)
                self._start([(f'({x:.2f},{y:.2f})', x, y, yaw)], 'NAVIGATING')
            elif cmd == 'save' and len(args) == 1:
                p = self.robot_pose()
                if p is None:
                    return self._warn('no robot pose in map yet')
                self.waypoints[args[0]] = p
                self._save_waypoints()
                self.get_logger().info(f'saved "{args[0]}" = ({p[0]:.2f}, {p[1]:.2f}, {math.degrees(p[2]):.0f} deg)')
            elif cmd == 'delete' and len(args) == 1:
                self.waypoints.pop(args[0], None)
                self._save_waypoints()
            elif cmd == 'list':
                for k, (x, y, yaw) in self.waypoints.items():
                    self.get_logger().info(f'  {k}: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)')
            elif cmd == 'patrol':
                if not self.waypoints:
                    return self._warn('no waypoints saved; use "save <name>" first')
                self.patrol_loops = int(args[0]) if args else 1
                self._start([(k, *v) for k, v in self.waypoints.items()], 'PATROL')
            elif cmd == 'object' and len(args) == 1:
                self._goto_object(args[0])
            elif cmd == 'init' and len(args) in (2, 3):
                self._publish_initial_pose(float(args[0]), float(args[1]), float(args[2]) if len(args) == 3 else 0.0)
            elif cmd == 'relocalize':
                self._relocalize()
            elif cmd in ('cancel', 'stop'):
                self._cancel('user cancel')
            else:
                self._warn(f'unknown command "{m.data}"')
        except ValueError as e:
            self._warn(f'bad command "{m.data}": {e}')

    def _warn(self, msg):
        self.get_logger().warn(msg)

    def _yaw_towards(self, x, y):
        p = self.robot_pose()
        return math.atan2(y - p[1], x - p[0]) if p else 0.0

    def _goto_object(self, oid):
        obj = self.objects.get(oid)
        if obj is None:
            return self._warn(f'object {oid} not in /objects/json (have: {list(self.objects)})')
        p = self.robot_pose()
        if p is None:
            return self._warn('no robot pose in map yet')
        gx, gy, gyaw = standoff_goal((p[0], p[1]), (obj['x'], obj['y']), self.standoff)
        self._start([(f"object {oid} ({obj['cls']})", gx, gy, gyaw)], 'NAVIGATING')

    # ---------------- localization helpers ----------------
    def _publish_initial_pose(self, x, y, yaw_deg):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        q = yaw_to_quat(math.radians(yaw_deg))
        o = m.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25 ** 2
        cov[35] = math.radians(15) ** 2
        m.pose.covariance = cov
        self.init_pub.publish(m)
        self.localized = False
        self.get_logger().info(f'initial pose sent: ({x:.2f}, {y:.2f}, {yaw_deg:.0f} deg)')

    def _relocalize(self):
        if not self.global_loc.service_is_ready():
            return self._warn('AMCL global localization service not available (localization mode only)')
        self.global_loc.call_async(EmptySrv.Request())
        self.localized = False
        if self.spin_client.server_is_ready():
            g = Spin.Goal()
            g.target_yaw = 2 * math.pi
            self.spin_client.send_goal_async(g)
            self.get_logger().info('global localization: particles scattered, spinning 360 deg')

    # ---------------- navigation ----------------
    def _start(self, goals, state):
        if not self.nav.server_is_ready():
            return self._warn('Nav2 navigate_to_pose server not ready - is navigation running and active?')
        if self.require_localized and not self.localized:
            return self._warn('not localized yet: use "init x y yaw" or RViz 2D Pose Estimate, or "relocalize"')
        self.queue = list(goals)
        self.state = state
        self._send_next()

    def _send_next(self):
        if not self.queue:
            if self.state == 'PATROL' and self.patrol_loops != 1:
                if self.patrol_loops:
                    self.patrol_loops -= 1
                self.queue = [(k, *v) for k, v in self.waypoints.items()]
            else:
                if self.state == 'PATROL':
                    self._event('patrol_done')
                self.state, self.current = 'IDLE', None
                return
        label, x, y, yaw = self.queue.pop(0)
        self.current = {'label': label, 'x': x, 'y': y, 'yaw': yaw, 'tries': 0}
        self._send_current()

    def _send_current(self):
        c = self.current
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = c['x'], c['y']
        o = goal.pose.pose.orientation
        o.x, o.y, o.z, o.w = yaw_to_quat(c['yaw'])
        self.seq += 1
        seq = self.seq
        self.fb = {}
        self.get_logger().info(f"-> {c['label']} ({c['x']:.2f}, {c['y']:.2f}, {math.degrees(c['yaw']):.0f} deg)")
        fut = self.nav.send_goal_async(goal, feedback_callback=lambda f, s=seq: self._on_feedback(s, f))
        fut.add_done_callback(lambda f, s=seq: self._on_accepted(s, f))

    def _on_accepted(self, seq, fut):
        if seq != self.seq:
            return
        gh = fut.result()
        if not gh.accepted:
            self._event('rejected', goal=self.current['label'])
            self.state, self.current, self.queue = 'IDLE', None, []
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(lambda f, s=seq: self._on_result(s, f))

    def _on_feedback(self, seq, msg):
        if seq != self.seq:
            return
        f = msg.feedback
        self.fb = {'distance_remaining': round(f.distance_remaining, 2),
                   'eta_s': round(f.estimated_time_remaining.sec + f.estimated_time_remaining.nanosec * 1e-9, 1),
                   'recoveries': f.number_of_recoveries}

    def _on_result(self, seq, fut):
        if seq != self.seq:
            return  # result of a preempted goal
        status = fut.result().status
        self.goal_handle = None
        label = self.current['label'] if self.current else '?'
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._event('arrived', goal=label)
            self._send_next()
        elif status == GoalStatus.STATUS_CANCELED:
            self._event('canceled', goal=label)
        else:
            if self.current and self.current['tries'] < self.retry:
                self.current['tries'] += 1
                self.get_logger().warn(f'{label}: {STATUS_NAMES.get(status, status)}, retrying')
                self._send_current()
                return
            self._event('failed', goal=label, status=STATUS_NAMES.get(status, str(status)))
            if self.state == 'PATROL':
                self._send_next()  # skip the unreachable waypoint, keep patrolling
            else:
                self.state, self.current = 'IDLE', None

    def _cancel(self, why):
        self.seq += 1
        self.queue = []
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.state, self.current = 'IDLE', None
        self.cmd_vel_pub.publish(Twist())
        self._event('canceled', reason=why)

    def _event(self, kind, **kw):
        ev = {'t': round(time.time(), 2), 'event': kind, **kw}
        self.event_pub.publish(String(data=json.dumps(ev)))
        self.get_logger().info(f'event: {ev}')

    # ---------------- periodic ----------------
    def _tick(self):
        if self.pending_init is not None and self.init_pub.get_subscription_count() > 0:
            self._publish_initial_pose(*self.pending_init)
            self.pending_init = None
        p = self.robot_pose()
        st = {'state': self.state, 'mode': self.mode, 'localized': self.localized,
              'loc_std': [round(v, 3) for v in self.loc_std],
              'pose': [round(p[0], 2), round(p[1], 2), round(math.degrees(p[2]))] if p else None,
              'goal': self.current['label'] if self.current else None,
              'queue': len(self.queue), 'nav2_ready': self.nav.server_is_ready(), **self.fb}
        self.status_pub.publish(String(data=json.dumps(st)))


def main():
    rclpy.init()
    node = NavManager()
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
