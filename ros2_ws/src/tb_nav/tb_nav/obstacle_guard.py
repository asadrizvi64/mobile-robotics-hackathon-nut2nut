"""obstacle_guard: lidar safety layer + simple autonomous wander.

Reads /scan, measures the closest obstacle in a front cone (and a rear cone).

Modes (std_msgs/String on /guard/mode):
  guard   (default) pass /cmd_vel_raw -> /cmd_vel, but block forward motion when
          something is closer than stop_dist in front (turning and reversing still allowed).
          Speed is scaled down smoothly between slow_dist and stop_dist.
  wander  drive forward on its own; when blocked, turn toward the more open side; repeat.
  off     forward everything unchanged.

Teleop must publish to /cmd_vel_raw for the guard to sit in between:
  ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r cmd_vel:=cmd_vel_raw

Outputs:
  /guard/front_blocked   std_msgs/Bool
  /guard/status          std_msgs/String JSON {front_m, rear_m, left_m, right_m, blocked, mode}
  /guard/markers         visualization_msgs/MarkerArray (front cone: green / orange / red)
"""
import json
import math
import time

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray


def sector_min(scan, center, half_width, max_range=None):
    """Robust closest range in [center - half_width, center + half_width] (rad, robot frame).

    LDS-01 reports 0.0 for 'no return', so zeros / inf / nan are ignored.
    Uses the 2nd-smallest valid reading to reject a single noisy point.
    """
    vals = []
    a = scan.angle_min
    rmax = max_range or scan.range_max
    for r in scan.ranges:
        d = math.atan2(math.sin(a - center), math.cos(a - center))
        if abs(d) <= half_width and math.isfinite(r) and scan.range_min < r < rmax:
            vals.append(r)
        a += scan.angle_increment
    if not vals:
        return float('inf')
    vals.sort()
    return vals[1] if len(vals) > 1 else vals[0]


class ObstacleGuard(Node):
    def __init__(self):
        super().__init__('obstacle_guard')
        dp = self.declare_parameter
        self.mode = dp('mode', 'guard').value
        self.stop_dist = dp('stop_dist', 0.35).value       # m, measured from the lidar
        self.slow_dist = dp('slow_dist', 0.70).value
        self.half_fov = math.radians(dp('front_half_angle_deg', 25.0).value)
        self.wander_speed = dp('wander_speed', 0.08).value
        self.wander_turn = dp('wander_turn', 0.5).value
        self.cmd_timeout = dp('cmd_timeout', 0.5).value
        self.base_frame = dp('base_frame', 'base_scan').value

        self.scan = None
        self.t_scan = 0.0
        self.raw = Twist()
        self.t_raw = 0.0
        self.turn_dir = 0.0
        self.d = {'front': float('inf'), 'rear': float('inf'), 'left': float('inf'), 'right': float('inf')}

        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel_raw', self._on_raw, 10)
        self.create_subscription(String, '/guard/mode', self._on_mode, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.blocked_pub = self.create_publisher(Bool, '/guard/front_blocked', 10)
        self.status_pub = self.create_publisher(String, '/guard/status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/guard/markers', 10)
        self.create_timer(0.05, self._tick)   # 20 Hz control
        self.create_timer(0.25, self._report)
        self.was_blocked = False
        self.get_logger().info(f'obstacle_guard up: mode={self.mode} stop={self.stop_dist} m slow={self.slow_dist} m')

    def _on_mode(self, m):
        mode = m.data.strip().lower()
        if mode in ('guard', 'wander', 'off'):
            self.mode = mode
            self.turn_dir = 0.0
            self.cmd_pub.publish(Twist())
            self.get_logger().info(f'mode -> {mode}')
        else:
            self.get_logger().warn('mode must be guard | wander | off')

    def _on_scan(self, m):
        self.scan, self.t_scan = m, time.monotonic()
        self.d['front'] = sector_min(m, 0.0, self.half_fov)
        self.d['rear'] = sector_min(m, math.pi, self.half_fov)
        self.d['left'] = sector_min(m, math.pi / 2, math.radians(40))
        self.d['right'] = sector_min(m, -math.pi / 2, math.radians(40))

    def _on_raw(self, m):
        self.raw, self.t_raw = m, time.monotonic()

    def _scale(self, dist):
        """1 = full speed, 0 = stop; linear ramp between stop_dist and slow_dist."""
        if dist <= self.stop_dist:
            return 0.0
        if dist >= self.slow_dist:
            return 1.0
        return (dist - self.stop_dist) / (self.slow_dist - self.stop_dist)

    def _tick(self):
        now = time.monotonic()
        scan_ok = self.scan is not None and now - self.t_scan < 1.0
        out = Twist()

        if self.mode == 'off':
            if now - self.t_raw < self.cmd_timeout:
                self.cmd_pub.publish(self.raw)
            return

        if not scan_ok:
            # no lidar = no motion (except in 'off'); warn but keep publishing zeros
            self.get_logger().warn('no /scan - holding still', throttle_duration_sec=2.0)
            self.cmd_pub.publish(out)
            return

        if self.mode == 'guard':
            if now - self.t_raw > self.cmd_timeout:
                return  # teleop idle: publish nothing, so Nav2 can still drive if used
            out.angular.z = self.raw.angular.z
            v = self.raw.linear.x
            if v > 0:
                v *= self._scale(self.d['front'])
            elif v < 0:
                v *= self._scale(self.d['rear'])
            out.linear.x = v
            self.cmd_pub.publish(out)

        elif self.mode == 'wander':
            f = self.d['front']
            if f <= self.stop_dist * 1.3 or self.turn_dir != 0.0:
                if self.turn_dir == 0.0:  # pick the more open side once, then commit
                    self.turn_dir = 1.0 if self.d['left'] > self.d['right'] else -1.0
                out.angular.z = self.turn_dir * self.wander_turn
                if f > self.slow_dist:  # front is clear again
                    self.turn_dir = 0.0
            else:
                out.linear.x = self.wander_speed * max(0.3, self._scale(f))
            self.cmd_pub.publish(out)

    def _report(self):
        blocked = self.d['front'] <= self.stop_dist
        if blocked != self.was_blocked:
            self.get_logger().info(f"front {'BLOCKED' if blocked else 'clear'} ({self.d['front']:.2f} m)")
            self.was_blocked = blocked
        self.blocked_pub.publish(Bool(data=blocked))
        st = {k + '_m': (round(v, 2) if math.isfinite(v) else None) for k, v in self.d.items()}
        st.update(blocked=blocked, mode=self.mode)
        self.status_pub.publish(String(data=json.dumps(st)))
        self._markers(blocked)

    def _markers(self, blocked):
        f = self.d['front']
        mk = Marker()
        mk.header.frame_id = self.base_frame
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns, mk.id, mk.type, mk.action = 'guard', 0, Marker.TRIANGLE_LIST, Marker.ADD
        mk.pose.orientation.w = 1.0
        mk.scale.x = mk.scale.y = mk.scale.z = 1.0
        reach = min(f, self.slow_dist) if math.isfinite(f) else self.slow_dist
        n = 8
        for i in range(n):
            a0 = -self.half_fov + 2 * self.half_fov * i / n
            a1 = -self.half_fov + 2 * self.half_fov * (i + 1) / n
            mk.points += [Point(), Point(x=reach * math.cos(a0), y=reach * math.sin(a0)),
                          Point(x=reach * math.cos(a1), y=reach * math.sin(a1))]
        if blocked:
            mk.color.r, mk.color.g, mk.color.b = 1.0, 0.1, 0.1
        elif f < self.slow_dist:
            mk.color.r, mk.color.g, mk.color.b = 1.0, 0.6, 0.0
        else:
            mk.color.r, mk.color.g, mk.color.b = 0.1, 0.9, 0.2
        mk.color.a = 0.5
        txt = Marker()
        txt.header = mk.header
        txt.ns, txt.id, txt.type, txt.action = 'guard', 1, Marker.TEXT_VIEW_FACING, Marker.ADD
        txt.pose.position.x, txt.pose.position.z = 0.4, 0.3
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.12
        txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
        txt.text = ('BLOCKED ' if blocked else '') + (f'{f:.2f} m' if math.isfinite(f) else 'clear')
        self.marker_pub.publish(MarkerArray(markers=[mk, txt]))

    def destroy_node(self):
        self.cmd_pub.publish(Twist())
        super().destroy_node()


def main():
    rclpy.init()
    node = ObstacleGuard()
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
