#!/usr/bin/env python3
"""straight_avoid: drive along a straight line, swerve around obstacles, rejoin the line.

Runs ON THE ROBOT (no Wi-Fi in the control loop). Needs only /scan and /odom.

    python3 ~/straight_avoid.py                       # 3 m straight ahead, 0.12 m/s
    python3 ~/straight_avoid.py --ros-args -p distance:=5.0 -p speed:=0.15
    distance:=0  -> drive forever (until Ctrl+C)
    -p legs:=4   -> back and forth on the SAME line: out, turn around, back, turn, ... (0 = forever)
  When finished it prints DONE and exits.

How it works (every 50 ms):
  1. The line = robot pose at start (x0, y0, heading0).
  2. Desired heading = pure pursuit back onto that line (aim at a point `lookahead` m ahead on it).
  3. For candidate headings -90..+90 deg, compute how far the robot's footprint
     (a corridor robot_width + 2*margin wide) can travel before hitting a lidar point.
  4. Pick the candidate closest to the desired heading whose corridor is free for >= clear_dist.
     Hysteresis keeps it on the side it picked, so it does not zig-zag in front of an obstacle.
  5. Nothing free -> turn in place toward the most open direction.
  Hard stop if anything is closer than stop_dist in front. Ctrl+C sends zero velocity.
Status on /straight/status (std_msgs/String).
"""
import math
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def corridor_free(points, theta, half_w, max_d):
    """Distance the robot can drive along relative heading theta before its corridor hits a point."""
    c, s = math.cos(theta), math.sin(theta)
    best = max_d
    for x, y in points:
        along = x * c + y * s
        if along <= 0.0 or along >= best:
            continue
        if abs(-x * s + y * c) < half_w:
            best = along
    return best


def scan_points(scan, max_r):
    pts = []
    a = scan.angle_min
    for r in scan.ranges:
        if math.isfinite(r) and scan.range_min < r < max_r:
            pts.append((r * math.cos(a), r * math.sin(a)))
        a += scan.angle_increment
    return pts


class StraightAvoid(Node):
    def __init__(self):
        super().__init__('straight_avoid')
        dp = self.declare_parameter
        self.distance = dp('distance', 3.0).value        # 0 = forever
        self.speed = dp('speed', 0.12).value             # m/s cruise
        self.w_max = dp('max_turn', 0.8).value           # rad/s
        self.robot_w = dp('robot_width', 0.31).value     # Waffle ~0.306 m
        self.margin = dp('margin', 0.08).value           # extra clearance each side
        self.clear_dist = dp('clear_dist', 0.6).value    # corridor must be free this far
        self.stop_dist = dp('stop_dist', 0.18).value     # hard stop (from lidar centre)
        self.lookahead = dp('lookahead', 0.8).value      # pure pursuit onto the line
        self.legs = int(dp('legs', 1).value)             # 1 = one way; N = back and forth; 0 = forever
        self.leg = 1
        self.turn_target = None
        self.scan_off = dp('lidar_x_offset', -0.064).value  # base_scan is 6.4 cm behind base on waffle
        self.half_w = self.robot_w / 2 + self.margin
        self.cands = [math.radians(d) for d in range(-90, 91, 5)]

        self.line = None
        self.pose = None
        self.scan = None
        self.side = 0.0          # +1 left, -1 right, 0 none (hysteresis)
        self.done = False
        self.state = 'WAIT'
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status = self.create_publisher(String, '/straight/status', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self._tick)
        self.create_timer(0.5, self._report)
        self.get_logger().info(f'straight_avoid: distance={self.distance} m speed={self.speed} m/s '
                               f'legs={self.legs or "forever"}')

    def _on_odom(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.line is None:
            self.line = self.pose
            self.get_logger().info(f'line start ({self.line[0]:.2f}, {self.line[1]:.2f}) '
                                   f'heading {math.degrees(self.line[2]):.0f} deg')

    def _on_scan(self, m):
        self.scan = (self.get_clock().now(), m)

    def _publish(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd.publish(t)

    def _tick(self):
        if self.done or self.pose is None or self.line is None or self.scan is None:
            return
        stamp, scan = self.scan
        if (self.get_clock().now() - stamp).nanoseconds > 0.5e9:
            self.state = 'NO_SCAN'
            self._publish(0, 0)
            self.get_logger().warn('no lidar data for 0.5 s - holding still', throttle_duration_sec=2.0)
            return

        x0, y0, h0 = self.line
        x, y, yaw = self.pose
        dx, dy = x - x0, y - y0
        progress = dx * math.cos(h0) + dy * math.sin(h0)
        lateral = -dx * math.sin(h0) + dy * math.cos(h0)       # + = left of the line
        self.progress, self.lateral = progress, lateral
        if self.turn_target is not None:                   # turning around at the end of a leg
            err = wrap(self.turn_target - yaw)
            if abs(err) < math.radians(8):
                self.turn_target = None
                self.get_logger().info(f'leg {self.leg} start')
            else:
                self.state = 'TURN_AROUND'
                self._publish(0.0, max(-0.6, min(0.6, 1.5 * err)))
                return

        if self.distance > 0 and progress >= self.distance:
            self.get_logger().info(f'leg {self.leg} finished: {progress:.2f} m along line, '
                                   f'{lateral:+.2f} m off it')
            if self.legs == 0 or self.leg < self.legs:
                # same geometric line, opposite direction, starting at the end point ON the line
                ex, ey = x0 + self.distance * math.cos(h0), y0 + self.distance * math.sin(h0)
                self.line = (ex, ey, wrap(h0 + math.pi))
                self.turn_target = self.line[2]
                self.leg += 1
                self.side = 0.0
                self._publish(0, 0)
                return
            self.state = 'DONE'
            self.done = True
            self._publish(0, 0)
            self.get_logger().info('DONE - all legs finished')

        # lidar points in base frame (shift by lidar offset)
        pts = [(px + self.scan_off, py) for px, py in scan_points(scan, 3.0)]
        front = [math.hypot(px, py) for px, py in pts if px > 0 and abs(math.atan2(py, px)) < math.radians(40)]
        if front and min(front) < self.stop_dist:
            self.state = 'TOO_CLOSE'
            self.get_logger().warn(f'obstacle {min(front):.2f} m in front - backing off', throttle_duration_sec=1.0)
            # back off slightly and turn toward the freer side
            left = corridor_free(pts, math.radians(60), self.half_w, 2.0)
            right = corridor_free(pts, math.radians(-60), self.half_w, 2.0)
            self._publish(-0.03, self.w_max * (1 if left >= right else -1))
            return

        desired = wrap(h0 + math.atan2(-lateral, self.lookahead) - yaw)   # relative to robot
        free = {th: corridor_free(pts, th, self.half_w, 2.0) for th in self.cands}
        ok = [th for th in self.cands if free[th] >= self.clear_dist]

        if not ok:
            self.state = 'TURNING'
            best = max(self.cands, key=lambda th: free[th])
            self.side = 1.0 if best > 0 else -1.0
            self._publish(0.0, self.w_max * self.side)
            return

        blocked_ahead = free[min(self.cands, key=lambda th: abs(wrap(th - desired)))] < self.clear_dist

        def cost(th):
            c = abs(wrap(th - desired))
            # only while blocked: stay on the side already chosen (no zig-zag in front of it)
            if blocked_ahead and self.side and th * self.side < -0.05:
                c += 0.6
            return c
        theta = min(ok, key=cost)

        if blocked_ahead:
            self.state = 'AVOIDING'
            if not self.side:
                self.side = 1.0 if theta > 0 else -1.0
        else:
            self.state = 'RETURNING' if abs(lateral) > 0.08 else 'FOLLOWING'
            self.side = 0.0

        w = max(-self.w_max, min(self.w_max, 2.0 * theta))
        slow = min(1.0, free[theta] / 1.0)
        v = self.speed * max(0.0, math.cos(theta)) * max(0.3, slow)
        if abs(theta) > math.radians(50):
            v = min(v, 0.03)
        self._publish(v, w)

    def _report(self):
        p = getattr(self, 'progress', 0.0)
        l = getattr(self, 'lateral', 0.0)
        if self.done:
            return
        if self.pose is None or self.scan is None:
            missing = ' and '.join(n for n, v in (('/odom', self.pose), ('/scan', self.scan)) if v is None)
            self.get_logger().warn(f'WAITING for {missing} - is the robot bringup running?',
                                   throttle_duration_sec=2.0)
            return
        self.status.publish(String(data=f'leg {self.leg} {self.state} progress={p:.2f}m lateral={l:+.2f}m'))
        self.get_logger().info(f'leg {self.leg} {self.state:11s} progress {p:5.2f} m  lateral {l:+.2f} m',
                               throttle_duration_sec=1.0)


STOP = {'flag': False}


def _on_signal(signum, _frame):
    STOP['flag'] = True


def main():
    # Own signal handling: rclpy's default handler shuts ROS down on Ctrl+C BEFORE we can
    # publish the final zero velocity, and the TurtleBot keeps driving with the last command.
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):   # Ctrl+C, kill, SSH drop
        signal.signal(sig, _on_signal)
    node = StraightAvoid()
    try:
        while rclpy.ok() and not STOP['flag'] and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    except SystemExit:
        pass
    finally:
        for _ in range(10):                      # make sure "stop" really reaches the robot
            node._publish(0.0, 0.0)
            time.sleep(0.02)
        node.get_logger().info('stopped - zero velocity sent')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
