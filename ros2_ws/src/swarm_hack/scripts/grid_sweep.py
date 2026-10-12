#!/usr/bin/env python3
"""grid_sweep: drive a grid of waypoints over a square, stop at each point,
take a lidar reading and detect objects from it (map frame).

Earlier experiment (before target_mission): lidar-only object finding on a snake grid, no camera.
Runs on the robot:  python3 grid_sweep.py --ros-args -p side:=2.0 -p points:=4
Results: ~/sweep/points.json (per waypoint) and ~/sweep/detections.json (merged objects).

Params: side (m), points (per side, 5 = 5x5 grid), cx, cy (centre, map frame),
speed, stop_dist, dwell (s), scans (scans averaged per point), out (folder).
"""
import json, math, os, signal, time, warnings
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Sweep(Node):

    def __init__(self):
        super().__init__('grid_sweep')
        p = self.declare_parameter
        p('side', 3.0)
        p('points', 5)
        p('cx', 0.0)
        p('cy', 0.0)
        p('speed', 0.10)
        p('turn_speed', 0.5)
        p('stop_dist', 0.30)
        p('dwell', 3.0)
        p('scans', 10)
        p('cluster_gap', 0.08)     # m, neighbouring lidar points closer than this are one object
        p('max_object', 0.60)      # m, bigger blobs are walls, not objects
        p('merge', 0.20)           # m, detections closer than this are the same object
        p('out', os.path.expanduser('~/sweep'))
        g = lambda k: self.get_parameter(k).value
        self.side = float(g('side'))
        self.n = int(g('points'))
        self.cx, self.cy = float(g('cx')), float(g('cy'))
        self.speed = float(g('speed'))
        self.turn_speed = float(g('turn_speed'))
        self.stop_dist = float(g('stop_dist'))
        self.dwell = float(g('dwell'))
        self.nscans = int(g('scans'))
        self.gap = float(g('cluster_gap'))
        self.max_obj = float(g('max_object'))
        self.merge_r = float(g('merge'))
        self.out = str(g('out'))
        os.makedirs(self.out, exist_ok=True)

        self.ang = None
        self.front = float('inf')
        self.buf = deque(maxlen=self.nscans)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.detections = []       # merged lidar objects: {x, y, hits}
        self.log_points = []

    # ------------------------------------------------------------ inputs
    def on_scan(self, m):
        r = np.asarray(m.ranges, dtype=float)
        self.ang = m.angle_min + np.arange(r.size) * m.angle_increment
        self.buf.append(r)
        ok = np.isfinite(r) & (r > 0.12) & (np.abs(self.ang) < math.radians(25))
        self.front = float(r[ok].min()) if ok.any() else float('inf')

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    # ------------------------------------------------------------ motion
    def cmd(self, v, w):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.pub.publish(t)

    def stop(self):
        for _ in range(3):
            self.pub.publish(Twist())
            time.sleep(0.02)

    def drive_to(self, tx, ty):
        """Returns 'ok', 'blocked' or 'timeout'."""
        t_end = time.monotonic() + 90.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            pose = self.pose()
            if pose is None:
                self.stop()
                continue
            x, y, yaw = pose
            dist = math.hypot(tx - x, ty - y)
            if dist < 0.08:
                self.stop()
                return 'ok'
            if time.monotonic() > t_end:
                self.stop()
                return 'timeout'
            alpha = wrap(math.atan2(ty - y, tx - x) - yaw)
            w = max(-self.turn_speed, min(self.turn_speed, 2.0 * alpha))
            if abs(alpha) > 0.25:
                self.cmd(0.0, w)          # turn on the spot first
            else:
                if self.front < self.stop_dist:
                    self.stop()
                    return 'blocked'
                v = self.speed * (1.0 - abs(alpha) / 0.25 * 0.5)
                self.cmd(v, w)
        return 'timeout'

    # ------------------------------------------------------------ lidar objects
    def lidar_objects(self, pose):
        """Median of recent scans -> points -> clusters -> object centroids (map frame)."""
        if self.ang is None or len(self.buf) == 0:
            return []
        arr = np.array(self.buf)
        arr = np.where((arr > 0.25) & (arr < 3.5), arr, np.nan)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            med = np.nanmedian(arr, axis=0)
        x, y, yaw = pose
        xb = med * np.cos(self.ang)
        yb = med * np.sin(self.ang)
        valid = ~np.isnan(med)

        clusters, cur = [], []
        for i in range(len(med)):
            if not valid[i]:
                if cur:
                    clusters.append(cur)
                    cur = []
                continue
            if cur and math.hypot(xb[i] - xb[cur[-1]], yb[i] - yb[cur[-1]]) > self.gap:
                clusters.append(cur)
                cur = []
            cur.append(i)
        if cur:
            clusters.append(cur)

        found = []
        for c in clusters:
            if len(c) < 2:
                continue
            pts = [(xb[i], yb[i]) for i in c]
            span = max(math.hypot(a[0] - b[0], a[1] - b[1]) for a in pts for b in pts)
            if span > self.max_obj:
                continue          # a wall or a large surface
            bx = sum(p[0] for p in pts) / len(pts)
            by = sum(p[1] for p in pts) / len(pts)
            mx = x + math.cos(yaw) * bx - math.sin(yaw) * by
            my = y + math.sin(yaw) * bx + math.cos(yaw) * by
            found.append((mx, my, len(c), span))
        return found

    def merge(self, found):
        for mx, my, npts, span in found:
            best = None
            for d in self.detections:
                if math.hypot(d['x'] - mx, d['y'] - my) < self.merge_r:
                    best = d
                    break
            if best is None:
                self.detections.append({'x': mx, 'y': my, 'hits': 1,
                                        'points': npts, 'size': round(span, 3)})
            else:
                k = best['hits']
                best['x'] = (best['x'] * k + mx) / (k + 1)
                best['y'] = (best['y'] * k + my) / (k + 1)
                best['hits'] = k + 1

    # ------------------------------------------------------------ the sweep
    def waypoints(self):
        step = self.side / (self.n - 1)
        pts = []
        for j in range(self.n):
            y = self.cy - self.side / 2 + j * step
            xs = [self.cx - self.side / 2 + i * step for i in range(self.n)]
            if j % 2 == 1:
                xs.reverse()        # snake: no long jumps back
            pts += [(x, y) for x in xs]
        return pts

    def run(self):
        wps = self.waypoints()
        self.get_logger().info(f'grid {self.n}x{self.n} over {self.side} m square, {len(wps)} points')
        for k, (tx, ty) in enumerate(wps, 1):
            res = self.drive_to(tx, ty)
            if res != 'ok':
                self.get_logger().warn(f'point {k}/{len(wps)} ({tx:.2f}, {ty:.2f}): {res}, skipped')
                self.log_points.append({'k': k, 'target': [tx, ty], 'status': res})
                continue
            self.spin_for(self.dwell)
            pose = self.pose()
            self.buf.clear()
            self.spin_for(0.1)
            while len(self.buf) < self.nscans and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
            pose = self.pose() or pose
            found = self.lidar_objects(pose)
            self.merge(found)
            self.log_points.append({'k': k, 'target': [tx, ty], 'status': 'ok',
                                    'pose': [round(pose[0], 3), round(pose[1], 3), round(pose[2], 3)],
                                    'objects': [[round(a, 3), round(b, 3), c] for a, b, c, _ in found]})
            self.get_logger().info(f'point {k}/{len(wps)} ({tx:.2f}, {ty:.2f}): '
                                   f'{len(found)} lidar object(s) here')
            self.save()
        self.stop()
        self.save()
        self.get_logger().info(f'DONE: {len(self.detections)} unique lidar objects')
        for i, d in enumerate(self.detections, 1):
            self.get_logger().info(f'  object {i}: x={d["x"]:.2f} y={d["y"]:.2f} hits={d["hits"]} size={d["size"]} m')

    def save(self):
        with open(os.path.join(self.out, 'points.json'), 'w') as f:
            json.dump(self.log_points, f, indent=1)
        with open(os.path.join(self.out, 'detections.json'), 'w') as f:
            json.dump(self.detections, f, indent=1)


def main():
    rclpy.init()
    node = Sweep()

    def on_signal(signum, frame):
        raise KeyboardInterrupt()
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, on_signal)

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn('interrupted: stopping the robot')
    finally:
        node.stop()
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
