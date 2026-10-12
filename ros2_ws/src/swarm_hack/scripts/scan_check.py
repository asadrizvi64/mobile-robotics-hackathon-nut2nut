#!/usr/bin/env python3
"""scan_check: is the lidar healthy, or is the room just big?  Runs ON THE ROBOT, ~4 s.

    python3 ~/scan_check.py

Collects 20 scans and reports, per 45-deg sector:
  valid   = range_min < r < range_max      (real returns)
  zero    = r == 0 or inf                  (LDS-01: no return = nothing within ~3.5 m, OR dirty/blocked)
  stale   = beams whose value is IDENTICAL in all 20 scans (a live lidar always jitters a few mm)
Also: the most repeated exact value inside one scan (a broken half-circle shows up as e.g. 1.34 x 150).
"""
import collections
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

N = 20


class Check(Node):
    def __init__(self):
        super().__init__('scan_check')
        self.scans = []
        self.t0 = None
        self.create_subscription(LaserScan, '/scan', self.cb, qos_profile_sensor_data)

    def cb(self, m):
        if self.t0 is None:
            self.t0 = time.time()
        self.scans.append(m)


def main():
    rclpy.init()
    n = Check()
    end = time.time() + 15
    while rclpy.ok() and len(n.scans) < N and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    S = n.scans
    if not S:
        print('NO /scan received in 15 s -> lidar driver not running / wrong domain')
        return
    m = S[-1]
    hz = (len(S) - 1) / (time.time() - n.t0) if len(S) > 1 else 0
    nb = len(m.ranges)
    print(f'{len(S)} scans, {hz:.1f} Hz, {nb} beams, range {m.range_min:.2f}..{m.range_max:.2f} m, '
          f'frame {m.header.frame_id}')
    valid = [0] * nb
    zero = [0] * nb
    close = [0] * nb      # 0 < r <= range_min: something touching / on top of the lidar
    far = [0] * nb        # r >= range_max
    for s in S:
        for i, r in enumerate(s.ranges[:nb]):
            if math.isfinite(r) and s.range_min < r < s.range_max:
                valid[i] += 1
            elif r == 0.0 or not math.isfinite(r):
                zero[i] += 1
            elif r <= s.range_min:
                close[i] += 1
            else:
                far[i] += 1
    stale = [i for i in range(nb)
             if valid[i] == len(S) and len({round(s.ranges[i], 4) for s in S}) == 1]
    print('\nsector (deg, 0=front, CCW)  valid%  no-return%  too-close%  beyond-max%  stale')
    for k in range(8):
        a0 = -180 + 45 * k
        idx = [i for i in range(nb)
               if a0 <= math.degrees(math.atan2(math.sin(m.angle_min + i * m.angle_increment),
                                                math.cos(m.angle_min + i * m.angle_increment))) < a0 + 45]
        if not idx:
            continue
        v = 100 * sum(valid[i] for i in idx) / (len(idx) * len(S))
        z = 100 * sum(zero[i] for i in idx) / (len(idx) * len(S))
        st = sum(1 for i in idx if i in set(stale))
        c = 100 * sum(close[i] for i in idx) / (len(idx) * len(S))
        f = 100 * sum(far[i] for i in idx) / (len(idx) * len(S))
        print(f'  {a0:5d}..{a0 + 45:<5d}            {v:5.0f}   {z:8.0f}   {c:9.0f}   {f:10.0f}   {st:5d}')
    tot_valid = sum(valid) / len(S)
    print(f'\navg valid points per scan: {tot_valid:.0f} / {nb}')
    c = collections.Counter(round(r, 3) for r in m.ranges if math.isfinite(r) and m.range_min < r < m.range_max)
    val, cnt = c.most_common(1)[0] if c else (0, 0)
    print(f'most repeated range in one scan: {val} m x {cnt} beams')
    print(f'stale beams (identical in all {len(S)} scans): {len(stale)}')
    vr = [r for r in m.ranges if math.isfinite(r) and m.range_min < r < m.range_max]
    if vr:
        print(f'valid ranges: min {min(vr):.2f} m, max {max(vr):.2f} m')
    blocked = [k for k in range(nb) if close[k] > len(S) // 2]
    if blocked:
        degs = sorted({round(math.degrees(m.angle_min + k * m.angle_increment)) % 360 for k in blocked})
        runs, start = [], degs[0]           # contiguous angle runs, e.g. 150-210
        for x, y in zip(degs, degs[1:] + [None]):
            if y is None or y - x > 3:
                runs.append(f'{start}-{x}'); start = y
        print(f'BLOCKED beams (<{m.range_min:.2f} m, something on/against the lidar): {len(blocked)}, '
              f'at {", ".join(runs)} deg (0 = front, 90 = left, 180 = rear, 270 = right)')
    print('\nVERDICT:')
    if len(blocked) > 10:
        print(f'  {len(blocked)} beams are BLOCKED right next to the lidar -> look at those angles: cable, '
              'tape, camera/drone/phone placed on the top plate, loose screw. Remove it.')
    if len(stale) > 20 or cnt > 40:
        print('  BROKEN/STALE data: same values repeat -> driver/serial problem. Restart the lidar driver, '
              'check the USB cable; if it persists, power-cycle the robot.')
    elif tot_valid < 0.5 * nb and vr and max(vr) > 2.5:
        print('  Probably FINE: missing points are beyond ~3.5 m (open room). Put boxes/walls within 3 m '
              'around the test square so SLAM has something to see.')
    elif tot_valid < 0.5 * nb:
        print('  SUSPICIOUS: few returns AND none far away -> dirty dome, something on top of the lidar, '
              'or the sensor is failing. Clean it and check nothing blocks it (cables, camera mount).')
    elif len(blocked) <= 10:
        print('  HEALTHY.')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
