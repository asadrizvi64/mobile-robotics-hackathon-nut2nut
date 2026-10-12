import os
import sys, math, random, importlib.util
spec = importlib.util.spec_from_file_location('tm', os.path.join(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ros2_ws", "src", "swarm_hack"), 'scripts', 'target_mission.py'))
tm = importlib.util.module_from_spec(spec); spec.loader.exec_module(tm)

def run(bottles, boxes, start=(0, 0, 0), region=(-1.5, 1.5, -1.5, 1.5), seed=0, T=600, verbose=False):
    random.seed(seed)
    p = dict(tm.DEFAULTS); p['region'] = list(region)
    core = tm.MissionCore(p)
    x, y, yaw = start
    fence = 2.3
    circles = [(bx, by, 0.045) for bx, by in bottles] + [(cx, cy, r) for cx, cy, r in boxes]
    def ray(a):
        best = 3.5
        dx, dy = math.cos(a), math.sin(a)
        for cx, cy, r in circles:
            ox, oy = cx - x, cy - y; b = ox*dx + oy*dy; c = ox*ox + oy*oy - r*r; disc = b*b - c
            if disc >= 0:
                t = b - math.sqrt(disc)
                if 0 < t < best: best = t
        for t in [((fence - x)/dx) if dx > 1e-9 else 1e9, ((-fence - x)/dx) if dx < -1e-9 else 1e9,
                  ((fence - y)/dy) if dy > 1e-9 else 1e9, ((-fence - y)/dy) if dy < -1e-9 else 1e9]:
            if 0 < t < best: best = t
        return best
    seen = {i: 0 for i in range(len(bottles))}
    est = {}
    dt = 0.05; mind = 9; t = 0.0; maxr = 0.0; wind = 0.0; wmax = 0.0  # WIND
    for k in range(int(T/dt)):
        t = k*dt
        # lidar in base frame
        pts = []
        for i in range(360):
            a = math.radians(i)
            r = ray(yaw + a)
            if 0.12 < r < 3.0: pts.append((r*math.cos(a), r*math.sin(a)))
        # camera/YOLO at 4 Hz
        if k % 5 == 0:
            for i, (bx, by) in enumerate(bottles):
                d = math.hypot(bx-x, by-y); brg = tm.wrap(math.atan2(by-y, bx-x) - yaw)
                vis = abs(brg) < math.radians(28) and 0.3 < d < 2.5 and ray(math.atan2(by-y, bx-x)) > d - 0.1 and abs(globals().get('BLUR_W', 0)) < 0.2  # BLUR: no detection while turning fast
                if vis:
                    seen[i] += 1
                    if seen[i] >= 3:
                        est[i] = (bx + random.gauss(0, 0.03), by + random.gauss(0, 0.03))
        dets = [{'id': i, 'cls': 'bottle', 'x': e[0], 'y': e[1], 'z': .15, 'conf': .9, 'hits': seen[i]} for i, e in est.items()]
        v, w = core.step(t, (x, y, yaw), pts, dets)
        globals()['BLUR_W'] = w
        # RAMP: same acceleration limiter as the robot node
        pv, pw = globals().get('_lc', (0.0, 0.0))
        def _lim(prev, tgt, up, down):
            st = (down if (abs(tgt) < abs(prev) and tgt * prev >= 0) else up) * dt
            return prev + max(-st, min(st, tgt - prev))
        v = _lim(pv, v, tm.DEFAULTS['acc_lin'], 1.0); w = _lim(pw, w, tm.DEFAULTS['acc_ang'], 4.0)
        globals()['_lc'] = (v, w)
        wind += w*dt; wmax = max(wmax, abs(wind)); yaw = tm.wrap(yaw + w*dt); x += v*math.cos(yaw)*dt; y += v*math.sin(yaw)*dt
        c_, s_ = math.cos(yaw), math.sin(yaw)
        for cx, cy, r in circles:
            lx, ly = (cx-x)*c_ + (cy-y)*s_, -(cx-x)*s_ + (cy-y)*c_
            dx_ = max(-0.205 - lx, 0, lx - 0.077); dy_ = max(-0.155 - ly, 0, ly - 0.155)
            mind = min(mind, math.hypot(dx_, dy_) - r)
        for ccx, ccy in [(-0.205,-0.155),(-0.205,0.155),(0.077,-0.155),(0.077,0.155)]:
            wx, wy = x + ccx*c_ - ccy*s_, y + ccx*s_ + ccy*c_
            mind = min(mind, fence - max(abs(wx), abs(wy)))
        maxr = max(maxr, math.hypot(x, y))
        if core.state == 'DONE': break
    for tt, m in core.log:
        if verbose or 'RELEASED' in m or 'skipped' in m or 'home' in m or 'timeout' in m: print(f'  {tt:6.1f}s {m}')
    errs = []
    for r in core.released:
        e = min(math.hypot(r['x']-bx, r['y']-by) for bx, by in bottles); errs.append(round(e, 3))
    print(f'  state={core.state} t={t:.0f}s released={len(core.released)}/{len(bottles)} pos_err={errs} '
          f'FOOTPRINT_clearance={mind:.3f} m end=({x:.2f},{y:.2f},{math.degrees(yaw):.0f}deg) MAX_DIST_FROM_A={maxr:.2f} CABLE_WIND_MAX={wmax/6.283:.1f}turns END={wind/6.283:+.1f}')
    return core

print('A: 3 bottles, one box obstacle between')
run([(1.2, 0.6), (-1.0, 1.0), (0.3, -1.2)], [(0.6, 0.3, 0.15)])
print('B: bottle behind the robot + bottle right next to a box')
run([(-1.2, -0.2), (1.0, -1.0)], [(1.0, -0.6, 0.12), (0.0, 0.8, 0.2)])
print('C: no bottles')
run([], [(0.5, 0.5, 0.15)])
