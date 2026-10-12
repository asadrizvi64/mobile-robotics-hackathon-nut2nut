"""End-to-end offline test: simulated robot (real target_mission logic) -> scout_dispatch (real logic)
-> fake cf_fleet drones (kinematic, same status protocol) -> /scout/drone_state back to the robot.

Checks, per bottle: the drone flew over the PRECISE position (not the camera estimate), how far the robot
was from the drone while it hovered there, and that every bottle was flown once.
    python3 sim_drones.py
"""
import importlib.util
import math
import os
import random

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    'sd', os.path.join(HERE, '..', 'drone', 'cf_target_flight', 'scout_dispatch.py'))
sd = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sd)

src = open(os.path.join(HERE, 'sim_mission.py')).read().split("print('A:")[0]
# camera range bias (like sim_bias.py) so camera estimate and precise position really differ
src = src.replace("est[i] = (bx + random.gauss(0, 0.03), by + random.gauss(0, 0.03))",
                  "ux_, uy_ = (bx - x) / d, (by - y) / d; "
                  "est[i] = (bx + BIAS * ux_ + random.gauss(0, 0.03), by + BIAS * uy_ + random.gauss(0, 0.03))")
exec(src)                                        # defines tm (target_mission) and run()


class FakeDrone:
    """cf_fleet's status protocol: idle/landed -> busy (takeoff, to_go, hovering, to_home, landing) -> idle."""

    def __init__(self, name, ox, oy, speed=0.2):
        self.name, self.ox, self.oy, self.speed = name, ox, oy, speed
        self.x, self.y = ox, oy
        self.state, self.phase, self.job, self.done, self.msg = 'idle', 'landed', '', [], ''
        self.plan = []                      # list of (phase, action, x, y, duration)
        self.t_phase = 0.0
        self.track = []                     # (t, phase, x, y)

    def route(self, r, t):
        if self.state == 'busy':
            return                          # cf_fleet ignores routes while busy
        self.job, self.done, self.msg, self.state = r['job'], [], '', 'busy'
        self.plan = [('takeoff', None, None, None, 4.0)]
        for wp in r['waypoints']:
            a = wp['action']
            if a == 'hover':
                self.plan.append(('hovering', a, None, None, float(wp['t'])))
            elif a == 'home':
                self.plan.append(('to_home', a, self.ox, self.oy, None))
            else:
                self.plan.append((f'to_{a}', a, wp['x'], wp['y'], None))
        self.plan.append(('landing', None, None, None, 4.0))
        self._next(t)

    def _next(self, t):
        if not self.plan:
            self.state, self.phase = 'idle', 'landed'
            return
        self.cur = self.plan.pop(0)
        self.phase, self.t_phase = self.cur[0], t

    def step(self, t, dt):
        if self.state != 'busy':
            return
        ph, act, gx, gy, dur = self.cur
        if gx is not None:
            d = math.hypot(gx - self.x, gy - self.y)
            s = min(d, self.speed * dt)
            if d > 1e-6:
                self.x += (gx - self.x) / d * s
                self.y += (gy - self.y) / d * s
            if d < 0.01:
                if act:
                    self.done.append(act)
                self._next(t)
        elif t - self.t_phase >= dur:
            if act:
                self.done.append(act)
            self._next(t)
        self.track.append((t, self.phase, self.x, self.y))

    def status(self):
        return {'name': self.name, 'state': self.state, 'phase': self.phase, 'job': self.job,
                'done': list(self.done), 'x': self.x, 'y': self.y, 'z': 0.5, 'vbat': 3.9, 'miss': 0.0,
                'msg': self.msg}


def scenario(name, bottles, boxes, wait, bias=0.2, max_targets=0, seed=0):
    global BIAS
    BIAS = bias
    disp = sd.DispatchCore(drones=['cf1', 'cf2'], fence=[-2.0, 2.0, -2.0, 2.0])
    drones = {'cf1': FakeDrone('cf1', 0.0, 0.6), 'cf2': FakeDrone('cf2', 0.0, 1.0)}
    robot_track = []
    Base = tm.MissionCore
    camera_first = {}

    class Linked(Base):
        def step(self, t, pose, pts, dets):
            out = Base.step(self, t, pose, pts, dets)
            robot_track.append((t, pose[0], pose[1]))
            k = round(t / 0.05)
            if k % 4 == 0:                                   # robot publishes /objects/json at 5 Hz
                if self.active:
                    camera_first.setdefault(self.active['id'], (self.active['x'], self.active['y']))
                disp.on_objects({'objects': [dict(self.active)] if self.active else []}, t)
            if k % 2 == 0:                                   # cf_fleet status at 10 Hz
                for d, dr in drones.items():
                    disp.on_status(d, dr.status(), t)
            if k % 10 == 0:                                  # dispatcher tick 2 Hz, drone_state back to robot
                disp.tick(t)
                self.drone_state = {k_: v for k_, v in disp.drone_state()['targets'].items()}
            for d, r in disp.routes:
                drones[d].route(r, t)
            disp.routes.clear()
            for dr in drones.values():
                dr.step(t, 0.05)
            return out

    tm.MissionCore = Linked
    tm.DEFAULTS['wait_for_drone'] = wait
    tm.DEFAULTS['max_targets'] = max_targets
    print(f'\n{name} (wait_for_drone={wait}, camera range error {bias:+.2f} m)')
    try:
        core = run(bottles, boxes, seed=seed, T=900)
    finally:
        tm.MissionCore = Base
        tm.DEFAULTS['wait_for_drone'] = False
        tm.DEFAULTS['max_targets'] = 0
    # finish the drone flights after the robot is done
    t = robot_track[-1][0] if robot_track else 0.0
    for _ in range(int(120 / 0.05)):
        t += 0.05
        for d, dr in drones.items():
            disp.on_status(d, dr.status(), t)
        disp.tick(t)
        for d, r in disp.routes:
            drones[d].route(r, t)
        disp.routes.clear()
        for dr in drones.values():
            dr.step(t, 0.05)
    for tt, m in core.log:
        if 'waiting here' in m or ' after ' in m and 'drone' in m:
            print(f'   robot {tt:6.1f}s: {m}')
    for _, e in disp.events:
        if 'flew over' in e or 'skipping' in e or 'benched' in e:
            print('   dispatch:', e)

    ok = True
    flown = 0
    for dname, dr in drones.items():
        hov = [(tt, x, y) for tt, ph, x, y in dr.track if ph == 'hovering']
        if not hov:
            continue
        # split hover samples into separate jobs
        jobs, cur = [], [hov[0]]
        for a, b in zip(hov, hov[1:]):
            if b[0] - a[0] > 1.0:
                jobs.append(cur)
                cur = []
            cur.append(b)
        jobs.append(cur)
        for j in jobs:
            flown += 1
            hx, hy = j[0][1], j[0][2]
            err = min(math.hypot(hx - bx, hy - by) for bx, by in bottles)
            near = [math.hypot(rx - hx, ry - hy) for (tt, rx, ry) in robot_track if j[0][0] <= tt <= j[-1][0]]
            rmin = min(near) if near else float('nan')
            cam_err = None
            print(f'   {dname} hovered over ({hx:.3f}, {hy:.3f}): error to true bottle {err * 100:.1f} cm, '
                  f'robot centre >= {rmin:.2f} m away while hovering'
                  + ('' if near else ' (robot already finished)'))
            if err > 0.05:
                ok = False
    for rid, (cx, cy) in camera_first.items():
        e = min(math.hypot(cx - bx, cy - by) for bx, by in bottles)
        print(f'   robot target {rid}: first (camera) estimate was {e * 100:.0f} cm off - drone did NOT fly there')
    n_rel = len(core.released)
    if flown != n_rel:
        ok = False
    print(f'   bottles released by robot {n_rel}, flown by drones {flown}  ->  {"PASS" if ok else "FAIL"}')
    return ok


if __name__ == '__main__':
    random.seed(0)
    res = [
        scenario('A: 2 bottles, no handshake', [(1.0, 0.5), (-0.8, -0.6)], [(0.3, -0.9, 0.1)], wait=False),
        scenario('B: 2 bottles, robot waits for the drone', [(1.0, 0.5), (-0.8, -0.6)], [(0.3, -0.9, 0.1)], wait=True),
        scenario('C: today layout, bias -0.15, robot waits', [(0.50, 0.19), (1.21, -0.28)],
                 [(0.72, -0.35, 0.08), (0.9, 0.1, 0.08)], wait=True, bias=-0.15, max_targets=2),
    ]
    print('\nALL PASS' if all(res) else '\nSOME FAILED')
