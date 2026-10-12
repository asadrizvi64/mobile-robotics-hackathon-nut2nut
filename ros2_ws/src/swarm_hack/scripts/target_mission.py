#!/usr/bin/env python3
"""target_mission: find bottles, drive up to each one, hand its location to the drone, go to the next.

Runs ON THE ROBOT (like straight_avoid): driving needs only /scan + TF, no Wi-Fi in the control loop.

    python3 ~/target_mission.py
    python3 ~/target_mission.py --ros-args -p region:="[-1.5,1.5,-1.5,1.5]" -p standoff:=0.5 -p dwell:=4.0

Inputs
  /objects/detections  (std_msgs/String, from object_mapper on the laptop) - everything YOLO+lidar knows
  /scan, TF map->base_footprint (SLAM on the robot)
Outputs
  /objects/json        (std_msgs/String) - ONLY targets the robot has reached = what the DRONE flies to.
                        {"objects":[{"id":1,"cls":"bottle","x":..,"y":..,"z":..,"conf":..}, ...]} at 2 Hz
  /mission/status      (std_msgs/String) - human-readable state
  /cmd_vel

Behaviour (state machine, 20 Hz)
  SEARCH  : go to the next search waypoint and spin 360 deg there slowly (camera sees all around).
  GOTO    : a new confirmed bottle -> drive to a point `standoff` m in front of it (lidar corridor avoidance,
            the bottle itself is never driven into).
  FACE    : turn to face the bottle.
  RELEASE : publish its position to the drone (/objects/json), wait `dwell` s.
  -> next unvisited bottle (nearest first), else continue SEARCH.  All waypoints done -> HOME -> DONE.
Ctrl+C always sends zero velocity.
"""
import json
import math
import signal
import time


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def corridor_free(points, theta, half_w, max_d):
    """How far the robot can drive along relative heading theta before its corridor hits a point."""
    c, s = math.cos(theta), math.sin(theta)
    best = max_d
    for x, y in points:
        along = x * c + y * s
        if along <= 0.0 or along >= best:
            continue
        if abs(-x * s + y * c) < half_w:
            best = along
    return best


class MissionCore:
    """ROS-free logic so it can be simulated. step() -> (v, w)."""

    def __init__(self, p):
        self.p = p
        xmin, xmax, ymin, ymax = p['region']
        cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
        qx, qy = (xmax - xmin) / 4, (ymax - ymin) / 4
        # spin at the start spot first, then the 4 quadrant centres, then the middle
        wps = [(cx + qx, cy + qy), (cx - qx, cy + qy), (cx - qx, cy - qy), (cx + qx, cy - qy), (cx, cy)]
        hx, hy = p['home']
        R = p['max_radius'] - 0.35                      # spin points well inside the fence
        for i, (wx, wy) in enumerate(wps):
            d = math.hypot(wx - hx, wy - hy)
            if d > R:
                wps[i] = (hx + (wx - hx) * R / d, hy + (wy - hy) * R / d)
        self.waypoints = [None] + wps
        self.wp_i = 0
        self.state = 'SEARCH'
        self.sub = 'MOVE'            # SEARCH sub-state: MOVE | SPIN
        self.spun = 0.0
        self.last_yaw = None
        self.target = None           # dict: x, y, cls, conf, z, approach (unit vector), t0
        self.released = []           # history of handed-over targets (so they are not visited twice)
        self.active = None           # the target published to the drone right now (only during the dwell)
        self.skipped = []            # positions given up on (unreachable)
        self.side = 0.0
        self.t_state = 0.0
        self.log = []
        self.ignored = []
        self.half_w = p['robot_width'] / 2 + p['margin']
        self.cands = [math.radians(d) for d in range(-180, 180, 5)]
        self.prog = None
        self.back_from = None
        self.turn_from = 0.0
        self.look_until = 0.0
        self.spin_dir = -1.0
        self.wind = 0.0
        self.prev_yaw = None
        self.spin_t = 0.0
        self.escape = None

    # ---------- helpers ----------
    def _say(self, now, msg):
        self.log.append((round(now, 1), msg))

    def _set(self, now, state):
        if state != self.state:
            self._say(now, f'{self.state} -> {state}')
        self.state, self.t_state = state, now

    def _in_region(self, x, y, m=0.3):
        xmin, xmax, ymin, ymax = self.p['region']
        return xmin - m <= x <= xmax + m and ymin - m <= y <= ymax + m

    def _known(self, x, y):
        r = self.p['visited_radius']
        return any(math.hypot(x - o['x'], y - o['y']) < r for o in self.released + self.skipped)

    def candidates(self, dets):
        """Bottles worth driving to. self.ignored = [(x, y, reason)] for the ones the mapper sees but we skip."""
        out, self.ignored = [], []
        hx, hy = self.p['home']
        for o in dets:
            if o.get('cls') not in self.p['classes']:
                continue
            x, y = float(o['x']), float(o['y'])
            why = None
            if o.get('hits', 99) < self.p['min_hits']:
                why = 'not confirmed yet'
            elif float(o.get('conf', 1.0)) < self.p['min_conf']:
                why = f"low confidence {float(o.get('conf', 0)):.2f}"
            elif 'track' in o and o['track'] not in ('camera', 'camera+lidar'):
                why = f"camera not seeing it now ({o['track']})"     # remembered position, may be stale
            elif float(o.get('age_s', 0.0)) > self.p['max_age']:
                why = 'stale'
            elif math.hypot(x - hx, y - hy) > self.p['max_radius']:
                why = f"outside {self.p['max_radius']} m of mark A ({math.hypot(x - hx, y - hy):.2f} m)"
            elif not self._in_region(x, y):
                why = 'outside the search region'
            elif self._known(x, y):
                why = f"already visited (within {self.p['visited_radius']} m of a done bottle)"
            if why:
                self.ignored.append((x, y, why))
            else:
                out.append(o)
        return out

    def _drive_to(self, pose, pts, gx, gy, now):
        """Corridor-avoidance drive toward (gx, gy). Returns (v, w, dist).
        Candidate headings cover the full circle; the one closest to the goal direction whose corridor is
        free wins. Big heading changes are done turning in place. `side` (which way round the obstacle,
        relative to the straight line to the goal) has hysteresis; if no progress for 12 s it flips."""
        x, y, yaw = pose
        hx, hy = self.p['home']
        R = self.p['max_radius']
        gd = math.hypot(gx - hx, gy - hy)
        if gd > R - 0.1:                                  # goal outside the fence -> pull it inside
            gx, gy = hx + (gx - hx) * (R - 0.1) / gd, hy + (gy - hy) * (R - 0.1) / gd
        dist = math.hypot(gx - x, gy - y)
        desired = wrap(math.atan2(gy - y, gx - x) - yaw)
        rd = math.hypot(x - hx, y - hy)
        out_dir = math.atan2(y - hy, x - hx)              # world direction pointing out of the fence
        # progress watchdog
        key = (round(gx, 1), round(gy, 1))
        if self.prog is None or self.prog[2] != key:
            self.prog = [dist, now, key]
            self.side = 0.0
        elif dist < self.prog[0] - 0.1:
            self.prog[0], self.prog[1] = dist, now
        elif now - self.prog[1] > 12.0:
            self.side = -self.side if self.side else 1.0
            self.prog[0], self.prog[1] = dist, now
            self._say(now, f'no progress for 12 s - trying the other side ({"left" if self.side > 0 else "right"})')
        front = [math.hypot(px, py) for px, py in pts if px > 0 and abs(math.atan2(py, px)) < math.radians(40)]
        if front and min(front) < self.p['stop_dist']:
            left = corridor_free(pts, math.radians(60), self.half_w, 2.0)
            right = corridor_free(pts, math.radians(-60), self.half_w, 2.0)
            return -0.03, self.p['max_turn'] * (1 if left >= right else -1), dist
        need = min(self.p['clear_dist'], dist + 0.05)
        free = {th: corridor_free(pts, th, self.half_w, 2.0) for th in self.cands}
        ok = [th for th in self.cands if free[th] >= need]
        if rd > R - 0.25:   # near the 1.5 m fence: no headings that lead further out
            ok = [th for th in ok if math.cos(wrap(yaw + th - out_dir)) < 0.2]
            if not ok:      # turn back toward mark A
                back = wrap(out_dir + math.pi - yaw)
                return 0.0, max(-self.p['max_turn'], min(self.p['max_turn'], 1.5 * back)), dist
        if not ok:   # boxed in: rotate toward the most open direction
            best = max(self.cands, key=lambda th: free[th])
            return 0.0, self.p['max_turn'] * (1.0 if best > 0 else -1.0), dist
        nearest = min(self.cands, key=lambda th: abs(wrap(th - desired)))
        blocked = free[nearest] < need

        def cost(th):
            off = wrap(th - desired)
            c = abs(off)
            if blocked and self.side and off * self.side < -0.05:
                c += 0.8
            return c
        th = min(ok, key=cost)
        if blocked and not self.side:
            self.side = 1.0 if wrap(th - desired) > 0 else -1.0
        if not blocked and dist < self.prog[0] + 0.3:
            self.side = 0.0
        w = max(-self.p['max_turn'], min(self.p['max_turn'], 2.0 * th))
        if abs(th) > math.radians(50):                # big turn: (almost) in place
            return (0.02 if abs(th) < math.radians(80) else 0.0), w, dist
        v = self.p['speed'] * math.cos(th) * max(0.3, min(1.0, free[th] / 1.0))
        v *= min(1.0, max(0.35, dist / 0.4))          # slow down near the goal
        return v, w, dist

    def _turn_to(self, yaw, target_yaw):
        err = wrap(target_yaw - yaw)
        return err, max(-0.6, min(0.6, 1.5 * err))

    def _pick_target(self, pose, cands, now):
        x, y, _ = pose
        o = min(cands, key=lambda o: math.hypot(float(o['x']) - x, float(o['y']) - y))
        tx, ty = float(o['x']), float(o['y'])
        d = math.hypot(x - tx, y - ty)
        ux, uy = ((x - tx) / d, (y - ty) / d) if d > 1e-3 else (1.0, 0.0)
        self.target = {'x': tx, 'y': ty, 'x0': tx, 'y0': ty, 'cls': o.get('cls'), 'conf': float(o.get('conf', 1.0)),
                       'z': float(o.get('z', 0.0)), 'ux': ux, 'uy': uy, 'src_id': o.get('id'), 't0': now}
        self.side = 0.0
        rid = len(self.released) + 1
        entry = {'id': rid, 'cls': o.get('cls'), 'x': round(tx, 3), 'y': round(ty, 3),
                 'z': round(float(o.get('z', 0.0)), 3), 'conf': round(float(o.get('conf', 1.0)), 2)}
        self.released.append(entry)
        self.active = entry if self.p['announce_early'] else None
        self.target['entry'] = entry
        self.target['meas'] = []
        self._say(now, f"TARGET {rid} {o.get('cls')} seen at ({tx:.2f}, {ty:.2f}) {d:.2f} m away -> "
                       + ('SENT TO DRONE now (camera position), driving there' if self.active else 'driving there'))
        self._set(now, 'GOTO')

    def _cluster(self, pose, pts, gx, gy, isolate=True, gate=0.25):
        """Object near map point (gx, gy) from the robot's own lidar.
        Returns {'c': centre (map), 'surf': distance axle->nearest point}, 'too_big', or None."""
        x, y, yaw = pose
        c, s_ = math.cos(yaw), math.sin(yaw)
        mp = [(x + px * c - py * s_, y + px * s_ + py * c, math.hypot(px, py)) for px, py in pts]
        near = [q for q in mp if math.hypot(q[0] - gx, q[1] - gy) < gate]
        if len(near) < 2:
            return None
        seed = min(near, key=lambda q: math.hypot(q[0] - gx, q[1] - gy))
        # grow the WHOLE connected object (neighbours < 5 cm apart), so a big block is not cut down to bottle size
        local = [q for q in mp if math.hypot(q[0] - seed[0], q[1] - seed[1]) < 0.6]
        cl, frontier, used = [seed], [seed], {seed}
        while frontier:
            q = frontier.pop()
            for r_ in local:
                if r_ not in used and math.hypot(r_[0] - q[0], r_[1] - q[1]) < 0.05:
                    used.add(r_)
                    cl.append(r_)
                    frontier.append(r_)
        if len(cl) < 2:
            return None
        cx, cy = sum(q[0] for q in cl) / len(cl), sum(q[1] for q in cl) / len(cl)
        if max(math.hypot(q[0] - cx, q[1] - cy) for q in cl) > self.p['max_half_width']:
            return 'too_big'                     # wider than a bottle: block / box / chair leg pair
        if isolate and len([q for q in mp if 0.12 < math.hypot(q[0] - cx, q[1] - cy) < 0.22]) > 2:
            return None                          # not isolated
        rx, ry = cx - x, cy - y                  # lidar sees the front face -> shift to the centre
        r = math.hypot(rx, ry)
        cen = (cx + rx / r * self.p['object_radius'], cy + ry / r * self.p['object_radius']) if r > 1e-3 else (cx, cy)
        return {'c': cen, 'surf': min(q[2] for q in cl)}

    def _cluster_on_ray(self, pose, pts, gx, gy):
        """First object ALONG the line of sight to (gx, gy): camera bearing is accurate, its range is not.
        Returns {'c': centre (map), 'surf': axle->nearest point, 'raw': cluster centre (map)}, ('too_big', centre), or None."""
        x, y, yaw = pose
        c, s_ = math.cos(yaw), math.sin(yaw)
        bx, by = (gx - x) * c + (gy - y) * s_, -(gx - x) * s_ + (gy - y) * c
        b0, r0 = math.atan2(by, bx), math.hypot(bx, by)
        tol = max(math.radians(6), math.atan2(0.07, max(r0, 0.1)))      # +-7 cm sideways or +-6 deg
        ray = [(px, py) for px, py in pts
               if abs(wrap(math.atan2(py, px) - b0)) < tol and math.hypot(px, py) < r0 + 0.35]
        if not ray:
            return None
        seed = min(ray, key=lambda q: math.hypot(q[0], q[1]))
        local = [q for q in pts if math.hypot(q[0] - seed[0], q[1] - seed[1]) < 0.6]
        cl, frontier, used = [seed], [seed], {seed}
        while frontier:
            q = frontier.pop()
            for r_ in local:
                if r_ not in used and math.hypot(r_[0] - q[0], r_[1] - q[1]) < 0.05:
                    used.add(r_)
                    cl.append(r_)
                    frontier.append(r_)
        cx, cy = sum(q[0] for q in cl) / len(cl), sum(q[1] for q in cl) / len(cl)
        to_map = lambda u, v: (x + u * c - v * s_, y + u * s_ + v * c)
        if len(cl) < 2:
            return None
        if max(math.hypot(q[0] - cx, q[1] - cy) for q in cl) > self.p['max_half_width']:
            return ('too_big', to_map(cx, cy))
        r = math.hypot(cx, cy)
        k = (r + self.p['object_radius']) / r if r > 1e-3 else 1.0          # front face -> centre
        return {'c': to_map(cx * k, cy * k), 'surf': min(math.hypot(q[0], q[1]) for q in cl)}

    def _lidar_measure(self, pose, pts, gx, gy):
        r = self._cluster(pose, pts, gx, gy, isolate=False)
        return r if r in (None, 'too_big') else r['c']

    def _reject_not_bottle(self, now):
        t = self.target
        e = t['entry']
        self._say(now, f"target {e['id']} at ({t['x']:.2f}, {t['y']:.2f}) is wider than a bottle (lidar) - "
                       f"NOT sent, ignoring it")
        if e in self.released:
            self.released.remove(e)
        bx, by = t.get('big_at', (t['x'], t['y']))
        self.skipped.append({'x': bx, 'y': by})          # only the wide object's spot - the bottle stays findable
        self.target = None
        self.active = None
        self.sub = 'MOVE'
        self._set(now, 'BACKOFF')

    def _refresh_target(self, dets, gate=0.25):
        """Follow the mapper's improving estimate of the same bottle (closer = more accurate)."""
        t = self.target
        best = None
        for o in dets:
            if o.get('cls') != t['cls']:
                continue
            # only CAMERA-confirmed updates, close to the current AND the first-seen position: the mapper's lidar
            # tracker can latch onto a nearby block and walk away - the target must never follow that
            if 'track' in o and o['track'] not in ('camera', 'camera+lidar'):
                continue
            ox, oy = float(o['x']), float(o['y'])
            d = math.hypot(ox - t['x'], oy - t['y'])
            if d < gate and math.hypot(ox - t['x0'], oy - t['y0']) < gate + 0.15 and (best is None or d < best[0]):
                best = (d, o)
        if best:
            t['x'], t['y'] = float(best[1]['x']), float(best[1]['y'])
            t['conf'] = max(t['conf'], float(best[1].get('conf', 1.0)))
            e = t.get('entry')
            if e is not None and not t.get('frozen'):   # closer = better estimate
                e['x'], e['y'], e['conf'] = round(t['x'], 3), round(t['y'], 3), round(t['conf'], 2)

    # ---------- main ----------
    def _hits_footprint(self, pts, dtheta, dx=0.0):
        """Would the Waffle footprint (+margin), moved dx forward and rotated by dtheta, contain a lidar point?"""
        x0, x1, y0, y1 = self.p['footprint']
        m = self.p['fp_margin']
        c, s = math.cos(dtheta), math.sin(dtheta)
        for px, py in pts:
            if px * px + py * py > 0.16:          # nothing beyond 0.4 m can be touched
                continue
            qx, qy = px - dx, py
            lx, ly = qx * c + qy * s, -qx * s + qy * c      # point in the rotated robot frame
            if x0 - m < lx < x1 + m and y0 - m < ly < y1 + m:
                return True
        return False

    def _rect_dist(self, lx, ly):
        x0, x1, y0, y1 = self.p['footprint']
        return math.hypot(max(x0 - lx, 0.0, lx - x1), max(y0 - ly, 0.0, ly - y1))

    def _safe(self, pts, v, w, look=0.4):
        """Footprint check along the commanded arc. Unsafe = some lidar point ends up within fp_margin of the
        body AND closer than it is now (moving AWAY from a touching object, e.g. reversing, is allowed)."""
        m = self.p['fp_margin']
        for f in (0.5, 1.0):
            dth, dx = w * look * f, v * look * f
            c, s_ = math.cos(dth), math.sin(dth)
            for px, py in pts:
                if px * px + py * py > 0.25:
                    continue
                qx, qy = px - dx, py
                after = self._rect_dist(qx * c + qy * s_, -qx * s_ + qy * c)
                if after < m and after < self._rect_dist(px, py) - 1e-4:
                    return False
        return True

    def step(self, now, pose, pts, dets):
        """Mission step + footprint guard. The Waffle is asymmetric (rear 0.205 m behind the axle, front 0.077 m),
        so turning swings the rear out. Every command that turns or reverses is checked against the real
        footprint 0.4 s ahead; if it would touch a lidar point, the first safe alternative is used:
        straight on, creep forward, creep back, turn the other way, else stand still."""
        if self.prev_yaw is not None:
            self.wind += wrap(pose[2] - self.prev_yaw)      # total rotation since start (cable twist)
        self.prev_yaw = pose[2]
        v, w = self._step(now, pose, pts, dets)
        if self.escape and now < self.escape[2]:        # committed escape move (no dithering)
            if self._safe(pts, self.escape[0], self.escape[1]):
                return self.escape[0], self.escape[1]
            self.escape = None
        if not pts or (abs(w) < 0.05 and v >= 0):     # straight forward: the corridor check already covers it
            return v, w
        if self._safe(pts, v, w):
            return v, w
        close = [(px, py) for px, py in pts if px * px + py * py < 0.16]
        nearest = min(close, key=lambda q: q[0] ** 2 + q[1] ** 2) if close else (-1.0, 0.0)
        fwd, back = (0.05, 0.0, 1.2), (-0.04, 0.0, 2.0)
        alts = []
        if v > 0.03:
            alts.append((v, 0.0, 1.2))
        # obstacle in front of the axle -> back away first; behind -> creep forward first
        alts += [back, fwd] if nearest[0] > -0.06 else [fwd, back]
        alts.append((0.0, -0.5 * w if abs(w) > 0.05 else 0.4, 1.2))
        for v2, w2, dur in alts:
            if self._safe(pts, v2, w2):
                self.escape = (v2, w2, now + dur)
                return v2, w2
        return 0.0, 0.0

    def _step(self, now, pose, pts, dets):
        """pose = (x, y, yaw) in map; pts = lidar points in base frame [(x, y)]; dets = list of mapper objects."""
        p = self.p
        x, y, yaw = pose
        if self.state == 'DONE':
            return 0.0, 0.0

        if self.state == 'SEARCH':
            cands = self.candidates(dets)
            if cands:
                self._pick_target(pose, cands, now)
                return 0.0, 0.0
            if self.wp_i >= len(self.waypoints) or (p['max_targets'] and len(self.released) >= p['max_targets']):
                self._set(now, 'HOME')
                return 0.0, 0.0
            wp = self.waypoints[self.wp_i]
            if self.sub == 'MOVE':
                if wp is None or math.hypot(wp[0] - x, wp[1] - y) < 0.15 or now - self.t_state > p['goto_timeout']:
                    self.sub, self.spun, self.last_yaw, self.spin_t = 'SPIN', 0.0, yaw, now
                    self.turn_from, self.look_until = 0.0, now + p['look_s']    # look first, then turn
                    # spin the way that UNWINDS the total rotation so far: a power cable never winds up
                    self.spin_dir = -1.0 if self.wind > 0 else 1.0
                    self._say(now, f'spin at waypoint {self.wp_i} ({x:.2f}, {y:.2f})')
                    return 0.0, 0.0
                v, w, _ = self._drive_to(pose, pts, wp[0], wp[1], now)
                return v, w
            # SPIN = turn-and-look: turn spin_step, STOP look_s (sharp, steady images for YOLO over Wi-Fi), repeat.
            # Any bottle confirmed during a look -> the top of SEARCH picks it and the robot drives there at once.
            step_ = abs(wrap(yaw - self.last_yaw))
            self.spun += step_
            self.last_yaw = yaw
            if self.spun >= 2 * math.pi - 0.05:
                self.wp_i += 1
                self.sub = 'MOVE'
                self.t_state = now
                return 0.0, 0.0
            if self.look_until and now < self.look_until:
                return 0.0, 0.0                                   # looking
            if self.look_until:                                    # look finished -> next turn
                self.look_until = 0.0
                self.turn_from = self.spun
                self.spin_t = now
            if self.spun - self.turn_from >= math.radians(p['spin_step']):
                self.look_until = now + p['look_s']               # turned one step -> look
                return 0.0, 0.0
            if step_ > 0.002:
                self.spin_t = now
            elif now - self.spin_t > 10.0:
                self._say(now, f'spin blocked for 10 s at waypoint {self.wp_i} - skipping it')
                self.spun = 2 * math.pi
            return 0.0, p['spin_speed'] * self.spin_dir

        if self.state == 'GOTO':
            self._refresh_target(dets)
            t = self.target
            gx, gy = t['x'] + p['standoff'] * t['ux'], t['y'] + p['standoff'] * t['uy']
            d_obj = math.hypot(t['x'] - x, t['y'] - y)
            v, w, dist = self._drive_to(pose, pts, gx, gy, now)
            if d_obj < 1.0 and now > t.get('no_lock_until', 0.0):   # bottle-sized thing on the line of sight -> lock
                r = self._cluster_on_ray(pose, pts, t['x'], t['y'])
                if isinstance(r, dict):
                    t['x'], t['y'] = r['c']
                    t['dock_c'] = [r['c']]
                    self._say(now, f"lidar locked on the bottle at ({r['c'][0]:.2f}, {r['c'][1]:.2f}) - docking")
                    self._set(now, 'DOCK')
                    return 0.0, 0.0
            if d_obj < 0.8 and self.prog is not None and now - self.prog[1] > 12.0:
                self._say(now, f"no progress for 12 s, {d_obj:.2f} m from the bottle - docking straight in")
                self._set(now, 'DOCK')
                return 0.0, 0.0
            if dist < p['arrive_tol'] or d_obj < p['standoff'] + 0.08:
                self._say(now, f"near target ({d_obj:.2f} m) - docking on it with the lidar")
                self._set(now, 'DOCK')
                return 0.0, 0.0
            if now - self.t_state > p['goto_timeout']:
                if d_obj < p['standoff'] + 0.6:
                    self._say(now, f'timeout but {d_obj:.2f} m from target - docking from here')
                    self._set(now, 'DOCK')
                else:
                    self._say(now, f"timeout, target ({t['x']:.2f}, {t['y']:.2f}) unreachable - skipped")
                    self.skipped.append({'x': t['x'], 'y': t['y']})
                    self.active = None
                    self.target = None
                    self.sub = 'MOVE'
                    self._set(now, 'SEARCH')
                return 0.0, 0.0
            return v, w

        if self.state == 'FACE':
            t = self.target
            err, w = self._turn_to(yaw, math.atan2(t['y'] - y, t['x'] - x))
            if abs(err) < math.radians(8) or now - self.t_state > 6.0:
                self._refresh_target(dets)
                self._say(now, f"AT TARGET {t['entry']['id']}: robot at ({x:.2f}, {y:.2f}), "
                               f"{math.hypot(t['x'] - x, t['y'] - y):.2f} m from it - measuring with lidar")
                self._set(now, 'MEASURE')
                return 0.0, 0.0
            return 0.0, w

        if self.state == 'DOCK':         # final approach steered by the robot's OWN lidar: as close as possible
            t = self.target
            front = p['footprint'][1]
            stop_at = front + p['dock_gap'] + 0.0
            if not t.get('dock_c'):          # no lidar lock yet: take the camera's CURRENT line of sight
                self._refresh_target(dets, gate=0.40)
            res = self._cluster_on_ray(pose, pts, t['x'], t['y'])
            if isinstance(res, tuple):                        # ('too_big', where)
                if math.hypot(res[1][0] - t['x'], res[1][1] - t['y']) > 0.15:
                    # a wide object IN FRONT of the bottle (occluding it): an obstacle, not the target ->
                    # go back to the avoiding approach and come at the bottle from a free direction
                    self._say(now, f"obstacle at ({res[1][0]:.2f}, {res[1][1]:.2f}) between robot and bottle - "
                                   f"going around it")
                    t['dock_c'] = []
                    self._set(now, 'GOTO')
                    t['no_lock_until'] = now + 6.0
                    return 0.0, 0.0
                t['big'] = t.get('big', 0) + 1
                t['big_at'] = res[1]
                if t['big'] > 8:
                    self._reject_not_bottle(now)
                return 0.0, 0.0
            if res:
                t['x'], t['y'] = 0.3 * t['x'] + 0.7 * res['c'][0], 0.3 * t['y'] + 0.7 * res['c'][1]
                t.setdefault('dock_c', []).append(res['c'])
                surf = res['surf']
            else:                        # lidar does not see it (too low / clear plastic): camera estimate,
                surf = math.hypot(t['x'] - x, t['y'] - y) - p['object_radius']   # which can be 20 cm off ->
                if not t.get('dock_c'):                                          # keep a big safety margin
                    surf -= p['blind_margin']
            # HARD RULE: never drive into anything - whatever is in the robot's path, measured by the lidar
            ahead = corridor_free(pts, 0.0, p['footprint'][3] + 0.02, 1.0)
            surf = min(surf, ahead)
            c_, s_ = math.cos(yaw), math.sin(yaw)
            bx, by = (t['x'] - x) * c_ + (t['y'] - y) * s_, -(t['x'] - x) * s_ + (t['y'] - y) * c_
            bearing = math.atan2(by, bx)
            if surf <= stop_at + 0.005 or now - self.t_state > 25.0:
                self._say(now, f"DOCKED: bumper {max(0.0, surf - front):.2f} m from the bottle "
                               f"({'lidar' if res else 'camera estimate'}) - measuring")
                self._set(now, 'MEASURE')
                return 0.0, 0.0
            w = max(-0.5, min(0.5, 2.0 * bearing))
            v = p['dock_speed'] * max(0.3, min(1.0, (surf - stop_at) / 0.10)) if abs(bearing) < 0.3 else 0.0
            return v, w

        if self.state == 'BACKOFF':      # reverse straight back along the way we came in (that path is free)
            if self.back_from is None:
                self.back_from = (x, y)
            gone = math.hypot(x - self.back_from[0], y - self.back_from[1])
            rear_free = corridor_free(pts, math.pi, p['footprint'][3] + 0.02, 1.0)   # from the axle, backwards
            if gone >= p['backoff'] or now - self.t_state > 10.0 or rear_free < -p['footprint'][0] + 0.06:
                self.back_from = None
                self.sub = 'MOVE'
                self._set(now, 'SEARCH')
                self.t_state = now
                return 0.0, 0.0
            return -0.06, 0.0

        if self.state == 'MEASURE':      # standing still: precise position from the robot's own lidar
            t = self.target
            m = self._cluster_on_ray(pose, pts, t['x'], t['y'])
            if isinstance(m, tuple):
                if math.hypot(m[1][0] - t['x'], m[1][1] - t['y']) <= 0.15:   # only a wide thing AT the bottle counts
                    t['big'] = t.get('big', 0) + 1
                    t['big_at'] = m[1]
            elif m:
                t['meas'].append(m['c'])
            if now - self.t_state >= p['measure_s']:
                e = t['entry']
                if t.get('big', 0) > max(3, len(t['meas'])):     # mostly "too big" -> YOLO was wrong
                    self._reject_not_bottle(now)
                    return 0.0, 0.0
                if len(t['meas']) < 3 and len(t.get('dock_c', [])) >= 3:   # use the docking measurements
                    t['meas'] = t['dock_c'][-10:]
                if len(t['meas']) >= 3:
                    xs = sorted(q[0] for q in t['meas'])
                    ys = sorted(q[1] for q in t['meas'])
                    mx, my = xs[len(xs) // 2], ys[len(ys) // 2]
                    src = f'lidar ({len(t["meas"])} scans)'
                else:
                    self._refresh_target(dets)
                    mx, my = t['x'], t['y']
                    src = 'camera (lidar could not isolate it)'
                e['x'], e['y'] = round(mx, 3), round(my, 3)
                t['x'], t['y'], t['frozen'] = mx, my, True
                self.active = e
                self._say(now, f"SENT TO DRONE (precise): target {e['id']} {e['cls']} at ({mx:.3f}, {my:.3f}) "
                               f"from {src}; publishing {p['dwell']:.0f} s")
                self._set(now, 'RELEASE')
            return 0.0, 0.0

        if self.state == 'RELEASE':
            # first clear the space next to the bottle for the drone (it hovers just short of the bottle):
            # reverse 'clear_dist_drone' m along the way we came, still publishing; then stand still
            t = self.target
            if t is not None and not t.get('cleared'):
                if self.back_from is None:
                    self.back_from = (x, y)
                gone = math.hypot(x - self.back_from[0], y - self.back_from[1])
                rear_free = corridor_free(pts, math.pi, p['footprint'][3] + 0.02, 1.0)
                if gone < p['clear_for_drone'] and now - self.t_state < 8.0 and rear_free > -p['footprint'][0] + 0.06:
                    return -0.08, 0.0
                t['cleared'] = True
                self.back_from = None
                self._say(now, f"moved {gone:.2f} m back from the bottle - space clear for the drone")
            if now - self.t_state >= p['dwell']:
                self._say(now, f"target {self.active['id']}: stood {p['dwell']:.0f} s at it - dropped, moving on")
                self.active = None
                self.target = None
                self.sub = 'MOVE'
                self._set(now, 'BACKOFF')
            return 0.0, 0.0

        if self.state == 'HOME':
            if not p['return_home']:
                self._set(now, 'DONE')
                return 0.0, 0.0
            hx, hy = p['home']
            if math.hypot(hx - x, hy - y) < 0.12:
                err, w = self._turn_to(yaw, 0.0)
                if abs(err) < math.radians(8):
                    self._say(now, f'home, {len(self.released)} target(s) released')
                    self._set(now, 'DONE')
                    return 0.0, 0.0
                return 0.0, w
            if now - self.t_state > 2 * p['goto_timeout']:
                self._say(now, 'could not reach home - stopping here')
                self._set(now, 'DONE')
                return 0.0, 0.0
            v, w, _ = self._drive_to(pose, pts, hx, hy, now)
            return v, w
        return 0.0, 0.0

    def status(self):
        t = self.target
        tgt = f" target=({t['x']:.2f},{t['y']:.2f})" if t else ''
        if self.active:
            tgt += f" PUBLISHING #{self.active['id']} ({self.active['x']:.2f},{self.active['y']:.2f})"
        sub = f'/{self.sub} wp {self.wp_i}/{len(self.waypoints)}' if self.state == 'SEARCH' else ''
        return f'{self.state}{sub}{tgt} released={len(self.released)}'


DEFAULTS = {
    'region': [-1.5, 1.5, -1.5, 1.5],   # search square in MAP metres: xmin, xmax, ymin, ymax (mark A = 0,0)
    'max_radius': 1.5,        # HARD limit: robot stays within this distance of mark A (home); targets beyond ignored
    'classes': ['bottle'],
    'min_hits': 3,
    'min_conf': 0.45,
    'max_age': 1.5,           # s: a target must have been updated (seen) this recently
    'standoff': 0.35,         # stop this far from the bottle centre (robot front is ~0.08 m ahead of the axle)
    'arrive_tol': 0.05,
    'visited_radius': 0.45,   # a detection this close to a released target = the same bottle
    'dwell': 10.0,            # s the robot stands at a bottle; its location is published ONLY during this time
    'speed': 0.12,
    'max_turn': 0.6,
    'spin_speed': 0.35,       # rad/s while turning between looks
    'spin_step': 45.0,        # deg turned between looks (camera sees ~62 deg -> overlap)
    'look_s': 2.5,            # s standing still per look, so YOLO gets sharp frames despite Wi-Fi delay
    'robot_width': 0.31,
    'margin': 0.06,
    'clear_dist': 0.5,
    'stop_dist': 0.17,
    'goto_timeout': 60.0,
    'max_targets': 0,         # 0 = visit EVERY bottle found in the area; N = go home after N bottles
    'announce_early': True,   # publish the camera position the MOMENT a bottle is seen; refined by lidar when docked
    'measure_s': 1.5,         # s standing at the bottle, measuring it with the lidar, before publishing
    'dock_gap': 0.06,         # m between the robot's front bumper and the bottle when it stops ("as close as possible")
    'dock_speed': 0.08,       # m/s final approach
    'backoff': 0.25,          # m reversed after each bottle before turning away
    'clear_for_drone': 0.45,  # m the robot backs away from the bottle while publishing (drone hovers there)
    'blind_margin': 0.20,     # m extra stop distance if the lidar never saw the bottle (camera-only estimate)
    'return_home': True,
    'home': [0.0, 0.0],
    'lidar_x_offset': -0.064,
    'acc_lin': 0.25,          # m/s^2 max speed-up (smooth = no current spikes = no power dips)
    'acc_ang': 1.2,           # rad/s^2 max turn-rate increase
    'object_radius': 0.04,
    'max_half_width': 0.065,  # lidar cluster half-width limit: a bottle (~8 cm) passes, a 14+ cm block does not
    'footprint': [-0.205, 0.077, -0.155, 0.155],   # Waffle body around base_footprint: xmin, xmax, ymin, ymax
    'fp_margin': 0.03,
}


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener

    class Mission(Node):
        def __init__(self):
            super().__init__('target_mission')
            p = {}
            for k, v in DEFAULTS.items():
                p[k] = self.declare_parameter(k, v).value
            p['region'] = [float(a) for a in p['region']]
            p['home'] = [float(a) for a in p['home']]
            p['classes'] = list(p['classes'])
            self.p = p
            self.core = MissionCore(p)
            self.tf = Buffer()
            TransformListener(self.tf, self)
            self.scan = None
            self.dets = []
            self.dets_t = 0.0
            self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
            self.last_cmd = (0.0, 0.0)
            self.out = self.create_publisher(String, '/objects/json', 10)
            self.stat = self.create_publisher(String, '/mission/status', 10)
            self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
            be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(String, '/objects/detections', self._on_dets, be)
            # an OLD laptop mapper still publishes its detections on /objects/json (the drone topic). Use them too,
            # so the mission works either way - but warn, because then the drone also gets every raw detection.
            # Mapper messages carry a "stamp" key; this node's own /objects/json output does not.
            self.create_subscription(String, '/objects/json', self._on_old, be)
            self.old_mapper = False
            self.create_timer(0.05, self._tick)
            self.create_timer(0.2, self._publish)
            self.n_log = 0
            self.done = False
            self.get_logger().info(f"target_mission v17: within {p['max_radius']} m of mark A, region {p['region']}, "
                                   f"standoff {p['standoff']} m, "
                                   f"classes {p['classes']}, waypoints {self.core.waypoints[1:]}")

        def _on_scan(self, m):
            self.scan = (time.monotonic(), m)

        def _on_dets(self, m):
            try:
                d = json.loads(m.data)
            except ValueError:
                return
            self.dets = d.get('objects', []) if isinstance(d, dict) else d
            self.dets_t = time.monotonic()

        def _on_old(self, m):
            try:
                d = json.loads(m.data)
            except ValueError:
                return
            if not (isinstance(d, dict) and 'stamp' in d):
                return                                   # our own output (or the drone side) - ignore
            if time.monotonic() - self.dets_t < 1.0 and not self.old_mapper:
                return                                   # the new mapper is talking on /objects/detections
            if not self.old_mapper:
                self.get_logger().warn('Using detections from /objects/json = OLD laptop mapper. The drone also '
                                       'sees every raw detection directly! Rebuild the mapper on the laptop.')
            self.old_mapper = True
            self.dets = d.get('objects', [])
            self.dets_t = time.monotonic()

        def _pose(self):
            try:
                t = self.tf.lookup_transform('map', 'base_footprint', Time())
            except TransformException as e:
                self.get_logger().warn(f'no TF map->base_footprint: {e}', throttle_duration_sec=2.0)
                return None
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return t.transform.translation.x, t.transform.translation.y, yaw

        def send(self, v, w, ramp=True):
            # Smooth acceleration: motor current spikes on abrupt starts/turns were rebooting the robot (power dips).
            # Speeding up / turning harder is rate-limited; slowing down toward 0 is allowed fast (safety).
            if ramp:
                dt = 0.05
                pv, pw = self.last_cmd
                def lim(prev, tgt, up, down):
                    if abs(tgt) < abs(prev) and tgt * prev >= 0:      # slowing down (same direction)
                        step = down * dt
                    else:
                        step = up * dt
                    return prev + max(-step, min(step, tgt - prev))
                v = lim(pv, v, self.p['acc_lin'], 1.0)
                w = lim(pw, w, self.p['acc_ang'], 4.0)
            self.last_cmd = (v, w)
            m = Twist()
            m.linear.x, m.angular.z = float(v), float(w)
            self.cmd.publish(m)

        def _tick(self):
            now = time.monotonic()
            pose = self._pose()
            if pose is None or self.scan is None:
                self.send(0, 0)
                return
            st, scan = self.scan
            if now - st > 0.6:
                self.send(0, 0)
                self.get_logger().warn('no lidar for 0.6 s - holding still', throttle_duration_sec=2.0)
                return
            pts = []
            a = scan.angle_min
            for r in scan.ranges:
                if math.isfinite(r) and scan.range_min < r < 3.0:
                    pts.append((r * math.cos(a) + self.p['lidar_x_offset'], r * math.sin(a)))
                a += scan.angle_increment
            # The mapper (laptop) is the robot's eyes. It publishes /objects/detections at 5 Hz even when it sees
            # nothing. No message for 3 s = laptop mapper not running / old version / bridge or Wi-Fi down:
            # searching blind is pointless (it would just drive around), so HOLD STILL until it is back.
            if now - self.dets_t > 3.0:
                self.send(0, 0)
                self.get_logger().warn('WAITING: no /objects/detections from the laptop mapper for '
                                       f'{now - self.dets_t:.0f} s - holding still. (Mapper updated + running? '
                                       'bridge up?)' if self.dets_t else
                                       'WAITING for /objects/detections from the laptop mapper - holding still. '
                                       '(Rebuild/restart the mapper on the laptop.)', throttle_duration_sec=3.0)
                return
            v, w = self.core.step(now, pose, pts, self.dets)
            for (ix, iy, why) in self.core.ignored:
                if 'not confirmed' not in why:
                    self.get_logger().info(f'ignoring bottle at ({ix:.2f}, {iy:.2f}): {why}', throttle_duration_sec=4.0)
            self.send(v, w)
            for t, msg in self.core.log[self.n_log:]:
                self.get_logger().info(msg)
            self.n_log = len(self.core.log)
            if self.core.state == 'DONE' and not self.done:
                self.done = True
                self.get_logger().info('DONE - nothing more to do. Ctrl+C to exit.')

        def _publish(self):
            # ONLY the bottle the robot is standing at, during its dwell; [] otherwise (5 Hz -> 50 msgs in 10 s)
            active = self.core.active
            self.out.publish(String(data=json.dumps({'objects': [active] if active else []})))
            self.stat.publish(String(data=self.core.status()))
            self.get_logger().info(self.core.status(), throttle_duration_sec=2.0)

    stop = {'flag': False}

    def on_sig(*_):
        stop['flag'] = True
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(s, on_sig)
    node = Mission()
    try:
        while rclpy.ok() and not stop['flag']:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        for _ in range(10):
            node.send(0.0, 0.0, ramp=False)
            time.sleep(0.02)
        node.get_logger().info(f'stopped - zero velocity sent. released: {node.core.released}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
