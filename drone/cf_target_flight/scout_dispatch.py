#!/usr/bin/env python3
"""scout_dispatch: robot finds a bottle -> the nearest free drone flies over it -> home.

This is the glue between the TurtleBot (target_mission.py) and cf_fleet.py. It replaces
mission.py for the "scout + responder" demo. mission.py is a pick-and-drop referee that
merges detections by position and freezes the first position it confirms. Our robot
works differently, and this node is written for the robot's actual behaviour:

  * The robot publishes ONE bottle at a time on /objects/json, with an integer `id`.
  * It sends the CAMERA position first (it can be ~20 cm off). After it has docked and
    measured with the lidar, it sends the PRECISE position under the SAME id
    ("precise": true from target_mission v18 on).
  * After its dwell the robot drops the bottle from the topic ({"objects": []}) and moves on.
    The bottle has not gone anywhere, so this node keeps it until a drone has flown it.

Policy:
  * Latest position per robot id wins. Dispatch when the position is precise. For robots older
    than v18, which send no flag, dispatch when the position has been stable (< `stable_tol`)
    for `settle_s`.
  * A route is `go` over the bottle (`offset` m short of it, toward the drone's home; default
    0 = directly above), then `hover` `hover_s` seconds, then `home`, where the drone lands.
    With `mark: dip`, it uses `pick` (dip to cf_fleet's dip_height beside the bottle) instead.
  * Closest free drone first. A free drone is idle or failed, landed, has a fresh status, is not
    benched and its battery is fine (same rules as mission.py).
  * Lost route: resend twice after 3 s each, then queue the target again.
    Drift or error: bench the drone, and another drone retries; after `max_tries`, skip the target.
    Rejected (outside the fence): skip.
  * Robot restarted (ids start at 1 again): a known id that jumps more than `same_dist` m is a
    new bottle. A new id within `same_dist` of a bottle already flown is the same bottle,
    and is not flown again.

  in   /objects/json          std_msgs/String   from target_mission (robot)
  in   /<drone>/status        std_msgs/String   from cf_fleet
  in   /scout/cmd             std_msgs/String   start | pause | reset | status
  out  /<drone>/route         std_msgs/String   to cf_fleet
  out  /scout/drone_state     std_msgs/String   {"targets": {"<id>": "queued|flying|done|skipped"}}, 2 Hz
                                                (target_mission v18 with wait_for_drone:=true waits on it)
  out  /scout/events          std_msgs/String   readable log for the screen

STATUS: logic unit-tested offline (sim/test_dispatch.py). NOT flown with real drones yet.
"""
import json
import math
import time


class Target:
    def __init__(self, rid, cls, x, y, now):
        self.rid, self.cls = rid, cls
        self.x, self.y = x, y
        self.precise = None          # None = robot sends no flag (pre-v18), else bool
        self.n = 1
        self.t_change = now          # last time the position moved more than stable_tol
        self.t_seen = now
        self.state = 'seen'          # seen -> queued -> flying -> done | skipped
        self.drone = ''
        self.job = ''
        self.tries = 0
        self.sent_at = 0.0
        self.resends = 0
        self.key = 0                 # internal unique number (robot ids repeat after a restart)


class DispatchCore:
    """Pure logic, no ROS. All times are seconds (monotonic)."""

    DEFAULTS = {
        'drones': ['cf1', 'cf2'],
        'classes': ['bottle'],
        'min_conf': 0.4,
        'stable_tol': 0.03,
        'settle_s': 3.0,
        'same_dist': 0.30,
        'hover_s': 5.0,
        'offset': 0.0,
        'mark': 'hover',            # hover | dip
        'max_tries': 2,
        'min_vbat': 3.4,
        'status_timeout': 2.0,
        'fence': [-1.0, 3.5, -1.5, 2.0],
        'auto_start': True,
    }

    def __init__(self, **kw):
        self.p = dict(self.DEFAULTS)
        self.p.update({k: v for k, v in kw.items() if v is not None})
        self.targets = []
        self.next_key = 1
        self.status = {}             # drone -> last status dict (+ '_t')
        self.active = {}             # drone -> Target
        self.benched = set()
        self.running = bool(self.p['auto_start'])
        self.events = []             # (t, text)
        self.routes = []             # (drone, route dict) to publish

    # ------------------------------------------------------------------ helpers
    def event(self, now, text):
        self.events.append((now, text))

    def _by_rid(self, rid):
        for t in reversed(self.targets):
            if t.rid == rid and t.rid is not None:
                return t
        return None

    def _near_flown(self, x, y, exclude=None):
        for t in self.targets:
            if t is exclude or t.state not in ('done', 'flying', 'queued'):
                continue
            if math.hypot(t.x - x, t.y - y) < self.p['same_dist']:
                return t
        return None

    def ready(self, t, now):
        if t.precise is True:
            return True
        if t.precise is False:
            return False
        return t.n >= 3 and now - t.t_change >= self.p['settle_s']

    # ------------------------------------------------------------------ inputs
    def on_objects(self, data, now):
        """data: parsed JSON from /objects/json."""
        if isinstance(data, dict):
            data = data.get('objects', [data] if 'x' in data else [])
        if not isinstance(data, list):
            return
        for o in data:
            try:
                cls = str(o.get('cls', o.get('class', 'object')))
                x, y = float(o['x']), float(o['y'])
                conf = float(o.get('conf', 1.0))
                rid = o.get('id')
            except (KeyError, TypeError, ValueError, AttributeError):
                continue
            if not (math.isfinite(x) and math.isfinite(y)) or conf < self.p['min_conf']:
                continue
            if self.p['classes'] and '*' not in self.p['classes'] and cls not in self.p['classes']:
                continue
            self._update(rid, cls, x, y, o.get('precise'), now)

    def _update(self, rid, cls, x, y, precise, now):
        if rid is None:              # raw detections without ids: match by position
            near = [t for t in self.targets if math.hypot(t.x - x, t.y - y) < self.p['same_dist']]
            t = min(near, key=lambda t: math.hypot(t.x - x, t.y - y)) if near else None
        else:
            t = self._by_rid(rid)
            if t is not None and math.hypot(t.x - x, t.y - y) > self.p['same_dist'] + 0.25:
                t = None             # same id, far away: the robot restarted and counts from 1 again
        if t is None:
            dup = self._near_flown(x, y)
            if dup is not None:
                return               # a bottle we already have (robot restarted, or it is re-announcing)
            t = Target(rid, cls, x, y, now)
            t.key = self.next_key
            self.next_key += 1
            self.targets.append(t)
            self.event(now, f'robot found {cls} #{rid} at ({x:.2f}, {y:.2f})'
                            + (' (camera estimate, waiting for the robot to measure it)' if precise is False else ''))
        else:
            if t.state in ('seen', 'queued'):        # not frozen yet: latest position wins
                if math.hypot(t.x - x, t.y - y) > self.p['stable_tol']:
                    t.t_change = now
                t.x, t.y = x, y
            t.n += 1
        t.t_seen = now
        if precise is not None:
            if precise and t.precise is not True and t.state in ('seen', 'queued'):
                self.event(now, f'{cls} #{rid}: precise position from the robot ({x:.3f}, {y:.3f})')
            t.precise = bool(precise) or bool(t.precise)
        if t.state == 'seen' and self.ready(t, now):
            t.state = 'queued'
            self.event(now, f'{cls} #{rid} queued for a drone at ({t.x:.3f}, {t.y:.3f})')

    def on_status(self, d, st, now):
        st = dict(st)
        st['_t'] = now
        self.status[d] = st
        t = self.active.get(d)
        if t is None or st.get('job') != t.job:
            return
        if st.get('state') in ('idle', 'failed') and st.get('phase') == 'landed':
            del self.active[d]
            done = st.get('done', [])
            first = 'pick' if self.p['mark'] == 'dip' else 'go'
            if st.get('state') == 'idle' and first in done:
                t.state = 'done'
                self.event(now, f'{d} flew over {t.cls} #{t.rid} and landed - '
                                f'{self.count("done")} bottle(s) done')
                return
            why = st.get('msg') or 'unknown reason'
            t.drone = ''
            if not why.startswith('rejected') and why != 'aborted':
                self.benched.add(d)
                self.event(now, f'{d} benched ({why}); other drones carry on. Send "start" to un-bench.')
            if why.startswith('rejected') or t.tries >= self.p['max_tries']:
                t.state = 'skipped'
                self.event(now, f'skipping {t.cls} #{t.rid}: {why}')
            else:
                t.state = 'queued'
                self.event(now, f'{d} could not finish {t.cls} #{t.rid} ({why}); will retry')

    def on_cmd(self, c, now):
        c = c.strip().lower()
        if c == 'start':
            self.running = True
            self.benched.clear()
            self.event(now, 'dispatch running')
        elif c == 'pause':
            self.running = False
            self.event(now, 'dispatch paused (drones in the air finish their job)')
        elif c == 'reset':
            if self.active:
                self.event(now, 'reset refused: a drone is flying')
            else:
                self.targets = []
                self.event(now, 'all targets forgotten')
        elif c == 'status':
            self.event(now, self.summary())

    # ------------------------------------------------------------------ the dispatcher
    def count(self, *states):
        return sum(1 for t in self.targets if t.state in states)

    def summary(self):
        s = ', '.join(f'#{t.rid}:{t.state}' for t in self.targets) or 'no targets'
        return f'{s}; flying: {", ".join(f"{d}->#{t.rid}" for d, t in self.active.items()) or "none"}'

    def drone_free(self, d, now):
        st = self.status.get(d)
        if st is None or now - st['_t'] > self.p['status_timeout'] or d in self.active or d in self.benched:
            return False
        if st.get('state') not in ('idle', 'failed') or st.get('phase') != 'landed':
            return False
        vbat = float(st.get('vbat', 0.0))
        return not (0.1 < vbat < self.p['min_vbat'])

    def aim_point(self, d, t):
        """Where the drone goes: the bottle, or `offset` m short of it toward the drone's position."""
        st = self.status.get(d, {})
        hx, hy = float(st.get('x', 0.0)), float(st.get('y', 0.0))
        vx, vy = hx - t.x, hy - t.y
        L = math.hypot(vx, vy)
        off = min(self.p['offset'], L / 2.0)
        if L < 1e-3 or off <= 0.0:
            return t.x, t.y
        return t.x + vx / L * off, t.y + vy / L * off

    def route(self, d, t):
        ax, ay = self.aim_point(d, t)
        if self.p['mark'] == 'dip':
            wps = [{'x': round(ax, 3), 'y': round(ay, 3), 'action': 'pick'}]
        else:
            wps = [{'x': round(ax, 3), 'y': round(ay, 3), 'action': 'go'},
                   {'action': 'hover', 't': float(self.p['hover_s'])}]
        wps.append({'action': 'home'})       # cf_fleet lands at the last waypoint - never on the bottle
        return {'job': t.job, 'waypoints': wps}

    def in_fence(self, x, y):
        xmin, xmax, ymin, ymax = self.p['fence']
        return xmin <= x <= xmax and ymin <= y <= ymax

    def assign(self, d, t, now):
        t.tries += 1
        t.state, t.drone = 'flying', d
        t.job = f'{t.cls.replace(" ", "_")}{t.rid}-k{t.key}-try{t.tries}'
        t.sent_at, t.resends = now, 0
        self.active[d] = t
        self.routes.append((d, self.route(d, t)))
        self.event(now, f'{d} -> {t.cls} #{t.rid} at ({t.x:.3f}, {t.y:.3f})')

    def tick(self, now):
        for d, t in list(self.active.items()):
            st = self.status.get(d)
            if st is not None and st.get('job') == t.job:
                continue
            if now - t.sent_at > 3.0:
                if t.resends < 2:
                    t.resends += 1
                    t.sent_at = now
                    self.routes.append((d, self.route(d, t)))
                else:
                    del self.active[d]
                    t.state, t.drone = 'queued', ''
                    t.tries -= 1                 # never started: does not count as a try
                    self.event(now, f'{d} did not answer; {t.cls} #{t.rid} queued again')
        # targets the robot published without a flag become ready with time, even after it stopped publishing
        for t in self.targets:
            if t.state == 'seen' and self.ready(t, now):
                t.state = 'queued'
                self.event(now, f'{t.cls} #{t.rid} queued for a drone at ({t.x:.3f}, {t.y:.3f})')
            if t.state == 'queued' and not self.in_fence(t.x, t.y):
                t.state = 'skipped'
                self.event(now, f'skipping {t.cls} #{t.rid}: ({t.x:.2f}, {t.y:.2f}) is outside the drone fence')
        if not self.running:
            return
        free = [d for d in self.p['drones'] if self.drone_free(d, now)]
        queued = [t for t in self.targets if t.state == 'queued']
        while free and queued:
            def dist(pair):
                d, t = pair
                st = self.status[d]
                return math.hypot(t.x - float(st.get('x', 0.0)), t.y - float(st.get('y', 0.0)))
            d, t = min(((d, t) for d in free for t in queued), key=dist)
            self.assign(d, t, now)
            free.remove(d)
            queued.remove(t)

    def drone_state(self):
        """For the robot: state of each robot id (latest target with that id)."""
        out = {}
        for t in self.targets:
            if t.rid is not None:
                out[str(t.rid)] = t.state
        return {'targets': out}


# ---------------------------------------------------------------------------- ROS node
def main(args=None):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class ScoutDispatch(Node):
        def __init__(self):
            super().__init__('scout_dispatch')
            kw = {}
            for k, v in DispatchCore.DEFAULTS.items():
                self.declare_parameter(k, v)
                kw[k] = self.get_parameter(k).value
            self.declare_parameter('objects_topic', '/objects/json')
            self.core = DispatchCore(**kw)
            self.route_pubs = {d: self.create_publisher(String, f'/{d}/route', 10) for d in self.core.p['drones']}
            for d in self.core.p['drones']:
                self.create_subscription(String, f'/{d}/status', lambda m, d=d: self._status(d, m), 10)
            self.create_subscription(String, self.get_parameter('objects_topic').value, self._objects, 10)
            self.create_subscription(String, '/scout/cmd', self._cmd, 10)
            self.ev_pub = self.create_publisher(String, '/scout/events', 10)
            self.ds_pub = self.create_publisher(String, '/scout/drone_state', 10)
            self.n_ev = 0
            self.create_timer(0.5, self._tick)
            self.get_logger().info(f"scout_dispatch: drones {self.core.p['drones']}, mark={self.core.p['mark']}, "
                                   f"hover {self.core.p['hover_s']} s, offset {self.core.p['offset']} m, "
                                   f"{'running' if self.core.running else 'PAUSED (send start on /scout/cmd)'}")

        def _objects(self, m):
            try:
                self.core.on_objects(json.loads(m.data), time.monotonic())
            except ValueError:
                return
            self._flush()

        def _status(self, d, m):
            try:
                self.core.on_status(d, json.loads(m.data), time.monotonic())
            except ValueError:
                return
            self._flush()

        def _cmd(self, m):
            self.core.on_cmd(m.data, time.monotonic())
            self._flush()

        def _tick(self):
            self.core.tick(time.monotonic())
            self.ds_pub.publish(String(data=json.dumps(self.core.drone_state())))
            self._flush()

        def _flush(self):
            for d, r in self.core.routes:
                self.route_pubs[d].publish(String(data=json.dumps(r)))
            self.core.routes.clear()
            for _, text in self.core.events[self.n_ev:]:
                self.get_logger().info(text)
                self.ev_pub.publish(String(data=text))
            self.n_ev = len(self.core.events)

    rclpy.init(args=args)
    node = None
    try:
        node = ScoutDispatch()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
