#!/usr/bin/env python3
"""cf_fleet: fly one or more Crazyflies on ONE Crazyradio, in the robot's map frame.

Per drone (name = cf1, cf2, ...):
  in   /<name>/route   std_msgs/String, JSON like:
         {"job": "bottle1-try1",
          "waypoints": [{"x": 1.0, "y": -0.3, "action": "pick"},
                        {"x": -0.5, "y": 1.5, "action": "drop"},
                        {"action": "home"}]}
       Coordinates are MAP coordinates (metres, same as the robot's SLAM map).
       pick / drop = dip down, wait, climb back (the pickup itself is simulated).
       home        = fly back over the drone's own tape mark.
       go          = just fly there.
       hover       = stay where it is for "t" seconds (hover test).
       The drone always lands at the last waypoint.
  out  /<name>/status  std_msgs/String, JSON:
         {name, state, phase, job, done, x, y, z, vbat, miss, msg}
         state: idle | busy | failed      phase: landed | takeoff | to_pick | picking | ...
         done:  actions finished in the current job, e.g. ["pick"]
         miss:  worst distance (m) between a target and where the drone thinks it got to
  out  /<name>/pose    geometry_msgs/PointStamped in "map"
  out  TF  map -> <name>
Fleet-wide:
  in   /fleet/land_all std_msgs/Empty:  every drone lands where it is.
  in   /fleet/rezero   std_msgs/String: "cf1" or "all". Use after you put a landed drone
                       back on its tape mark by hand: its (0, 0) is set again.

Each drone's (0, 0) is set when this node starts. So start the node with every drone
sitting on its tape mark (its "origin" parameter), facing along its heading line.
Routes end with "home", so the drones come back there by themselves.

Safety checks (from the Flow-deck lessons):
  * After take-off the drone hovers hover_check seconds. If its position wanders more
    than hover_tol, it lands at once: that is almost always a bad floor.
  * After each leg it compares where it thinks it is with the target (arrive_tol),
    waits up to arrive_wait seconds for it to settle, and reports any miss.

sim:=true runs fake drones (no radio needed) to rehearse the whole mission.
"""
import json
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class Aborted(Exception):
    """Raised inside a flight when /fleet/land_all (or shutdown) asks to stop."""


class Drifting(Exception):
    """Raised when the hover check after take-off fails."""


# --------------------------------------------------------------------------- backends

class SimBackend:
    """Fake drone. Moves in straight lines with the timing it was asked for."""

    def __init__(self):
        self._lock = threading.Lock()
        self._a = (0.0, 0.0, 0.0)
        self._b = (0.0, 0.0, 0.0)
        self._t0 = 0.0
        self._d = 1e-6
        self.vbat = 4.0
        self.wobble = 0.0  # tests: fake position noise while airborne (metres)

    def position(self):
        with self._lock:
            s = min(1.0, max(0.0, (time.monotonic() - self._t0) / self._d))
            p = [a + (b - a) * s for a, b in zip(self._a, self._b)]
        if self.wobble and p[2] > 0.05:
            p[0] += self.wobble * math.sin(time.monotonic() * 7.0)
        return tuple(p)

    def go_to(self, x, y, z, duration):
        start = self.position()
        with self._lock:
            self._a, self._b = start, (x, y, z)
            self._t0, self._d = time.monotonic(), max(duration, 1e-3)

    def takeoff(self, height, duration):
        x, y, _ = self.position()
        self.go_to(x, y, height, duration)

    def land(self, duration):
        x, y, _ = self.position()
        self.go_to(x, y, 0.0, duration)

    def rezero(self):
        with self._lock:
            self._a = self._b = (0.0, 0.0, 0.0)
            self._d = 1e-6

    def arm(self, on):
        pass

    def stop(self):
        pass

    def close(self):
        pass


class RealBackend:
    """Real Crazyflie through cflib's high-level commander (drone plans its own path)."""

    def __init__(self, uri, deck, logger):
        from cflib.crazyflie import Crazyflie
        from cflib.crazyflie.log import LogConfig
        from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
        from cflib.utils.reset_estimator import reset_estimator

        self.log = logger
        self._reset = reset_estimator
        self._lock = threading.Lock()
        self._pos = (0.0, 0.0, 0.0)
        self.vbat = 0.0
        cache = os.path.expanduser('~/.cache/cf_fleet')
        os.makedirs(cache, exist_ok=True)
        self.scf = SyncCrazyflie(uri, cf=Crazyflie(rw_cache=cache))
        self.scf.open_link()
        self.scf.wait_for_params()
        self.cf = self.scf.cf
        if deck and int(float(self.cf.param.get_value(f'deck.{deck}'))) != 1:
            self.scf.close_link()
            raise RuntimeError(f'{uri}: positioning deck "{deck}" not detected')
        lc = LogConfig(name='fleet', period_in_ms=100)
        for v in ('stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z', 'pm.vbat'):
            lc.add_variable(v, 'float')
        self.cf.log.add_config(lc)
        lc.data_received_cb.add_callback(self._on_log)
        lc.start()
        self.log.info(f'{uri}: setting (0, 0) here, keep the drone still ...')
        self._reset(self.cf)
        self.hlc = self.cf.high_level_commander

    def _on_log(self, _ts, data, _conf):
        with self._lock:
            self._pos = (data['stateEstimate.x'], data['stateEstimate.y'],
                         data['stateEstimate.z'])
            self.vbat = data['pm.vbat']

    def position(self):
        with self._lock:
            return self._pos

    def go_to(self, x, y, z, duration):
        self.hlc.go_to(x, y, z, 0.0, duration, relative=False)

    def takeoff(self, height, duration):
        self.hlc.takeoff(height, duration)

    def land(self, duration):
        self.hlc.land(0.0, duration)

    def rezero(self):
        self._reset(self.cf)

    def arm(self, on):
        try:
            self.cf.supervisor.send_arming_request(on)
        except AttributeError:
            self.log.warn('This cflib has no send_arming_request. Update cflib '
                          '(the Brushless will not spin its motors without arming).')

    def stop(self):
        self.hlc.stop()

    def close(self):
        self.scf.close_link()


# --------------------------------------------------------------------------- one drone

class Drone:

    def __init__(self, node, name, backend, origin, height):
        self.node, self.name, self.cf = node, name, backend
        self.ox, self.oy = float(origin[0]), float(origin[1])
        self.oyaw = math.radians(float(origin[2]))
        self.height = float(height)
        self.lock = threading.Lock()
        self.state, self.phase, self.job, self.done, self.msg = 'idle', 'landed', '', [], ''
        self.miss = 0.0
        self.flying = False
        self.abort = threading.Event()
        self.thread = None
        self.status_pub = node.create_publisher(String, f'/{name}/status', 10)
        self.pose_pub = node.create_publisher(PointStamped, f'/{name}/pose', 10)
        node.create_subscription(String, f'/{name}/route', self.on_route, 10)

    # map = origin + rotate(yaw) * local
    def to_local(self, x, y):
        dx, dy = x - self.ox, y - self.oy
        c, s = math.cos(self.oyaw), math.sin(self.oyaw)
        return c * dx + s * dy, -s * dx + c * dy

    def to_map(self, lx, ly):
        c, s = math.cos(self.oyaw), math.sin(self.oyaw)
        return self.ox + c * lx - s * ly, self.oy + s * lx + c * ly

    def map_position(self):
        lx, ly, z = self.cf.position()
        x, y = self.to_map(lx, ly)
        return x, y, z

    def set(self, **kw):
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)
        if 'phase' in kw:
            self.node.get_logger().info(f'{self.name}: {self.phase}')

    def status(self):
        x, y, z = self.map_position()
        with self.lock:
            return {'name': self.name, 'state': self.state, 'phase': self.phase,
                    'job': self.job, 'done': list(self.done),
                    'x': round(x, 3), 'y': round(y, 3), 'z': round(z, 3),
                    'vbat': round(float(self.cf.vbat), 2), 'miss': round(self.miss, 3),
                    'msg': self.msg}

    def busy(self):
        return self.thread is not None and self.thread.is_alive()

    # ---- incoming route
    def on_route(self, msg):
        log = self.node.get_logger()
        try:
            req = json.loads(msg.data)
            job = str(req.get('job', ''))
            wps = req['waypoints']
            if not isinstance(wps, list) or not wps:
                raise ValueError('no waypoints')
        except Exception as e:
            log.warn(f'{self.name}: bad route message: {e}')
            return
        if self.busy():
            log.warn(f'{self.name}: busy, route "{job}" ignored')
            return
        try:
            err = self.node.check_route(self, wps)
        except Exception as e:
            err = f'bad waypoint: {e}'
        if err:
            log.warn(f'{self.name}: route "{job}" rejected: {err}')
            self.set(state='failed', job=job, done=[], msg=f'rejected: {err}')
            return
        self.abort.clear()
        self.set(state='busy', phase='takeoff', job=job, done=[], msg='', miss=0.0)
        self.thread = threading.Thread(target=self.run, args=(wps,), daemon=True)
        self.thread.start()

    def rezero(self):
        if self.busy() or self.flying:
            self.node.get_logger().warn(f'{self.name}: cannot re-zero while flying')
            return
        self.set(phase='rezeroing')
        self.thread = threading.Thread(target=self._rezero, daemon=True)
        self.thread.start()

    def _rezero(self):
        try:
            self.cf.rezero()
            self.set(state='idle', phase='landed', msg='re-zeroed on its mark')
        except Exception as e:
            self.set(state='failed', phase='landed', msg=f're-zero failed: {e}')

    # ---- flying (runs in its own thread)
    def wait(self, seconds):
        if self.abort.wait(seconds):
            raise Aborted()

    def watch(self, cx, cy, seconds):
        """Stay put for `seconds`; return the biggest x/y wander from (cx, cy)."""
        worst = 0.0
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.wait(0.1)
            x, y, _ = self.cf.position()
            worst = max(worst, math.hypot(x - cx, y - cy))
        return worst

    def fly(self, lx, ly, z, duration=None):
        n = self.node
        if duration is None:
            cx, cy, cz = self.cf.position()
            dist = math.sqrt((lx - cx) ** 2 + (ly - cy) ** 2 + (z - cz) ** 2)
            duration = max(dist / n.speed, n.min_leg_time)
        self.cf.go_to(lx, ly, z, duration)
        self.wait(duration + n.settle_time)
        # Did it get there (as far as the drone itself knows)? Give it a moment.
        end = time.monotonic() + n.arrive_wait
        while True:
            x, y, _ = self.cf.position()
            miss = math.hypot(x - lx, y - ly)
            if miss <= n.arrive_tol or time.monotonic() >= end:
                break
            self.wait(0.1)
        if miss > n.arrive_tol:
            n.get_logger().warn(f'{self.name}: {miss:.2f} m short of its target')
        with self.lock:
            self.miss = max(self.miss, miss)

    def run(self, wps):
        n = self.node
        try:
            self.cf.arm(True)
            self.wait(n.arm_time)
            gx, gy, _ = self.cf.position()
            self.flying = True
            self.cf.takeoff(self.height, n.takeoff_time)
            self.wait(n.takeoff_time + n.settle_time)
            if n.hover_check > 0.0:
                self.set(phase='hover_check')
                worst = self.watch(gx, gy, n.hover_check)
                if worst > n.hover_tol:
                    raise Drifting(f'drifted {worst:.2f} m right after take-off '
                                   '(check the floor and light)')
            for wp in wps:
                act = wp.get('action', 'go')
                if act == 'hover':
                    self.set(phase='hovering')
                    cx, cy, _ = self.cf.position()
                    worst = self.watch(cx, cy, float(wp.get('t', 5.0)))
                    self.set(msg=f'hover test: wandered up to {worst:.2f} m')
                    with self.lock:
                        self.done.append(act)
                    continue
                if act == 'home':
                    lx, ly = 0.0, 0.0
                else:
                    lx, ly = self.to_local(float(wp['x']), float(wp['y']))
                self.set(phase=f'to_{act}')
                self.fly(lx, ly, self.height)
                if act in ('pick', 'drop'):
                    self.set(phase='picking' if act == 'pick' else 'dropping')
                    self.fly(lx, ly, min(n.dip_height, self.height), n.dip_time)
                    self.wait(n.dwell)
                    self.fly(lx, ly, self.height, n.dip_time)
                with self.lock:
                    self.done.append(act)
            self.set(phase='landing')
            self.cf.land(n.land_time)
            self.wait(n.land_time + n.settle_time)
            self.cf.stop()
            self.cf.arm(False)
            self.flying = False
            self.set(state='idle', phase='landed')
        except Aborted:
            self.land_here('aborted')
        except Drifting as e:
            n.get_logger().error(f'{self.name}: {e}. Landing.')
            self.land_here(str(e))
        except Exception as e:
            n.get_logger().error(f'{self.name}: flight error: {e}')
            self.land_here(f'error: {e}')

    def land_here(self, why):
        n = self.node
        try:
            if self.flying:
                self.set(phase='landing')
                self.cf.land(n.land_time)
                time.sleep(n.land_time + n.settle_time)
                self.cf.stop()
            self.cf.arm(False)
        except Exception as e:
            n.get_logger().error(f'{self.name}: landing error: {e}')
        self.flying = False
        self.set(state='failed', phase='landed', msg=why)


# --------------------------------------------------------------------------- the node

class Fleet(Node):

    def __init__(self):
        super().__init__('cf_fleet')
        p = self.declare_parameter
        p('sim', False)
        p('drones', ['cf1'])
        p('deck', 'bcFlow2')
        p('speed', 0.2)            # m/s average along each leg (slower = more accurate)
        p('dip_height', 0.25)      # height of the pick / drop dip
        p('dwell', 2.0)            # seconds spent down at the dip
        p('dip_time', 1.5)
        p('takeoff_time', 4.0)     # same gentle timings as the tested fly_to_target
        p('land_time', 4.0)
        p('arm_time', 1.0)
        p('min_leg_time', 1.0)
        p('settle_time', 0.5)
        p('hover_check', 2.0)      # seconds of hover after take-off (0 = skip)
        p('hover_tol', 0.15)       # land if it wanders more than this during the check
        p('arrive_tol', 0.15)      # "arrived" if within this of the target
        p('arrive_wait', 2.0)      # extra seconds allowed to arrive
        p('fence', [-3.0, 3.0, -3.0, 3.0])   # xmin, xmax, ymin, ymax (map metres)
        p('max_leg', 4.0)                     # refuse any single leg longer than this

        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.sim = bool(g('sim'))
        for k in ('speed', 'dip_height', 'dwell', 'dip_time', 'takeoff_time', 'land_time',
                  'arm_time', 'min_leg_time', 'settle_time', 'hover_check', 'hover_tol',
                  'arrive_tol', 'arrive_wait', 'max_leg'):
            setattr(self, k, float(g(k)))
        self.fence = [float(v) for v in g('fence')]
        deck = g('deck')

        names = [str(n) for n in g('drones')]
        if not self.sim:
            import cflib.crtp
            cflib.crtp.init_drivers()
        self.drones = []
        try:
            for i, name in enumerate(names):
                p(f'{name}.uri', f'radio://0/80/2M/E7E7E7E70{i + 1}')
                p(f'{name}.origin', [0.0, 0.5 * (i + 1), 0.0])  # x, y, facing in degrees
                p(f'{name}.height', 0.5 + 0.3 * i)  # different heights: no mid-air crash
                if self.sim:
                    backend = SimBackend()
                else:
                    self.get_logger().info(f'{name}: connecting {g(f"{name}.uri")} ...')
                    backend = RealBackend(g(f'{name}.uri'), deck, self.get_logger())
                d = Drone(self, name, backend, g(f'{name}.origin'), g(f'{name}.height'))
                self.drones.append(d)
                self.get_logger().info(
                    f'{name}: ready{" (SIM)" if self.sim else ""}, base at map '
                    f'({d.ox:.2f}, {d.oy:.2f}), facing {math.degrees(d.oyaw):.0f} deg, '
                    f'cruise {d.height:.2f} m')
        except Exception:
            for d in self.drones:
                d.cf.close()
            raise

        self.tf = TransformBroadcaster(self)
        self.create_subscription(Empty, '/fleet/land_all', self.on_land_all, 10)
        self.create_subscription(String, '/fleet/rezero', self.on_rezero, 10)
        self.create_timer(0.1, self.tick)

    def check_route(self, d, wps):
        px, py, _ = d.map_position()
        xmin, xmax, ymin, ymax = self.fence
        for wp in wps:
            act = wp.get('action', 'go')
            if act not in ('go', 'pick', 'drop', 'home', 'hover'):
                return f'unknown action "{act}"'
            if act == 'hover':
                t = float(wp.get('t', 5.0))
                if not 0.0 < t <= 120.0:
                    return 'hover time must be between 0 and 120 s'
                continue
            if act == 'home':
                x, y = d.ox, d.oy
            else:
                x, y = float(wp['x']), float(wp['y'])
                if not (math.isfinite(x) and math.isfinite(y)):
                    return 'waypoint is not a number'
            if not (xmin <= x <= xmax and ymin <= y <= ymax):
                return f'({x:.2f}, {y:.2f}) is outside the fence'
            if math.hypot(x - px, y - py) > self.max_leg:
                return f'leg to ({x:.2f}, {y:.2f}) is longer than {self.max_leg} m'
            px, py = x, y
        return ''

    def on_land_all(self, _msg):
        self.get_logger().warn('LAND ALL')
        for d in self.drones:
            d.abort.set()

    def on_rezero(self, msg):
        want = msg.data.strip()
        for d in self.drones:
            if want in ('all', d.name):
                d.rezero()

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        tfs = []
        for d in self.drones:
            st = d.status()
            sm = String()
            sm.data = json.dumps(st)
            d.status_pub.publish(sm)

            ps = PointStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = 'map'
            ps.point.x, ps.point.y, ps.point.z = float(st['x']), float(st['y']), float(st['z'])
            d.pose_pub.publish(ps)

            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'map'
            t.child_frame_id = d.name
            t.transform.translation.x = ps.point.x
            t.transform.translation.y = ps.point.y
            t.transform.translation.z = ps.point.z
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(d.oyaw / 2.0)
            t.transform.rotation.w = math.cos(d.oyaw / 2.0)
            tfs.append(t)
        self.tf.sendTransform(tfs)

    def shutdown(self):
        for d in self.drones:
            d.abort.set()
        for d in self.drones:
            if d.thread is not None:
                d.thread.join(timeout=self.land_time + 3.0)
        for d in self.drones:
            try:
                d.cf.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Fleet()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()  # lands anything still flying
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
