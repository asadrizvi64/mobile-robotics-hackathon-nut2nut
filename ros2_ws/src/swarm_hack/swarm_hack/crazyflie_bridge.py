"""Crazyflie bridge: N drones on ONE Crazyradio, all in this one process.

Why one process: a Crazyradio USB dongle can only be opened by one process,
so two separate bridge nodes would fight over it.

Localization model (Flow deck only, no Lighthouse/UWB/mocap):
  * The Kalman estimator on each drone gives pose in its own odometry frame
    <name>/odom: origin = where it was when the estimator was reset,
    x = its nose direction at reset, z = height from the ToF sensor.
  * At takeoff we look up where the TurtleBot is in the SLAM map and add the
    drone's known spawn offset on/next to the robot. That gives the fixed
    transform map -> <name>/odom (the "anchor").
  * map -> <name>/odom -> <name>/base_link is published on TF, so every drone
    has a 3D pose in the robot's SLAM map. Drift grows with distance flown
    (optical flow), so re-anchor by landing at the spawn spot and taking off again.

Topics per drone (name = cf1, cf2, ...):
  sub  /<name>/goal     geometry_msgs/PoseStamped  (frame: map)
  pub  /<name>/pose     geometry_msgs/PoseStamped  (frame: map)
  pub  /<name>/status   std_msgs/String            (IDLE/RESETTING/TAKEOFF/FLYING/LANDING/ESTOP)
  pub  /<name>/battery  std_msgs/Float32           (V)
  pub  /<name>/zrange   std_msgs/Float32           (m, raw downward ToF)
  srv  /<name>/takeoff, /<name>/land               std_srvs/Trigger
Swarm-wide:
  sub  /swarm/estop     std_msgs/Empty   -> motors off NOW
  srv  /swarm/takeoff_all, /swarm/land_all, /swarm/reset_estop
"""
import math
import threading
import time
from types import SimpleNamespace

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Empty, Float32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from .geom import clamp, quat_from_yaw, se2_apply, se2_inverse_apply, wrap, yaw_from_quat


class SimLink:
    """Fake Crazyflie for sim:=true. First-order response to position setpoints."""

    def __init__(self):
        self.pos = [0.0, 0.0, 0.0, 0.0]
        self.sp = None

    def reset(self):
        self.pos = [0.0, 0.0, 0.0, 0.0]
        self.sp = None

    def setpoint(self, x, y, z, yaw):
        self.sp = [x, y, z, yaw]

    def stop(self):
        self.sp = None

    def step(self, dt):
        if self.sp is None:
            self.pos[2] = max(0.0, self.pos[2] - 1.5 * dt)
            return
        k = min(1.0, dt / 0.35)
        for i in range(3):
            self.pos[i] += k * (self.sp[i] - self.pos[i])
        self.pos[3] = wrap(self.pos[3] + k * wrap(self.sp[3] - self.pos[3]))


class Drone:
    def __init__(self, node, name, uri, spawn, sim):
        self.node, self.name, self.uri, self.spawn, self.sim = node, name, uri, spawn, sim
        self.log = node.get_logger()
        self.lock = threading.Lock()
        self.est = None            # (x, y, z, yaw) in <name>/odom
        self.t_est = 0.0
        self.vbat = float('nan')
        self.zrange = float('nan')
        self.anchor = None         # (x, y, yaw) of <name>/odom in map
        self.state = 'DISCONNECTED'
        self.t_state = time.monotonic()
        self.sp = None             # current rate-limited setpoint in odom [x, y, z, yaw]
        self.goal_map = None       # last commanded goal in map (x, y, z, yaw)
        self.t_goal = 0.0
        self.cf = None
        self.scf = None
        self.simlink = SimLink() if sim else None

    # ---------------- connection ----------------
    def connect(self):
        if self.sim:
            self.set_state('IDLE')
            return
        from cflib.crazyflie import Crazyflie
        from cflib.crazyflie.log import LogConfig
        from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

        self.scf = SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cf_cache'))
        self.scf.open_link()
        self.cf = self.scf.cf

        lc = LogConfig(name='state', period_in_ms=20)
        for v in ('stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z', 'stabilizer.yaw'):
            lc.add_variable(v, 'float')
        aux = LogConfig(name='aux', period_in_ms=100)
        aux.add_variable('pm.vbat', 'float')
        aux.add_variable('range.zrange', 'uint16_t')
        for conf, cb in ((lc, self._on_state), (aux, self._on_aux)):
            self.cf.log.add_config(conf)
            conf.data_received_cb.add_callback(cb)
            conf.start()
        self.set_state('IDLE')

    def close(self):
        if self.cf is not None:
            try:
                self.cf.commander.send_stop_setpoint()
                self.scf.close_link()
            except Exception:
                pass

    def _on_state(self, _ts, data, _conf):
        with self.lock:
            self.est = (data['stateEstimate.x'], data['stateEstimate.y'],
                        data['stateEstimate.z'], math.radians(data['stabilizer.yaw']))
            self.t_est = time.monotonic()

    def _on_aux(self, _ts, data, _conf):
        self.vbat = data['pm.vbat']
        self.zrange = data['range.zrange'] / 1000.0

    # ---------------- state ----------------
    def set_state(self, s):
        if s != self.state:
            self.log.info(f'[{self.name}] {self.state} -> {s}')
        self.state = s
        self.t_state = time.monotonic()

    def odom_to_map(self, p):
        x, y = se2_apply(self.anchor, (p[0], p[1]))
        return x, y, p[2], wrap(p[3] + self.anchor[2])

    def map_to_odom(self, g):
        x, y = se2_inverse_apply(self.anchor, (g[0], g[1]))
        return x, y, g[2], wrap(g[3] - self.anchor[2])

    # ---------------- commands ----------------
    def takeoff(self, anchor):
        if self.state != 'IDLE':
            return False, f'{self.name} is {self.state}, not IDLE'
        self.anchor = anchor
        self.set_state('RESETTING')
        threading.Thread(target=self._reset_then_takeoff, daemon=True).start()
        return True, f'{self.name}: resetting estimator, then taking off'

    def _reset_then_takeoff(self):
        """Runs in its own thread so the 25 Hz setpoint stream of the OTHER drone never stalls."""
        try:
            if self.sim:
                self.simlink.reset()
                time.sleep(0.5)
            else:
                try:
                    if int(self.cf.param.get_value('deck.bcFlow2')) != 1:
                        self.log.error(f'[{self.name}] Flow deck v2 not detected - refusing takeoff')
                        self.set_state('IDLE')
                        return
                except Exception:
                    self.log.warn(f'[{self.name}] could not read deck.bcFlow2, continuing')
                self.cf.param.set_value('kalman.resetEstimation', '1')
                time.sleep(0.1)
                self.cf.param.set_value('kalman.resetEstimation', '0')
                time.sleep(2.0)
                with self.lock:
                    est = self.est
                if est is None or abs(est[0]) > 0.1 or abs(est[1]) > 0.1 or est[2] > 0.15:
                    self.log.error(f'[{self.name}] estimator did not settle ({est}) - refusing takeoff')
                    self.set_state('IDLE')
                    return
                arm = getattr(self.cf.platform, 'send_arming_request', None)  # Brushless needs arming
                if arm is not None:
                    arm(True)
                    time.sleep(1.0)
            self.sp = [0.0, 0.0, 0.0, 0.0]
            self.set_state('TAKEOFF')
        except Exception as e:  # noqa: BLE001
            self.log.error(f'[{self.name}] takeoff failed: {e}')
            self.set_state('IDLE')

    def land(self):
        if self.state in ('TAKEOFF', 'FLYING'):
            self.set_state('LANDING')
            return True, f'{self.name} landing'
        return False, f'{self.name} is {self.state}'

    def estop(self):
        self.set_state('ESTOP')
        self._send_stop()

    def _send_sp(self, sp):
        if self.sim:
            self.simlink.setpoint(*sp)
        else:
            self.cf.commander.send_position_setpoint(sp[0], sp[1], sp[2], math.degrees(sp[3]))

    def _send_stop(self):
        if self.sim:
            self.simlink.stop()
        elif self.cf is not None:
            self.cf.commander.send_stop_setpoint()

    def _disarm(self):
        if not self.sim and self.cf is not None:
            arm = getattr(self.cf.platform, 'send_arming_request', None)
            if arm is not None:
                arm(False)

    # ---------------- control loop (called at rate_hz) ----------------
    def step(self, now, dt, P):
        if self.sim and self.state != 'DISCONNECTED':
            self.simlink.step(dt)
            with self.lock:
                self.est = tuple(self.simlink.pos)
                self.t_est = now
            self.vbat, self.zrange = 4.0, self.simlink.pos[2]

        st = self.state
        if st in ('DISCONNECTED', 'IDLE', 'RESETTING'):
            return
        if st == 'ESTOP':
            self._send_stop()
            return

        with self.lock:
            est, t_est = self.est, self.t_est

        # --- safety checks ---
        if st in ('TAKEOFF', 'FLYING'):
            if est is None or now - t_est > P.est_timeout:
                self.log.error(f'[{self.name}] state estimate stale - landing')
                self.set_state('LANDING')
            elif not math.isnan(self.vbat) and self.vbat < P.low_bat and now - self.t_state > 2.0:
                self.log.warn(f'[{self.name}] battery {self.vbat:.2f} V - landing')
                self.set_state('LANDING')
        st = self.state
        sp = self.sp

        if st == 'TAKEOFF':
            sp[2] = min(P.takeoff_z, sp[2] + P.vz_max * dt)
            if est is not None and est[2] > 0.85 * P.takeoff_z and now - self.t_state > 1.0:
                self.goal_map = self.odom_to_map(sp)
                self.t_goal = now
                self.set_state('FLYING')

        elif st == 'FLYING':
            age = now - self.t_goal
            if age > P.land_timeout:
                self.log.warn(f'[{self.name}] no goal for {age:.1f}s - landing')
                self.set_state('LANDING')
                target = sp
            elif age > P.cmd_timeout or self.goal_map is None:
                target = sp  # coordinator silent: hold position
            else:
                gx = clamp(self.goal_map[0], P.fence_min[0], P.fence_max[0])
                gy = clamp(self.goal_map[1], P.fence_min[1], P.fence_max[1])
                gz = clamp(self.goal_map[2], P.z_min, P.z_max)
                target = self.map_to_odom((gx, gy, gz, self.goal_map[3]))
            dx, dy = target[0] - sp[0], target[1] - sp[1]
            d, lim = math.hypot(dx, dy), P.vxy_max * dt
            if d > lim:
                dx, dy = dx * lim / d, dy * lim / d
            sp[0] += dx
            sp[1] += dy
            sp[2] += clamp(target[2] - sp[2], -P.vz_max * dt, P.vz_max * dt)
            sp[3] = wrap(sp[3] + clamp(wrap(target[3] - sp[3]), -P.yaw_rate * dt, P.yaw_rate * dt))

        if self.state == 'LANDING':
            sp[2] = max(0.0, sp[2] - P.vz_land * dt)
            if sp[2] <= 0.05 or (est is not None and est[2] < 0.08):
                self._send_stop()
                self._disarm()
                self.set_state('IDLE')
                return

        self._send_sp(sp)


class CrazyflieBridge(Node):
    def __init__(self):
        super().__init__('crazyflie_bridge')
        dp = self.declare_parameter
        names = dp('names', ['cf1', 'cf2']).value
        uris = dp('uris', ['radio://0/80/2M/E7E7E7E701', 'radio://0/80/2M/E7E7E7E702']).value
        self.sim = dp('sim', False).value
        self.map_frame = dp('map_frame', 'map').value
        self.spawn_frame = dp('spawn_frame', 'base_footprint').value
        rate = dp('rate_hz', 25.0).value
        self.P = SimpleNamespace(
            takeoff_z=dp('takeoff_z', 0.6).value,
            vxy_max=dp('vxy_max', 0.4).value,       # m/s, setpoint slew limit
            vz_max=dp('vz_max', 0.3).value,
            vz_land=dp('vz_land', 0.25).value,
            yaw_rate=math.radians(dp('yaw_rate_deg', 60.0).value),
            cmd_timeout=dp('cmd_timeout', 0.5).value,   # hold if no goal for this long
            land_timeout=dp('land_timeout', 3.0).value,  # land if no goal for this long
            est_timeout=dp('est_timeout', 0.5).value,
            low_bat=dp('low_battery_v', 3.2).value,
            fence_min=dp('fence_min', [-3.0, -3.0]).value,
            fence_max=dp('fence_max', [3.0, 3.0]).value,
            z_min=dp('z_min', 0.3).value,
            z_max=dp('z_max', 1.4).value,
        )
        if len(names) != len(uris):
            raise ValueError('names and uris must have the same length')

        if not self.sim:
            import cflib.crtp
            cflib.crtp.init_drivers()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)

        self.drones, self.pubs = {}, {}
        for i, (n, u) in enumerate(zip(names, uris)):
            spawn = dp(f'spawn.{n}', [-0.4, 0.4 if i % 2 == 0 else -0.4, 0.0]).value
            d = Drone(self, n, u, tuple(spawn), self.sim)
            try:
                d.connect()
                self.get_logger().info(f'{n} connected ({"sim" if self.sim else u})')
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'{n}: cannot connect to {u}: {e}')
            self.drones[n] = d
            self.pubs[n] = SimpleNamespace(
                pose=self.create_publisher(PoseStamped, f'/{n}/pose', 10),
                status=self.create_publisher(String, f'/{n}/status', 10),
                battery=self.create_publisher(Float32, f'/{n}/battery', 10),
                zrange=self.create_publisher(Float32, f'/{n}/zrange', 10),
            )
            self.create_subscription(PoseStamped, f'/{n}/goal', lambda m, d=d: self._on_goal(d, m), 10)
            self.create_service(Trigger, f'/{n}/takeoff', lambda rq, rs, d=d: self._takeoff(d, rs))
            self.create_service(Trigger, f'/{n}/land', lambda rq, rs, d=d: self._land(d, rs))

        self.create_subscription(Empty, '/swarm/estop', self._on_estop, 10)
        self.create_service(Trigger, '/swarm/takeoff_all', self._takeoff_all)
        self.create_service(Trigger, '/swarm/land_all', self._land_all)
        self.create_service(Trigger, '/swarm/reset_estop', self._reset_estop)

        self.t_last = time.monotonic()
        self.tick_n = 0
        self.create_timer(1.0 / rate, self._tick)

    # ---------------- helpers ----------------
    def _spawn_anchor(self, d):
        """Pose of the drone's odom origin in map = robot pose (+) spawn offset."""
        if self.spawn_frame == self.map_frame:
            base = (0.0, 0.0, 0.0)
        else:
            try:
                tf = self.tf_buffer.lookup_transform(self.map_frame, self.spawn_frame, Time())
                t = tf.transform.translation
                base = (t.x, t.y, yaw_from_quat(tf.transform.rotation))
            except TransformException as e:
                if not self.sim:
                    self.get_logger().error(f'no TF {self.map_frame}->{self.spawn_frame}: {e}')
                    return None
                base = (0.0, 0.0, 0.0)
        sx, sy, syaw = d.spawn
        px, py = se2_apply(base, (sx, sy))
        return px, py, wrap(base[2] + syaw)

    def _on_goal(self, d, m):
        if m.header.frame_id not in ('', self.map_frame):
            self.get_logger().warn(f'{d.name}: goal must be in {self.map_frame}', throttle_duration_sec=2.0)
            return
        p = m.pose.position
        d.goal_map = (p.x, p.y, p.z, yaw_from_quat(m.pose.orientation))
        d.t_goal = time.monotonic()

    def _takeoff(self, d, res):
        anchor = self._spawn_anchor(d)
        if anchor is None:
            res.success, res.message = False, 'no robot TF - is SLAM running?'
        else:
            res.success, res.message = d.takeoff(anchor)
        return res

    def _land(self, d, res):
        res.success, res.message = d.land()
        return res

    def _takeoff_all(self, _req, res):
        msgs = [self._takeoff(d, Trigger.Response()) for d in self.drones.values()]
        res.success = all(m.success for m in msgs)
        res.message = '; '.join(m.message for m in msgs)
        return res

    def _land_all(self, _req, res):
        msgs = [d.land() for d in self.drones.values()]
        res.success, res.message = True, '; '.join(m for _, m in msgs)
        return res

    def _on_estop(self, _msg):
        self.get_logger().error('E-STOP')
        for d in self.drones.values():
            d.estop()

    def _reset_estop(self, _req, res):
        for d in self.drones.values():
            if d.state == 'ESTOP':
                d._disarm()
                d.set_state('IDLE')
        res.success, res.message = True, 'ESTOP cleared'
        return res

    # ---------------- main loop ----------------
    def _tick(self):
        now = time.monotonic()
        dt, self.t_last = min(now - self.t_last, 0.1), now
        self.tick_n += 1
        stamp = self.get_clock().now().to_msg()
        tfs = []
        for n, d in self.drones.items():
            try:
                d.step(now, dt, self.P)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'{n} step failed: {e}', throttle_duration_sec=1.0)
            pub = self.pubs[n]
            if self.tick_n % 5 == 0:
                pub.status.publish(String(data=d.state))
                pub.battery.publish(Float32(data=float(d.vbat)))
                pub.zrange.publish(Float32(data=float(d.zrange)))
            with d.lock:
                est = d.est
            if est is None or d.anchor is None:
                continue
            a = TransformStamped()
            a.header.stamp, a.header.frame_id, a.child_frame_id = stamp, self.map_frame, f'{n}/odom'
            a.transform.translation.x, a.transform.translation.y = d.anchor[0], d.anchor[1]
            a.transform.rotation = quat_from_yaw(d.anchor[2])
            b = TransformStamped()
            b.header.stamp, b.header.frame_id, b.child_frame_id = stamp, f'{n}/odom', f'{n}/base_link'
            b.transform.translation.x, b.transform.translation.y, b.transform.translation.z = est[:3]
            b.transform.rotation = quat_from_yaw(est[3])
            tfs += [a, b]
            mx, my, mz, myaw = d.odom_to_map(est)
            ps = PoseStamped()
            ps.header.stamp, ps.header.frame_id = stamp, self.map_frame
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = mx, my, mz
            ps.pose.orientation = quat_from_yaw(myaw)
            pub.pose.publish(ps)
        if tfs:
            self.tf_pub.sendTransform(tfs)

    def destroy_node(self):
        for d in self.drones.values():
            d.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = CrazyflieBridge()
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
