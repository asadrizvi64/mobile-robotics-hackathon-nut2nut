#!/usr/bin/env python3
"""mission: the search-and-rescue "referee".

The robot searches and finds objects (YOLO -> /objects/json, map frame).
For each object this node gives a job to the nearest free drone:
  fly to the object -> dip ("pick up") -> fly to the safe zone -> dip ("drop")
  -> fly back to base -> land.
The pickup is simulated. RViz shows the package hanging under the drone on a
rope and then sitting in the safe zone, so the audience sees the whole story.

  in   /objects/json     std_msgs/String  (from object_mapper)
  in   /<drone>/status   std_msgs/String  (from cf_fleet)
  in   /mission/cmd      std_msgs/String  start | pause | land | reset | status
  in   /fleet/land_all   std_msgs/Empty   (anyone's e-stop also pauses the mission)
  out  /<drone>/route    std_msgs/String  (to cf_fleet)
  out  /mission/markers  visualization_msgs/MarkerArray  (add this in RViz)
  out  /mission/events   std_msgs/String  (human-readable log, nice on screen)
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

RED = (1.0, 0.25, 0.2, 1.0)
AMBER = (1.0, 0.7, 0.1, 1.0)
GREEN = (0.2, 0.85, 0.35, 1.0)
GRAY = (0.6, 0.6, 0.6, 0.8)
BLUE = (0.2, 0.5, 1.0, 1.0)
WHITE = (1.0, 1.0, 1.0, 1.0)
ZONE = (0.2, 0.85, 0.35, 0.25)


class Item:
    """One found object. States: seen -> waiting -> assigned -> carried -> delivered,
    or skipped. x, y stay at the object's real spot (it never really moves), so YOLO
    seeing it again later does not create a duplicate."""

    def __init__(self, iid, cls, x, y, conf):
        self.id, self.cls, self.x, self.y, self.conf = iid, cls, x, y, conf
        self.n = 1
        self.state = 'seen'
        self.drone = ''
        self.job = ''
        self.slot = None
        self.tries = 0
        self.sent_at = 0.0
        self.resends = 0


class Mission(Node):

    def __init__(self):
        super().__init__('mission')
        p = self.declare_parameter
        p('drones', ['cf1', 'cf2'])
        p('objects_topic', '/objects/json')
        p('classes', 'bottle,cup,teddy bear,sports ball,backpack')  # "*" = any class
        p('min_conf', 0.4)
        p('min_sightings', 3)        # YOLO must see it this many times before it counts
        p('merge_radius', 0.35)      # detections closer than this are the same object
        p('drop_zone', [0.0, 1.5])   # safe zone centre in map metres
        p('zone_size', 0.6)
        p('slot_spacing', 0.2)       # packages are laid out in a 3x3 grid in the zone
        p('approach_offset', 0.2)    # dip this far beside the object, not on top of it
        p('return_home', True)       # after the drop, fly back to the drone's base
        p('min_vbat', 3.4)           # don't start a job below this battery voltage
        p('auto_start', False)
        p('test_objects', '')        # e.g. "bottle:1.0,-0.3; cup:2.0,0.4" (rehearsal)

        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.drones = [str(d) for d in g('drones')]
        cls = [c.strip() for c in str(g('classes')).split(',') if c.strip()]
        self.classes = set() if '*' in cls else set(cls)
        self.min_conf = float(g('min_conf'))
        self.min_sightings = int(g('min_sightings'))
        self.merge_radius = float(g('merge_radius'))
        self.zone = (float(g('drop_zone')[0]), float(g('drop_zone')[1]))
        self.zone_size = float(g('zone_size'))
        self.slot_spacing = float(g('slot_spacing'))
        self.offset = float(g('approach_offset'))
        self.return_home = bool(g('return_home'))
        self.min_vbat = float(g('min_vbat'))
        self.running = bool(g('auto_start'))

        self.items = []
        self.next_id = 1
        self.slots_used = 0
        self.active = {}       # drone -> Item it is working on
        self.status = {}       # drone -> last status dict
        self.low_batt_warned = set()
        self.benched = set()   # drones that failed (e.g. drifted) stay down until 'start'
        self.completed = False

        self.events = self.create_publisher(String, '/mission/events', 10)
        self.markers = self.create_publisher(MarkerArray, '/mission/markers', 10)
        self.land_pub = self.create_publisher(Empty, '/fleet/land_all', 10)
        self.route_pubs = {d: self.create_publisher(String, f'/{d}/route', 10)
                           for d in self.drones}
        for d in self.drones:
            self.create_subscription(String, f'/{d}/status',
                                     lambda m, d=d: self.on_status(d, m), 10)
        self.create_subscription(String, g('objects_topic'), self.on_objects, 10)
        self.create_subscription(String, '/mission/cmd', self.on_cmd, 10)
        self.create_subscription(Empty, '/fleet/land_all', self.on_land_all, 10)
        self.create_timer(0.5, self.tick)
        self.create_timer(0.2, self.publish_markers)

        for part in str(g('test_objects')).split(';'):
            self.add_test_object(part)
        hint = 'Running.' if self.running else "Send 'start' on /mission/cmd."
        self.event(f'Mission ready. Drones: {", ".join(self.drones)}. Safe zone at '
                   f'({self.zone[0]:.2f}, {self.zone[1]:.2f}). {hint}')

    # ------------------------------------------------------------------ helpers
    def event(self, text):
        self.get_logger().info(text)
        m = String()
        m.data = text
        self.events.publish(m)

    def count(self, *states):
        return sum(1 for it in self.items if it.state in states)

    def add_test_object(self, part):
        part = part.strip()
        if not part:
            return
        try:
            cls, _, xy = part.rpartition(':')
            x, y = (float(v) for v in xy.split(','))
        except ValueError:
            self.get_logger().warn(f'test_objects: cannot read "{part}"')
            return
        it = Item(self.next_id, cls.strip() or 'package', x, y, 1.0)
        self.next_id += 1
        self.items.append(it)
        self.accept(it)

    def accept(self, it):
        it.state = 'waiting'
        self.completed = False
        self.event(f'Found {it.cls} #{it.id} at ({it.x:.2f}, {it.y:.2f})')

    # ------------------------------------------------------------------ inputs
    def on_objects(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(data, dict):
            data = data.get('objects', [data] if 'x' in data else [])
        if not isinstance(data, list):
            return
        for o in data:
            try:
                cls = str(o.get('cls', o.get('class', o.get('label', 'object'))))
                x, y = float(o['x']), float(o['y'])
                conf = float(o.get('conf', o.get('score', 1.0)))
            except (KeyError, TypeError, ValueError, AttributeError):
                continue
            self.add_detection(cls, x, y, conf)

    def add_detection(self, cls, x, y, conf):
        if not (math.isfinite(x) and math.isfinite(y)) or conf < self.min_conf:
            return
        if self.classes and cls not in self.classes:
            return
        best, best_d = None, self.merge_radius
        for it in self.items:
            dd = math.hypot(it.x - x, it.y - y)
            if dd < best_d:
                best, best_d = it, dd
        if best is None:
            half = self.zone_size / 2.0
            if abs(x - self.zone[0]) < half and abs(y - self.zone[1]) < half:
                return  # ignore anything inside the safe zone
            it = Item(self.next_id, cls, x, y, conf)
            self.next_id += 1
            self.items.append(it)
            if it.n >= self.min_sightings:
                self.accept(it)
            return
        if best.state in ('seen', 'waiting'):
            k = min(best.n, 20)  # running average, so the position settles
            best.x = (best.x * k + x) / (k + 1)
            best.y = (best.y * k + y) / (k + 1)
            best.n += 1
            best.conf = max(best.conf, conf)
            if best.state == 'seen' and best.n >= self.min_sightings:
                self.accept(best)

    def on_status(self, d, msg):
        try:
            st = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        st['_t'] = time.monotonic()
        self.status[d] = st
        it = self.active.get(d)
        if it is None or st.get('job') != it.job:
            return
        done = st.get('done', [])
        if it.state == 'assigned' and 'pick' in done:
            it.state = 'carried'
            self.event(f'{d} picked up {it.cls} #{it.id}')
        if it.state == 'carried' and 'drop' in done:
            it.state = 'delivered'
            self.event(f'{d} delivered {it.cls} #{it.id} to the safe zone '
                       f'({self.count("delivered")}/{self.count_found()})')
        if st.get('state') in ('idle', 'failed') and st.get('phase') == 'landed':
            del self.active[d]
            if it.state == 'delivered':
                self.event(f'{d} landed{" back at base" if self.return_home else ""}')
                return
            why = st.get('msg') or 'unknown reason'
            it.drone = ''
            if not why.startswith('rejected') and why != 'aborted':
                # Drift or an error: keep this drone on the ground, the others carry on.
                self.benched.add(d)
                self.event(f'{d} is benched ({why}). The other drones carry on. '
                           "Fix it, then send 'start' again.")
            if why.startswith('rejected') or it.tries >= 2:
                it.state = 'skipped'
                self.event(f'Skipping {it.cls} #{it.id}: {why}')
            else:
                it.state = 'waiting'
                self.event(f'{d} could not finish {it.cls} #{it.id} ({why}); will retry')

    def on_cmd(self, msg):
        c = msg.data.strip().lower()
        if c == 'start':
            self.running = True
            if self.benched:
                self.event(f'Back in service: {", ".join(sorted(self.benched))}')
                self.benched.clear()
            self.event('Mission started')
        elif c == 'pause':
            self.running = False
            self.event('Mission paused (drones in the air finish their job)')
        elif c == 'land':
            self.running = False
            self.land_pub.publish(Empty())
        elif c == 'reset':
            if self.active:
                self.event('Cannot reset while drones are working')
                return
            self.items, self.next_id, self.slots_used = [], 1, 0
            self.completed = False
            clear = Marker()
            clear.action = Marker.DELETEALL
            self.markers.publish(MarkerArray(markers=[clear]))
            self.event('Mission reset')
        elif c == 'status':
            self.event(f'running={self.running} found={self.count_found()} '
                       f'delivered={self.count("delivered")} '
                       f'busy={", ".join(self.active) or "none"} '
                       f'benched={", ".join(sorted(self.benched)) or "none"}')
        else:
            self.event(f'Unknown command "{c}" (start | pause | land | reset | status)')

    def on_land_all(self, _msg):
        if self.running:
            self.event('LAND ALL received: mission paused')
        self.running = False

    def count_found(self):
        return sum(1 for it in self.items if it.state not in ('seen', 'skipped'))

    # ------------------------------------------------------------------ the referee
    def drone_free(self, d, now):
        st = self.status.get(d)
        if st is None or now - st['_t'] > 2.0 or d in self.active or d in self.benched:
            return False
        if st.get('state') not in ('idle', 'failed') or st.get('phase') != 'landed':
            return False
        vbat = float(st.get('vbat', 0.0))
        if 0.1 < vbat < self.min_vbat:
            if d not in self.low_batt_warned:
                self.event(f'{d} battery low ({vbat:.2f} V): not sending it out')
                self.low_batt_warned.add(d)
            return False
        self.low_batt_warned.discard(d)
        return True

    def dist(self, d, it):
        st = self.status[d]
        return math.hypot(it.x - float(st.get('x', 0.0)), it.y - float(st.get('y', 0.0)))

    def next_slot(self):
        k = self.slots_used
        self.slots_used += 1
        col, row = k % 3 - 1, (k // 3) % 3 - 1
        return (self.zone[0] + col * self.slot_spacing, self.zone[1] + row * self.slot_spacing)

    def pick_point(self, it):
        vx, vy = self.zone[0] - it.x, self.zone[1] - it.y
        length = math.hypot(vx, vy)
        if length < 1e-3 or self.offset <= 0.0:
            return it.x, it.y
        off = min(self.offset, length / 2.0)
        return it.x + vx / length * off, it.y + vy / length * off

    def send_route(self, d, it):
        px, py = self.pick_point(it)
        wps = [{'x': round(px, 3), 'y': round(py, 3), 'action': 'pick'},
               {'x': round(it.slot[0], 3), 'y': round(it.slot[1], 3), 'action': 'drop'}]
        if self.return_home:
            wps.append({'action': 'home'})
        m = String()
        m.data = json.dumps({'job': it.job, 'waypoints': wps})
        self.route_pubs[d].publish(m)

    def assign(self, d, it, now):
        it.tries += 1
        it.state, it.drone = 'assigned', d
        it.job = f'{it.cls.replace(" ", "_")}{it.id}-try{it.tries}'
        if it.slot is None:
            it.slot = self.next_slot()
        it.sent_at, it.resends = now, 0
        self.active[d] = it
        self.send_route(d, it)
        self.event(f'{d} is going for {it.cls} #{it.id}')

    def tick(self):
        now = time.monotonic()
        # Jobs that were sent but the drone never started: resend twice, then give up.
        for d, it in list(self.active.items()):
            st = self.status.get(d)
            if st is not None and st.get('job') == it.job:
                continue
            if now - it.sent_at > 3.0:
                if it.resends < 2:
                    it.resends += 1
                    it.sent_at = now
                    self.send_route(d, it)
                else:
                    del self.active[d]
                    it.state, it.drone = 'waiting', ''
                    self.event(f'{d} did not answer; {it.cls} #{it.id} is waiting again')
        if not self.running:
            return
        # Give waiting objects to free drones: closest drone-object pair first.
        free = [d for d in self.drones if self.drone_free(d, now)]
        waiting = [it for it in self.items if it.state == 'waiting']
        while free and waiting:
            d, it = min(((d, it) for d in free for it in waiting),
                        key=lambda pair: self.dist(*pair))
            self.assign(d, it, now)
            free.remove(d)
            waiting.remove(it)
        if (not self.completed and not self.active and self.count('delivered') > 0
                and self.count('seen', 'waiting', 'assigned', 'carried') == 0):
            self.completed = True
            self.event(f'Mission complete: {self.count("delivered")} delivered to the safe zone')

    # ------------------------------------------------------------------ RViz
    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        ms = []

        def mk(ns, mid, mtype, x, y, z, scale, rgba, text=''):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns, m.id, m.type, m.action = ns, int(mid), mtype, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
            m.color.r, m.color.g, m.color.b, m.color.a = (float(v) for v in rgba)
            m.text = text
            ms.append(m)
            return m

        zx, zy = self.zone
        zs = self.zone_size
        mk('zone', 0, Marker.CUBE, zx, zy, 0.005, (zs, zs, 0.01), ZONE)
        mk('zone', 1, Marker.TEXT_VIEW_FACING, zx, zy, 0.45, (0.0, 0.0, 0.12), WHITE,
           f'Safe zone  {self.count("delivered")}/{self.count_found()}')

        for i, d in enumerate(self.drones):
            st = self.status.get(d)
            if st is None:
                continue
            x, y, z = float(st['x']), float(st['y']), float(st['z'])
            mk('drones', 2 * i, Marker.CYLINDER, x, y, z, (0.12, 0.12, 0.03), BLUE)
            mk('drones', 2 * i + 1, Marker.TEXT_VIEW_FACING, x, y, z + 0.15,
               (0.0, 0.0, 0.08), WHITE, f'{d}: {st.get("phase", "")}')

        labels = {'waiting': 'waiting', 'assigned': 'drone on the way',
                  'carried': 'being carried', 'delivered': 'safe', 'skipped': 'skipped'}
        colors = {'waiting': RED, 'assigned': AMBER, 'carried': AMBER,
                  'delivered': GREEN, 'skipped': GRAY}
        for it in self.items:
            if it.state == 'seen':
                continue
            x, y, z = it.x, it.y, 0.04
            st = self.status.get(it.drone)
            carried = it.state == 'carried' and st is not None
            if carried:
                x, y, z = float(st['x']), float(st['y']), max(float(st['z']) - 0.12, 0.04)
            elif it.state == 'delivered' and it.slot is not None:
                x, y = it.slot
            mk('items', 3 * it.id, Marker.CUBE, x, y, z, (0.08, 0.08, 0.08), colors[it.state])
            who = f' ({it.drone})' if it.state in ('assigned', 'carried') else ''
            mk('items', 3 * it.id + 1, Marker.TEXT_VIEW_FACING, x, y, z + 0.12,
               (0.0, 0.0, 0.07), WHITE, f'{it.cls} #{it.id}: {labels[it.state]}{who}')
            rope = mk('items', 3 * it.id + 2, Marker.LINE_LIST, 0.0, 0.0, 0.0,
                      (0.01, 0.0, 0.0), GRAY)
            if carried:
                rope.points = [Point(x=x, y=y, z=float(st['z'])), Point(x=x, y=y, z=z)]
            else:
                rope.action = Marker.DELETE
        self.markers.publish(MarkerArray(markers=ms))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Mission()
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
