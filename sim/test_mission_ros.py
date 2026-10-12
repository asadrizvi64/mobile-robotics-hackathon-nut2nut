import os
# Stub ROS to exercise the real ROS wrapper code paths of target_mission.py (catches typos / API misuse).
import sys, types, math, json
pubs = {}
class Pub:
    def __init__(self, topic): self.topic = topic; self.msgs = []; pubs[topic] = self
    def publish(self, m): self.msgs.append(m)
class Logger:
    def info(self, m, **k): print('[INFO]', m)
    def warn(self, m, **k): print('[WARN]', m)
class Param:
    def __init__(self, v): self.value = v
class Node:
    def __init__(self, name): self.subs = {}; self.timers = []
    def declare_parameter(self, k, v): return Param(v)
    def create_publisher(self, t, topic, q): return Pub(topic)
    def create_subscription(self, t, topic, cb, q): self.subs[topic] = cb
    def create_timer(self, p, cb): self.timers.append(cb)
    def get_logger(self): return Logger()
    def destroy_node(self): pass
class Twist:
    def __init__(self): self.linear = types.SimpleNamespace(x=0.0); self.angular = types.SimpleNamespace(z=0.0)
class String:
    def __init__(self, data=''): self.data = data
class TfEx(Exception): pass
POSE = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
class Buffer:
    def lookup_transform(self, a, b, t):
        q = types.SimpleNamespace(x=0.0, y=0.0, z=math.sin(POSE['yaw']/2), w=math.cos(POSE['yaw']/2))
        return types.SimpleNamespace(transform=types.SimpleNamespace(rotation=q, translation=types.SimpleNamespace(x=POSE['x'], y=POSE['y'])))
NODE = {}
def spin_once(node, timeout_sec=0):
    NODE['n'] = node
    raise SystemExit   # stop main loop immediately; we drive callbacks manually
mods = {
 'rclpy': types.SimpleNamespace(init=lambda **k: None, ok=lambda: True, spin_once=spin_once, shutdown=lambda: None),
 'rclpy.node': types.SimpleNamespace(Node=Node),
 'rclpy.qos': types.SimpleNamespace(QoSProfile=lambda **k: None, ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT=1), qos_profile_sensor_data=None),
 'rclpy.time': types.SimpleNamespace(Time=lambda: None),
 'rclpy.signals': types.SimpleNamespace(SignalHandlerOptions=types.SimpleNamespace(NO=0)),
 'geometry_msgs': types.ModuleType('g'), 'geometry_msgs.msg': types.SimpleNamespace(Twist=Twist),
 'sensor_msgs': types.ModuleType('s'), 'sensor_msgs.msg': types.SimpleNamespace(LaserScan=object),
 'std_msgs': types.ModuleType('st'), 'std_msgs.msg': types.SimpleNamespace(String=String),
 'tf2_ros': types.SimpleNamespace(Buffer=Buffer, TransformException=TfEx, TransformListener=lambda b, n: None),
}
sys.modules.update(mods)
import importlib.util
spec = importlib.util.spec_from_file_location('tm', os.path.join(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ros2_ws", "src", "swarm_hack"), 'scripts', 'target_mission.py'))
tm = importlib.util.module_from_spec(spec); spec.loader.exec_module(tm)
try:
    tm.main()
except SystemExit:
    pass
n = NODE['n']
scan = types.SimpleNamespace(angle_min=0.0, angle_increment=math.radians(1), range_min=0.12, ranges=[2.0]*360)
n.subs['/scan'](scan)
n.subs['/objects/detections'](String(json.dumps({'stamp': 1, 'objects': [{'id': 0, 'cls': 'bottle', 'x': 1.5, 'y': 0.5, 'z': 0.15, 'conf': 0.9, 'hits': 10, 'track': 'camera+lidar'}]})))
for _ in range(3):
    for cb in n.timers: cb()
print('cmd_vel msgs:', len(pubs['/cmd_vel'].msgs), 'last v,w =', pubs['/cmd_vel'].msgs[-1].linear.x, round(pubs['/cmd_vel'].msgs[-1].angular.z, 2))
print('/objects/json:', pubs['/objects/json'].msgs[-1].data)
print('/mission/status:', pubs['/mission/status'].msgs[-1].data)
