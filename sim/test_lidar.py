import os
import sys, types, math, time, json
import numpy as np
# --- stub ROS modules ---
def mod(n, **kw):
    m = types.ModuleType(n); m.__dict__.update(kw); sys.modules[n] = m; return m
class Node:
    def __init__(self, *a, **k): pass
mod('rclpy', init=lambda *a, **k: None); mod('rclpy.node', Node=Node); mod('rclpy.qos', qos_profile_sensor_data=None)
mod('rclpy.time', Time=lambda *a, **k: None)
class Any:
    def __call__(self, *a, **k): return Any()
    def __init__(self, *a, **k): self.__dict__.update(k)
    def __getattr__(self, n):
        v = Any(); setattr(self, n, v); return v
mod('sensor_msgs'); mod('sensor_msgs.msg', CameraInfo=Any, CompressedImage=Any, Image=Any, LaserScan=Any)
mod('std_msgs'); mod('std_msgs.msg', String=Any)
mod('tf2_ros', Buffer=Any, TransformException=Exception, TransformListener=Any)
class Mk(Any):
    DELETEALL=3; CUBE=1; TEXT_VIEW_FACING=9; ADD=0
mod('visualization_msgs'); mod('visualization_msgs.msg', Marker=Mk, MarkerArray=lambda: types.SimpleNamespace(markers=[]))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ros2_ws", "src", "swarm_hack"))
import importlib.util
pkg = mod('swarm_hack'); pkg.__path__ = [os.path.join(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ros2_ws", "src", "swarm_hack"), 'swarm_hack')]
from swarm_hack import object_mapper as om

M = om.ObjectMapper.__new__(om.ObjectMapper)
M.map_frame='map'; M.max_range=4.0; M.min_hits=3
M.lidar_track=True; M.lidar_gate=0.35; M.lidar_cluster=0.12; M.lidar_max_size=0.30; M.obj_radius=0.04
out = {}
M._publish = lambda now: out.update(now=now)
# robot at map (1,0) yaw 90deg; lidar frame = robot
yaw = math.pi/2; rx, ry = 1.0, 0.0
T = np.eye(4); T[:2,:2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]; T[0,3]=rx; T[1,3]=ry
M._lookup = lambda a, b: T

def make_scan(bottle_map, wall_x=None):
    n = 360; ranges = np.full(n, np.inf); inc = 2*math.pi/n
    angs = -math.pi + np.arange(n)*inc
    Tinv = np.linalg.inv(T)
    def ray_circle(c, r):
        cl = (Tinv @ np.array([c[0], c[1], 0, 1]))[:2]
        for i,a in enumerate(angs):
            d = np.array([math.cos(a), math.sin(a)]); b = d@cl; disc = b*b - (cl@cl - r*r)
            if disc >= 0 and b - math.sqrt(disc) > 0: ranges[i] = min(ranges[i], b - math.sqrt(disc))
    ray_circle(bottle_map, 0.045)
    if wall_x is not None:   # wall x = wall_x (map) -> in robot frame
        for i,a in enumerate(angs):
            d_map = np.array([math.cos(a+yaw), math.sin(a+yaw)])
            if d_map[0] > 1e-3:
                t = (wall_x - rx)/d_map[0]
                if 0 < t < ranges[i]: ranges[i] = t
    return types.SimpleNamespace(header=types.SimpleNamespace(frame_id='base_scan'), ranges=list(ranges),
                                 angle_min=-math.pi, angle_increment=inc, range_min=0.12, range_max=3.5)

def obj(p, hits=5):
    return {'id':0,'cls':'bottle','p':np.array([p[0],p[1],0.15]),'w':0.08,'h':0.3,'conf':0.8,'hits':hits,
            'last':time.monotonic()-10,'method':'lidar','lidar_last':0.0,'lidar_hits':0}

# 1) bottle BEHIND the robot (map (1,-0.5)), old estimate off by 15 cm -> converges
true = (1.0, -0.5); M.objects=[obj((1.12, -0.42))]; M.scan = make_scan(true)
for _ in range(10): M._track_and_publish()
o = M.objects[0]; print('behind:', np.round(o['p'][:2],3), 'err', round(np.linalg.norm(o['p'][:2]-true),3), om.ObjectMapper._track_state(o, time.monotonic()))
# 2) bottle moves 15 cm sideways while out of view -> follows
true = (1.15, -0.5); M.scan = make_scan(true)
for _ in range(10): M._track_and_publish()
o = M.objects[0]; print('moved:', np.round(o['p'][:2],3), 'err', round(np.linalg.norm(o['p'][:2]-true),3))
# 3) bottle removed, wall 25 cm away -> must NOT snap to the wall
M.objects=[obj((1.2,-0.5))]; M.scan = make_scan((50,50), wall_x=1.4)
for _ in range(10): M._track_and_publish()
o = M.objects[0]; print('wall only:', np.round(o['p'][:2],3), 'lidar_hits', o['lidar_hits'], om.ObjectMapper._track_state(o, time.monotonic()))
# 4) unconfirmed object (hits<3) -> untouched
M.objects=[obj((1.1,-0.45),hits=1)]; M.scan = make_scan((1.0,-0.5)); M._track_and_publish()
print('unconfirmed lidar_hits', M.objects[0]['lidar_hits'])
# 5) bottle 15 cm from a wall (lidar 1.0,-0.5, wall at x=1.25) -> isolation check
M.objects=[obj((1.05,-0.5))]; M.scan = make_scan((1.0,-0.5), wall_x=1.25)
for _ in range(5): M._track_and_publish()
print('next to wall: lidar_hits', M.objects[0]['lidar_hits'], np.round(M.objects[0]['p'][:2],3))
# real _publish with stubbed publishers
del M._publish
sent = {}
M.json_pub = types.SimpleNamespace(publish=lambda m: sent.update(j=m.data))
M.marker_pub = types.SimpleNamespace(publish=lambda m: sent.update(mk=m))
M.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: None))
M.objects=[obj((1.1,-0.45))]; M.scan = make_scan((1.0,-0.5))
for _ in range(3): M._track_and_publish()
print(sent.get('j'))
