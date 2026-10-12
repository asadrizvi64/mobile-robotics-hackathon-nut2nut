import os
import sys, types, math, random
class Node:
    def __init__(self,*a,**k): pass
    def create_subscription(self,*a): pass
    def destroy_node(self): pass
st={'n':0}
def spin_once(node, timeout_sec=0):
    r=[]
    for i in range(360):
        a=math.radians(i)
        if i<180: r.append(1.34 + random.uniform(-0.01,0.01))                       # stale half circle
        elif i<250: r.append(0.0)
        elif i<300: r.append(0.05)
        else: r.append(2.0+random.uniform(-0.01,0.01))
    node.cb(types.SimpleNamespace(ranges=r, angle_min=0.0, angle_increment=math.radians(1), range_min=0.12, range_max=3.5, header=types.SimpleNamespace(frame_id='base_scan')))
sys.modules['rclpy']=types.SimpleNamespace(init=lambda:None, ok=lambda:True, spin_once=spin_once, shutdown=lambda:None)
sys.modules['rclpy.node']=types.SimpleNamespace(Node=Node)
sys.modules['rclpy.qos']=types.SimpleNamespace(qos_profile_sensor_data=None)
sys.modules['sensor_msgs']=types.ModuleType('sensor_msgs'); sys.modules['sensor_msgs.msg']=types.SimpleNamespace(LaserScan=object)
exec(open(os.path.join(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ros2_ws", "src", "swarm_hack"), 'scripts', 'scan_check.py')).read().replace("if __name__ == '__main__':\n    main()",'main()'))
