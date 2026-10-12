import os
exec(open('test_lidar.py').read().split('# 1)')[0])   # reuse stubs + scan maker (robot at (1,0) facing +y)
M.get_logger = lambda: types.SimpleNamespace(info=print, warn=print)
M.single=True; M.drop_unseen_s=3.0; M.view_range=2.0; M.forget_s=5.0; M.assoc_r=0.5
M.K = np.array([[500,0,320],[0,500,240],[0,0,1.0]])
Tmc = np.eye(4); Tmc[:3,0]=[1,0,0]; Tmc[:3,1]=[0,0,-1]; Tmc[:3,2]=[0,1,0]; Tmc[:3,3]=[1.0,0,0.2]
def run(objs, scan, n=8, dt=0.5, matched=()):
    M.objects = objs; M.scan = scan
    for _ in range(n): M._negative_evidence(set(matched), Tmc, 640, 480, dt)
    return [o['id'] for o in M.objects]
def ob(i,p): o=obj(p); o['id']=i; o['p'][2]=0.15; return o
print('visible, YOLO silent -> dropped:', run([ob(1,(1.0,1.0))], make_scan((1.0,1.0))))
print('behind robot -> kept:', run([ob(2,(1.0,-0.6))], make_scan((1.0,-0.6))))
print('too far (3 m) -> kept:', run([ob(3,(1.0,3.0))], make_scan((1.0,3.0))))
o4=ob(4,(1.0,1.5)); 
print('occluded by box at y=0.8 -> kept:', run([o4], make_scan((1.0,0.8))))
o5=ob(5,(1.0,1.0)); print('visible AND matched -> kept:', run([o5], make_scan((1.0,1.0)), matched=[id(o5)]))
# single_per_class through _associate
M.objects=[ob(7,(0.5,2.8))]; M.next_id=10
for _ in range(3): new = M._associate('bottle', np.array([-2.0,0.17,0.15]), .08,.3,.8,'lidar', time.monotonic())
print('new hits', new['hits'])
import textwrap, re
src = open(os.path.join(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ros2_ws", "src", "swarm_hack"), 'swarm_hack', 'object_mapper.py')).read()
blk = src[src.index('        if self.single:'):src.index("        self.objects = [o for o in self.objects\n                        if o['hits'] >= self.min_hits or now - o['last']")]
matched = {id(new)}
exec(textwrap.dedent(blk), {'self': M, 'matched': matched})
print('after single_per_class:', [(o['id'], round(o['p'][0],2)) for o in M.objects])
