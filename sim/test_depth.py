exec(open('test_neg.py').read().split("def run(")[0])
M.max_range = 4.0; M.Kinv = np.linalg.inv(M.K)
# Tmc: camera at (1,0,0.2) looking +y. bottle on floor 1.0 m ahead -> box bottom pixel:
v2 = 240 + 500*0.2/1.0; u1, u2 = 300, 340
Zg = M._depth_from_ground(Tmc, 320, v2); print('ground prior', round(Zg,3))
wall = np.array([[x, 0.03, 2.4] for x in np.linspace(-0.05, 0.05, 6)])        # cam frame: x right, z fwd
bott = np.array([[x, 0.03, 1.0] for x in np.linspace(-0.02, 0.02, 3)])
print('clear bottle (only wall in band): ungated', round(M._depth_from_lidar(wall,u1,u2),2), '| gated', M._depth_from_lidar(wall,u1,u2,Zg))
print('opaque bottle + wall: gated', round(M._depth_from_lidar(np.vstack([wall,bott]),u1,u2,Zg),2))
