exec(open('sim_mission.py').read().split("print('A:")[0])
print('A: 3 bottles, box between'); run([(1.2, 0.6), (-1.0, 1.0), (0.3, -1.2)], [(0.6, 0.3, 0.15)])
print('B: bottle behind + bottle next to a box'); run([(-1.2, -0.2), (1.0, -1.0)], [(1.0, -0.6, 0.12), (0.0, 0.8, 0.2)])
print('C: no bottles'); run([], [(0.5, 0.5, 0.15)])
tm.DEFAULTS['max_targets'] = 2
print('D: max_targets=2, box between bottle 2 and home'); run([(1.2, 0.6), (-1.0, 1.0)], [(0.6, 0.3, 0.15)])
print('E: max_targets=2, 60cm wall of boxes between bottle and home'); run([(1.3, 1.0), (-1.0, -1.0)], [(0.5, 0.25, 0.1), (0.62, 0.38, 0.1), (0.38, 0.12, 0.1), (0.74, 0.5, 0.1)])
print('F: max_targets=2, other start heading, 4 random obstacles'); run([(1.0, -0.4), (-0.8, 1.2)], [(0.5, -0.9, 0.12), (-0.6, 0.3, 0.12), (1.1, 0.9, 0.15), (-1.0, -0.8, 0.15)], start=(0, 0, 2.0))
