src = open('sim_mission.py').read().split("print('A:")[0]
# camera-visible bottles get track camera+lidar; add a mapper 'ghost' that walks away onto a block (track lidar)
src = src.replace("        dets = [{'id': i, 'cls': 'bottle', 'x': e[0], 'y': e[1], 'z': .15, 'conf': .9, 'hits': seen[i]} for i, e in est.items()]",
"""        dets = [{'id': i, 'cls': 'bottle', 'x': e[0], 'y': e[1], 'z': .15, 'conf': .9, 'hits': seen[i], 'track': 'camera+lidar'} for i, e in est.items()]
        if GHOST and 0 in est:
            f = min(1.0, t / 15.0)
            dets.append({'id': 99, 'cls': 'bottle', 'x': bottles[0][0] + f * (GHOST[0] - bottles[0][0]), 'y': bottles[0][1] + f * (GHOST[1] - bottles[0][1]), 'z': .15, 'conf': .9, 'hits': 50, 'track': 'lidar'})
            dets.append({'id': 98, 'cls': 'bottle', 'x': GHOST[0], 'y': GHOST[1], 'z': .15, 'conf': .9, 'hits': 50, 'track': 'camera+lidar'} if t > 15 else dets[0])""")
exec(src)
GHOST = (1.0, 0.45)
tm.DEFAULTS['max_targets'] = 1
print('bottle at (1.2, 0.0), blocks at (0.95, 0.4) and (0.7, -0.35); mapper ghost walks onto the block')
run([(1.2, 0.0)], [(0.95, 0.4, 0.07), (0.7, -0.35, 0.07)], verbose=True)
