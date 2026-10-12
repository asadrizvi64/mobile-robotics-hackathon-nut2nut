src = open('sim_mission.py').read().split("print('A:")[0]
src = src.replace("est[i] = (bx + random.gauss(0, 0.03), by + random.gauss(0, 0.03))",
 "ux_, uy_ = (bx - x) / d, (by - y) / d; est[i] = (bx + BIAS * ux_ + random.gauss(0, 0.03), by + BIAS * uy_ + random.gauss(0, 0.03))")
exec(src)
for BIAS in (0.20, -0.15, 0.30):
    tm.DEFAULTS['max_targets'] = 2
    print(f'camera range error {BIAS:+.2f} m (today: +0.20)')
    run([(0.45, 0.15), (-0.7, 0.8)], [(0.2, -0.6, 0.12)], verbose=True)
