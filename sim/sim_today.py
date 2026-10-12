src = open('sim_mission.py').read().split("print('A:")[0]
src = src.replace("est[i] = (bx + random.gauss(0, 0.03), by + random.gauss(0, 0.03))",
 "ux_, uy_ = (bx - x) / d, (by - y) / d; est[i] = (bx + BIAS * ux_ + random.gauss(0, 0.03), by + BIAS * uy_ + random.gauss(0, 0.03))")
exec(src)
tm.DEFAULTS['max_targets'] = 2
for BIAS in (0.2, -0.15):
    print(f'TODAY layout, camera range error {BIAS:+.2f}: bottles (0.50,0.19) & (1.21,-0.28), blocks (0.72,-0.35) (0.9,0.1)')
    run([(0.50, 0.19), (1.21, -0.28)], [(0.72, -0.35, 0.08), (0.9, 0.1, 0.08)], verbose=True)
    print(f'block 15 cm beside the bottle, camera error {BIAS:+.2f}')
    run([(0.9, 0.3), (-0.6, 0.6)], [(0.9, 0.08, 0.07)], verbose=True)
