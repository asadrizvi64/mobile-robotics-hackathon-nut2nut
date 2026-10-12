exec(open('sim_mission.py').read().split("print('A:")[0])
import random as R
fails = 0
for seed in range(8):
    R.seed(100 + seed)
    nb, no = R.randint(2, 3), R.randint(3, 5)
    def free(px, py, objs, d):
        return math.hypot(px, py) > 0.55 and all(math.hypot(px - a, py - b) > d for a, b, *_ in objs)
    obs, bot = [], []
    while len(obs) < no:
        p_ = (R.uniform(-1.4, 1.4), R.uniform(-1.4, 1.4), R.uniform(0.08, 0.18))
        if free(p_[0], p_[1], obs, 0.7): obs.append(p_)
    while len(bot) < nb:
        p_ = (R.uniform(-1.3, 1.3), R.uniform(-1.3, 1.3))
        if free(p_[0], p_[1], obs + [(a, b) for a, b in bot], 0.6): bot.append(p_)
    tm.DEFAULTS['max_targets'] = nb
    print(f'seed {seed}: {nb} bottles, {no} obstacles')
    c = run(bot, obs, start=(0, 0, R.uniform(-3, 3)), seed=seed, T=500)
    ok = c.state == 'DONE' and len(c.released) == nb
    fails += not ok
print('FAILS', fails)
