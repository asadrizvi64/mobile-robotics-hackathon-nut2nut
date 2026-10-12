"""Unit tests for drone/cf_target_flight/scout_dispatch.py (pure logic, no ROS).
    python3 test_dispatch.py
"""
import importlib.util
import os

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    'sd', os.path.join(HERE, '..', 'drone', 'cf_target_flight', 'scout_dispatch.py'))
sd = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sd)

FAILS = []


def check(name, cond):
    print(('  ok   ' if cond else '  FAIL ') + name)
    if not cond:
        FAILS.append(name)


def idle(x=0.0, y=0.6, job='', state='idle', msg='', done=(), vbat=3.9):
    return {'state': state, 'phase': 'landed', 'job': job, 'done': list(done), 'x': x, 'y': y, 'vbat': vbat, 'msg': msg}


def busy(job, x=0.0, y=0.6):
    return {'state': 'busy', 'phase': 'to_go', 'job': job, 'done': [], 'x': x, 'y': y, 'vbat': 3.9, 'msg': ''}


def obj(i, x, y, precise=None, cls='bottle'):
    o = {'id': i, 'cls': cls, 'x': x, 'y': y, 'z': 0.12, 'conf': 0.9}
    if precise is not None:
        o['precise'] = precise
    return {'objects': [o]}


def core(**kw):
    c = sd.DispatchCore(drones=['cf1', 'cf2'], fence=[-2.0, 2.0, -2.0, 2.0], **kw)
    c.on_status('cf1', idle(0.0, 0.6), 0.0)
    c.on_status('cf2', idle(0.0, 1.0), 0.0)
    return c


def feed_status(c, t, s1=None, s2=None):
    c.on_status('cf1', s1 or idle(0.0, 0.6), t)
    c.on_status('cf2', s2 or idle(0.0, 1.0), t)


print('1) camera estimate is NOT flown; precise position (same id) is, and only that')
c = core()
for k in range(10):
    c.on_objects(obj(1, 1.20, -0.30, precise=False), k * 0.2)
    feed_status(c, k * 0.2)
    c.tick(k * 0.2)
check('nothing sent while camera-only', not c.routes)
c.on_objects(obj(1, 1.02, -0.28, precise=True), 2.2)
feed_status(c, 2.2)
c.tick(2.2)
check('one route sent after precise', len(c.routes) == 1)
d, r = c.routes[0]
check('closest drone (cf1) chosen', d == 'cf1')
check('route = go over bottle, hover, home', [w['action'] for w in r['waypoints']] == ['go', 'hover', 'home'])
check('flies to the precise spot', abs(r['waypoints'][0]['x'] - 1.02) < 1e-6 and abs(r['waypoints'][0]['y'] + 0.28) < 1e-6)
job = r['job']
c.routes.clear()
for k in range(20):                                  # robot keeps publishing during its dwell
    c.on_objects(obj(1, 1.02, -0.28, precise=True), 2.4 + k * 0.2)
    feed_status(c, 2.4 + k * 0.2, s1=busy(job))
    c.tick(2.4 + k * 0.2)
check('no duplicate route during the dwell', not c.routes)
check('drone_state says flying', c.drone_state()['targets'].get('1') == 'flying')
c.on_objects({'objects': []}, 7.0)
c.on_status('cf1', idle(0.0, 0.6, job=job, done=['go', 'hover', 'home']), 30.0)
check('done after the drone landed', c.drone_state()['targets'].get('1') == 'done')

print('2) lost route: resend twice, then queue again (not counted as a try)')
c = core()
c.on_objects(obj(1, 1.0, 0.0, precise=True), 0.0)
c.tick(0.0)
n = len(c.routes)
for k in range(1, 30):
    feed_status(c, k * 0.5)                          # cf1 keeps saying idle, no job
    c.tick(k * 0.5)
t1 = c.targets[0]
check('resent', len(c.routes) > n)
check('eventually reassigned', t1.state in ('flying', 'queued'))

print('3) drift on cf1 -> cf1 benched, cf2 retries; second failure -> skipped')
c = core()
c.on_objects(obj(1, 1.0, 0.0, precise=True), 0.0)
c.tick(0.0)
d, r = c.routes.pop()
c.on_status(d, busy(r['job']), 1.0)
c.on_status(d, idle(job=r['job'], state='failed', msg='drifted 0.20 m right after take-off'), 10.0)
check('cf1 benched', 'cf1' in c.benched)
feed_status(c, 10.5)
c.tick(10.5)
d2, r2 = c.routes.pop()
check('cf2 retries', d2 == 'cf2')
c.on_status('cf2', busy(r2['job'], 0.0, 1.0), 11.0)
c.on_status('cf2', idle(0.0, 1.0, job=r2['job'], state='failed', msg='error: link lost'), 20.0)
check('skipped after 2 tries', c.targets[0].state == 'skipped')

print('4) rejected route (outside fence) -> skipped, drone not benched')
c = core()
c.on_objects(obj(1, 1.0, 0.0, precise=True), 0.0)
c.tick(0.0)
d, r = c.routes.pop()
c.on_status(d, idle(job=r['job'], state='failed', msg='rejected: (1.00, 0.00) is outside the fence'), 1.0)
check('skipped', c.targets[0].state == 'skipped')
check('not benched', not c.benched)
c = core()
c.on_objects(obj(1, 3.0, 0.0, precise=True), 0.0)
c.tick(0.0)
check('target outside own fence never sent', not c.routes and c.targets[0].state == 'skipped')

print('5) robot restart: id 1 again, far away = new bottle; id 7 at a flown spot = same bottle')
c = core()
c.on_objects(obj(1, 1.0, 0.0, precise=True), 0.0)
c.tick(0.0)
d, r = c.routes.pop()
c.on_status(d, idle(job=r['job'], done=['go', 'hover', 'home']), 30.0)
c.on_objects(obj(1, -0.8, 0.9, precise=True), 40.0)
check('new target created', len(c.targets) == 2)
c.on_objects(obj(7, 1.05, 0.02, precise=True), 41.0)
check('re-announced bottle ignored', len(c.targets) == 2)

print('6) old robot (no precise flag): fly once the position has been stable for settle_s')
c = core()
c.on_objects(obj(1, 1.20, 0.0), 0.0)
c.on_objects(obj(1, 1.20, 0.0), 0.2)
c.on_objects(obj(1, 1.01, 0.0), 0.4)                # lidar refinement jumps it
for k in range(3, 12):
    c.on_objects(obj(1, 1.01, 0.0), k * 0.2)
    feed_status(c, k * 0.2)
    c.tick(k * 0.2)
check('not yet (stable < 3 s)', not c.routes)
c.on_objects({'objects': []}, 3.0)                  # robot drops it after its dwell
feed_status(c, 3.6)
c.tick(3.6)
check('sent after settling, at the refined spot', c.routes and abs(c.routes[0][1]['waypoints'][0]['x'] - 1.01) < 1e-6)

print('7) low battery drone not used; class filter; dip mode; offset')
c = core()
c.on_status('cf1', idle(0.0, 0.6, vbat=3.2), 0.0)
c.on_objects(obj(1, 1.0, 0.0, precise=True), 0.0)
c.on_objects(obj(2, 0.5, 0.5, precise=True, cls='person'), 0.0)
c.tick(0.0)
check('cf2 sent instead of low-battery cf1', len(c.routes) == 1 and c.routes[0][0] == 'cf2')
check('person ignored', len(c.targets) == 1)
c = core(mark='dip', offset=0.2)
c.on_objects(obj(1, 1.0, 0.6, precise=True), 0.0)
c.tick(0.0)
w = c.routes[0][1]['waypoints']
check('dip mode uses pick + home', [x['action'] for x in w] == ['pick', 'home'])
check('offset 0.2 m toward the drone', abs(w[0]['x'] - 0.8) < 1e-3 and abs(w[0]['y'] - 0.6) < 1e-3)

print('8) pause / start')
c = core(auto_start=False)
c.on_objects(obj(1, 1.0, 0.0, precise=True), 0.0)
c.tick(0.0)
check('paused: nothing sent', not c.routes)
c.on_cmd('start', 0.5)
c.tick(0.5)
check('start: sent', len(c.routes) == 1)

print('\nALL PASS' if not FAILS else f'\n{len(FAILS)} FAILED: {FAILS}')
