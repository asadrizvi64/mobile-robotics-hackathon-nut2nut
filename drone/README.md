# Drone side: `cf_target_flight`

These files run on the **drone PC**, which is native Ubuntu 22.04 with ROS 2 Humble and has the Crazyradio plugged in. That PC has to be on the same Wi-Fi as the robot and use `ROS_DOMAIN_ID=30`.

| File | What it is | Status |
|---|---|---|
| `cf_target_flight/cf_fleet.py` | Flies N Crazyflies on one radio. It takes map-frame routes on `/cfN/route` and reports `/cfN/status` (JSON, 10 Hz). It also has a hover check, an arrival check, e-stop on `/fleet/land_all`, and `/fleet/rezero`. | Written for the hackathon. Tested only against a fake cflib/rclpy. |
| `cf_target_flight/mission.py` | The original "referee". Simulated pick and drop into a safe zone, RViz markers. | Same. **Superseded for the scout demo** by `scout_dispatch.py`; see below. |
| `cf_target_flight/scout_dispatch.py` | **Glue between robot and drones**: when the robot finds a bottle, the closest free drone flies over it, hovers, and goes home. | New. Logic is unit-tested (`sim/test_dispatch.py`) and tested end to end with the robot simulator (`sim/sim_drones.py`). **Not flown yet.** |
| `fleet_mission.yaml` | Parameters for `cf_fleet` and `mission`. | |
| `scout_dispatch.yaml` | Parameters for `scout_dispatch`. | |

The full design notes for `cf_fleet` and `mission` are in [`docs/FINAL_HANDOFF.md`](../docs/FINAL_HANDOFF.md), sections 8–10.

## Why `scout_dispatch` and not `mission.py`

`mission.py` merges detections by position and **freezes** an object's position as soon as it has been seen 3 times. The robot, though, sends the camera estimate first, which is often 15–25 cm off. A few seconds later, after docking, it sends the **lidar-measured** position under the **same `id`**. With `mission.py` the drone would fly to the camera estimate. `scout_dispatch` does this instead:

- It keeps the latest position for each robot `id`, and flies only once the position is precise. From `target_mission` v18 the robot marks this with `"precise": true`. With older robots, it flies once the position has been stable for 3 s.
- It keeps the bottle after the robot drops it from the topic, so a drone that is busy can take it later.
- It flies **directly over** the bottle (`offset: 0.0`), hovers there, then goes home and lands. It never lands on the bottle, because `cf_fleet` lands at the last waypoint, and that is always `home`.
- It publishes `/scout/drone_state`. With `-p wait_for_drone:=true`, the robot stands still until the drone has flown its bottle. This keeps the robot from driving under a hovering drone, which would confuse the Flow deck's height sensor.
- It handles failures the same way as `mission.py`: closest free drone first, resend if a route is lost, bench a drifting drone and let the other one retry, skip a target that was rejected or has failed twice.
- If the robot is restarted, it counts ids from 1 again. A known id that suddenly jumps far away is treated as a new bottle. A new id at a bottle that has already been flown is ignored.

## Install (drone PC)

```bash
# into the existing package next to fly_to_target.py
cp cf_target_flight/{cf_fleet.py,mission.py,scout_dispatch.py} ~/ros2_ws/src/cf_target_flight/cf_target_flight/
cp fleet_mission.yaml scout_dispatch.yaml ~/
# setup.py -> entry_points -> console_scripts, add:
#   'cf_fleet = cf_target_flight.cf_fleet:main',
#   'mission = cf_target_flight.mission:main',
#   'scout_dispatch = cf_target_flight.scout_dispatch:main',
cd ~/ros2_ws && colcon build --packages-select cf_target_flight && source install/setup.bash
export ROS_DOMAIN_ID=30
python3 -c "import cflib, cflib.utils.reset_estimator; print(cflib.__file__)"   # must work
```

If cflib lives in a venv, see `docs/FINAL_HANDOFF.md` §10. In short: build without `--symlink-install`, and add `[build_scripts] executable = /usr/bin/env python3` to `setup.cfg`.

## Arena setup (do this before anything flies)

1. **Mark A** is the robot's start spot, and it is the map origin (0, 0). +x is the way the robot faces at start, and +y is to its left.
2. **Drone bases must be OUTSIDE the robot's 1.5 m circle.** A landed Crazyflie is about 3 cm tall, and the robot's lidar scans at 17 cm, so **the robot cannot see a landed drone**. Example: `cf1.origin: [0.0, 1.9, 0.0]` and `cf2.origin: [0.0, -1.9, 0.0]`. Measure each base from mark A with a tape measure, and put the values in `fleet_mission.yaml`.
3. In `fleet_mission.yaml`, set `fence` to at least the robot's area plus the bases, e.g. `[-1.7, 1.7, -2.1, 2.1]`. The old value `[-1.0, 3.5, -1.5, 2.0]` would reject bottles at x < -1.0.
4. Use a bottle at least 25 cm tall, wrapped in tape, so the robot's lidar can see it.

## Run order

```bash
# 1. ROBOT (on mark A) + laptop: as usual, from the laptop
bash go.sh -p wait_for_drone:=true

# 2. DRONE PC, terminal 1: drones on their tape marks (the estimator resets at start)
ros2 run cf_target_flight cf_fleet --ros-args --params-file ~/fleet_mission.yaml
#    rehearsal without flying:  ... --params-file ~/fleet_mission.yaml -p sim:=true

# 3. DRONE PC, terminal 2
ros2 run cf_target_flight scout_dispatch --ros-args --params-file ~/scout_dispatch.yaml

# watch
ros2 topic echo /scout/events
ros2 topic echo /objects/json std_msgs/msg/String --field data
ros2 topic echo /cf1/status std_msgs/msg/String --field data

# E-STOP: every drone lands where it is
ros2 topic pub --once /fleet/land_all std_msgs/msg/Empty
```

**Alignment check** (2 minutes, worth it): put a bottle on a measured spot, e.g. (1.0, 0.0), and send one drone there by hand:

```bash
ros2 topic pub --once /cf1/route std_msgs/msg/String \
  "{data: '{\"job\":\"check\",\"waypoints\":[{\"x\":1.0,\"y\":0.0,\"action\":\"go\"},{\"action\":\"hover\",\"t\":5.0},{\"action\":\"home\"}]}'}"
```

If it hovers beside the bottle instead of over it, that drone's `origin` or the robot's start heading is off. Fix the YAML.

## Known risks (honest list)

- Flow deck drift is about 5–15 cm per minute of flight. The robot's position is accurate to 1–3 cm, but the drone will not be that accurate.
- Directly above a bottle, the Flow deck's ToF sensor reads the bottle top. The drone may bob up by the bottle's height. If it does, use `offset: 0.15` or a higher cruise height.
- `cf_fleet` has not flown real drones from this code. Do the `-p sim:=true` rehearsal and one single-drone `hover` route first.
