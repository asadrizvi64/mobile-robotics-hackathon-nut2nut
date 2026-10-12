# TurtleBot + Crazyflie: ground scout for a drone team

Built at a robotics hackathon in October 2026 at TU Dresden.

A **TurtleBot3 Waffle Pi** finds target objects (bottles), measures exactly where they are, and hands each position to a team of **Crazyflie 2.1 Brushless** drones. Robot and drones share one coordinate frame.

The idea: drones are fast but know poorly where they are, because the Flow deck drifts. The ground robot is slow but precise. So the robot is the **scout**, and the drones are the **responders**.

## What the robot does

1. **Maps** the arena with its lidar (`slam_toolbox`). The map origin `(0, 0)` is a taped mark on the floor, **mark A**, and the drones' start spots are measured from the same mark.
2. **Detects** bottles with the Pi camera and **YOLOv8n**, running on a laptop. Camera and lidar are fused: the camera says *what* and *which direction*, the lidar says *where*.
3. **Searches** within 1.5 m of mark A by turning and looking: turn 45°, stand still 2.5 s so YOLO gets sharp frames over Wi-Fi, repeat.
4. **Tells the drone** the moment a bottle is seen (camera position), then **drives to it**.
5. **Docks** on the bottle using its own lidar along the camera's line of sight, and stops with the bumper about 6 cm away. It **measures** the bottle there (about 1–3 cm accuracy) and **replaces** the drone's target with that precise position, under the same `id`.
6. **Backs off** to clear the space for the drone, publishes for 10 s, then moves on to the next bottle. At the end it returns to mark A.

**Safety, all running on the robot:**

- Obstacle avoidance that uses the Waffle's real asymmetric footprint, including the rear swinging out during turns.
- A hard stop for anything within 6 cm of the bumper.
- A 1.5 m geofence around mark A.
- Smooth acceleration, to avoid current spikes.
- If the perception link (Wi-Fi) drops, the robot holds still.

## Drone interface

Topic `/objects/json`, type `std_msgs/msg/String`, 5 Hz:

```json
{"objects": [{"id": 1, "cls": "bottle", "x": 1.212, "y": -0.278, "z": 0.124, "conf": 0.93}]}
```

- `x, y` are in metres, in the SLAM `map` frame: (0,0) is mark A, +x is the direction the robot faced at the start, +y is to its left.
- The list holds at most one bottle: the one the robot is working on. Otherwise it is `{"objects": []}`.
- The position of an `id` gets more precise over time, so **fly to the latest one**.

The raw detections from the camera + lidar mapper are on `/objects/detections`.

## Repository layout

```
ros2_ws/src/swarm_hack/                ROS 2 (Humble) package
  swarm_hack/object_mapper.py          laptop: YOLO + lidar fusion, 360° lidar tracking -> /objects/detections
  swarm_hack/video_recorder.py         laptop: record camera / YOLO debug video to MP4
  swarm_hack/crazyflie_bridge.py,
  swarm_coordinator.py                 earlier drone-side prototype (simulated drones)
  scripts/target_mission.py            ROBOT: search -> dock -> measure -> publish -> next (the main mission)
  scripts/straight_avoid.py            ROBOT: drive a straight line, swerve around obstacles
  scripts/scan_check.py                ROBOT: lidar health check (open room vs blocked vs frozen beams)
  scripts/go.sh                        LAPTOP: one command - checks everything, then runs the mission
  scripts/robot_start.sh               ROBOT: bringup + SLAM + camera + bridge, all at once
  scripts/robot_extras.sh              ROBOT: camera (15 fps, JPEG 60) + zenoh bridge
  scripts/laptop.sh                    LAPTOP: tmux session with the bridge + mapper
  config/swarm.yaml                    parameters
ros2_ws/src/tb_nav/                    SLAM / Nav2 configs, RViz layout, obstacle guard, nav manager
sim/                                   2-D simulator + tests for the mission (no ROS needed)
docs/                                  runbooks, handovers, drone-team interface notes
```

## Setup

| Machine | Software | Network |
|---|---|---|
| **Robot** | Raspberry Pi, Ubuntu 22.04, ROS 2 Humble, `camera_ros`, `slam_toolbox`, `ros-humble-zenoh-bridge-dds` | Fast DDS, `ROS_DOMAIN_ID=30` |
| **Laptop** | Windows 10 + WSL2 Ubuntu 22.04, ROS 2 Humble, `ultralytics` (YOLOv8n) | joins through `zenoh-bridge-dds` over TCP (WSL NAT has no multicast) |
| **Drones** | separate laptop, `cf_fleet` / mission node | same Fast DDS domain, reads `/objects/json` |

```bash
# laptop
cd ~/ws && colcon build --symlink-install --packages-select swarm_hack tb_nav
# copy the robot scripts once
scp ros2_ws/src/swarm_hack/scripts/{robot_start.sh,robot_extras.sh,target_mission.py} turtle@<robot-ip>:~/
# robot on mark A, then from the laptop:
bash ros2_ws/src/swarm_hack/scripts/go.sh            # or: go.sh -p max_targets:=2   /   go.sh fix
```

Watch it run:

```bash
ros2 topic echo /mission/status
ros2 topic echo /objects/json std_msgs/msg/String --field data
```

## Simulator and tests

`sim/` contains a kinematic simulator of the Waffle, with its exact footprint, a 360-beam lidar, and a camera model that has latency, motion blur and range bias. It runs the real `target_mission.py` logic unchanged.

```bash
cd sim
python3 sim_all.py      # 6 hand-made arenas
python3 sim_rand.py     # 8 random arenas
python3 sim_bias.py     # camera range error +20 / -15 / +30 cm
python3 sim_today.py    # the real arena layout from the hackathon + a block right beside a bottle
python3 sim_hop.py      # the mapper track jumping onto a block / a block mislabelled as a bottle
python3 test_lidar.py; python3 test_mission_ros.py; python3 test_scan_check.py
```

Each run prints how many bottles were released, the position error, the minimum footprint clearance, the furthest distance from mark A and the cable twist.

## Lessons from the real runs

- **Clear plastic bottles** are half-invisible to the LDS-01 lidar, and short bottles only show their neck at the 17 cm scan height. Use a tall bottle wrapped in tape.
- **Camera range** from a 12 cm-high camera can be 20 cm off. Trust the camera's bearing, and use the lidar for range.
- **Wi-Fi bridge clocks:** the zenoh bridge drops messages stamped more than 100 ms in the future, so keep Windows, WSL and robot clocks synced (NTP).
- **Power:** current spikes on abrupt starts and turns can brown out the robot. Hence the acceleration limits.

See `docs/` for the full runbooks and handovers.
