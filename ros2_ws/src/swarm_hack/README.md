# swarm_hack — TurtleBot SLAM + 2× Crazyflie (Flow deck) swarm

The TurtleBot is the "mothership": it owns the map (lidar SLAM) and the eyes (camera + YOLO).
The drones have no global sensing, so they borrow the robot's map.

- **3D localization of the drones:** each drone's Flow deck gives a pose in its own odometry frame.
  At takeoff we read the robot's SLAM pose and add the drone's known spawn offset.
  That fixes `map -> cfN/odom`, so `map -> cfN/odom -> cfN/base_link` is a full 3D pose in the robot's map.
- **Robot → drones:** the drones fly a formation that moves with the robot.
- **Robot → drones:** objects the robot detects get handed to a drone, which flies over and hovers above them.
- **Drones → robot:** `/swarm/events` reports each inspection, with an experimental object-height
  estimate from the drone's downward ToF.

```
TurtleBot (Wi-Fi, same ROS_DOMAIN_ID)          Laptop
  lidar -> slam_toolbox -> TF map->base  ---->  swarm_coordinator --/cfN/goal--> crazyflie_bridge ==radio==> cf1, cf2
  camera ------------------------------------>  object_mapper --/objects/json--^         |
                                                                                    TF map->cfN/odom->cfN/base_link
```

Three processes on purpose: YOLO inference must never stall the 25 Hz setpoint stream.

## Install (laptop, ROS 2 Humble)

```bash
pip install cflib ultralytics "numpy<2"
mkdir -p ~/ws/src && cp -r swarm_hack ~/ws/src/
cd ~/ws && colcon build --symlink-install && source install/setup.bash
```

USB permissions for the Crazyradio: follow the udev rules in the cflib docs.
Give each drone its own radio address (cfclient → Configure 2.x) on the same channel.

## Run

```bash
# On the TurtleBot
ros2 launch turtlebot3_bringup robot.launch.py          # or turtlebot4 bringup
# On the laptop
ros2 launch slam_toolbox online_async_launch.py
ros2 launch swarm_hack swarm.launch.py                  # sim:=true for fake drones
rviz2                                                   # Fixed frame: map. Add TF, Map, /objects/markers, /swarm/targets
```

Fly:

```bash
ros2 service call /swarm/takeoff_all std_srvs/srv/Trigger
ros2 run teleop_twist_keyboard teleop_twist_keyboard    # drive the robot; drones follow
ros2 topic pub --once /swarm/mode std_msgs/String "data: auto"        # auto-inspect new objects
ros2 topic pub --once /swarm/mode std_msgs/String "data: inspect 3"   # send a drone to object 3
ros2 topic pub --once /swarm/mode std_msgs/String "data: hold"
ros2 topic pub --once /swarm/mode std_msgs/String "data: goto 1.5 0.5"      # nearest drone -> map point
ros2 topic pub --once /swarm/mode std_msgs/String "data: goto cf2 1.0 -0.5 1.0"
ros2 topic pub --once /swarm/mode std_msgs/String "data: fly table"          # waypoint saved by tb_nav
ros2 topic pub --once /swarm/mode std_msgs/String "data: home"               # back to formation
# or: RViz toolbar "Publish Point" -> click on the map -> nearest drone flies there
ros2 topic echo /swarm/events
ros2 service call /swarm/land_all std_srvs/srv/Trigger
ros2 topic pub --once /swarm/estop std_msgs/Empty "{}"               # motors off now
```

## One-day plan

| Time | Goal | Done when |
|---|---|---|
| 0–2 h | Robot SLAM + TF on the laptop | `ros2 run tf2_ros tf2_echo map base_footprint` works on the laptop |
| 2–3 h | Full stack with `sim:=true` | Fake drones follow the robot in RViz while you teleop |
| 3–5 h | One real drone, follow mode | Hovers in its slot while the robot drives slowly |
| 5–7 h | object_mapper | Stable markers on 2–3 objects, `/objects/json` ids don't churn |
| 7–9 h | Both drones + `auto` mode | A drone flies to a detected object and returns |
| 9+ h | Rehearse the demo, record a video backup | |

## Things that will bite you

- **Spawn placement is your calibration.** Put tape marks for each drone at its `spawn` offset behind the robot, nose aligned with the robot. Every centimetre or degree of error here becomes position error in flight.
- **Flow deck drift.** Expect a few cm per metre flown. Keep runs short. Re-anchor by landing on the tape marks and taking off again.
- **Flow deck needs texture and light.** Shiny or uniform floors fail. Lay down a textured mat or paper with patterns.
- **Flying over objects.** The ToF thinks the floor came up, so the drone climbs by the object's height. That is safe, and it's also what makes the height estimate possible.
- **Robot speed.** Cap it around 0.15–0.2 m/s. Drone setpoints are slew-limited to `vxy_max`.
- **Time sync.** Run `chrony` between laptop and robot, or TF lookups go stale.
- **Brushless arming.** The bridge sends the arming request. If the supervisor locks after a crash, power-cycle the drone.
- **Camera frame.** `object_mapper` needs the optical frame (z forward). If your image `frame_id` is the body frame, set `camera_frame`.
- **Always land before Ctrl-C.** Killing the bridge cuts the setpoint stream and the drones drop.
- Safety: prop guards, a software geofence (`fence_min/max`, `z_max`), and one person on the e-stop.

## Pitch line

"The drones are blind. The robot is short. Together they're a 3D mapping team: the robot shares its map and its eyes, the drones share their altitude and range."
