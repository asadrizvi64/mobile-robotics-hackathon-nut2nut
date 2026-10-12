# tb_nav — SLAM, localization and navigation for the TurtleBot (ROS 2 Humble)

One launch file, two modes, one control node.

- `mode:=slam` runs slam_toolbox (mapping and localization at once) plus Nav2. The robot can navigate while it is still building the map.
- `mode:=localization` runs map_server and AMCL on a saved map plus Nav2. Use this for the demo: the map is fixed and repeatable.
- `nav_manager` sends goals to Nav2 and takes plain-text commands on `/nav/command`.

## Build (laptop, inside the Ubuntu 22.04 WSL)

```bash
mkdir -p ~/ws/src && cp -r tb_nav ~/ws/src/
cd ~/ws && colcon build --symlink-install && source install/setup.bash
echo "source ~/ws/install/setup.bash" >> ~/.bashrc
```

## 1. Make the map

Robot (over SSH):

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

Laptop, one command per terminal:

```bash
ros2 launch tb_nav tb_nav.launch.py mode:=slam
ros2 run turtlebot3_teleop teleop_keyboard
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

Drive slowly around the arena and close at least one loop. While you drive, save the spots you'll want later:

```bash
ros2 topic pub --once /nav/command std_msgs/String "data: save start"
ros2 topic pub --once /nav/command std_msgs/String "data: save table"
ros2 topic pub --once /nav/command std_msgs/String "data: save door"
```

Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/arena_map
```

This writes `~/arena_map.yaml` and `~/arena_map.pgm`. Waypoints are saved in `~/tb_waypoints.yaml`.

## 2. Localize and navigate on the saved map

Stop the slam launch and run:

```bash
ros2 launch tb_nav tb_nav.launch.py mode:=localization map:=$HOME/arena_map.yaml
```

Tell AMCL where the robot is. Either click **2D Pose Estimate** in RViz, or type the pose:

```bash
ros2 topic pub --once /nav/command std_msgs/String "data: init 0 0 0"   # x y yaw_deg
```

If you don't know where it is, try `relocalize`. It scatters particles and spins the robot. This works in distinctive rooms and is unreliable in symmetric ones.

Wait until `/nav/status` shows `"localized": true`, then drive:

```bash
ros2 topic pub --once /nav/command std_msgs/String "data: goto table"
ros2 topic pub --once /nav/command std_msgs/String "data: goto 1.5 -0.5 90"
ros2 topic pub --once /nav/command std_msgs/String "data: patrol 2"     # 0 = forever
ros2 topic pub --once /nav/command std_msgs/String "data: object 3"     # from object_mapper
ros2 topic pub --once /nav/command std_msgs/String "data: cancel"
ros2 topic echo /nav/status
ros2 topic echo /nav/events
```

`object <id>` reads `/objects/json` from `swarm_hack/object_mapper`. It stops 0.5 m in front of the object, facing it.

RViz's **Nav2 Goal** button still works alongside the commands.

## How the frames fit

```
map --(slam_toolbox or AMCL)--> odom --(wheel odometry)--> base_footprint --> base_scan, camera
```

The swarm package uses the same `map` frame, so drones, objects and robot goals all share one coordinate system.

## Troubleshooting

- **"Nav2 navigate_to_pose server not ready".** Nav2 is still activating, or a lifecycle node failed. Look for red errors in the launch terminal.
- **AMCL error about `robot_model_type`.** The TB3 params file uses an old name. Copy the file, set `robot_model_type: "nav2_amcl::DifferentialMotionModel"`, and pass `params_file:=/path/to/your.yaml`.
- **"not localized yet".** AMCL's covariance is above the threshold. Give an initial pose and drive a little. You can loosen `localized_xy_std` in `config/nav_manager.yaml`.
- **The robot plans fine but doesn't move.** Check that `/cmd_vel` reaches the robot (`ros2 topic echo /cmd_vel` on the robot side). If not, it's the network or DDS config.
- **The robot spins or oscillates at the goal.** The map and laser disagree. Re-localize, or rebuild the map driving slower.
- **Goals near walls fail.** The inflation radius makes them unreachable. Pick goals at least about 0.3 m from walls.
