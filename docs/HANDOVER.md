# Robotics Hackathon: Context Handover

Last updated 2026-10-10, 23:50. Paste this into a new chat to continue.

## 1. Goal

Win the hackathon with a **TurtleBot3 + 2× Crazyflie 2.1 Brushless** "robot–drone team".

- The **TurtleBot** is the mothership. It builds the map (lidar SLAM), sees objects (Pi camera + YOLO), and gives every object a position in the shared **world frame**: the SLAM `map` frame, in metres, with its origin where SLAM started.
- The **drones** only have a Flow deck (optical flow + downward ToF) and no global position. They borrow the robot's map. At takeoff, the robot's map pose plus a known spawn offset fixes `map → cfN/odom`. The robot can then send them to map coordinates, for example "fly over the bottle the robot saw".
- **Demo story:** the robot drives a straight line and swerves around obstacles. YOLO detects objects and puts them on the map with x/y coordinates. A drone flies to a detected object's coordinates. Camera and YOLO video are recorded.
- **Pitch line:** "The drones are blind, the robot is short: together they're a 3D mapping team."

## 2. Hardware, network, credentials

| Item | Value |
|---|---|
| Robot | TurtleBot3 **Waffle Pi** (configured as `TURTLEBOT3_MODEL=waffle` on the robot), Raspberry Pi, Ubuntu 22.04 aarch64, ROS 2 **Humble**, LDS-01 lidar, Pi camera (libcamera/unicam) |
| Robot login | `ssh turtle@172.20.10.3`, password: *(ask the team)*; the prompt shows `[TURTLEBOT] turtle@ubuntu` |
| Network | Phone hotspot **`bD`**, password: *(ask the team)* (WPA2, 2.4 GHz). Subnet `172.20.10.x` (iPhone). Laptop Wi-Fi .2/.4; robot **.3** |
| Middleware | **Fast DDS** (team decision), `ROS_DOMAIN_ID=30` on all machines. Zenoh RMW was removed from the robot by a teammate. |
| Asad's laptop | **Windows 10 (build 19045)** + WSL2 **Ubuntu-22.04** (default distro), ROS 2 Humble. WSL is behind **NAT** (Win10 has no mirrored networking), so it can't join Fast DDS directly and uses a **bridge**. |
| Laptop prompt | `asadrizvi64@DESKTOP-GBMJ4NA`. PowerShell `wsl` opens 22.04. The 24.04 distro also exists but has **no ROS**. |
| Teammates | One other laptop (Fast DDS, joins directly) also drives the robot. **Only one bringup and one teleop at a time.** |

## 3. Current architecture

```
ROBOT (Pi, Fast DDS domain 30) - tmux session "robot"
  bringup : ros2 launch turtlebot3_bringup robot.launch.py   (/scan /odom /tf; ONLY ONE copy!)
  slam    : ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$HOME/slam_tb3.yaml
  camera  : ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480 -p format:=RGB888
            -> /camera/image_raw/compressed (~30 Hz), /camera/camera_info
  bridge  : zenoh_bridge_dds 0.5.0  -d 30 -l tcp/0.0.0.0:7447 -a 'scan|tf|map|camera_info|compressed|cmd_vel|odom|joint_states|robot_description|goal_pose|plan|initialpose'
  (foreground SSH) python3 ~/straight_avoid.py ...   (line-following obstacle avoidance)

         one TCP link (laptop -> robot:7447)

LAPTOP WSL (Fast DDS locally) - tmux session "lap" (started by laptop.sh)
  bridge  : zenoh_bridge_dds -d 30 -e tcp/172.20.10.3:7447 --no-multicast-scouting
  mapper  : ros2 run swarm_hack object_mapper   (YOLOv8n CPU; image + lidar -> /objects/json in map frame)
  swarm   : ros2 launch swarm_hack swarm.launch.py sim:=true mapper:=false   (FAKE drones + coordinator)
  rec_*   : ros2 run swarm_hack video_recorder   (optional, MP4 -> C:\Users\HP\Videos)
  separate windows: RViz (light.rviz), commands
```

## 4. Code (all written in this chat)

Latest zips are in **`C:\Users\HP\Downloads\`**, unpacked to `~/ws/src/` in WSL and built with `colcon build --symlink-install`. `~/.bashrc` sources `~/ws/install/setup.bash`.

### `swarm_hack` (ament_python)

- `crazyflie_bridge.py`: all drones on one Crazyradio, run through cflib.
  - Flow-deck pose in `cfN/odom`. The anchor `map→cfN/odom` comes from the robot TF plus `spawn` offsets.
  - Takes map-frame goals on `/cfN/goal` and rate-limits the setpoints.
  - Safety: geofence, hold or land on goal timeout, low-battery land, stale-estimate land, e-stop, brushless arming.
  - `sim:=true` gives fake drones.
  - Services: `/swarm/takeoff_all`, `/swarm/land_all`, `/cfN/takeoff|land`. Topic `/swarm/estop` (Empty).
- `swarm_coordinator.py`: formation slots in the robot frame, plus commands on `/swarm/mode` (String):
  - `follow | auto | hold | inspect <id> | goto [cfN] x y [z] | fly <waypoint> | home`
  - RViz **Publish Point** (`/clicked_point`) sends the nearest drone there.
  - Events on `/swarm/events`. Also reports an EXPERIMENTAL object height from the drone ToF.
- `object_mapper.py`: YOLO (ultralytics) on `/camera/image_raw/compressed`.
  - Depth: depth image if any, otherwise lidar points projected into the box, otherwise a ground-plane ray.
  - Intrinsics from `hfov_deg=62.2` while the camera is uncalibrated (camera_info K is zero).
  - Uses `camera_frame: camera_rgb_optical_frame`.
  - Outputs `/objects/json` (`id, cls, x, y, z, z_top, w, h, conf...` in **map** frame), `/objects/markers`, `/objects/debug_image`.
- `video_recorder.py`: any image topic to MP4 at real-time speed. Ctrl+C finalises the file, plus an `_h264.mp4` copy made with ffmpeg.
- `geom.py`: SE(2) and quaternion helpers.
- `config/swarm.yaml`: all parameters, including drone URIs, spawn offsets, slots, fence and mapper topics.
- `scripts/robot_extras.sh`: on the robot, starts the camera (camera_ros) and bridge windows in tmux `robot`.
- `scripts/laptop.sh [ROBOT_IP]`: on the laptop, tmux `lap` with bridge, mapper, swarm (sim) and a cmd window.
- `scripts/straight_avoid.py` (**v3.1**): runs ON THE ROBOT. Holds a straight line from its start pose, picks the free heading closest to "back onto the line" with a footprint corridor check over the lidar, keeps to its chosen side while blocked, then rejoins the line.
  - Params: `distance` (0 = forever), `speed`, `legs` (back and forth on the same line, 0 = forever), `margin`, `clear_dist`, `stop_dist`.
  - Uses its own signal handling, so **Ctrl+C, kill and SSH drop all send zero velocity**. Logs which topic it's waiting for.
  - Simulated on 7 scenarios: rejoins the line within 5 cm, keeps at least 8 cm body clearance.

### `tb_nav` (ament_python)

- `nav_manager.py`: Nav2 action client with commands on `/nav/command`: `goto x y [yaw]`, `goto <name>`, `save <name>`, `patrol N`, `object <id>`, `init`, `relocalize`, `cancel`. Waypoints are stored in `~/tb_waypoints.yaml`.
- `obstacle_guard.py`: lidar safety gate (`/cmd_vel_raw → /cmd_vel`) with modes `guard | wander | off`.
- `launch/tb_nav.launch.py`: `mode:=slam|localization`, `run_slam:=false` (SLAM on the robot). Defaults to **Nav2's own Humble params**, because the TB3 params use Jazzy plugin names that fail on Humble.
- `config/slam_tb3.yaml`: tuned for LDS-01 (`max_laser_range 3.5`, `map_update_interval 1.0`, `minimum_travel 0.1 m / 0.15 rad`). A copy is on the robot as `~/slam_tb3.yaml`.
- `config/light.rviz`: Map, LaserScan, Robot axes, Objects, Detections image, DroneTargets, TF, Plan; tools for Publish Point and 2D Goal.
- `scripts/start_robot.sh` and `start_laptop.sh`: **OBSOLETE** (from the zenoh era). Use `robot_extras.sh` and `laptop.sh` instead.

### Project docs

`claude/runbook_fastdds_bridge.md`, `claude/tb_nav_README.md`, `claude/swarm_hack_README.md`, and this handover.

## 5. Start-up runbook (current, working setup)

**Robot** (`ssh turtle@172.20.10.3`):

```bash
ros2 node list | grep -E "turtlebot3_node|hlds" | sort | uniq -c     # must be exactly 1 each
# if missing: tmux new-window -t robot -n bringup "bash -ic 'ros2 launch turtlebot3_bringup robot.launch.py'"
# SLAM, if not running: tmux new-window -t robot -n slam "bash -ic 'ros2 launch slam_toolbox online_async_launch.py slam_params_file:=\$HOME/slam_tb3.yaml'"
bash ~/robot_extras.sh            # camera + bridge
timeout 5 ros2 topic hz /scan      # ~5 Hz
```

**Laptop** (WSL 22.04):

```bash
bash ~/ws/src/swarm_hack/scripts/laptop.sh 172.20.10.3
ros2 daemon stop                   # once, if ros2 CLI sees nothing
ros2 topic hz /scan                # ~4-5 Hz through the bridge
rviz2 -d ~/ws/install/tb_nav/share/tb_nav/config/light.rviz   # Map display: Durability = Volatile
ros2 topic echo /objects/json --once
```

**Drive** (on the robot, in a foreground SSH session, no teleop running anywhere):

```bash
python3 ~/straight_avoid.py --ros-args -p distance:=2.5 -p speed:=0.10 -p legs:=2
```

**Fake drones:**

```bash
ros2 service call /swarm/takeoff_all std_srvs/srv/Trigger
ros2 topic pub --once /swarm/mode std_msgs/String "data: inspect 1"     # or: goto 1.0 0.5 / auto / home
ros2 topic echo /swarm/events
```

**Record video:**

```bash
tmux new-window -t lap -n rec_cam "bash -ic 'ros2 run swarm_hack video_recorder'"
tmux new-window -t lap -n rec_yolo "bash -ic 'ros2 run swarm_hack video_recorder --ros-args -p topic:=/objects/debug_image -p name:=yolo'"
# stop:
tmux send-keys -t lap:rec_cam C-c; tmux send-keys -t lap:rec_yolo C-c     # files land in C:\Users\HP\Videos
```

**Emergency stop:** lift the robot or use its power switch, or run `ros2 topic pub --times 5 /cmd_vel geometry_msgs/msg/Twist "{}"` from any machine.

## 6. Status

**Verified working**

- WSL ROS Humble.
- The robot ↔ laptop bridge: `/scan` arrives on the laptop at about 4.2 Hz.
- SLAM on the robot.
- Pi camera through camera_ros at 30 Hz.
- YOLO detection with a 3D map position: a bottle was detected at map (0.19, −0.39).
- RViz with the detection image and object markers.
- `straight_avoid` drove a line and avoided objects on the real robot (v1).
- `/odom` at 20 Hz.

**Not yet verified / pending**

1. **straight_avoid v3.1 on the robot.** Last run was still v1 (the stop bug). Copy with `scp /mnt/c/Users/HP/Downloads/straight_avoid.py turtle@172.20.10.3:~/`, then check with `grep -c SignalHandlerOptions ~/straight_avoid.py` (it should print at least 1).
2. **`/scan` missing on the robot in the last run** (`WAIT` after `line start`). Probably duplicate or killed bringups fighting over `/dev/ttyUSB0`. Fix: kill all bringups and start exactly one (§7, item 9).
3. **Fake-drone `inspect`** was set up but not run end to end.
4. **Real Crazyflie flight** not started. Needs: Crazyradio into WSL with `usbipd bind/attach --wsl` (Admin PowerShell), `pip3 install cflib`, udev rules, radio addresses in `swarm.yaml`, tape marks for the spawn offsets, a textured floor for the flow deck, and one drone first. Alternative: plug the radio into a native-Ubuntu teammate laptop.
5. **The map isn't displayed** unless RViz Map Durability is set to Volatile, because the bridge doesn't keep transient-local delivery.
6. **Nav2 on the laptop** isn't running in the Fast DDS setup. If needed: `ros2 launch tb_nav tb_nav.launch.py mode:=slam run_slam:=false`, which also needs `/goal_pose` and `/cmd_vel` through the bridge (already in the allow list).
7. **Camera calibration** (optional, improves object x/y): checkerboard with `camera_calibration`, or set `hfov_deg` for the real lens (v1 53.5°, v2 62.2°).
8. **Demo video and slides.**

## 7. Blockers hit, and their solutions

| # | Problem | Cause | Solution |
|---|---|---|---|
| 1 | Humble wouldn't install | WSL default distro was Ubuntu 24.04 | Installed Ubuntu-22.04, `wsl --set-default Ubuntu-22.04` |
| 2 | Robot not reachable from WSL | Win10 WSL2 = NAT; mirrored mode needs Win11 | Bridge with an outgoing TCP link (first rmw_zenoh, now `zenoh_bridge_dds` from ROS apt) |
| 3 | `zenoh-bridge-ros2dds` (Eclipse apt / musl zip) wouldn't run | Built for glibc 2.38 / missing musl libgcc | Use `ros-humble-zenoh-bridge-dds` 0.5.0 from ROS apt. Its flags: `-d -l -e -a`, **no** max-frequency |
| 4 | Pasted command blocks ran only partly | `add-apt-repository` and `sudo` swallow stdin | Run sudo/apt commands one line at a time |
| 5 | "exceeding delta 500ms" zenoh errors, SLAM stalling | **Windows clock off by about 1.6–2 s**; WSL re-syncs to it | `chrony` in WSL (`makestep 0.2 -1`, `minpoll 3 maxpoll 4`); Windows Settings → Date & time → Sync now |
| 6 | Nav2 planner FATAL | TB3 Nav2 params use Jazzy-style `::` plugin names | Use `nav2_bringup/params/nav2_params.yaml` (now the default) |
| 7 | SLAM on the laptop dropped scans and smeared the map | Wi-Fi latency + clock + driving too fast (0.26 m/s, spinning at 1.8 rad/s) | SLAM runs **on the robot** with tuned params; drive at 0.05–0.1 m/s, turns at most 0.5 |
| 8 | Commands ran on the wrong machine | Confusing prompts | Robot = `[TURTLEBOT] turtle@ubuntu`; laptop = `asadrizvi64@DESKTOP-GBMJ4NA`. RViz, YOLO and the recorder run on the laptop; bringup, SLAM, camera and straight_avoid on the robot |
| 9 | Duplicate `/turtlebot3_node`, lidar missing | Two bringups (ours + a teammate's) on the same serial ports | Exactly one bringup: `tmux kill-window -t robot:bringup; pkill -f hlds_laser; pkill -f turtlebot3_ros`, then start one |
| 10 | Programs froze | Windows console "Select"/"Mark" mode (clicking inside a terminal) pauses the process | Press Esc; click title bars only |
| 11 | Everything died when the hotspot dropped | Processes were tied to SSH sessions | Run robot processes in **tmux**. iPhone: keep the Personal Hotspot screen open / Maximize Compatibility |
| 12 | `librmw_zenoh_cpp.so` not found on the robot | Teammate removed zenoh, but `.bashrc` still exported it | `sed -i '/rmw_zenoh/d' ~/.bashrc` (robot and laptop) |
| 13 | `ros2 topic echo` sees nothing | Stale ros2 daemon from the zenoh era | `ros2 daemon stop` |
| 14 | `v4l2_camera`: "Failed stream start: Invalid argument" | Pi camera on 22.04 uses libcamera (unicam) | `sudo apt install ros-humble-camera-ros`; `ros2 run camera_ros camera_node ...` |
| 15 | Robot kept driving after Ctrl+C | rclpy shut down before the final zero Twist; the TB3 keeps its last cmd_vel | straight_avoid v3: own SIGINT/SIGTERM/SIGHUP handler, sends 10 zero Twists before shutdown |
| 16 | `straight_avoid` stuck in `WAIT` | No `/odom` or `/scan` (bringup not running or duplicated) | Check `ros2 topic hz /odom /scan`; one bringup only |
| 17 | Teleop fighting autonomy | TB3 teleop republishes cmd_vel continuously | Close all teleops (`ros2 topic info /cmd_vel -v`) |

## 8. Next steps (priority order)

1. Clean robot state: one bringup, SLAM, camera and bridge (`robot_extras.sh`). Confirm `/scan` and `/odom` on the robot.
2. Copy straight_avoid v3.1 and test Ctrl+C stopping. Run the `legs:=2` demo while recording camera and YOLO video.
3. Fake-drone demo: `takeoff_all`, then `inspect <object id>`, and watch the TF frame `cf1` move to the object in RViz.
4. Real drone: Crazyradio via usbipd (or a teammate's native laptop), one drone, tape-mark calibration, then `goto cf1 <robot x> <robot y> 0.8` to hover over the robot and check the error (it should be under 10 cm).
5. Combined demo: robot drives and avoids, YOLO maps an object, a drone flies to its coordinates, all recorded.
6. Slides and pitch: "robot = map + eyes, drones = altitude".
