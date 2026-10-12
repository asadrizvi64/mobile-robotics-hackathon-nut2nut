# Handover: lidar, map and exploration work (11 Oct 2026, ~13:25)

Pasted from a parallel chat. It continues `FINAL_HANDOFF.md`, `MORNING_RUNBOOK.md` and the lidar+YOLO fusion done earlier today.

**Goal:** a TurtleBot3 Waffle Pi builds a SLAM map and detects objects (YOLO on the laptop). Crazyflie drones fly to detected objects. The robot must reliably map a square area and find objects in it.

## Machines and access

- **Robot:** `turtle@172.20.10.3`, password: *(ask the team)*. Prompt `[TURTLEBOT] turtle@ubuntu`. Everything runs in tmux session `robot`, with windows bringup, slam, camera, bridge, explore and sweep.
- **Laptop:** Windows 10 + WSL2 Ubuntu-22.04, prompt `asadrizvi64@DESKTOP-GBMJ4NA`. RViz, the YOLO mapper and the zenoh bridge run here. Windows downloads go to `C:\Users\HP\Downloads\`.
- **Network:** iPhone hotspot `bD` (password: *(ask the team)*). It drops often, but robot processes survive in tmux. ROS_DOMAIN_ID=30, Fast DDS, zenoh bridge on the laptop side.

## Current state

- **Robot stack:** bringup, SLAM (`slam_toolbox`, params `~/slam_tb3.yaml`) and the lidar driver are running, one instance each (verified). `robot_extras.sh` starts the camera and the bridge.
- **Lidar: HEALTHY** (checked with `scan_check.py` at 13:30). 5.0 Hz, 0 frozen beams, 0 blocked beams. Only about 55 of 360 beams return, because the room is open: most directions have nothing within 3.5 m. The rear readings are beyond 3.5 m, not 0. The speckled maps come from SLAM matching on these sparse scans. Fix: put objects at least 20 cm tall around the square.
- **SLAM map:** noisy, with black speckle and grey unknown patches. The best saved map is `~/box_map_1307.pgm`/`.yaml`: 152×147 cells at 5 cm, about 7.6 × 7.4 m, with the origin at the robot's odometry start.
- **`~/square_map.py`** (on laptop and robot): removes lone speckle cells and crops the map to a metric square. The last crop showed mostly free floor, with leftover specks and unknown patches near the top-right diagonal and the bottom corners.

## Code from that session (all in the robot's home folder)

- **`scan_explore.py`:** frontier explorer.
  - Plans a BFS path to the nearest unexplored boundary.
  - Inflates obstacles by 0.2 m and uses live lidar hits (≥0.25 m).
  - Parameters: `max_radius` (default 5 m) and `region_box` (`xmin,xmax,ymin,ymax`, in map metres).
  - Stops when no frontier is reachable. When blocked, it drops the frontier, backs off and replans.
  - Known issues: it ends early with "no path" when the robot is enclosed, and the map origin is the odometry origin of the last bringup, not where SLAM started.
- **`grid_sweep.py`:** NOT YET RUN.
  - Drives a 5×5 grid over a square (default 3 m, centred on (0,0)).
  - Dwells at each point, averages the lidar scans, and detects objects from lidar clusters 0.08–0.6 m in size.
  - Output: `~/sweep/points.json` and `~/sweep/detections.json`. Log: `~/sweep.log`.
- **`square_map.py`:** crops a PGM/YAML map to a metric square and reports the occupied, free and unknown cells inside it.

## Decisions

- Use SLAM only; no Nav2, which fails on the Humble params and isn't needed for coverage.
- Keep objects out of the square, so the robot can move freely inside it.
- Only ever one bringup and one SLAM. Duplicate SLAM processes corrupted the pose earlier.
- Stop a window with `tmux send-keys -t robot:<window> C-c`, then `ros2 topic pub --times 5 /cmd_vel geometry_msgs/msg/Twist "{}"`.
- Save the map before stopping SLAM; stopping it discards the map.

## Open items, in priority order

1. ~~Lidar check~~: done, healthy. Next: surround the square with boxes, bags or chairs (at least 20 cm tall, 0.5–1 m outside its edges). Then `scan_check.py` from the centre should show valid above 150.
2. Run `grid_sweep` on a clear 3 m square and send the end of `~/sweep.log`.
3. Confirm `/objects/json` reaches the laptop (camera detections) during the sweep.
4. Fix the explorer's early "no path" stop, if a full-square map is still needed.
5. Fix the map frame origin: restart bringup and SLAM together, with the robot on mark A.
6. Drone integration (`cf_fleet`, `mission`) is untested on real hardware. See `FINAL_HANDOFF.md`.

## Object mapper state (from the main chat, same day)

- `object_mapper` fuses YOLO with the lidar.
  - The camera gives class and bearing; lidar points inside the box give range, gated by the box's floor-contact depth.
  - Confirmed bottles are then tracked 360° by lidar.
- `/objects/json` has a `track` field: `camera`, `lidar`, `camera+lidar` or `lost`.
- Only one bottle is published: the incumbent target stays unless it has been unseen for 2 s.
- A clear plastic bottle is partly invisible to the lidar. Wrap it in tape 10–25 cm above the floor.
- Check from the CLI with `ros2 topic echo --full-length /objects/json std_msgs/msg/String`.
