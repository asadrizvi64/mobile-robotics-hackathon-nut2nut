# Runbook: robot (Fast DDS) + WSL laptop via zenoh-bridge-dds

Team decision (2026-10-10): the robot and the teammates use plain ROS 2 Fast DDS (domain 30) on the `bD` phone hotspot. Asad's laptop runs Windows 10 + WSL2 (NAT, no mirrored networking), so it joins through a DDS bridge over a single TCP link.

## Robot (Raspberry Pi, 172.20.10.3, user `turtle`)

- Only ONE bringup may run. Check `ros2 node list` for a duplicate `/turtlebot3_node`.
- tmux session `robot` has four windows: bringup, slam (slam_toolbox, `~/slam_tb3.yaml`), camera and bridge (listening on tcp/0.0.0.0:7447).
- `bash ~/robot_extras.sh` adds the camera and the bridge without touching bringup or SLAM.

## Laptop (WSL Ubuntu 22.04)

- `bash ~/ws/src/swarm_hack/scripts/laptop.sh 172.20.10.3` creates tmux session `lap` with windows for the bridge, the mapper (YOLO), the swarm (simulated drones) and a command shell.
- RViz: `rviz2 -d ~/ws/install/tb_nav/share/tb_nav/config/light.rviz`
- Clock: keep the laptop clock synced. The Windows clock drifts, and WSL copies it.

## Frames

- World = SLAM `map` frame (origin = where SLAM started, in metres).
- Objects: `/objects/json` (map coordinates).
- Drones: `/swarm/mode` commands (`goto`, `fly <waypoint>`, `inspect <id>`, `auto`, `home`).
