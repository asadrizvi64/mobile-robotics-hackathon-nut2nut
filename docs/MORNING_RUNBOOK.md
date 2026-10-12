# Morning runbook (Asad): YOLO + video + robot drive, bottle = drone target

**Goal:** the robot drives its line and avoids **every** object. YOLO watches the camera. Only the **bottle** is reported as a target, with map x/y on `/objects/json`, which is what calls the drone. Camera and YOLO videos are recorded.

Work one step at a time, and check the ✅ result before moving on.

Prompts:

- **Laptop:** `asadrizvi64@DESKTOP-GBMJ4NA`
- **Robot:** `[TURTLEBOT] turtle@ubuntu`

---

## 0. Arena (2 minutes, before anything else)

- **Objects must be at least 20 cm tall.** The lidar scans one flat plane about 17 cm above the floor. Anything lower is invisible to it, and the robot will hit it. For the bottle, use a 1.5 L bottle (about 30 cm), or stand a small one on a box.
- **Place the bottle 0.5–1 m beside the robot's line, 1–2 m ahead of the start.** The camera only sees about ±30° ahead, up to about 2.5 m.
- Put the other obstacles **on** the line, so the robot has to swerve around them.
- **Wrap the bottle in paper or masking tape 10–25 cm above the floor** (or use an opaque bottle). The lidar's laser passes through clear plastic, so an unwrapped bottle is half-invisible to the lidar, to the tracking and to collision avoidance.
- **Keep the bottle at least 30 cm from walls and other objects.** After YOLO has confirmed it, the lidar keeps tracking it all the way round (360°), but it only locks onto a small, *isolated* cluster.
- Put the robot on **mark A**, facing the agreed direction. SLAM's (0, 0) is set where SLAM starts.

## 1. Clocks (the bridge DROPS data if laptop and robot differ by more than 0.1 s)

The real culprit is the **Windows clock**, which can be about 0.2 s off. WSL copies it, so fix Windows first.

**Administrator PowerShell:**

```powershell
w32tm /stripchart /computer:pool.ntp.org /samples:3 /dataonly
```

If the values are above ±0.05 s, add the shown offset, for example `Set-Date -Adjust 00:00:00.200`, then check again.

**WSL:** keep it re-synced every 10 s:

```bash
sudo tmux new-session -d -s clock "while true; do ntpdate -b -s pool.ntp.org; sleep 10; done"; sleep 12; ntpdate -q pool.ntp.org | tail -1
```

✅ The offset is below 0.05. (Chrony in WSL doesn't work here: `chronyc makestep` gives `500 Failure`.)

**Robot:**

```bash
ssh -t turtle@172.20.10.3 "sudo ntpdate -b pool.ntp.org"
```

## 2. Laptop: copy the robot files

Skip this if you already did it last night.

```bash
scp /mnt/c/Users/HP/Downloads/robot_extras.sh /mnt/c/Users/HP/Downloads/straight_avoid.py turtle@172.20.10.3:~/ && ssh turtle@172.20.10.3 "grep -c objects ~/robot_extras.sh; grep -c SignalHandlerOptions ~/straight_avoid.py"
```

✅ Both numbers should be **1 or more**.

## 3. Robot: check what's still running

```bash
ssh turtle@172.20.10.3
```

```bash
tmux ls; ros2 node list | grep -E "turtlebot3_node|hlds|slam|camera" | sort | uniq -c
```

- **Each of `/turtlebot3_node`, `/hlds_laser_publisher`, `/slam_toolbox` and `/camera` appears exactly once:** everything survived the night. Go to step 4.
- **Something is missing, or appears twice:** start clean.

  Stop everything:

  ```bash
  tmux kill-server; pkill -f hlds_laser; pkill -f turtlebot3_ros; pkill -f slam_toolbox; pkill -f camera_node; pkill -f zenoh_bridge; sleep 3
  ```

  Bringup:

  ```bash
  tmux new-session -d -s robot -n bringup "bash -ic 'ros2 launch turtlebot3_bringup robot.launch.py'"; sleep 15
  ```

  SLAM, with the robot on mark A:

  ```bash
  tmux new-window -t robot -n slam "bash -ic 'ros2 launch slam_toolbox online_async_launch.py slam_params_file:=\$HOME/slam_tb3.yaml'"; sleep 5
  ```

## 4. Robot: camera and bridge

The bridge now also passes `objects`.

```bash
bash ~/robot_extras.sh; sleep 10; timeout 4 ros2 topic hz /scan; timeout 4 ros2 topic hz /odom; timeout 4 ros2 topic hz /camera/image_raw/compressed
```

✅ `/scan` about 5 Hz, `/odom` about 20 Hz, camera about 30 Hz. Then `exit`.

## 5. Laptop: bottle-only config, then start

Make YOLO report **only bottles** (the video still shows boxes on everything):

```bash
sed -i "s/^    classes: \[''\]/    classes: [bottle]/" ~/ws/src/swarm_hack/config/swarm.yaml && grep -n "^    classes" ~/ws/src/swarm_hack/config/swarm.yaml && cd ~/ws && colcon build --symlink-install --packages-select swarm_hack && source install/setup.bash
```

✅ It shows `classes: [bottle]`, then `1 package finished`.

```bash
bash ~/ws/src/swarm_hack/scripts/laptop.sh 172.20.10.3; sleep 3; tmux kill-window -t lap:swarm 2>/dev/null; ros2 daemon stop
```

This starts the bridge and the YOLO mapper. The old fake-drone window is closed, because it would clash with the real drones.

Wait 20 seconds, then:

```bash
timeout 5 ros2 topic hz /scan; tmux capture-pane -pt lap:mapper -S -30 | grep -v "^$" | tail -5
```

✅ `/scan` about 4–5 Hz; the mapper log shows `YOLO model ... loaded` and `camera intrinsics ...`.

## 6. Laptop: RViz (new tab: `wsl`)

```bash
rviz2 -d ~/ws/install/tb_nav/share/tb_nav/config/light.rviz
```

Set **Map → Topic → Durability Policy = Volatile**. You should see the map, the laser, the Detections image, and a green marker on the bottle.

## 7. Laptop: start recording (ONCE; check with `tmux list-windows -t lap`)

```bash
tmux new-window -t lap -n rec_cam; tmux send-keys -t lap:rec_cam "ros2 run swarm_hack video_recorder --ros-args -p out_dir:=/mnt/c/Users/HP/Videos" C-m; tmux new-window -t lap -n rec_yolo; tmux send-keys -t lap:rec_yolo "ros2 run swarm_hack video_recorder --ros-args -p topic:=/objects/debug_image -p name:=yolo -p out_dir:=/mnt/c/Users/HP/Videos" C-m
```

`out_dir` is needed because this PC has a second Windows user, and the default can pick the wrong Videos folder.

## 8. Robot: drive

**No teleop running anywhere.** Check with `ros2 topic info /cmd_vel -v | grep "Publisher count"`; it should be 0.

```bash
ssh turtle@172.20.10.3
```

```bash
python3 ~/straight_avoid.py --ros-args -p distance:=2.5 -p speed:=0.10 -p legs:=2
```

**Ctrl+C stops the robot at once** and prints `stopped - zero velocity sent`.

## 9. Laptop: the bottle's coordinates

Since 14:30 there are two topics:

- `/objects/detections`: everything the YOLO+lidar mapper knows.
- `/objects/json`: only the bottles the robot has driven up to. This is the one the drone reads, and `target_mission` publishes it (step 11).

To see what the mapper knows:

```bash
ros2 topic echo --full-length /objects/detections std_msgs/msg/String --once
```

✅ `{"objects": [{"id": ..., "cls": "bottle", "x": ..., "y": ..., "track": ..., ...}]}`. **x and y are map metres.**

`track` says where the position comes from:

- `camera+lidar` or `camera`: in camera view.
- `lidar`: out of view (too close, beside or behind the robot) but still updated by the lidar.
- `lost`: no update in the last second; the last known position is kept.

In RViz the marker is green for camera, blue for lidar only and grey for lost.

**Drone PC:** run the same command there. It must show the same bottle, which proves it crosses the robot bridge. Their `mission` should use `classes:=bottle`.

## 10. Stop recording (laptop)

```bash
pkill -INT -f "lib/swarm_hack/video_recorder"; sleep 15; ls -lh /mnt/c/Users/HP/Videos/*.mp4
```

Use the `_h264.mp4` files. They play in any player.

## 11. Robot: target mission (find a bottle, drive to it, hand it to the drone, next)

Get the drone team's OK first. Every bottle the robot reaches goes straight to the drone. For a dry run, the drone node is stopped.

```bash
python3 ~/target_mission.py
```

What it does:

1. It looks around on mark A: it turns 45°, stops 1.5 s so YOLO gets a sharp image, and repeats eight times for 360°.
2. When the camera confirms a bottle, it drives toward it, avoiding obstacles with the lidar and the real Waffle footprint. For the last part it **docks on the bottle with its own lidar** and stops with the front bumper about 6 cm from it (`dock_gap`). Hard rule: it stops for **anything** in its path at 6 cm, measured by the lidar. If the lidar can't see the bottle at all, it stops 20 cm short of the camera estimate.
3. Standing still, it **measures the bottle with its own lidar** for 1.5 s, then **publishes that exact position on `/objects/json` for 10 s** (5 Hz). Then it drops the bottle, reverses 25 cm along the way it came in, and goes to the next one.
   - Without `max_targets` it visits every bottle within 1.5 m of mark A.
   - The camera position goes to the drone the moment the bottle is seen (`announce_early`, on by default). The precise lidar position replaces it once docked. `-p announce_early:=false` publishes only the precise one.
4. It moves on to the next bottle.
5. When none are left, it searches the 4 quarters of the square (spinning in each) and then the middle. It returns to mark A when `max_targets` bottles are done or everything has been searched. It never drives more than **1.5 m from mark A** (`max_radius`), and bottles further out are ignored.

The search square is ±1.5 m around mark A by default. If mark A is not in the middle, pass the square in map metres, for example `-p region:="[0.0,3.0,-1.5,1.5]"`.

Watch it from the laptop:

```bash
ros2 topic echo /mission/status
```

**Ctrl+C stops the robot at once.**

---

## Emergency stops

- **Robot:** Ctrl+C in the straight_avoid tab, lift the robot, or flip its power switch. From any machine: `ros2 topic pub --times 5 /cmd_vel geometry_msgs/msg/Twist "{}"`
- **Drones:** `ros2 topic pub --once /fleet/land_all std_msgs/msg/Empty`

## If something fails

| Symptom | Fix |
|---|---|
| straight_avoid says `WAITING for /scan` | Two bringups, or none. Step 3, start clean. |
| Robot drives into an object | The object is shorter than the lidar plane (about 17 cm). Use taller objects. |
| `ros2 topic echo` sees nothing on the laptop | `ros2 daemon stop` |
| Red `exceeding delta 500ms` lines | Clock (step 1) |
| A program froze | The window is in "Select" mode: press Esc. Click title bars only. |
| No bottle in `/objects/json` | Bottle outside the camera view, or too far (over 2.5 m). Check the Detections image in RViz. The camera must see it 3 times first; the lidar can't classify. |
| `track` stays `lost` when the bottle is beside the robot | Bottle closer than 30 cm to something else, or shorter than about 20 cm. |
