# Robot–Drone Search-and-Rescue Team: Final Project Handoff

**Last updated:** Sunday 11 October 2026 (hackathon day).
**What this is:** the single, complete context for continuing the project in a new AI chat.
It merges three earlier handovers (Asad's robot/laptop handover, Bilal's drone context,
and the drone operator's "Obstacle Scout" handover) with the mission design and code
written afterwards. Where those documents disagree, this one wins.

**Companion files** (delivered with this document):

- `cf_mission_bundle.zip`: `cf_fleet.py`, `mission.py`, `fleet_mission.yaml`,
  `MISSION_README.md` and the offline test harness (`tests/`).
- The same three source files are also reproduced verbatim in the Appendix of this document.

---

## 0. Instructions for the AI reading this

You are taking over as technical guide for a hackathon team. You cannot see or touch
their machines; they run your instructions and report back.

1. **Plain language.** The team asked for simple words. Explain any technical term the
   first time you use it. Keep code blocks short in chat; give whole files as files.
2. **One step at a time.** Give an exact command, say what output to expect, and wait for
   the result before the next step. Hardware steps especially.
3. **Ask which machine.** There are four machines (robot, Asad's Windows/WSL laptop, the
   drone PC, possibly Bilal's laptop). Wrong-machine commands were a real past problem.
   Prompts: robot = `[TURTLEBOT] turtle@ubuntu`, Asad = `asadrizvi64@DESKTOP-GBMJ4NA`,
   drone PC user = `alijodat`.
4. **Facts vs guesses.** Section 16 lists what is not confirmed. Check before building on it.
5. **Time is short.** It is demo day and there are about 6 working hours. Prefer the fix
   that works today over the elegant one. Protect the fallback demo (Section 14).
6. **Safety.** Propeller guards on. Clear the flight area. Every script must land the drone
   on Ctrl+C (the provided code does). Know the e-stop commands in Section 11.7.
7. **Don't reintroduce rejected designs** without a reason (Section 17 explains each decision).
8. **Version-sensitive facts** (cflib API, firmware) were correct on 10–11 Oct 2026.
   Verify against Bitcraze/ROS docs if something behaves differently.

---

## 1. The project in one paragraph

A **TurtleBot3** ground robot and **two Crazyflie 2.1 Brushless** nano-drones work as one
search-and-rescue team. The robot drives through an area, builds a map with its lidar
(SLAM) and recognises objects with its camera (YOLO). Every object gets x, y coordinates
on the robot's map. The **closest free drone** then takes off, flies to the object, dips
down to "pick it up", flies to a marked **safe zone**, dips again to "drop" it, flies back
to its own base mark and lands. The robot keeps searching meanwhile, and the second drone
takes the next object. If one drone fails, the other takes over. RViz shows the whole
story live: packages appear red when found, ride under the drone on a "rope" when
carried, and turn green in the safe zone.

**The pickup is simulated** (the drone dips; nothing is grabbed). The team says so openly
in the pitch. The drone can carry up to 40 g, so a small gripper or magnet would make the
same flight path a real pickup.

**Pitch line:** "The drones are blind, the robot is short. Together they search and rescue."

---

## 2. Team, machines and roles

| Who / what | Machine | Runs | Notes |
|---|---|---|---|
| **Robot** | TurtleBot3 **Waffle Pi** (Raspberry Pi), Ubuntu 22.04 aarch64, ROS 2 Humble | bringup, SLAM, camera, zenoh bridge, `straight_avoid.py` | Configured as `TURTLEBOT3_MODEL=waffle`. LDS-01 lidar, Pi camera (libcamera/unicam). Login `ssh turtle@172.20.10.3`, password: *(ask the team)*. Everything runs in tmux session `robot`. |
| **Asad** | Windows 10 (build 19045) + WSL2 **Ubuntu-22.04**, ROS 2 Humble | zenoh bridge, YOLO `object_mapper`, video recorder, RViz | WSL2 on Win10 is behind NAT, so it cannot join Fast DDS directly and uses a zenoh bridge (one TCP link to the robot). tmux session `lap`. Videos land in `C:\Users\HP\Videos`. A 24.04 distro also exists but has no ROS. |
| **Drone operator** | Ubuntu 22.04 PC, user `alijodat` (native vs VM **unknown**) | Crazyradio, `cf_fleet`, `mission`, RViz for the demo screen | `fly_to_target.py` already connects, arms, takes off and lands here. Workspace `/home/alijodat/ros2_ws`, package `cf_target_flight`. |
| **Bilal** | Ubuntu 22.04.3 native, ROS 2 Humble | (optional) second place to run drone code | Wrote the first drone node. Installed cflib in a venv `~/cf_venv` (`--system-site-packages`). **Unknown** whether Bilal and the drone operator are the same person/PC. |
| **Another teammate laptop** | Fast DDS, joins directly | sometimes drives the robot | Only **one** bringup and **one** teleop may run at a time, team-wide. |

**Recommended split for demo day**

- Asad: robot state, YOLO, recordings, slides.
- Drone operator: floor fix, drone tests, then `cf_fleet` + `mission` + RViz demo screen.
- Third person: slides, video editing, rehearsal.

---

## 3. Network

| Item | Value |
|---|---|
| Wi-Fi | iPhone hotspot **`bD`**, password: *(ask the team)* (WPA2, 2.4 GHz). Keep the Personal Hotspot screen open; enable Maximize Compatibility. |
| Subnet | `172.20.10.x`. Robot **.3**; laptops .2 / .4 (DHCP). |
| ROS middleware | **Fast DDS** (team decision; Zenoh RMW was removed). |
| Domain | `export ROS_DOMAIN_ID=30` on **every** machine and terminal. |
| WSL link | `zenoh_bridge_dds` 0.5.0 (ROS apt package `ros-humble-zenoh-bridge-dds`). Robot side listens on `tcp/0.0.0.0:7447` with an allow-list (`-a`); laptop side connects with `-e tcp/172.20.10.3:7447 --no-multicast-scouting`. Flags of this version: `-d -l -e -a` only (no max-frequency flag). |

**Current robot bridge allow-list** (in `~/robot_extras.sh` on the robot):

```
scan|tf|map|camera_info|compressed|cmd_vel|odom|joint_states|robot_description|goal_pose|plan|initialpose
```

**Required change (not yet done):** add `|objects` so the YOLO output made on Asad's WSL
reaches the drone PC. Restart the bridge window afterwards. Test on the drone PC:
`ros2 topic echo /objects/json --once`.

**Bridge limitation:** it does not keep "transient-local" (latched) delivery. That is why
RViz on Asad's WSL needs Map Durability = Volatile. The drone PC joins Fast DDS directly,
so RViz there shows the map normally. **Use the drone PC's RViz as the demo screen.**

**If the drone PC is a virtual machine** with NAT networking, it will not see the robot
either (same problem as WSL). Fix: bridged networking in the VM, or a zenoh bridge like
Asad's. Check with `ros2 topic list | grep scan`.

**Clock:** Asad's Windows clock was 1.6–2 s off, which broke the bridge. `chrony` is set up
in WSL (`makestep 0.2 -1`, `minpoll 3 maxpoll 4`); also Windows Settings → Date & time →
Sync now.

---

## 4. Hardware facts

### 4.1 Crazyflie 2.1 Brushless (×2)

- Open-source research nano-drone, 32 g, about **10 minutes** flight on the stock
  battery, **40 g** maximum recommended payload (Bitcraze product page). Widely used for
  indoor swarm research.
- Firmware platform **`cf21bl`**, release **2026.08** flashed on 2026-10-10 (drone 1).
- **Needs arming** before the motors spin: `cf.supervisor.send_arming_request(True)`.
- **Flow deck v2** underneath (`deck.bcFlow2 = 1`): optical flow + downward laser range.
  This gives **relative** position only: (0, 0) is wherever the estimator was last reset.
  It needs a **matte, textured, well-lit floor**. Glossy floors make the estimate wander.
  Expect slow drift (a few cm per minute on a good floor).
- **Multi-ranger deck:** not mounted; unknown whether the team has one. **Not needed** for
  this plan (the robot finds objects).
- No Lighthouse/Loco/mocap system, so no absolute positioning.
- Charging dock (12 V or USB-C) has no button or self-test. Its pad is black; **take off
  from the mat, not the dock**. Flow-deck accuracy is not enough to land back on the dock.

**Drone 1 (confirmed):** URI `radio://0/80/2M/E7E7E7E701`. Its sticker says "03" (reason
unknown, maybe kit numbering; other teams may have Crazyflies on the same channel).
**Drone 2:** not configured yet. Set address `E7E7E7E702` (Section 11.4).

### 4.2 Crazyradio

USB ID `1915:7777`. Bilal's notes say Crazyradio 2.0; the drone operator's notes say the
model was not recorded. **Only one program at a time can open the radio**: close cfclient,
`fly_to_target`, and Asad's `crazyflie_bridge` before running `cf_fleet`. One radio can
serve both drones if both are on the same channel with different addresses, inside one
process (which `cf_fleet` does).

### 4.3 LED reference (Crazyflie)

- Both blue LEDs solid, front-right red blinking twice a second: normal, self-test passed.
- Five short red pulses repeating: self-test failed.
- Blue LEDs blinking: bootloader mode (URI `radio://0/0/2M`).
- Red blinking once every 2 s: sensors not calibrated; put it level and still.
- Front-left flickering red/green: radio link active.
- Back-left blue blinking: charging; solid: full.

### 4.4 TurtleBot3

Waffle Pi, about 14 cm tall. LDS-01 lidar (`/scan` ~5 Hz on the robot, ~4.2 Hz through
the bridge), `/odom` 20 Hz, Pi camera via `camera_ros` at 640×480 RGB888, ~30 Hz on
`/camera/image_raw/compressed`. Camera is **uncalibrated** (`camera_info` K is all zero),
so the mapper uses `hfov_deg=62.2` (v2 camera; v1 would be 53.5°).

---

## 5. System architecture

```
ROBOT (Pi)  tmux "robot"                      Fast DDS, domain 30
  bringup   ros2 launch turtlebot3_bringup robot.launch.py   -> /scan /odom /tf  (ONLY ONE!)
  slam      slam_toolbox online_async  (params ~/slam_tb3.yaml)  -> /map, TF map->odom
  camera    camera_ros camera_node 640x480 RGB888  -> /camera/image_raw/compressed
  bridge    zenoh_bridge_dds -d 30 -l tcp/0.0.0.0:7447 -a '<allow-list>|objects'
  drive     python3 ~/straight_avoid.py  (foreground SSH or tmux)
        |                                   |
        | one TCP link (zenoh)              | Fast DDS over the hotspot (direct)
        v                                   v
ASAD'S WSL  tmux "lap"                    DRONE PC (Ubuntu 22.04)
  bridge  zenoh_bridge_dds -e tcp/...     cf_fleet  (Crazyradio -> cf1, cf2)
  mapper  object_mapper (YOLOv8n CPU)     mission   (referee: jobs, markers, events)
          -> /objects/json (map frame) ---> mission (via robot bridge, needs |objects)
  rec_*   video_recorder -> MP4           RViz      (demo screen: map, TF, /mission/markers)
  RViz    light.rviz (optional)
```

Data flow for one rescue:

1. Camera image + lidar → `object_mapper` → `/objects/json` (map coordinates).
2. `mission` collects detections, confirms an object after 3 sightings, picks the closest
   free drone, sends it a **route** in map coordinates on `/cfN/route`.
3. `cf_fleet` converts map → drone coordinates using that drone's tape-mark pose, and flies
   it with the Crazyflie **high-level commander** (the drone plans its own smooth path).
4. `cf_fleet` reports `/cfN/status` (phase, done actions, position in map, battery).
5. `mission` updates the package state and RViz markers; logs to `/mission/events`.

---

## 6. Coordinate frames (the core idea)

**Map frame** (`map`): the robot's SLAM map. (0, 0) = where the robot was when SLAM
started, x = the direction the robot faced, y = to its left, metres. It does not drift.
All mission coordinates (objects, safe zone, drone bases, fence) are in this frame.

**Drone frame** (per drone): the Flow deck's own frame. (0, 0) = where the drone sat when
its estimator was reset, x = the way its antenna pointed, y = left. It drifts slowly.

**The link between them** is each drone's `origin` parameter: the map pose of its tape
mark, `[x0, y0, yaw_deg]`. With θ = yaw in radians:

```
map   = (x0 + cosθ·lx − sinθ·ly,   y0 + sinθ·lx + cosθ·ly)
local = ( cosθ·dx + sinθ·dy,       −sinθ·dx + cosθ·dy)    where dx = X − x0, dy = Y − y0
```

How it is fixed in practice:

1. Always start SLAM with the robot on the same tape mark ("mark A"), facing the same way.
   Then map (0, 0) is mark A, every run.
2. Tape a cross for each drone at a measured offset from mark A, plus a heading line.
   Default: cf1 at (0.0, 0.6), cf2 at (0.0, 1.0), both facing 0° (same as the robot).
3. Start `cf_fleet` with the drones sitting on their crosses, antenna along the line.
   That is the moment their (0, 0) is set.

**Sensitivity:** a 10° heading error puts the drone about 35 cm off after 2 m; position is
off 1:1 with any tape-measure error. Accuracy to expect overall: **10–20 cm** (estimate).
The design tolerates this: drones dip 20 cm *beside* objects, and the safe zone is 60 cm.

`cf_fleet` publishes TF `map → cf1`, `map → cf2` (dynamic `/tf`, 10 Hz, so it would also
cross the bridge) and `geometry_msgs/PointStamped` on `/cfN/pose` in `map`.

---

## 7. Software inventory

### 7.1 Existing code (written in earlier chats; status from those handovers)

Location on Asad's WSL: zips in `C:\Users\HP\Downloads\`, unpacked to `~/ws/src/`, built
with `colcon build --symlink-install`; `~/.bashrc` sources `~/ws/install/setup.bash`.

**`swarm_hack` (ament_python)**

- `object_mapper.py`: YOLO (ultralytics, YOLOv8n on CPU) on
  `/camera/image_raw/compressed`. Depth from a depth image if present, else lidar points
  projected into the box, else a ground-plane ray. `camera_frame: camera_rgb_optical_frame`.
  Outputs `/objects/json` (fields `id, cls, x, y, z, z_top, w, h, conf, ...`, **map
  frame**), `/objects/markers`, `/objects/debug_image`. **Verified:** a bottle mapped at
  (0.19, −0.39). **Unverified:** the exact JSON container shape (list vs `{"objects": [...]}`);
  `mission.py` accepts both.
- `video_recorder.py`: any image topic → MP4 at real-time speed; Ctrl+C finalises and makes
  an `_h264.mp4` copy with ffmpeg.
- `crazyflie_bridge.py` + `swarm_coordinator.py`: an earlier multi-drone design (formation
  slots, `/swarm/mode` commands, fake drones with `sim:=true`). **Superseded by `cf_fleet` +
  `mission` for the demo. Never run `crazyflie_bridge` together with `cf_fleet`.**
- `geom.py` (SE(2)/quaternion helpers), `config/swarm.yaml`.
- `scripts/robot_extras.sh` (robot: starts camera + bridge windows in tmux `robot`).
- `scripts/laptop.sh [ROBOT_IP]` (WSL: tmux `lap` with bridge, mapper, swarm sim, cmd window).
- `scripts/straight_avoid.py` **v3.1** (runs on the robot): holds a straight line from its
  start pose, chooses the free heading closest to "back onto the line" with a footprint
  corridor check, keeps its side while blocked, rejoins the line. Params `distance`
  (0 = forever), `speed`, `legs` (back-and-forth count, 0 = forever), `margin`,
  `clear_dist`, `stop_dist`. Own signal handling: Ctrl+C, kill and SSH drop all send zero
  velocity (10 zero Twists). Simulated in 7 scenarios (rejoins within 5 cm, ≥ 8 cm clearance).
  **v1 drove on the real robot; v3.1 not yet copied/tested there.**

**`tb_nav` (ament_python)**: `nav_manager.py` (Nav2 action client, `/nav/command`:
`goto x y [yaw]`, `goto <name>`, `save <name>`, `patrol N`, `object <id>`, `init`,
`relocalize`, `cancel`; waypoints in `~/tb_waypoints.yaml`), `obstacle_guard.py`
(`/cmd_vel_raw → /cmd_vel`, modes guard|wander|off), `launch/tb_nav.launch.py`
(`mode:=slam|localization`, `run_slam:=false`; uses Nav2's own Humble params because the
TB3 params use Jazzy plugin names), `config/slam_tb3.yaml` (LDS-01 tuned:
`max_laser_range 3.5`, `map_update_interval 1.0`, `minimum_travel 0.1 m / 0.15 rad`;
copy on robot `~/slam_tb3.yaml`), `config/light.rviz`. `start_robot.sh` /
`start_laptop.sh` are **obsolete** (zenoh era). Nav2 is **not** used in the demo.

**`cf_target_flight/fly_to_target.py`** (drone PC): single-drone "fly to a point and land"
node. Subscribes `/target_point` (geometry_msgs/Point, z ignored), publishes `/cf/position`
(PointStamped, frame `cf_origin`, 10 Hz). Latest version (pasted 2026-10-11): default URI
`radio://0/80/2M/E7E7E7E701`, height 0.5 m, speed 0.15 m/s, take-off and landing 4.0 s,
max target 4 m. **Flies, but does not reach its targets ("random" landings)**, most likely
because of a glossy floor. Keep it for single tests; `cf_fleet` replaces it for the mission.

### 7.2 New code for the mission (this handoff; full source in the Appendix)

| File | Node name | Purpose |
|---|---|---|
| `cf_fleet.py` | `cf_fleet` | Flies N Crazyflies on one radio; map-frame routes; safety checks; fake-drone mode. |
| `mission.py` | `mission` | Referee: object list, job assignment, retries, benching, RViz markers, events. |
| `fleet_mission.yaml` | both | Parameters for both nodes (one file, two sections). |
| `MISSION_README.md` | – | Human-facing install/run guide (shorter version of this document). |

Install them into the existing `cf_target_flight` package on the drone PC (Section 10).

---

## 8. `cf_fleet` in detail

### 8.1 Interfaces

| Direction | Topic | Type | Content |
|---|---|---|---|
| in | `/cfN/route` | `std_msgs/String` | JSON route (8.2) |
| out | `/cfN/status` | `std_msgs/String` | JSON status, 10 Hz (8.3) |
| out | `/cfN/pose` | `geometry_msgs/PointStamped` | position in `map`, 10 Hz |
| out | `/tf` | TF | `map → cfN`, 10 Hz (rotation = base heading) |
| in | `/fleet/land_all` | `std_msgs/Empty` | every drone lands where it is |
| in | `/fleet/rezero` | `std_msgs/String` | `"cf1"` or `"all"`: set (0, 0) again (drone must be landed, on its mark) |

### 8.2 Route message

```json
{"job": "bottle1-try1",
 "waypoints": [{"x": 1.0, "y": -0.3, "action": "pick"},
               {"x": -0.6, "y": 1.4, "action": "drop"},
               {"action": "home"}]}
```

Actions: `go` (fly there), `pick` / `drop` (fly there, dip to `dip_height`, wait `dwell`,
climb back), `home` (fly over the drone's own tape mark), `hover` (stay put `"t"` seconds,
0 < t ≤ 120; reports how far it wandered). The drone **always lands at the last waypoint**.
Coordinates are **map** coordinates.

A route is **rejected** (status `failed`, msg starts with `rejected:`) if: an action is
unknown, a number is not finite, a point is outside `fence`, or any leg is longer than
`max_leg`. A route sent while the drone is busy is **ignored** (logged), not queued.

### 8.3 Status message

```json
{"name": "cf1", "state": "busy", "phase": "to_drop", "job": "bottle1-try1",
 "done": ["pick"], "x": 0.41, "y": 0.52, "z": 0.5, "vbat": 3.95, "miss": 0.04, "msg": ""}
```

- `state`: `idle` (ready) · `busy` (flying a job) · `failed` (last job failed or was
  rejected; still ready for a new job).
- `phase`: `landed` · `takeoff` · `hover_check` · `to_pick` · `picking` · `to_drop` ·
  `dropping` · `to_home` · `to_go` · `hovering` · `landing` · `rezeroing`.
- `done`: actions finished in the current job (how `mission` knows "picked" / "dropped").
- `x, y, z`: drone position in **map** frame (z from the Flow deck).
- `vbat`: battery voltage (0.0 until the first log arrives).
- `miss`: worst distance between a target and where the drone *believes* it ended, this job.
- `msg`: reason text (`aborted`, `drifted 0.16 m right after take-off (check the floor
  and light)`, `rejected: ...`, `error: ...`, `hover test: wandered up to 0.03 m`,
  `re-zeroed on its mark`).

### 8.4 One job, step by step

1. Arm (`send_arming_request(True)`), wait `arm_time`.
2. Remember the ground position, take off to the drone's cruise `height` over `takeoff_time`.
3. **Hover check:** hover `hover_check` s; if x/y wanders more than `hover_tol` from the
   ground position → land at once, status `failed`, msg "drifted ...".
4. For each waypoint: fly at cruise height; duration = distance / `speed` (at least
   `min_leg_time`) + `settle_time`. **Arrival check:** wait up to `arrive_wait` s for the
   estimate to come within `arrive_tol`; record `miss`; log a warning if it never does
   (the job continues). For pick/drop: dip over `dip_time`, wait `dwell`, climb back.
5. Land over `land_time`, `hlc.stop()`, disarm. Status `idle`, phase `landed`.

Abort (`/fleet/land_all`, Ctrl+C, shutdown) at any point → land where it is, disarm,
status `failed`, msg `aborted`. Any exception → same, msg `error: ...`.

Typical job time with the YAML defaults: about **1 minute** (4 s take-off, 2 s hover check,
~6 s per dip, legs at 0.2 m/s, 4 s landing). Plan **2–3 jobs per battery**.

### 8.5 Coordinates and the estimator reset

- Each drone's estimator is reset **once, at node start** (`reset_estimator`). The drone
  must be on its tape mark then. Not resetting before each take-off is deliberate: routes
  end with `home`, so the drone comes back by itself, and resetting where it landed would
  bake its drift in.
- If someone carries a drone back to its mark (after an e-stop or a bad landing), send
  `/fleet/rezero` for it, or restart `cf_fleet`.

### 8.6 Parameters (code default → YAML value)

| Parameter | Code default | YAML | Meaning |
|---|---|---|---|
| `sim` | false | false | fake drones, no radio (override `-p sim:=true`) |
| `drones` | `[cf1]` | `[cf1, cf2]` | names; one URI/origin/height block each |
| `deck` | `bcFlow2` | `bcFlow2` | refuse to start if this deck isn't detected |
| `speed` | 0.2 | 0.2 | m/s average per leg (0.15 = the colleague's tested value) |
| `dip_height` | 0.25 | 0.25 | m, height of the pick/drop dip |
| `dwell` | 2.0 | 2.0 | s at the bottom of a dip |
| `dip_time` | 1.5 | – | s to go down / up |
| `takeoff_time` | 4.0 | 4.0 | s |
| `land_time` | 4.0 | 4.0 | s |
| `arm_time` | 1.0 | – | s wait after arming |
| `min_leg_time` | 1.0 | – | s, shortest leg duration |
| `settle_time` | 0.5 | – | s added after every move |
| `hover_check` | 2.0 | 2.0 | s hover after take-off (0 = skip) |
| `hover_tol` | 0.15 | 0.15 | m, land if it wanders more in the check |
| `arrive_tol` | 0.15 | 0.15 | m, "arrived" threshold |
| `arrive_wait` | 2.0 | – | s extra allowed to arrive |
| `fence` | `[-3,3,-3,3]` | `[-1.0, 3.5, -1.5, 2.0]` | xmin, xmax, ymin, ymax in map metres |
| `max_leg` | 4.0 | 4.0 | m, longest allowed single leg |
| `cfN.uri` | `radio://0/80/2M/E7E7E7E70N` | 01 / 02 | radio address per drone |
| `cfN.origin` | `[0, 0.5·N, 0]` | cf1 `[0.0, 0.6, 0.0]`, cf2 `[0.0, 1.0, 0.0]` | tape mark: x, y, heading in degrees |
| `cfN.height` | 0.5 + 0.3·(N−1) | cf1 0.5, cf2 0.8 | cruise height; different heights prevent mid-air collisions |

**YAML gotcha:** every number needs a decimal point (`0.0`, not `0`); ROS refuses a
parameter whose type differs from the declared default.

---

## 9. `mission` in detail

### 9.1 Interfaces

| Direction | Topic | Type | Content |
|---|---|---|---|
| in | `/objects/json` (param `objects_topic`) | `std_msgs/String` | YOLO detections in map frame |
| in | `/cfN/status` | `std_msgs/String` | from `cf_fleet` |
| in | `/mission/cmd` | `std_msgs/String` | `start` · `pause` · `land` · `reset` · `status` |
| in | `/fleet/land_all` | `std_msgs/Empty` | anyone's e-stop also pauses the mission |
| out | `/cfN/route` | `std_msgs/String` | jobs for `cf_fleet` |
| out | `/fleet/land_all` | `std_msgs/Empty` | sent by the `land` command |
| out | `/mission/markers` | `visualization_msgs/MarkerArray` | RViz, 5 Hz |
| out | `/mission/events` | `std_msgs/String` | readable log ("cf1 picked up bottle #1") |

### 9.2 Reading detections

Accepts a JSON list of objects, or a dict with an `objects` list, or a single object dict.
Per object it reads `cls` (or `class` / `label`), `x`, `y`, `conf` (or `score`; default 1.0).
It drops: non-finite positions, `conf < min_conf`, classes not in `classes` (unless `*`),
and anything inside the safe-zone square.

**Merging:** a detection within `merge_radius` of a known item is the same item. While the
item is `seen` or `waiting` its position is a running average (weight capped at 20). Once
assigned or delivered its position is frozen. Items keep their **original** spot even when
"delivered" (the real object never moves), so YOLO seeing it again doesn't create a duplicate.

**Confirmation:** an item becomes `waiting` (counts as "Found") after `min_sightings`
detections.

### 9.3 Item states

```
seen ──(min_sightings)──> waiting ──(assigned to closest free drone)──> assigned
assigned ──('pick' in done)──> carried ──('drop' in done)──> delivered
assigned/carried ──(drone failed, tries < 2)──> waiting   (retry, possibly other drone)
any ──(route rejected, or 2nd failure)──> skipped
```

### 9.4 Assigning jobs (every 0.5 s while running)

- A drone is **free** if its status is fresh (< 2 s old), `state` is `idle` or `failed`,
  `phase` is `landed`, it has no job, it isn't **benched**, and its battery is not below
  `min_vbat` (a reading ≤ 0.1 V means "unknown" and is allowed).
- **Closest pair first:** among all free drones × waiting items, assign the pair with the
  smallest distance (drone's current map position to the item), then repeat.
- The route: pick point = object position moved `approach_offset` (0.2 m, at most half the
  distance) **toward the safe zone**, so the drone dips beside the object; drop point = the
  item's own **slot** in the zone (3 × 3 grid, `slot_spacing` 0.2 m, wraps after 9); then
  `home` if `return_home`.
- Job id: `<class><id>-try<N>` (spaces replaced by `_`), e.g. `teddy_bear3-try1`.
- **Lost route:** if the drone's status hasn't shown the job id 3 s after sending, resend
  (up to 2 times), then put the item back to `waiting`.

### 9.5 Failures, benching, completion

- Drone ends a job with `state` idle/failed and `phase` landed: if the item was delivered,
  done. Otherwise the job failed:
  - msg starts with `rejected` → item `skipped` (e.g. outside the fence).
  - msg is `aborted` (e-stop) → item back to `waiting`; mission was paused anyway.
  - anything else (drift, error) → the drone is **benched** (no more jobs until the next
    `start`), and the item is retried by another drone; a second failure → `skipped`.
- **Mission complete** event when nothing is seen/waiting/assigned/carried, at least one
  item is delivered, and no drone is busy.
- Commands: `start` (run; also un-benches drones), `pause` (no new jobs), `land` (pause +
  `/fleet/land_all`), `reset` (forget items and slots; refused while drones work),
  `status` (summary event).

### 9.6 RViz markers (`/mission/markers`, frame `map`)

| Namespace | What |
|---|---|
| `zone` | translucent green 60 cm square + label "Safe zone d/n" |
| `drones` | blue disc at each drone + label "cfN: <phase>" |
| `items` | 8 cm cube + label "<cls> #<id>: <state>". Colours: red waiting, amber assigned/carried, green delivered, grey skipped. While carried: cube 12 cm under the drone plus a grey **rope** line to the drone. Delivered: cube sits in its zone slot. |

RViz setup: Fixed Frame `map`; add Map (`/map`), LaserScan (`/scan`), TF, MarkerArray
`/mission/markers`; optionally Image `/objects/debug_image` (YOLO boxes).

### 9.7 Parameters (code default → YAML value)

| Parameter | Code default | YAML | Meaning |
|---|---|---|---|
| `drones` | `[cf1, cf2]` | `[cf1, cf2]` | must match `cf_fleet` |
| `objects_topic` | `/objects/json` | same | |
| `classes` | `bottle,cup,teddy bear,sports ball,backpack` | same | COCO class names, comma-separated; `*` = any |
| `min_conf` | 0.4 | 0.4 | |
| `min_sightings` | 3 | 3 | |
| `merge_radius` | 0.35 | 0.35 | m |
| `drop_zone` | `[0.0, 1.5]` | `[-0.6, 1.4]` | safe zone centre, map metres |
| `zone_size` | 0.6 | 0.6 | m (tape this square on the floor) |
| `slot_spacing` | 0.2 | – | m |
| `approach_offset` | 0.2 | 0.2 | m |
| `return_home` | true | true | false = land in the drop slot instead |
| `min_vbat` | 3.4 | 3.4 | V |
| `auto_start` | false | false | true = run without the `start` command |
| `test_objects` | `""` | `""` | rehearsal objects: `"bottle:1.2,-0.4; cup:2.0,-0.3"` (class optional) |

---

## 10. Installing on the drone PC

Use the PC where `fly_to_target` already flies: radio, udev rules, firmware and cflib work
there, and the new files live in the same package.

1. Copy `cf_fleet.py` and `mission.py` into
   `/home/alijodat/ros2_ws/src/cf_target_flight/cf_target_flight/` (next to
   `fly_to_target.py`). Copy `fleet_mission.yaml` to `~/`.
2. `setup.py` → `entry_points` → `console_scripts`, add:
   ```python
   'cf_fleet = cf_target_flight.cf_fleet:main',
   'mission = cf_target_flight.mission:main',
   ```
3. `package.xml`: add `<exec_depend>` for `std_msgs`, `geometry_msgs`,
   `visualization_msgs`, `tf2_ros` (all come with `ros-humble-desktop`).
4. Rebuild the same way `fly_to_target` was built:
   ```bash
   cd ~/ros2_ws && source /opt/ros/humble/setup.bash
   colcon build --packages-select cf_target_flight
   source install/setup.bash
   ```
5. Check cflib is visible to the Python that `ros2 run` uses:
   `python3 -c "import cflib, cflib.utils.reset_estimator; print(cflib.__file__)"`.

**If cflib lives in Bilal's venv instead** (`~/cf_venv`, made with
`--system-site-packages`): `source ~/cf_venv/bin/activate` before building; the package's
`setup.cfg` must contain `[build_scripts] executable = /usr/bin/env python3`; build
**without** `--symlink-install` (symlink mode hard-codes the build Python). Reason for the
venv: cflib 0.1.34 wants numpy 2.2, while ROS Humble on 22.04 uses system numpy 1.21.

In every terminal on this PC: source ROS + `install/setup.bash`, and
`export ROS_DOMAIN_ID=30`.

---

## 11. Runbooks

### 11.1 Robot start-up (on the robot)

```bash
ros2 node list | grep -E "turtlebot3_node|hlds" | sort | uniq -c     # exactly 1 each
# bringup if missing:
tmux new-window -t robot -n bringup "bash -ic 'ros2 launch turtlebot3_bringup robot.launch.py'"
# SLAM if missing (robot on mark A, facing the agreed direction):
tmux new-window -t robot -n slam "bash -ic 'ros2 launch slam_toolbox online_async_launch.py slam_params_file:=\$HOME/slam_tb3.yaml'"
bash ~/robot_extras.sh             # camera + bridge (after adding |objects to its -a list)
timeout 5 ros2 topic hz /scan      # ~5 Hz
```

Duplicate bringups fight over `/dev/ttyUSB0` and the lidar disappears. Fix:
`tmux kill-window -t robot:bringup; pkill -f hlds_laser; pkill -f turtlebot3_ros`, then
start exactly one.

Copy straight_avoid v3.1 first (from Asad's WSL):
`scp /mnt/c/Users/HP/Downloads/straight_avoid.py turtle@172.20.10.3:~/`, then on the robot
`grep -c SignalHandlerOptions ~/straight_avoid.py` must print ≥ 1.

### 11.2 Asad's WSL

```bash
bash ~/ws/src/swarm_hack/scripts/laptop.sh 172.20.10.3   # bridge, mapper, (swarm sim) windows
ros2 daemon stop                                         # once, if the CLI sees nothing
ros2 topic hz /scan                                      # ~4-5 Hz through the bridge
ros2 topic echo /objects/json --once                     # YOLO output
```

`laptop.sh` also starts Asad's old fake-drone swarm window. It doesn't touch the radio, but
it publishes its own drone TF frames (likely `cf1`/`cf2`) that would clash with
`cf_fleet`'s. **Close that `swarm` window during the demo.**

Recording:

```bash
tmux new-window -t lap -n rec_cam "bash -ic 'ros2 run swarm_hack video_recorder'"
tmux new-window -t lap -n rec_yolo "bash -ic 'ros2 run swarm_hack video_recorder --ros-args -p topic:=/objects/debug_image -p name:=yolo'"
tmux send-keys -t lap:rec_cam C-c; tmux send-keys -t lap:rec_yolo C-c   # stop; MP4s in C:\Users\HP\Videos
```

Also screen-record the drone PC's RViz (any screen recorder) and film the arena with a phone.

### 11.3 Drone tests, in this order (gates)

**Step 0, the floor (do first).** Lay matte, patterned material (foam puzzle mats, rug,
printed paper) over about 2.5 × 2.5 m. Light it evenly. Tape mark A (robot), a cross plus
heading line per drone, and the 60 cm safe-zone square.

**Test 1, hover (one drone).** Set `drones: [cf1]` in both YAML sections. Drone on its mark.

```bash
ros2 run cf_target_flight cf_fleet --ros-args --params-file ~/fleet_mission.yaml
ros2 topic pub --once /cf1/route std_msgs/msg/String \
  '{data: "{\"job\": \"hover\", \"waypoints\": [{\"action\": \"hover\", \"t\": 30}, {\"action\": \"home\"}]}"}'
ros2 topic echo /cf1/status
```

**Gate:** it stays within about 10–15 cm of where it took off for 30 s (judge by eye
against the tape). If it lands after 2 s with "drifted", the floor or light is the problem.

**Test 2, out and back.** Five times:

```bash
ros2 topic pub --once /cf1/route std_msgs/msg/String \
  '{data: "{\"job\": \"t1\", \"waypoints\": [{\"x\": 1.0, \"y\": 0.6, \"action\": \"go\"}, {\"action\": \"home\"}]}"}'
```

(With cf1's mark at (0.0, 0.6), map (1.0, 0.6) is 1 m straight ahead.) **Gate:** landings
within about 15 cm of each other. Diagnosis: compare `miss` in the status with reality.
Small `miss` but physically elsewhere → floor/estimate. Big `miss` → timing/flight.

**Test 3, one-drone mission** with rehearsal objects placed on the floor at the same spots:

```bash
ros2 run cf_target_flight mission --ros-args --params-file ~/fleet_mission.yaml \
  -p test_objects:="bottle:1.2,-0.4"
ros2 topic pub --once /mission/cmd std_msgs/msg/String "data: start"
```

**Test 4, two drones.** Configure drone 2's address (11.4), set `drones: [cf1, cf2]` in
both sections, both drones on their marks, restart `cf_fleet`.

### 11.4 Configuring drone 2's radio address (cfclient)

1. Close everything that uses the radio. Power on drone 2, connect it by USB cable.
2. cfclient: Scan → select `usb://0` → Connect → **Connect → Configure 2.X**: channel 80,
   2M, address `0xE7E7E7E702` → **Write**.
3. Disconnect, unplug, **restart the drone** (new address applies after reboot).
4. Type `0xE7E7E7E702` in the main window's Address box, press Enter (it doesn't update
   itself), Scan, connect `radio://0/80/2M/E7E7E7E702` to confirm.
5. If other teams' Crazyflies are around (drone 1's sticker says "03"), move **both**
   drones to another channel (for example 60) the same way and change both `uri` lines.

Flashing firmware (if needed): cfclient bootloader dialog, platform `cf21bl`, release
2026.08, Program; if stuck on "Trying to connect in firmware mode", use the **Cold boot
(recovery)** tab. Enter bootloader on purpose: power off, hold the power button ~3 s until
the blue LEDs blink.

### 11.5 Rehearsal with fake drones (no hardware)

```bash
ros2 run cf_target_flight cf_fleet --ros-args --params-file ~/fleet_mission.yaml -p sim:=true
ros2 run cf_target_flight mission --ros-args --params-file ~/fleet_mission.yaml \
  -p test_objects:="bottle:1.2,-0.4; cup:2.0,-0.3"
ros2 topic pub --once /mission/cmd std_msgs/msg/String "data: start"
ros2 topic echo /mission/events
```

Leave out `test_objects` to use real YOLO detections with fake drones. This is also the
**fallback demo**.

### 11.6 The full demo

1. Robot: clean start (11.1), robot on mark A when SLAM starts.
2. Asad: bridge + mapper (11.2), close the old swarm window, start recorders.
3. Drone PC: drones on their marks → `cf_fleet` → `mission` → RViz.
4. Robot: `python3 ~/straight_avoid.py --ros-args -p distance:=2.5 -p speed:=0.10 -p legs:=2`
   (no teleop anywhere; check `ros2 topic info /cmd_vel -v`).
5. When the first object shows red in RViz: `mission` `start`.
6. Watch: drone dips at the object (amber, rope), dips at the safe zone (green), lands at
   base; second drone takes the next one; "Mission complete".

### 11.7 Emergency stops

- Drones: `ros2 topic pub --once /fleet/land_all std_msgs/msg/Empty` (any machine on the
  network), or `mission` command `land`, or Ctrl+C on `cf_fleet` (lands everything).
- Robot: lift it or use its power switch, or
  `ros2 topic pub --times 5 /cmd_vel geometry_msgs/msg/Twist "{}"`; Ctrl+C on
  straight_avoid v3.1 also stops it.
- After an e-stop, a drone stays where it landed and still knows where it is. If you carry
  it back to its mark: `ros2 topic pub --once /fleet/rezero std_msgs/msg/String "data: cf1"`.

---

## 12. Testing done (and not done)

**Done (offline, in a sandbox, Python 3.10 = ROS Humble's version):** both nodes run
against a fake rclpy/message layer and a fake cflib (harness in `tests/` of the bundle).

- `test_sim.py`, end to end with 2 fake drones: 3 objects confirmed only after 3
  sightings; wrong class and low confidence ignored; a far object inside the fence
  delivered; an object outside the fence skipped (route rejected); both drones used
  (closest pair); each drone actually dipped beside each pick point and in its slot; both
  ended landed on their own marks; cf2 cruised higher than cf1; each delivered package got
  a different slot inside the zone; re-detecting delivered objects created no new jobs;
  rope marker shown while carried; e-stop mid-carry → landed, mission paused, item back to
  waiting, then delivered after `start`; a drifting cf1 failed the hover check, landed, got
  benched, and cf2 delivered its job; the hover-test route and `/fleet/rezero` work. cf2's
  base was turned 90° in this test to prove the coordinate maths.
- `test_real.py`: the real-drone code path against a fake cflib. Exact call order:
  `reset_estimator` → arm → `takeoff(0.5)` → `go_to` in **local** coordinates (map (1, 3)
  from a base at (1, 2) facing 90° became local (1, 0)) → dips → `home` (0, 0) → `land` →
  `stop` → disarm → link closed; log variables `stateEstimate.x/y/z`, `pm.vbat`; deck check
  `deck.bcFlow2`.
- `test_yaml.py`: the YAML loads with the right types into both nodes; zone and bases sit
  inside the fence.

**Not done:** no real flight with `cf_fleet` yet; never run on real ROS 2 (only the fake
layer); `/objects/json` exact format not checked against `mission`; RViz look not seen;
two real drones on one radio not tried; flying near the moving robot not tried.

To re-run the tests: unzip the bundle, then `python3 tests/test_sim.py`,
`python3 tests/test_real.py`, `python3 tests/test_yaml.py` (the last needs PyYAML). They
don't need ROS.

---

## 13. Known risks, limits and gotchas

| # | Risk / gotcha | What to do |
|---|---|---|
| 1 | **Glossy floor → drone goes to "random" places** (current real symptom) | Matte patterned mats, even light; hover test gate before anything else |
| 2 | Flow deck drift accumulates | Short jobs; return home; re-zero after carrying a drone back |
| 3 | Heading error at the mark rotates every target | Heading line on the tape; antenna along it |
| 4 | Drone passing over the robot "sees" the floor jump up and bobs | Keep the safe zone and bases away from the robot's line; objects beside the line |
| 5 | Two drones colliding | Different cruise heights (0.5 / 0.8 m), separate bases and drop slots |
| 6 | Radio already in use (cfclient, `fly_to_target`, `crazyflie_bridge`) | Run only `cf_fleet` |
| 7 | Same radio address / channel as another drone or team | Unique addresses 01/02; change channel if others nearby |
| 8 | `/objects/json` blocked by the robot bridge allow-list | Add `objects` to the robot bridge allow-list (Section 3), restart it |
| 9 | Latched topics (map, static TF) don't cross the bridge | Demo RViz on the drone PC; `cf_fleet` uses dynamic TF |
| 10 | YAML integers vs floats | Always write `0.0` |
| 11 | Battery: about 10 min per drone | 2–3 jobs per battery; `min_vbat` 3.4 V keeps empty drones grounded; charge between runs |
| 12 | Duplicate robot bringup → no `/scan` | Exactly one bringup (11.1) |
| 13 | Teleop fighting straight_avoid | Close all teleops |
| 14 | Windows console "Select" mode freezes programs | Press Esc; click title bars only |
| 15 | Hotspot drop kills SSH sessions | Robot processes in tmux; keep hotspot screen open |
| 16 | YOLO false positives / duplicates | `min_sightings` 3, `merge_radius` 0.35, class list |
| 17 | Object positions only as good as the uncalibrated camera | Optional: calibrate, or set the right `hfov_deg` |
| 18 | Precise landing on the dock is impossible with the Flow deck | Land on the mat beside it |
| 19 | Old swarm sim on Asad's laptop publishes clashing drone TF frames | Close its window during the demo |
| 20 | Accuracy overall 10–20 cm (estimate) | Dip beside objects; 60 cm safe zone |

---

## 14. Six-hour plan, fallback and pitch

| Time | Robot / Asad | Drone operator | Done when |
|---|---|---|---|
| 0:00–0:45 | Clean robot start; copy straight_avoid v3.1; add `objects` to the bridge allow-list | Lay mats, tape marks; install `cf_fleet` + `mission`; fake-drone rehearsal in RViz | `/scan`, `/odom` live; sim mission completes on the drone PC |
| 0:45–1:30 | Record robot demo (`legs:=2`) + YOLO video = safety-net footage | Hover test, out-and-back test | Gates in 11.3 passed |
| 1:30–2:30 | Check `/objects/json` arrives on the drone PC; place 2 objects beside the line | One-drone mission with `test_objects`, then with real YOLO | One real delivery |
| 2:30–3:30 | Support | Configure drone 2; two-drone mission | Both drones deliver |
| **3:30** | **Go / no-go** | | Real drones reliable? |
| 3:30–4:30 | Full demo takes, all recorded (3+ takes) | | One clean take |
| 4:30–5:30 | Edit video, slides | | Pitch ready |
| 5:30–6:00 | Buffer, rehearse | | |

**No-go fallback at 3:30:** demo the real robot + real YOLO + **fake drones** in RViz
(`sim:=true`), plus a separate clip of one real drone flying one hop. It still tells the
whole story.

**Demo script (what judges see):** robot drives and swerves; RViz builds the map; YOLO finds
a bottle (red package, "Found bottle #1"); the closest drone takes off, dips at the bottle
(amber package on a rope), flies to the safe zone, dips (green), lands at base; the robot
keeps searching; the second drone takes the next object; "Safe zone 2/2", "Mission complete".

**Pitch lines**

- "The drones are blind, the robot is short. Together they search and rescue."
- "Robot = eyes and map. Drones = fast delivery over obstacles."
- "The closest free drone takes each job. If one drone fails, the other takes over."
- "Today the pickup is simulated. The Brushless can carry up to 40 g, so with a small
  gripper or magnet this exact flight path becomes a real pickup."

---

## 15. Troubleshooting log (all resolved earlier; recognise the symptoms)

| Symptom | Cause | Fix |
|---|---|---|
| Humble wouldn't install in WSL | Default distro was 24.04 | Installed Ubuntu-22.04, `wsl --set-default Ubuntu-22.04` |
| Robot unreachable from WSL | Win10 WSL2 NAT (mirrored mode needs Win11) | zenoh bridge over one outgoing TCP link |
| `zenoh-bridge-ros2dds` wouldn't run | Built for glibc 2.38 / missing musl libgcc | Use `ros-humble-zenoh-bridge-dds` 0.5.0 from ROS apt |
| Pasted command blocks ran partly | `add-apt-repository` / `sudo` swallow stdin | Run sudo/apt lines one at a time |
| "exceeding delta 500ms", SLAM stalling | Windows clock off 1.6–2 s | chrony in WSL + Windows time sync |
| Nav2 planner FATAL | TB3 params use Jazzy `::` plugin names | Nav2's own Humble params (default in `tb_nav`) |
| SLAM on laptop smeared the map | Wi-Fi latency + clock + driving too fast | SLAM on the robot; 0.05–0.1 m/s, turns ≤ 0.5 rad/s |
| Commands on the wrong machine | Confusing prompts | Check the prompt (Section 0) |
| Duplicate `/turtlebot3_node`, no lidar | Two bringups | One bringup (11.1) |
| Programs froze on Windows | Console Select/Mark mode | Esc |
| Everything died when the hotspot dropped | Processes tied to SSH | tmux |
| `librmw_zenoh_cpp.so` not found | `.bashrc` still exported zenoh RMW | `sed -i '/rmw_zenoh/d' ~/.bashrc` |
| `ros2 topic echo` sees nothing | Stale ros2 daemon | `ros2 daemon stop` |
| `v4l2_camera` "Failed stream start" | Pi camera uses libcamera on 22.04 | `ros-humble-camera-ros`, `camera_ros camera_node` |
| Robot kept driving after Ctrl+C | TB3 keeps last cmd_vel | straight_avoid v3 sends zero Twists on any exit |
| straight_avoid stuck in WAIT | No `/odom` or `/scan` | One bringup; `ros2 topic hz /odom /scan` |
| Teleop fights autonomy | Teleop republishes cmd_vel | Close teleops |
| cfclient Connect greyed out | No interface selected | Scan, pick from dropdown |
| Scan finds nothing | USB permissions | udev rules (below), `plugdev` group, log out/in |
| Stuck "Trying to connect in firmware mode" | Drone in bootloader | Cold boot (recovery) tab |
| Scan empty after flashing | Address not default; scan uses the Address box | Read address over USB, type it in the box |
| New address not found | Needs Write + reboot + same value in the box | Do all three |
| "No input-device found" | No gamepad | Not needed |

udev rules (`/etc/udev/rules.d/99-bitcraze.rules`), then `sudo udevadm control
--reload-rules && sudo udevadm trigger`, user in `plugdev`, log out/in:

```
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"
```

---

## 16. Open questions (ask; don't assume)

1. Is the drone PC (`alijodat`) native Ubuntu or a VM? (A VM needs bridged networking to
   see the robot.)
2. Are Bilal and the drone operator the same person / PC? Which PC will run `cf_fleet`?
3. How is cflib installed on the drone PC (system or venv) and which version?
   `pip3 show cflib cfclient`.
4. Crazyradio model: PA or 2.0?
5. Is the second drone flashed (`cf21bl` 2026.08) and has its address been changed?
6. Are there other Crazyflies at the event on channel 80? (Drone 1's sticker says "03".)
7. What floor will the demo be on, and are mats available?
8. Exact shape of `/objects/json` (`ros2 topic echo /objects/json --once`).
9. Arena size and layout: where is mark A, the robot's line, objects, safe zone, drone bases?
   Measure and put them in the YAML.
10. Which YOLO classes will the props be? (Default list: bottle, cup, teddy bear, sports
    ball, backpack.)
11. Does the team have a Multi-ranger deck? (Not needed now.)
12. Has straight_avoid v3.1 been copied to and tested on the robot?

---

## 17. Decision log (why things are the way they are)

| Decision | Why |
|---|---|
| Plain cflib, not Crazyswarm2 | Crazyswarm2 is built around motion capture; Flow-deck absolute flying is awkward there. A small node is simpler for 1–2 drones. |
| High-level commander (`takeoff`/`go_to`/`land`) | The drone plans and stabilises its own path; the PC only sends goals. |
| One `cf_fleet` process for both drones | Only one program can open the radio; one process can talk to several drones on one channel. |
| Routes in **map** coordinates; conversion inside `cf_fleet` | Mission logic never deals with drone frames; one origin parameter per drone. |
| Estimator reset once at start + `/fleet/rezero` | Multi-leg jobs and return-home need a stable origin; re-zero only when a human puts the drone back on its mark. |
| Hover check + arrival check | From the drone operator's Phase 1 plan: never fly a mission on a drifting estimate; never land "blind" without reporting. |
| Simulated pickup (dip + RViz rope) | No gripper; anything hanging under the drone blocks the Flow deck; 40 g payload limit. Honest framing in the pitch. |
| Dip **beside** the object (0.2 m toward the zone) | Flying over a tall object fools the Flow deck's height and risks a collision. |
| Different cruise heights | Cheap, reliable collision avoidance without coordination. |
| Closest-pair task allocation | Simple, explainable "swarm" behaviour; uses both drones well. |
| Benching a failed drone | The other drone carries on; good fault-tolerance story. |
| Return home after each job | The drone ends near its known mark; easy to re-zero; predictable for the demo. |
| Mission paused by any e-stop | Nobody gets surprised by a drone taking off right after an emergency. |
| Robot keeps searching independently | No coupling = fewer failure points; the robot's straight_avoid already works. |
| Fast DDS + zenoh bridge for WSL; SLAM on the robot | Win10 WSL can't join DDS; laptop SLAM over Wi-Fi smeared the map. |
| Multi-ranger exploration plan (drone operator's Phases 2–4) dropped | The robot's camera + YOLO already finds and locates objects. |

---

## 18. Future work (after the hackathon)

- Real pickup: a small magnet or servo gripper (≤ 40 g) mounted so it doesn't cover the
  Flow deck.
- Lighthouse positioning for absolute, drift-free coordinates and precise docking.
- Multi-ranger on top: obstacle safety stop and the drone confirming the object itself.
- Robot pause topic in straight_avoid so the robot waits while a drone works nearby.
- Camera calibration for better object positions; Nav2 patrols instead of a straight line.
- Battery-aware scheduling and charging-dock rotation.

---

## Appendix: full source

The three files below are the exact versions in `cf_mission_bundle.zip`.

### A.1 `cf_fleet.py`

```python
#!/usr/bin/env python3
"""cf_fleet: fly one or more Crazyflies on ONE Crazyradio, in the robot's map frame.

Per drone (name = cf1, cf2, ...):
  in   /<name>/route   std_msgs/String, JSON like:
         {"job": "bottle1-try1",
          "waypoints": [{"x": 1.0, "y": -0.3, "action": "pick"},
                        {"x": -0.5, "y": 1.5, "action": "drop"},
                        {"action": "home"}]}
       Coordinates are MAP coordinates (metres, same as the robot's SLAM map).
       pick / drop = dip down, wait, climb back (the pickup itself is simulated).
       home        = fly back over the drone's own tape mark.
       go          = just fly there.
       hover       = stay where it is for "t" seconds (hover test).
       The drone always lands at the last waypoint.
  out  /<name>/status  std_msgs/String, JSON:
         {name, state, phase, job, done, x, y, z, vbat, miss, msg}
         state: idle | busy | failed      phase: landed | takeoff | to_pick | picking | ...
         done:  actions finished in the current job, e.g. ["pick"]
         miss:  worst distance (m) between a target and where the drone thinks it got to
  out  /<name>/pose    geometry_msgs/PointStamped in "map"
  out  TF  map -> <name>
Fleet-wide:
  in   /fleet/land_all std_msgs/Empty:  every drone lands where it is.
  in   /fleet/rezero   std_msgs/String: "cf1" or "all". Use after you put a landed drone
                       back on its tape mark by hand: its (0, 0) is set again.

Each drone's (0, 0) is set when this node starts. So start the node with every drone
sitting on its tape mark (its "origin" parameter), facing along its heading line.
Routes end with "home", so the drones come back there by themselves.

Safety checks (from the Flow-deck lessons):
  * After take-off the drone hovers hover_check seconds. If its position wanders more
    than hover_tol, it lands at once: that is almost always a bad floor.
  * After each leg it compares where it thinks it is with the target (arrive_tol),
    waits up to arrive_wait seconds for it to settle, and reports any miss.

sim:=true runs fake drones (no radio needed) to rehearse the whole mission.
"""
import json
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class Aborted(Exception):
    """Raised inside a flight when /fleet/land_all (or shutdown) asks to stop."""


class Drifting(Exception):
    """Raised when the hover check after take-off fails."""


# --------------------------------------------------------------------------- backends

class SimBackend:
    """Fake drone. Moves in straight lines with the timing it was asked for."""

    def __init__(self):
        self._lock = threading.Lock()
        self._a = (0.0, 0.0, 0.0)
        self._b = (0.0, 0.0, 0.0)
        self._t0 = 0.0
        self._d = 1e-6
        self.vbat = 4.0
        self.wobble = 0.0  # tests: fake position noise while airborne (metres)

    def position(self):
        with self._lock:
            s = min(1.0, max(0.0, (time.monotonic() - self._t0) / self._d))
            p = [a + (b - a) * s for a, b in zip(self._a, self._b)]
        if self.wobble and p[2] > 0.05:
            p[0] += self.wobble * math.sin(time.monotonic() * 7.0)
        return tuple(p)

    def go_to(self, x, y, z, duration):
        start = self.position()
        with self._lock:
            self._a, self._b = start, (x, y, z)
            self._t0, self._d = time.monotonic(), max(duration, 1e-3)

    def takeoff(self, height, duration):
        x, y, _ = self.position()
        self.go_to(x, y, height, duration)

    def land(self, duration):
        x, y, _ = self.position()
        self.go_to(x, y, 0.0, duration)

    def rezero(self):
        with self._lock:
            self._a = self._b = (0.0, 0.0, 0.0)
            self._d = 1e-6

    def arm(self, on):
        pass

    def stop(self):
        pass

    def close(self):
        pass


class RealBackend:
    """Real Crazyflie through cflib's high-level commander (drone plans its own path)."""

    def __init__(self, uri, deck, logger):
        from cflib.crazyflie import Crazyflie
        from cflib.crazyflie.log import LogConfig
        from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
        from cflib.utils.reset_estimator import reset_estimator

        self.log = logger
        self._reset = reset_estimator
        self._lock = threading.Lock()
        self._pos = (0.0, 0.0, 0.0)
        self.vbat = 0.0
        cache = os.path.expanduser('~/.cache/cf_fleet')
        os.makedirs(cache, exist_ok=True)
        self.scf = SyncCrazyflie(uri, cf=Crazyflie(rw_cache=cache))
        self.scf.open_link()
        self.scf.wait_for_params()
        self.cf = self.scf.cf
        if deck and int(float(self.cf.param.get_value(f'deck.{deck}'))) != 1:
            self.scf.close_link()
            raise RuntimeError(f'{uri}: positioning deck "{deck}" not detected')
        lc = LogConfig(name='fleet', period_in_ms=100)
        for v in ('stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z', 'pm.vbat'):
            lc.add_variable(v, 'float')
        self.cf.log.add_config(lc)
        lc.data_received_cb.add_callback(self._on_log)
        lc.start()
        self.log.info(f'{uri}: setting (0, 0) here, keep the drone still ...')
        self._reset(self.cf)
        self.hlc = self.cf.high_level_commander

    def _on_log(self, _ts, data, _conf):
        with self._lock:
            self._pos = (data['stateEstimate.x'], data['stateEstimate.y'],
                         data['stateEstimate.z'])
            self.vbat = data['pm.vbat']

    def position(self):
        with self._lock:
            return self._pos

    def go_to(self, x, y, z, duration):
        self.hlc.go_to(x, y, z, 0.0, duration, relative=False)

    def takeoff(self, height, duration):
        self.hlc.takeoff(height, duration)

    def land(self, duration):
        self.hlc.land(0.0, duration)

    def rezero(self):
        self._reset(self.cf)

    def arm(self, on):
        try:
            self.cf.supervisor.send_arming_request(on)
        except AttributeError:
            self.log.warn('This cflib has no send_arming_request. Update cflib '
                          '(the Brushless will not spin its motors without arming).')

    def stop(self):
        self.hlc.stop()

    def close(self):
        self.scf.close_link()


# --------------------------------------------------------------------------- one drone

class Drone:

    def __init__(self, node, name, backend, origin, height):
        self.node, self.name, self.cf = node, name, backend
        self.ox, self.oy = float(origin[0]), float(origin[1])
        self.oyaw = math.radians(float(origin[2]))
        self.height = float(height)
        self.lock = threading.Lock()
        self.state, self.phase, self.job, self.done, self.msg = 'idle', 'landed', '', [], ''
        self.miss = 0.0
        self.flying = False
        self.abort = threading.Event()
        self.thread = None
        self.status_pub = node.create_publisher(String, f'/{name}/status', 10)
        self.pose_pub = node.create_publisher(PointStamped, f'/{name}/pose', 10)
        node.create_subscription(String, f'/{name}/route', self.on_route, 10)

    # map = origin + rotate(yaw) * local
    def to_local(self, x, y):
        dx, dy = x - self.ox, y - self.oy
        c, s = math.cos(self.oyaw), math.sin(self.oyaw)
        return c * dx + s * dy, -s * dx + c * dy

    def to_map(self, lx, ly):
        c, s = math.cos(self.oyaw), math.sin(self.oyaw)
        return self.ox + c * lx - s * ly, self.oy + s * lx + c * ly

    def map_position(self):
        lx, ly, z = self.cf.position()
        x, y = self.to_map(lx, ly)
        return x, y, z

    def set(self, **kw):
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)
        if 'phase' in kw:
            self.node.get_logger().info(f'{self.name}: {self.phase}')

    def status(self):
        x, y, z = self.map_position()
        with self.lock:
            return {'name': self.name, 'state': self.state, 'phase': self.phase,
                    'job': self.job, 'done': list(self.done),
                    'x': round(x, 3), 'y': round(y, 3), 'z': round(z, 3),
                    'vbat': round(float(self.cf.vbat), 2), 'miss': round(self.miss, 3),
                    'msg': self.msg}

    def busy(self):
        return self.thread is not None and self.thread.is_alive()

    # ---- incoming route
    def on_route(self, msg):
        log = self.node.get_logger()
        try:
            req = json.loads(msg.data)
            job = str(req.get('job', ''))
            wps = req['waypoints']
            if not isinstance(wps, list) or not wps:
                raise ValueError('no waypoints')
        except Exception as e:
            log.warn(f'{self.name}: bad route message: {e}')
            return
        if self.busy():
            log.warn(f'{self.name}: busy, route "{job}" ignored')
            return
        try:
            err = self.node.check_route(self, wps)
        except Exception as e:
            err = f'bad waypoint: {e}'
        if err:
            log.warn(f'{self.name}: route "{job}" rejected: {err}')
            self.set(state='failed', job=job, done=[], msg=f'rejected: {err}')
            return
        self.abort.clear()
        self.set(state='busy', phase='takeoff', job=job, done=[], msg='', miss=0.0)
        self.thread = threading.Thread(target=self.run, args=(wps,), daemon=True)
        self.thread.start()

    def rezero(self):
        if self.busy() or self.flying:
            self.node.get_logger().warn(f'{self.name}: cannot re-zero while flying')
            return
        self.set(phase='rezeroing')
        self.thread = threading.Thread(target=self._rezero, daemon=True)
        self.thread.start()

    def _rezero(self):
        try:
            self.cf.rezero()
            self.set(state='idle', phase='landed', msg='re-zeroed on its mark')
        except Exception as e:
            self.set(state='failed', phase='landed', msg=f're-zero failed: {e}')

    # ---- flying (runs in its own thread)
    def wait(self, seconds):
        if self.abort.wait(seconds):
            raise Aborted()

    def watch(self, cx, cy, seconds):
        """Stay put for `seconds`; return the biggest x/y wander from (cx, cy)."""
        worst = 0.0
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.wait(0.1)
            x, y, _ = self.cf.position()
            worst = max(worst, math.hypot(x - cx, y - cy))
        return worst

    def fly(self, lx, ly, z, duration=None):
        n = self.node
        if duration is None:
            cx, cy, cz = self.cf.position()
            dist = math.sqrt((lx - cx) ** 2 + (ly - cy) ** 2 + (z - cz) ** 2)
            duration = max(dist / n.speed, n.min_leg_time)
        self.cf.go_to(lx, ly, z, duration)
        self.wait(duration + n.settle_time)
        # Did it get there (as far as the drone itself knows)? Give it a moment.
        end = time.monotonic() + n.arrive_wait
        while True:
            x, y, _ = self.cf.position()
            miss = math.hypot(x - lx, y - ly)
            if miss <= n.arrive_tol or time.monotonic() >= end:
                break
            self.wait(0.1)
        if miss > n.arrive_tol:
            n.get_logger().warn(f'{self.name}: {miss:.2f} m short of its target')
        with self.lock:
            self.miss = max(self.miss, miss)

    def run(self, wps):
        n = self.node
        try:
            self.cf.arm(True)
            self.wait(n.arm_time)
            gx, gy, _ = self.cf.position()
            self.flying = True
            self.cf.takeoff(self.height, n.takeoff_time)
            self.wait(n.takeoff_time + n.settle_time)
            if n.hover_check > 0.0:
                self.set(phase='hover_check')
                worst = self.watch(gx, gy, n.hover_check)
                if worst > n.hover_tol:
                    raise Drifting(f'drifted {worst:.2f} m right after take-off '
                                   '(check the floor and light)')
            for wp in wps:
                act = wp.get('action', 'go')
                if act == 'hover':
                    self.set(phase='hovering')
                    cx, cy, _ = self.cf.position()
                    worst = self.watch(cx, cy, float(wp.get('t', 5.0)))
                    self.set(msg=f'hover test: wandered up to {worst:.2f} m')
                    with self.lock:
                        self.done.append(act)
                    continue
                if act == 'home':
                    lx, ly = 0.0, 0.0
                else:
                    lx, ly = self.to_local(float(wp['x']), float(wp['y']))
                self.set(phase=f'to_{act}')
                self.fly(lx, ly, self.height)
                if act in ('pick', 'drop'):
                    self.set(phase='picking' if act == 'pick' else 'dropping')
                    self.fly(lx, ly, min(n.dip_height, self.height), n.dip_time)
                    self.wait(n.dwell)
                    self.fly(lx, ly, self.height, n.dip_time)
                with self.lock:
                    self.done.append(act)
            self.set(phase='landing')
            self.cf.land(n.land_time)
            self.wait(n.land_time + n.settle_time)
            self.cf.stop()
            self.cf.arm(False)
            self.flying = False
            self.set(state='idle', phase='landed')
        except Aborted:
            self.land_here('aborted')
        except Drifting as e:
            n.get_logger().error(f'{self.name}: {e}. Landing.')
            self.land_here(str(e))
        except Exception as e:
            n.get_logger().error(f'{self.name}: flight error: {e}')
            self.land_here(f'error: {e}')

    def land_here(self, why):
        n = self.node
        try:
            if self.flying:
                self.set(phase='landing')
                self.cf.land(n.land_time)
                time.sleep(n.land_time + n.settle_time)
                self.cf.stop()
            self.cf.arm(False)
        except Exception as e:
            n.get_logger().error(f'{self.name}: landing error: {e}')
        self.flying = False
        self.set(state='failed', phase='landed', msg=why)


# --------------------------------------------------------------------------- the node

class Fleet(Node):

    def __init__(self):
        super().__init__('cf_fleet')
        p = self.declare_parameter
        p('sim', False)
        p('drones', ['cf1'])
        p('deck', 'bcFlow2')
        p('speed', 0.2)            # m/s average along each leg (slower = more accurate)
        p('dip_height', 0.25)      # height of the pick / drop dip
        p('dwell', 2.0)            # seconds spent down at the dip
        p('dip_time', 1.5)
        p('takeoff_time', 4.0)     # same gentle timings as the tested fly_to_target
        p('land_time', 4.0)
        p('arm_time', 1.0)
        p('min_leg_time', 1.0)
        p('settle_time', 0.5)
        p('hover_check', 2.0)      # seconds of hover after take-off (0 = skip)
        p('hover_tol', 0.15)       # land if it wanders more than this during the check
        p('arrive_tol', 0.15)      # "arrived" if within this of the target
        p('arrive_wait', 2.0)      # extra seconds allowed to arrive
        p('fence', [-3.0, 3.0, -3.0, 3.0])   # xmin, xmax, ymin, ymax (map metres)
        p('max_leg', 4.0)                     # refuse any single leg longer than this

        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.sim = bool(g('sim'))
        for k in ('speed', 'dip_height', 'dwell', 'dip_time', 'takeoff_time', 'land_time',
                  'arm_time', 'min_leg_time', 'settle_time', 'hover_check', 'hover_tol',
                  'arrive_tol', 'arrive_wait', 'max_leg'):
            setattr(self, k, float(g(k)))
        self.fence = [float(v) for v in g('fence')]
        deck = g('deck')

        names = [str(n) for n in g('drones')]
        if not self.sim:
            import cflib.crtp
            cflib.crtp.init_drivers()
        self.drones = []
        try:
            for i, name in enumerate(names):
                p(f'{name}.uri', f'radio://0/80/2M/E7E7E7E70{i + 1}')
                p(f'{name}.origin', [0.0, 0.5 * (i + 1), 0.0])  # x, y, facing in degrees
                p(f'{name}.height', 0.5 + 0.3 * i)  # different heights: no mid-air crash
                if self.sim:
                    backend = SimBackend()
                else:
                    self.get_logger().info(f'{name}: connecting {g(f"{name}.uri")} ...')
                    backend = RealBackend(g(f'{name}.uri'), deck, self.get_logger())
                d = Drone(self, name, backend, g(f'{name}.origin'), g(f'{name}.height'))
                self.drones.append(d)
                self.get_logger().info(
                    f'{name}: ready{" (SIM)" if self.sim else ""}, base at map '
                    f'({d.ox:.2f}, {d.oy:.2f}), facing {math.degrees(d.oyaw):.0f} deg, '
                    f'cruise {d.height:.2f} m')
        except Exception:
            for d in self.drones:
                d.cf.close()
            raise

        self.tf = TransformBroadcaster(self)
        self.create_subscription(Empty, '/fleet/land_all', self.on_land_all, 10)
        self.create_subscription(String, '/fleet/rezero', self.on_rezero, 10)
        self.create_timer(0.1, self.tick)

    def check_route(self, d, wps):
        px, py, _ = d.map_position()
        xmin, xmax, ymin, ymax = self.fence
        for wp in wps:
            act = wp.get('action', 'go')
            if act not in ('go', 'pick', 'drop', 'home', 'hover'):
                return f'unknown action "{act}"'
            if act == 'hover':
                t = float(wp.get('t', 5.0))
                if not 0.0 < t <= 120.0:
                    return 'hover time must be between 0 and 120 s'
                continue
            if act == 'home':
                x, y = d.ox, d.oy
            else:
                x, y = float(wp['x']), float(wp['y'])
                if not (math.isfinite(x) and math.isfinite(y)):
                    return 'waypoint is not a number'
            if not (xmin <= x <= xmax and ymin <= y <= ymax):
                return f'({x:.2f}, {y:.2f}) is outside the fence'
            if math.hypot(x - px, y - py) > self.max_leg:
                return f'leg to ({x:.2f}, {y:.2f}) is longer than {self.max_leg} m'
            px, py = x, y
        return ''

    def on_land_all(self, _msg):
        self.get_logger().warn('LAND ALL')
        for d in self.drones:
            d.abort.set()

    def on_rezero(self, msg):
        want = msg.data.strip()
        for d in self.drones:
            if want in ('all', d.name):
                d.rezero()

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        tfs = []
        for d in self.drones:
            st = d.status()
            sm = String()
            sm.data = json.dumps(st)
            d.status_pub.publish(sm)

            ps = PointStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = 'map'
            ps.point.x, ps.point.y, ps.point.z = float(st['x']), float(st['y']), float(st['z'])
            d.pose_pub.publish(ps)

            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'map'
            t.child_frame_id = d.name
            t.transform.translation.x = ps.point.x
            t.transform.translation.y = ps.point.y
            t.transform.translation.z = ps.point.z
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(d.oyaw / 2.0)
            t.transform.rotation.w = math.cos(d.oyaw / 2.0)
            tfs.append(t)
        self.tf.sendTransform(tfs)

    def shutdown(self):
        for d in self.drones:
            d.abort.set()
        for d in self.drones:
            if d.thread is not None:
                d.thread.join(timeout=self.land_time + 3.0)
        for d in self.drones:
            try:
                d.cf.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Fleet()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()  # lands anything still flying
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### A.2 `mission.py`

```python
#!/usr/bin/env python3
"""mission: the search-and-rescue "referee".

The robot searches and finds objects (YOLO -> /objects/json, map frame).
For each object this node gives a job to the nearest free drone:
  fly to the object -> dip ("pick up") -> fly to the safe zone -> dip ("drop")
  -> fly back to base -> land.
The pickup is simulated. RViz shows the package hanging under the drone on a
rope and then sitting in the safe zone, so the audience sees the whole story.

  in   /objects/json     std_msgs/String  (from object_mapper)
  in   /<drone>/status   std_msgs/String  (from cf_fleet)
  in   /mission/cmd      std_msgs/String  start | pause | land | reset | status
  in   /fleet/land_all   std_msgs/Empty   (anyone's e-stop also pauses the mission)
  out  /<drone>/route    std_msgs/String  (to cf_fleet)
  out  /mission/markers  visualization_msgs/MarkerArray  (add this in RViz)
  out  /mission/events   std_msgs/String  (human-readable log, nice on screen)
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

RED = (1.0, 0.25, 0.2, 1.0)
AMBER = (1.0, 0.7, 0.1, 1.0)
GREEN = (0.2, 0.85, 0.35, 1.0)
GRAY = (0.6, 0.6, 0.6, 0.8)
BLUE = (0.2, 0.5, 1.0, 1.0)
WHITE = (1.0, 1.0, 1.0, 1.0)
ZONE = (0.2, 0.85, 0.35, 0.25)


class Item:
    """One found object. States: seen -> waiting -> assigned -> carried -> delivered,
    or skipped. x, y stay at the object's real spot (it never really moves), so YOLO
    seeing it again later does not create a duplicate."""

    def __init__(self, iid, cls, x, y, conf):
        self.id, self.cls, self.x, self.y, self.conf = iid, cls, x, y, conf
        self.n = 1
        self.state = 'seen'
        self.drone = ''
        self.job = ''
        self.slot = None
        self.tries = 0
        self.sent_at = 0.0
        self.resends = 0


class Mission(Node):

    def __init__(self):
        super().__init__('mission')
        p = self.declare_parameter
        p('drones', ['cf1', 'cf2'])
        p('objects_topic', '/objects/json')
        p('classes', 'bottle,cup,teddy bear,sports ball,backpack')  # "*" = any class
        p('min_conf', 0.4)
        p('min_sightings', 3)        # YOLO must see it this many times before it counts
        p('merge_radius', 0.35)      # detections closer than this are the same object
        p('drop_zone', [0.0, 1.5])   # safe zone centre in map metres
        p('zone_size', 0.6)
        p('slot_spacing', 0.2)       # packages are laid out in a 3x3 grid in the zone
        p('approach_offset', 0.2)    # dip this far beside the object, not on top of it
        p('return_home', True)       # after the drop, fly back to the drone's base
        p('min_vbat', 3.4)           # don't start a job below this battery voltage
        p('auto_start', False)
        p('test_objects', '')        # e.g. "bottle:1.0,-0.3; cup:2.0,0.4" (rehearsal)

        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.drones = [str(d) for d in g('drones')]
        cls = [c.strip() for c in str(g('classes')).split(',') if c.strip()]
        self.classes = set() if '*' in cls else set(cls)
        self.min_conf = float(g('min_conf'))
        self.min_sightings = int(g('min_sightings'))
        self.merge_radius = float(g('merge_radius'))
        self.zone = (float(g('drop_zone')[0]), float(g('drop_zone')[1]))
        self.zone_size = float(g('zone_size'))
        self.slot_spacing = float(g('slot_spacing'))
        self.offset = float(g('approach_offset'))
        self.return_home = bool(g('return_home'))
        self.min_vbat = float(g('min_vbat'))
        self.running = bool(g('auto_start'))

        self.items = []
        self.next_id = 1
        self.slots_used = 0
        self.active = {}       # drone -> Item it is working on
        self.status = {}       # drone -> last status dict
        self.low_batt_warned = set()
        self.benched = set()   # drones that failed (e.g. drifted) stay down until 'start'
        self.completed = False

        self.events = self.create_publisher(String, '/mission/events', 10)
        self.markers = self.create_publisher(MarkerArray, '/mission/markers', 10)
        self.land_pub = self.create_publisher(Empty, '/fleet/land_all', 10)
        self.route_pubs = {d: self.create_publisher(String, f'/{d}/route', 10)
                           for d in self.drones}
        for d in self.drones:
            self.create_subscription(String, f'/{d}/status',
                                     lambda m, d=d: self.on_status(d, m), 10)
        self.create_subscription(String, g('objects_topic'), self.on_objects, 10)
        self.create_subscription(String, '/mission/cmd', self.on_cmd, 10)
        self.create_subscription(Empty, '/fleet/land_all', self.on_land_all, 10)
        self.create_timer(0.5, self.tick)
        self.create_timer(0.2, self.publish_markers)

        for part in str(g('test_objects')).split(';'):
            self.add_test_object(part)
        hint = 'Running.' if self.running else "Send 'start' on /mission/cmd."
        self.event(f'Mission ready. Drones: {", ".join(self.drones)}. Safe zone at '
                   f'({self.zone[0]:.2f}, {self.zone[1]:.2f}). {hint}')

    # ------------------------------------------------------------------ helpers
    def event(self, text):
        self.get_logger().info(text)
        m = String()
        m.data = text
        self.events.publish(m)

    def count(self, *states):
        return sum(1 for it in self.items if it.state in states)

    def add_test_object(self, part):
        part = part.strip()
        if not part:
            return
        try:
            cls, _, xy = part.rpartition(':')
            x, y = (float(v) for v in xy.split(','))
        except ValueError:
            self.get_logger().warn(f'test_objects: cannot read "{part}"')
            return
        it = Item(self.next_id, cls.strip() or 'package', x, y, 1.0)
        self.next_id += 1
        self.items.append(it)
        self.accept(it)

    def accept(self, it):
        it.state = 'waiting'
        self.completed = False
        self.event(f'Found {it.cls} #{it.id} at ({it.x:.2f}, {it.y:.2f})')

    # ------------------------------------------------------------------ inputs
    def on_objects(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(data, dict):
            data = data.get('objects', [data] if 'x' in data else [])
        if not isinstance(data, list):
            return
        for o in data:
            try:
                cls = str(o.get('cls', o.get('class', o.get('label', 'object'))))
                x, y = float(o['x']), float(o['y'])
                conf = float(o.get('conf', o.get('score', 1.0)))
            except (KeyError, TypeError, ValueError, AttributeError):
                continue
            self.add_detection(cls, x, y, conf)

    def add_detection(self, cls, x, y, conf):
        if not (math.isfinite(x) and math.isfinite(y)) or conf < self.min_conf:
            return
        if self.classes and cls not in self.classes:
            return
        best, best_d = None, self.merge_radius
        for it in self.items:
            dd = math.hypot(it.x - x, it.y - y)
            if dd < best_d:
                best, best_d = it, dd
        if best is None:
            half = self.zone_size / 2.0
            if abs(x - self.zone[0]) < half and abs(y - self.zone[1]) < half:
                return  # ignore anything inside the safe zone
            it = Item(self.next_id, cls, x, y, conf)
            self.next_id += 1
            self.items.append(it)
            if it.n >= self.min_sightings:
                self.accept(it)
            return
        if best.state in ('seen', 'waiting'):
            k = min(best.n, 20)  # running average, so the position settles
            best.x = (best.x * k + x) / (k + 1)
            best.y = (best.y * k + y) / (k + 1)
            best.n += 1
            best.conf = max(best.conf, conf)
            if best.state == 'seen' and best.n >= self.min_sightings:
                self.accept(best)

    def on_status(self, d, msg):
        try:
            st = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        st['_t'] = time.monotonic()
        self.status[d] = st
        it = self.active.get(d)
        if it is None or st.get('job') != it.job:
            return
        done = st.get('done', [])
        if it.state == 'assigned' and 'pick' in done:
            it.state = 'carried'
            self.event(f'{d} picked up {it.cls} #{it.id}')
        if it.state == 'carried' and 'drop' in done:
            it.state = 'delivered'
            self.event(f'{d} delivered {it.cls} #{it.id} to the safe zone '
                       f'({self.count("delivered")}/{self.count_found()})')
        if st.get('state') in ('idle', 'failed') and st.get('phase') == 'landed':
            del self.active[d]
            if it.state == 'delivered':
                self.event(f'{d} landed{" back at base" if self.return_home else ""}')
                return
            why = st.get('msg') or 'unknown reason'
            it.drone = ''
            if not why.startswith('rejected') and why != 'aborted':
                # Drift or an error: keep this drone on the ground, the others carry on.
                self.benched.add(d)
                self.event(f'{d} is benched ({why}). The other drones carry on. '
                           "Fix it, then send 'start' again.")
            if why.startswith('rejected') or it.tries >= 2:
                it.state = 'skipped'
                self.event(f'Skipping {it.cls} #{it.id}: {why}')
            else:
                it.state = 'waiting'
                self.event(f'{d} could not finish {it.cls} #{it.id} ({why}); will retry')

    def on_cmd(self, msg):
        c = msg.data.strip().lower()
        if c == 'start':
            self.running = True
            if self.benched:
                self.event(f'Back in service: {", ".join(sorted(self.benched))}')
                self.benched.clear()
            self.event('Mission started')
        elif c == 'pause':
            self.running = False
            self.event('Mission paused (drones in the air finish their job)')
        elif c == 'land':
            self.running = False
            self.land_pub.publish(Empty())
        elif c == 'reset':
            if self.active:
                self.event('Cannot reset while drones are working')
                return
            self.items, self.next_id, self.slots_used = [], 1, 0
            self.completed = False
            clear = Marker()
            clear.action = Marker.DELETEALL
            self.markers.publish(MarkerArray(markers=[clear]))
            self.event('Mission reset')
        elif c == 'status':
            self.event(f'running={self.running} found={self.count_found()} '
                       f'delivered={self.count("delivered")} '
                       f'busy={", ".join(self.active) or "none"} '
                       f'benched={", ".join(sorted(self.benched)) or "none"}')
        else:
            self.event(f'Unknown command "{c}" (start | pause | land | reset | status)')

    def on_land_all(self, _msg):
        if self.running:
            self.event('LAND ALL received: mission paused')
        self.running = False

    def count_found(self):
        return sum(1 for it in self.items if it.state not in ('seen', 'skipped'))

    # ------------------------------------------------------------------ the referee
    def drone_free(self, d, now):
        st = self.status.get(d)
        if st is None or now - st['_t'] > 2.0 or d in self.active or d in self.benched:
            return False
        if st.get('state') not in ('idle', 'failed') or st.get('phase') != 'landed':
            return False
        vbat = float(st.get('vbat', 0.0))
        if 0.1 < vbat < self.min_vbat:
            if d not in self.low_batt_warned:
                self.event(f'{d} battery low ({vbat:.2f} V): not sending it out')
                self.low_batt_warned.add(d)
            return False
        self.low_batt_warned.discard(d)
        return True

    def dist(self, d, it):
        st = self.status[d]
        return math.hypot(it.x - float(st.get('x', 0.0)), it.y - float(st.get('y', 0.0)))

    def next_slot(self):
        k = self.slots_used
        self.slots_used += 1
        col, row = k % 3 - 1, (k // 3) % 3 - 1
        return (self.zone[0] + col * self.slot_spacing, self.zone[1] + row * self.slot_spacing)

    def pick_point(self, it):
        vx, vy = self.zone[0] - it.x, self.zone[1] - it.y
        length = math.hypot(vx, vy)
        if length < 1e-3 or self.offset <= 0.0:
            return it.x, it.y
        off = min(self.offset, length / 2.0)
        return it.x + vx / length * off, it.y + vy / length * off

    def send_route(self, d, it):
        px, py = self.pick_point(it)
        wps = [{'x': round(px, 3), 'y': round(py, 3), 'action': 'pick'},
               {'x': round(it.slot[0], 3), 'y': round(it.slot[1], 3), 'action': 'drop'}]
        if self.return_home:
            wps.append({'action': 'home'})
        m = String()
        m.data = json.dumps({'job': it.job, 'waypoints': wps})
        self.route_pubs[d].publish(m)

    def assign(self, d, it, now):
        it.tries += 1
        it.state, it.drone = 'assigned', d
        it.job = f'{it.cls.replace(" ", "_")}{it.id}-try{it.tries}'
        if it.slot is None:
            it.slot = self.next_slot()
        it.sent_at, it.resends = now, 0
        self.active[d] = it
        self.send_route(d, it)
        self.event(f'{d} is going for {it.cls} #{it.id}')

    def tick(self):
        now = time.monotonic()
        # Jobs that were sent but the drone never started: resend twice, then give up.
        for d, it in list(self.active.items()):
            st = self.status.get(d)
            if st is not None and st.get('job') == it.job:
                continue
            if now - it.sent_at > 3.0:
                if it.resends < 2:
                    it.resends += 1
                    it.sent_at = now
                    self.send_route(d, it)
                else:
                    del self.active[d]
                    it.state, it.drone = 'waiting', ''
                    self.event(f'{d} did not answer; {it.cls} #{it.id} is waiting again')
        if not self.running:
            return
        # Give waiting objects to free drones: closest drone-object pair first.
        free = [d for d in self.drones if self.drone_free(d, now)]
        waiting = [it for it in self.items if it.state == 'waiting']
        while free and waiting:
            d, it = min(((d, it) for d in free for it in waiting),
                        key=lambda pair: self.dist(*pair))
            self.assign(d, it, now)
            free.remove(d)
            waiting.remove(it)
        if (not self.completed and not self.active and self.count('delivered') > 0
                and self.count('seen', 'waiting', 'assigned', 'carried') == 0):
            self.completed = True
            self.event(f'Mission complete: {self.count("delivered")} delivered to the safe zone')

    # ------------------------------------------------------------------ RViz
    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        ms = []

        def mk(ns, mid, mtype, x, y, z, scale, rgba, text=''):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns, m.id, m.type, m.action = ns, int(mid), mtype, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
            m.color.r, m.color.g, m.color.b, m.color.a = (float(v) for v in rgba)
            m.text = text
            ms.append(m)
            return m

        zx, zy = self.zone
        zs = self.zone_size
        mk('zone', 0, Marker.CUBE, zx, zy, 0.005, (zs, zs, 0.01), ZONE)
        mk('zone', 1, Marker.TEXT_VIEW_FACING, zx, zy, 0.45, (0.0, 0.0, 0.12), WHITE,
           f'Safe zone  {self.count("delivered")}/{self.count_found()}')

        for i, d in enumerate(self.drones):
            st = self.status.get(d)
            if st is None:
                continue
            x, y, z = float(st['x']), float(st['y']), float(st['z'])
            mk('drones', 2 * i, Marker.CYLINDER, x, y, z, (0.12, 0.12, 0.03), BLUE)
            mk('drones', 2 * i + 1, Marker.TEXT_VIEW_FACING, x, y, z + 0.15,
               (0.0, 0.0, 0.08), WHITE, f'{d}: {st.get("phase", "")}')

        labels = {'waiting': 'waiting', 'assigned': 'drone on the way',
                  'carried': 'being carried', 'delivered': 'safe', 'skipped': 'skipped'}
        colors = {'waiting': RED, 'assigned': AMBER, 'carried': AMBER,
                  'delivered': GREEN, 'skipped': GRAY}
        for it in self.items:
            if it.state == 'seen':
                continue
            x, y, z = it.x, it.y, 0.04
            st = self.status.get(it.drone)
            carried = it.state == 'carried' and st is not None
            if carried:
                x, y, z = float(st['x']), float(st['y']), max(float(st['z']) - 0.12, 0.04)
            elif it.state == 'delivered' and it.slot is not None:
                x, y = it.slot
            mk('items', 3 * it.id, Marker.CUBE, x, y, z, (0.08, 0.08, 0.08), colors[it.state])
            who = f' ({it.drone})' if it.state in ('assigned', 'carried') else ''
            mk('items', 3 * it.id + 1, Marker.TEXT_VIEW_FACING, x, y, z + 0.12,
               (0.0, 0.0, 0.07), WHITE, f'{it.cls} #{it.id}: {labels[it.state]}{who}')
            rope = mk('items', 3 * it.id + 2, Marker.LINE_LIST, 0.0, 0.0, 0.0,
                      (0.01, 0.0, 0.0), GRAY)
            if carried:
                rope.points = [Point(x=x, y=y, z=float(st['z'])), Point(x=x, y=y, z=z)]
            else:
                rope.action = Marker.DELETE
        self.markers.publish(MarkerArray(markers=ms))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Mission()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### A.3 `fleet_mission.yaml`

```yaml
# Parameters for cf_fleet and mission. Use with:  --ros-args --params-file ~/fleet_mission.yaml
# IMPORTANT: write every number with a decimal point (0.0, not 0), or ROS refuses it.
# All positions are MAP coordinates in metres (robot's SLAM map: (0, 0) = where SLAM started,
# x = the way the robot faced at start, y = to its left).

cf_fleet:
  ros__parameters:
    sim: false                 # true = fake drones, no radio needed
    drones: [cf1, cf2]         # first real flight: just [cf1]
    deck: bcFlow2
    speed: 0.2                 # m/s average along each leg (0.15 = colleague's tested value)
    dip_height: 0.25           # how low the drone dips to "pick up" / "drop"
    dwell: 2.0                 # seconds it stays down there
    takeoff_time: 4.0          # gentle, same as the fly_to_target that already flies
    land_time: 4.0
    hover_check: 2.0           # hover this long after take-off ...
    hover_tol: 0.15            # ... and land at once if it wanders more than this
    arrive_tol: 0.15           # warn if it ends more than this from a target
    fence: [-1.0, 3.5, -1.5, 2.0]   # xmin, xmax, ymin, ymax: never fly outside this box
    max_leg: 4.0               # refuse any single leg longer than this
    cf1:
      uri: radio://0/80/2M/E7E7E7E701   # the drone that already flies has this address
      origin: [0.0, 0.6, 0.0]           # its tape mark: x, y, facing (degrees, 0 = same as robot)
      height: 0.5                       # cruise height
    cf2:
      uri: radio://0/80/2M/E7E7E7E702   # set this on the second drone in cfclient
      origin: [0.0, 1.0, 0.0]
      height: 0.8                       # different height: the two drones can't collide

mission:
  ros__parameters:
    drones: [cf1, cf2]
    objects_topic: /objects/json
    classes: "bottle,cup,teddy bear,sports ball,backpack"   # "*" = any YOLO class
    min_conf: 0.4
    min_sightings: 3           # YOLO must see it 3 times before it counts
    merge_radius: 0.35         # closer than this = the same object
    drop_zone: [-0.6, 1.4]     # safe zone centre (tape a 60 cm square here)
    zone_size: 0.6
    approach_offset: 0.2       # dip 20 cm beside the object, not on top of it
    return_home: true          # after the drop, fly back to its tape mark and land
    min_vbat: 3.4              # don't send a drone out below this battery voltage
    auto_start: false          # wait for "start" on /mission/cmd
    test_objects: ""           # rehearsal without YOLO, e.g. "bottle:1.2,-0.4; cup:2.0,-0.3"
```