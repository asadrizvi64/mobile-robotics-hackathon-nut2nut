# What the TurtleBot side needs from the drone team

Oct 11, 2026 · from Asad (TurtleBot side), replying to "TurtleBot side: what the drone needs"

The TurtleBot side is ready as your note asks. SLAM was started on mark A, everything runs on domain 30, and `/objects/json` is live with `objects` in the robot bridge's allow-list. It reached the robot at 13:52. To finish the hand-off safely, please do or answer the points below. Each one ends with what we need back.

## 1. Confirm the topic arrives on your laptop (5 minutes)

On the drone laptop (with `export ROS_DOMAIN_ID=30`), run:

```bash
ros2 daemon stop; ros2 topic echo --full-length /objects/json std_msgs/msg/String --once
```

- Pass the message type explicitly. Without it, the CLI sometimes says "could not determine type".
- `--full-length` matters. Without it, the CLI cuts the string after 128 characters.
- You should see `{"stamp": ..., "objects": []}`. The list stays empty until the bottle is in front of the robot's camera.

**Need back:** a screenshot or paste of that output.

## 0. URGENT (16:55): hover offset = 0

The drone ended up **above the robot** instead of above the bottle. Your node hovers "about 0.3 m short of the object". The robot docks on the same side of the bottle it approached from, and that's usually the side your drone comes from too, so the hover point lands on the robot. **Please set the hover offset to 0** and hover directly above the published `x, y`.

Since v17, the robot also backs about 0.45 m away from the bottle right after measuring it, while it keeps publishing.

## 2. What arrives on `/objects/json` (v14, 16:30)

1. **The moment the robot's camera confirms a bottle**, the robot publishes it on `/objects/json`, 5 times per second. This is the camera position, which can be 10–20 cm off.
2. The robot drives to the bottle and keeps publishing. The position may improve along the way.
3. Docked about 6 cm from the bottle, it measures the bottle with its own lidar and **replaces the position with the precise one (1–3 cm), same `id`**. It keeps publishing that for 10 s.
4. Then it publishes `{"objects": []}` and moves on to the next bottle, `id` 2.

```json
{"objects": [{"id": 1, "cls": "bottle", "x": 1.21, "y": -0.28, "z": 0.12, "conf": 0.93}]}
```

**For best accuracy,** treat the **latest** position of each `id` as the truth, rather than averaging everything since the first message. Or wait until the position stops changing for 2 s.

**Need back:** confirm that your node keeps a target after it drops off the topic, and whether it uses the latest or the averaged position.

## 3. Agree on the shared frame: mark A and the axes

- **Map origin (0, 0)** is the robot's position on mark A when SLAM started.
- **Map +x** is the direction the robot faced on mark A. **Map +y** is 90° to its left (ROS convention, counter-clockwise).
- Units are metres.

**Need back:**
- Your drone start spot(s) in this frame, as `x, y, yaw`. How did you measure them (tape from mark A)?
- Which way the drone's nose points at take-off, relative to map +x.

## 4. One measured calibration test before any free flight

1. We place the bottle at a taped spot, measured from mark A, for example x = 1.5 m, y = +0.5 m.
2. We send you our `x, y` from `/objects/json`; it should be within about 10 cm.
3. You compute the drone-frame target and fly to it **without the bottle in place**, or at a safe height.
4. We tape-measure where the drone actually hovered.

**Need back:** the hover-position error. If it's more than about 20 cm, we check the start-spot offset and yaw before the demo.

## 5. Safety: arming and emergency stop

Your note says any message on `/objects/json` can make the drone take off. Our mapper publishes **continuously**. A bottle appears as soon as YOLO confirms it, about 1 s after it enters the camera view.

**Need back:**
- **How you arm and disarm.** Ideally, the drone only reacts while your node is armed by a person. Otherwise, start your mission node only when we agree, and we'll hide the bottle until then.
- **The exact emergency-land command,** for example `ros2 topic pub --once /fleet/land_all std_msgs/msg/Empty`. Who presses it during the demo?
- **Your geofence** in map coordinates (we had ±3 m in the earlier config). Is it still active?

## 6. Flight path and the robot

- The robot is about 0.2 m tall and drives inside the square. The bottle is about 0.3 m tall.
- You hover at 0.5 m, about 0.3 m short of the object.

**Need back:**
- Your cruise height on the way there.
- Which side "0.3 m short of the object" is measured from: the drone's approach direction, or a fixed side?
- That the path can't cross the robot at robot height.

## 7. Range and placement

You skip objects more than about 4 m from your start spot.

**Need back:** your start spot (see point 3), so we place the bottle within about 3 m of it, inside the robot's mapped square.

## 8. Wi-Fi (the hotspot is our bottleneck)

At 13:49 the robot ping was 255 ms with 10% loss. We cut the camera stream to 15 fps at JPEG quality 60.

**Please:**
- Don't subscribe to `/camera/...` images or run RViz with the camera on your laptop over the hotspot.
- Pause any big downloads, uploads or video calls on `bD` during the demo.
- If you use a Crazyradio, keep it on a high channel (e.g. 80, which is 2480 MHz), clear of Wi-Fi channels 1–11.

**Need back:** what else is running on the hotspot.

## 9. Flow deck limits

Position from the Flow deck drifts with distance and needs a textured, well-lit floor.

**Need back:**
- The position error you've seen after flying about 2–3 m.
- Whether the floor in the square works for the Flow deck (plain glossy floors are bad). If it's poor, we can lay down some tape or paper texture.

## 10. Optional, nice for the demo

A status topic from your side, for example `/drone/status` (`std_msgs/String`, values like `idle`, `flying_to x,y`, `hovering`, `landed`). We'd show it next to the map in our recording.

**Need back:** yes or no, and the topic name.

## Demo sequence (to agree)

1. The robot is on mark A, with SLAM, camera, bridge and mapper running. The drone sits on its start spot, disarmed.
2. Both sides confirm `/objects/json` is arriving (point 1).
3. The drone team arms.
4. The robot drives (`straight_avoid`). The bottle comes into view, gets confirmed and is published.
5. The drone flies to the bottle, hovers, then lands or returns.
6. Emergency stop is ready on both sides throughout:
   - **robot:** Ctrl+C, or `ros2 topic pub --times 5 /cmd_vel geometry_msgs/msg/Twist "{}"`;
   - **drone:** land_all.
