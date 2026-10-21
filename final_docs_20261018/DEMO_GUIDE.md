# Disaster Bot — Stationary Demonstration Guide

Source of truth: `~/disaster_bot_test_logs/detector_eval_20261018_000055/FINAL_DEMO_CHECKLIST.md`
(rehearsed and live-verified 2026-10-17 and 2026-10-18; corrected copy below).

> The robot **does not move** in this demo. Scene changes are Gazebo teleports.
> No autonomous-navigation, safety-controller, or obstacle-detector node is started.
> YOLO-World is **optional and experimental**. Segment A (YOLO11n) is the primary demo.

---

## Prerequisites

- Ubuntu 26.04, ROS 2 Lyrical, Gazebo Sim 10.4.0
- TurtleBot3 Gazebo packages installed
- `~/ros2_ws/yolo_env/` — Ultralytics Python environment
- `~/ros2_ws/yolo11n.pt` — production weights
- Demo kit: `~/disaster_bot_test_logs/detector_eval_20261018_000055/demo/`
- Frozen files verified: `cd ~/ros2_ws && sha256sum -c evidence/SHA256SUMS.frozen --quiet && echo "frozen 7/7 OK"`

---

## Step 0 — Pre-flight checks

Run these in any terminal **before opening demo terminals**:

```bash
cat /sys/class/power_supply/AC/online
# Must print 1

pgrep -af "ros2 |ros2-daemon|gz sim|gz-sim|parameter_bridge|victim_|yolo_detector|autonomous_search|safety_controller|obstacle_detector" || echo "clean"
# Must print "clean"

cd ~/ros2_ws && sha256sum -c evidence/SHA256SUMS.frozen --quiet && echo "frozen 7/7 OK"
# Must print "frozen 7/7 OK"
```

---

## Step 1 — Source in every terminal

Open **four** terminals (T1–T4). In **every** terminal, run first:

```bash
source ~/disaster_bot_test_logs/detector_eval_20261018_000055/demo/demo_env.sh
```

This sets ROS domain 42 (localhost only), Gazebo partition `demo42`, YOLO offline mode, and `$DEMO`.

---

## Step 2 — Start simulation (T1)

```bash
ros2 launch $DEMO/site/launch/demo_sim.launch.py x_pose:=3.0 y_pose:=-6.6 yaw:=1.5708
```

Add `gui:=true` if you want the Gazebo window (costs CPU).
**Wait for:** `Entity creation successful`

---

## Step 3 — Start perception stack (T2, Segment A — YOLO11n)

```bash
ros2 launch $DEMO/demo_stack.launch.py
```

**Wait ~5 s for all five startup lines:**
- `YOLO Detector Started`
- `Detector model yolo11n.pt, confidence >= 0.5`
- `Victim Distance Estimator Started`
- `Victim detection guard started … exclusive_cluster_check False`
- `Victim localizer started`

---

## Step 4 — Start monitor (T3)

```bash
python3 -B $DEMO/watch.py
```

This prints one line per second showing `/cmd_vel` publisher count and message count, and odometry displacement.
**Expected line:** `cmd_vel msgs 0 pubs 0/0  odom moved 0 mm` — throughout the entire demo.

---

## Step 5 — Confirm configuration (T4)

```bash
ros2 node list
# Expected: /robot_state_publisher /ros_gz_bridge /ros_gz_image
#           /yolo_detector /victim_distance_estimator /victim_detection_guard
#           /victim_localizer /demo_watch
# Must NOT include: autonomous_search, safety_controller, obstacle_detector

ros2 topic info -v /victim_detections_nav
# Publisher: victim_detection_guard — Subscriber: victim_distance_estimator

ros2 topic info /cmd_vel
# TwistStamped, Publisher count: 0

ros2 param get /victim_detection_guard exclusive_cluster_check
# False

ros2 param get /yolo_detector model_path
# yolo11n.pt  (Segment A)
```

---

## Step 6 — Segment A: YOLO11n scenes

Run each command in T4. **Wait ~10 s after each scene change**, then observe T3 and run the check commands.

### Empty scene
```bash
$DEMO/scene.sh empty
```
Expected: 0 detections, 0 `/victim_distance` messages on T3.

### Standing person (~1.8 m)
```bash
$DEMO/scene.sh standing
```
Expected: detections forwarded; `/victim_distance` published.
```bash
ros2 topic echo /victim_distance --once
# range_m ~1.71, bearing ~0.0°  (validated: 1.707–1.709 m)
```

### Person behind debris
```bash
$DEMO/scene.sh debris
```
Expected: detections withheld by guard; 0 `/victim_distance`.
```bash
ros2 topic echo /victim_detection_guard --once
# reason: lidar_hit_below_box
```

### Standing person behind lying proxy
```bash
$DEMO/scene.sh lying
```
Expected: 0 detections forwarded (standing person behind the proxy is withheld); 0 distances.

**Point out:** `$DEMO/scene.sh lying_alone` → YOLO11n detects nothing for the lying proxy. This is a known limitation.

### Two standing people, separated 2.6 m
```bash
$DEMO/scene.sh pair
```
Expected: both detected and localized; `/victim_distance` shows ~2.65 m.

---

## Step 7 — Segment B: YOLO-World v2-s or v2-m (OPTIONAL, EXPERIMENTAL)

> State clearly: YOLO-World is experimental, slower, not the default, and not validated for navigation.

In T2: **Ctrl+C** the current stack. Wait until the process exits (up to 15 s; press Ctrl+C again if still running).

```bash
ros2 launch $DEMO/demo_stack.launch.py detector:=worldv2s   # 3.4× slower, offline-validated
ros2 launch $DEMO/demo_stack.launch.py detector:=worldv2m   # 2.6× slower, better lying@0.60, live-validated
```

Note: use the updated `demo_stack.launch.py` in `detector_eval_20261018_000055/demo/` (not `$DEMO/demo_stack.launch.py`), which passes `inference_imgsz:=480` for worldv2m.

Both **worldv2s** and **worldv2m** are live-validated (2026-10-18). **Recommend worldv2m at imgsz=480** for the best lying-proxy detection: oblique view 100% above nav gate; side view **46.4%** above nav gate (up from 23.5% at 640). v2-m+imgsz=480 runs at **3.8 Hz** live (vs 1.65 Hz at 640; vs ~3.5 Hz for v2-s).

**Wait for:** `Detector model …/yolov8m-worldv2.pt, confidence >= 0.2, imgsz=480` (about 5 s).
Detector rate: ~3.5 Hz (v2-s) or ~3.8 Hz (v2-m at imgsz=480).

Confirm: `ros2 param get /yolo_detector model_path` → shows the worldv2s weights path.

```bash
$DEMO/scene.sh standing        # same ~1.706 m
$DEMO/scene.sh lying_alone     # lying proxy now detected; distances ~1.34 m (conf < 0.60: navigation would ignore)
$DEMO/scene.sh lying_oblique   # lying proxy detected; ~1.13 m (some conf ≥ 0.60)
$DEMO/scene.sh lying           # lying proxy box forwarded; ~1.29 m (correct for lying figure)
$DEMO/scene.sh debris          # still withheld (guard behaviour unchanged)
```

Optional limitation demonstration:
```bash
$DEMO/scene.sh pair_facing
ros2 topic echo /victim_detections --once
# YOLO-World sometimes draws one merged box spanning both people (x≈153–201)
```

To switch back to YOLO11n: **Ctrl+C T2**, then `ros2 launch $DEMO/demo_stack.launch.py`.
**Never run both detectors at once.**

---

## Step 8 — Robot stays stationary (T4, at any point)

```bash
ros2 topic info /cmd_vel
# Publisher count: 0

ros2 topic echo /cmd_vel --once   # should time out (no messages)
```

T3 must show `cmd_vel msgs 0 pubs 0/0` and `odom moved 0 mm` on every line.

---

## Step 9 — Cleanup

In order: **Ctrl+C in T3, T2, T1** (each exits within ~2 s).

Expected shutdown noise: `exit code -2`, `parameter_bridge … exit code -11`, KeyboardInterrupt tracebacks. All are normal.

Then in T4:
```bash
ros2 daemon stop

pgrep -af "ros2-daemon|gz-sim|parameter_bridge|image_bridge|robot_state_publisher|victim_|yolo_detector|watch.py" || echo "clean"
```

If the daemon still appears after a second, re-run `pgrep`. It can take a moment to exit.

---

## Limitations to state during the demo

1. **Stationary simulation only.** Autonomous navigation with the guard active is **not validated**.
2. **Standing people only** are validated live. Lying results use rigid proxy figures; sitting and kneeling have no test evidence.
3. **Camera/LiDAR are not synchronised.** After a scene change, readings from the previous scene can appear for ~1.7 s. The 10 s wait is a precaution, not a fix.
4. **Stale targets.** The frozen navigation node keeps readings for 30 s. In a moving-robot setting, stale readings could cause wrong approaches.
5. **YOLO-World is optional and experimental.** v2-s is 3.4× slower (~3.5 Hz live); v2-m is 8.2× slower (~1.65 Hz live). Both make merged boxes over close people at very low confidence (v2-m merged-box conf 0.216 live, below 0.50 threshold). v2-m live-validated: oblique lying 100% above nav gate, side lying 23.5% above nav gate.
6. **Everything is simulation.** One world, one victim placement, one rigid person mesh.

---

## Evidence to capture during the demo

- T2 startup lines: detector model/threshold, guard `exclusive_cluster_check` value.
- Output of `ros2 node list`, `ros2 topic info -v /victim_detections_nav`, `ros2 topic info /cmd_vel`.
- T3 lines for each scene.
- `ros2 topic echo /victim_distance --once` (standing scene).
- `ros2 topic echo /victim_detection_guard --once` (debris scene).
- In Segment B: T3 lines for `lying_alone` next to Segment A's "0 detections" for the same scene.
