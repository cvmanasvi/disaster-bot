# Disaster Bot — Architecture (production pipeline)

Verified against the source on 2026-10-14. The relevant files are unchanged since 2026-10-08 (SHA-256 in `evidence/SHA256SUMS.frozen`).
Line references point to `src/my_first_robot/my_first_robot/` unless stated otherwise.

## 1. What `disaster_bot_full.launch.py` starts

| Process | Source | Role |
|---|---|---|
| Gazebo Sim server (+ GUI client if `gui:=true`) | `disaster_site/launch/disaster_site.launch.py` → `ros_gz_sim/gz_sim.launch.py` | Simulates `disaster_site/world/earthquake_disaster.sdf` |
| `robot_state_publisher` | `turtlebot3_gazebo/launch/robot_state_publisher.launch.py` | TF for the robot |
| `create` (ros_gz_sim) | disaster_site launch | Spawns `turtlebot3_burger_cam` at (0.0, −1.2), yaw 0 (outside the building) |
| `parameter_bridge` | `turtlebot3_burger_cam_bridge.yaml` (installed TB3 package) | `/clock`, `/odom`, `/imu`, `/scan`, `/tf`, `/joint_states`, `/camera/camera_info` Gazebo→ROS; `/cmd_vel` (TwistStamped) ROS→Gazebo |
| `image_bridge` | disaster_site launch | `/camera/image_raw` Gazebo→ROS |
| `safety_controller` | `safety_controller.py` | The **only** `/cmd_vel` publisher |
| `yolo_detector` | `yolo_detector.py`, run with `<ws>/yolo_env/bin/python`, cwd `<ws>` | Person detection |
| `victim_distance_estimator` | `victim_distance_estimator.py` | Box + LiDAR → range and bearing |
| `autonomous_search` (started 10 s later) | `autonomous_search.py` (+ `exploration_grid.py`, `victim_prioritizer.py`) | Mission logic; publishes `/cmd_vel_raw` and `/victim_found` |

Launch arguments: `gui` (default `true`), `x_pose` / `y_pose` / `yaw` (0.0 / −1.2 / 0.0), `explore_max_steps` (30), `record_dir` (`''` = off).
The launch fixes `TURTLEBOT3_MODEL=burger_cam`, `use_sim_time` true for all project nodes, and on `autonomous_search` it sets
`victim_approach_enabled: true` and `victim_prioritization_enabled: true` (both default `false` in the node).
Before starting anything, it checks that `disaster_site.launch.py`, `yolo_env/bin/python`, the detector script and `yolo11n.pt` exist, and
raises `RuntimeError` if one is missing.

**Not started** (legacy entry points that are still installed): `obstacle_detector`, `victim_rescue`, `mission_control`, `mission_monitor`,
`battery_server`/`battery_client`, `keyboard_controller`, `camera_viewer`. They are not part of the validated system.

`obstacle_detector` (`obstacle_detector.py`) subscribes to `/scan`. When its closest valid LiDAR reading is < 0.8 m, it publishes a **zero-velocity
`geometry_msgs/Twist` on `/cmd_vel`**; otherwise it only logs. It never commands motion. It is nevertheless an **unvalidated additional `/cmd_vel`
publisher** outside the SafetyController, and its `Twist` type conflicts with the `TwistStamped` that the Gazebo bridge uses on `/cmd_vel`. Its effect
alongside the stack has not been tested. **Do not launch it alongside the validated stack.**

## 2. Data flow

```
 Gazebo (earthquake_disaster.sdf, burger_cam)
   │ /camera/image_raw (320×240 wide-angle)      │ /scan  │ /odom  │ /imu  │ /clock
   ▼                                              │        │        │
 yolo_detector (YOLO11n, class 0, conf ≥ 0.50)    │        │        │
   │ /victim_detections  (std_msgs/String, JSON list)       │        │
   ▼                                              ▼        │        │
 victim_distance_estimator ◄──────────────────── /scan     │        │
   │ /victim_distance  (std_msgs/String, JSON, 7 keys)      │        │
   ▼                                                       ▼        ▼
 autonomous_search ◄──────────────── /scan, /odom, /imu ─────────────
   │ /cmd_vel_raw (TwistStamped)        └─► /victim_found (String JSON, transient-local, depth 1)
   ▼
 safety_controller ◄── /scan
   │ /cmd_vel (TwistStamped)
   ▼
 parameter_bridge ─► Gazebo diff-drive

 disaster_bot_monitor (optional, separate `ros2 run`): subscribes only — /rosout, /victim_detections,
 /victim_distance, /victim_found, /cmd_vel_raw, /cmd_vel, /imu; read-only parameter query of autonomous_search.
```

## 3. Stage by stage

### 3.1 Person detection — `yolo_detector`
- Loads `YOLO('yolo11n.pt')`, resolved relative to the working directory, which the launch sets to the workspace root (`yolo_detector.py:28`).
  It uses pretrained COCO weights, unmodified.
- For each image it runs inference with `classes=[0]` (person) and `conf=0.50` (`:89–94`).
- When at least one box is found, it publishes `/victim_detections`: a JSON **list**, one dict per box:
  `victim_id` (**always 1**, no tracking), `class` (`"person"`), `confidence`, `bbox {x1,y1,x2,y2}`, `center {x,y}`, `timestamp`
  (wall-clock `time.time()`). Frames without a person publish nothing.
- It opens an OpenCV window (`cv2.imshow`), so it needs a display (`DISPLAY`). The venv's OpenCV has only the xcb backend.
- Detection says "a person-shaped object is in the image". It says nothing about the person's condition.

### 3.2 LiDAR-based distance estimation — `victim_distance_estimator`
- Selects **only the highest-confidence box** per message (`victim_distance_estimator.py:218–222`). Everyone else in the frame is ignored.
- Converts box edges and centre to bearings with the calibrated wide-angle lens model `bearing = −4·atan(((x−160)/160)·tan(3.183/8))`, where + is left
  (`:24–31`). CameraInfo's pinhole model is deliberately not used.
- Gates:
  - no output if the box touches the image edge (≤ 2 px);
  - rays nearer than `max(0.20 m, 0.25 m / box angular width)` are ignored (size gate);
  - the scan must be ≤ 0.5 s old.
- Distance: in the box's angular span (padded ±2°, minimum half-span 2°), it finds the **nearest range supported by ≥ 2 rays within 0.15 m**
  and returns the median of that cluster. The result is EMA-smoothed (α 0.35), and the filter resets on a jump > 0.5 m or when detections are stale > 1 s.
- Output on `/victim_distance`: a JSON dict with exactly **seven keys**. The set is asserted by `test/test_victim_distance_gate.py::test_output_format_unchanged`:

  | Key | Meaning |
  |---|---|
  | `victim_id` | copied from the detector (always 1) |
  | `class` | `"person"` |
  | `confidence` | YOLO confidence of the selected box (3 dp) |
  | `distance_m` | smoothed LiDAR range (m, 3 dp) |
  | `camera_x` | box centre x (px, 1 dp) |
  | `angle_deg` | centre bearing (deg, + = left, 2 dp) |
  | `detection_age_s` | time from message receipt to publication (≈ 0; **not** the image age) |

- Known risk: "nearest supported cluster" can pick up an object (debris, beam) that is inside the box span and nearer than the person
  (see VALIDATION_AND_LIMITATIONS §5).

### 3.3 Candidate confirmation and prioritization — inside `autonomous_search`
- `victim_measurement()` (`autonomous_search.py:965`) accepts a reading only if all of these hold: 0.20 < distance ≤ 3.5 m, |bearing| ≤ 100°,
  0.60 ≤ confidence ≤ 1, and age ≤ 0.5 s.
- Readings are used only once the mission phase is `INSIDE`, in the exploration scan/turn/move states, and while the IMU yaw rate is
  < 3 °/s ("not rotating"; this is **not** the same as "stationary"). A reading taken while the robot rotates is kept only as a bearing *cue*,
  which can trigger a look-toward turn.
- Each reading is placed in the robot's **nav frame** (odometry distance along the IMU heading, origin = spawn; a dead-reckoned estimate).
  `VictimMemory` groups readings into tracks within 0.50 m. A track is **confirmed** after 3 agreeing readings. Unconfirmed tracks expire after 30 s.
- With `victim_prioritization_enabled`, `victim_prioritizer.py` ranks the confirmed candidates at the post-scan decision point:

  `score = 0.35·confidence + 0.35·distance + 0.30·accessibility − 0.20·risk` (each term in [0, 1])

  - confidence = mean YOLO confidence, rescaled from [0.60, 1] to [0, 1];
  - distance = 1 at the stop distance, falling to 0 at 3.5 m;
  - accessibility = 1 / 0.75 / 0.5 for the first approach heading offset (0 / ±10 / ±20°) that passes the approach checks. If none passes, the candidate is not selectable;
  - risk = how close a narrow obstacle comes to the direct approach line.

  The target locks once its first approach move starts. Before that, switching needs a score margin of +0.10.
- **This is robotic, accessibility-based ranking. It is not medical triage.** No injury, consciousness or urgency information exists anywhere in the system.

### 3.4 Autonomous search and approach — `autonomous_search`
Mission phases are `OUTSIDE_SEARCH` → `ENTERING` → `INSIDE`. The main states (`State` enum, `:25`) are:

| Group | States | Purpose |
|---|---|---|
| Startup / reactive | `STARTUP`, `EXPLORING`, `TURNING`, `FOLLOWING_BOUNDARY`, `CROSSING_OPENING`, `RECOVERY` | Wait for fresh sensors; outside search and boundary following |
| Entrance entry | `ENTRANCE_APPROACH`, `ENTRANCE_ALIGN`, `ENTRANCE_CROSS`, `VERIFY_INSIDE` | IMU-heading-based doorway detection, alignment, crossing, interior verification |
| Interior exploration | `STOP_AND_SCAN`, `EXPLORE_TURN`, `EXPLORE_MOVE` | 360° scan → coarse occupancy grid (`exploration_grid.py`, not SLAM) → frontier move, at most `explore_max_steps` moves |
| Terminal | `VICTIM_HOLD`, `INSIDE_HOLD`, `FAULT_HOLD` | See §3.6 |

- After every completed scan, a confirmed victim takes priority over the next frontier (`:4028`).
- The approach uses the same `EXPLORE_TURN`/`EXPLORE_MOVE` steps. It tries heading offsets 0, +10, −10, +20 and −20° from the victim bearing, and each must pass
  the corridor check, the remembered narrow-obstacle keep-out, the entrance guard and a live-scan check before any motion.
- A move never aims closer than `victim_stop_distance_m` 0.80 m (LiDAR range to the estimate, with a 0.05 m tolerance).
- If the person isn't seen in 2 approach scans, or after 8 approach moves, the target is abandoned.
- Exploration moves are not victim-aware and can end closer than 0.80 m.

### 3.5 Safety-gated movement — `safety_controller`
- Every 50 ms it republishes the latest `/cmd_vel_raw` command to `/cmd_vel`, applying these rules:
  - **Forward:** front ±30° min range ≤ 0.30 m → linear 0 (rotation passes through); ≤ 0.55 m → linear capped at 0.15 m/s; otherwise capped at 0.60 m/s.
  - **Reverse:** rear ±30° ≤ 0.30 m → 0; ≤ 0.45 m → cap −0.08 m/s; otherwise cap −0.15 m/s.
  - **Freshness:** scan older than 1.5 s, or command older than 0.5 s (header stamps, sim time) → publishes zero.
- It does not use the camera or the victim topics. `autonomous_search` has its own, more conservative stops (front cone, keep-out, path checks).
  In every recorded mission the SafetyController zeroed or capped **0** forward commands, so its limiting paths have never been exercised live
  and have no unit tests.
- An IMU tilt guard in `autonomous_search` (|roll| or |pitch| ≥ 10°) sends an immediate zero command and latches a terminal `FAULT_HOLD`.

### 3.6 Mission completion
| End state | Trigger | Behaviour |
|---|---|---|
| **`VICTIM_HOLD`** | Approach reached: range − 0.80 m ≤ 0.05 m, or the person can no longer be seen at close range | Turns in place to face the estimate (gives up after 15 s), then holds zero velocity. Publishes `/victim_found` immediately and every 5 s. **Terminal**: the mission ends at the first victim |
| `INSIDE_HOLD` | Exploration budget used up, or no frontier left | Holds zero velocity; victim not reached |
| `FAULT_HOLD` | Stale scan/odom (recoverable), recovery exhausted or tilt guard (terminal) | Zero velocity; terminal cases need a node restart |

`/victim_found` (JSON) keys:
- always: `victim_x`, `victim_y`, `frame`, `distance_m`, `last_measured_distance_m`, `confidence` (max over readings), `observations`, `robot_x`, `robot_y`,
  `timestamp`, `state`, `reason`;
- with prioritization on, also: `candidate_id`, `priority_score`, `candidates_considered`.

Positions are nav-frame dead-reckoned estimates, not ground truth.

### 3.7 Mission Monitor — `disaster_bot_monitor` (read-only)
- A terminal dashboard. The node is created with `enable_rosout=False` and `start_parameter_services=False`, and has **no** `create_publisher` call.
  A live graph check showed only rclpy's internal `/parameter_events` publisher. It never commands the robot.
- It parses `autonomous_search` `/rosout` text (so it depends on the log format), counts SafetyController interventions by comparing `/cmd_vel_raw` with `/cmd_vel`,
  and reads `explore_max_steps` and the two victim flags with a read-only `GetParameters` call. Accessibility and risk terms are shown only when given
  `-p record_dir:=<autonomous_search scan_record_dir>`.
- Its timer uses the steady clock, so it refreshes even before `/clock` arrives. It cannot observe collisions (there is no contact topic).

## 4. Experimental work kept **outside** the production pipeline
Neither of these is referenced by any production file or launch file:
- The **Stage 1 multi-person estimator** and its **candidate ambiguity rule** live only in the private archive
  `~/disaster_bot_pose_eval_20261012/stage1_multi_person/`. The candidate **failed** independent validation on 2026-10-13 and is not eligible for integration.
- **YOLO11m-pose** (and YOLO11m) were evaluated offline and in isolated Gazebo copies only. Production remains **YOLO11n**.
