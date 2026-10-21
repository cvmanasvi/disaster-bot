# Disaster Bot — Final Project Overview

**ROS 2 Lyrical · Gazebo Sim 10.4.0 · YOLO11n · LiDAR fusion · Stationary demo validated 2026-10-18**

> Simulation research prototype. No real-world deployment, no hardware testing.
> All results are from Gazebo Sim on a laptop CPU.

---

## Problem Statement

After a structural disaster, first responders need to locate people inside damaged buildings quickly but without exposing additional personnel to risk. This project builds a simulated autonomous robot that enters a building through its doorway, explores the interior, and localises standing people using a camera and 2-D LiDAR — without any operator input during the mission.

## Objectives

1. Autonomous building entry (doorway detection and crossing).
2. LiDAR-based frontier exploration of an unknown floor plan.
3. Camera-based person detection (YOLO11n) fused with LiDAR for range and bearing.
4. Safe approach and stop beside the first confirmed person.
5. A navigation-facing perception guard that filters wrong-surface and wrong-person distance readings before they reach the navigation node.
6. Stationary simulation demonstration that shows the perception pipeline without initiating robot movement.

## Architecture (summary)

```
/camera/image_raw
    └─► yolo_detector (YOLO11n, yolo_env)
            └─► /victim_detections
                    └─► victim_detection_guard  ◄─── /scan
                            └─► /victim_detections_nav
                                    └─► victim_distance_estimator (frozen)  ◄─── /scan
                                            └─► /victim_distance
                                                    └─► autonomous_search (frozen; NOT in demo)
```

Advisory path (stationary demo only, does not affect distances):
```
/camera/image_raw + /victim_posture ──► victim_posture_estimator (YOLO11m-pose, advisory)
/victim_posture + /scan + /odom ──────► victim_localizer ──► /victim_localizations (nothing consumes)
```

Full diagram and topic details: `ARCHITECTURE.md`.

## Technologies

| Component | Technology |
|---|---|
| Robot | TurtleBot3 `burger_cam` (simulated) |
| Simulator | Gazebo Sim 10.4.0, `ros_gz` bridges |
| Framework | ROS 2 Lyrical (rclpy, Python 3.14) |
| Person detector | Ultralytics YOLO11n (COCO class 0, conf ≥ 0.50, CPU inference) |
| Optional detector | YOLO-World v2-s (open-vocabulary; experimental, conf 0.20, default off) |
| LiDAR | 360° 2-D scan via Gazebo sensor |
| Tests | pytest + ament_lint (497 passing in latest run) |
| Platform | Ubuntu 26.04, laptop CPU (i7-1355U), no GPU |

## Current Capabilities (verified in simulation)

| Capability | Verification level |
|---|---|
| Building entry (doorway detect, cross, verify) | Gazebo missions, 6/6 |
| LiDAR frontier exploration | Gazebo missions, 6/6 disaster-site runs reached VICTIM_HOLD |
| Standing-person detection at 1–3 m (0°/±45°) | Stage A offline + live stationary (100 %) |
| Camera–LiDAR range estimate, standing, 1.7 m | Live stationary: median 1.708 m (validated 1.709 m) |
| Navigation guard: debris/lying-proxy withheld | Live stationary: all withheld, 0 wrong distances |
| Guard: close-pair option (`exclusive_cluster_check`) | Offline only; default off; not Gazebo-validated |
| Advisory per-person localizer | Integrated; advisory only; nothing in the mission consumes it |
| YOLO-World v2-s lying-proxy detection | Live stationary, side/oblique views; conf mostly < 0.60 nav gate |

## What Is NOT Validated

- Autonomous navigation safety with the guard active (no moving-robot guarded run exists).
- Any posture other than standing in live simulation (lying uses rigid proxy figures; sitting/kneeling have no test assets).
- Real-world performance of any kind.
- YOLO-World merged-box risk in a moving-robot scenario.

## Demo Entry Point (stationary simulation)

The demo does **not** start autonomous navigation. The robot does not move.

See `DEMO_GUIDE.md` for the full step-by-step procedure.

**Prerequisites:** Ubuntu 26.04, ROS 2 Lyrical, Gazebo Sim 10, TurtleBot3 Gazebo packages,
`~/ros2_ws/yolo_env/` (Ultralytics environment), `~/ros2_ws/yolo11n.pt`.

```bash
# Source in every terminal:
source ~/disaster_bot_test_logs/detector_eval_20261018_000055/demo/demo_env.sh

# T1 — simulation
ros2 launch $DEMO/site/launch/demo_sim.launch.py x_pose:=3.0 y_pose:=-6.6 yaw:=1.5708

# T2 — perception stack (YOLO11n, default)
ros2 launch $DEMO/demo_stack.launch.py

# T3 — monitor
python3 -B $DEMO/watch.py

# T4 — scene commands
$DEMO/scene.sh empty|standing|debris|lying|pair
```

Demo kit: `~/disaster_bot_test_logs/detector_eval_20261018_000055/demo/`
Source of truth checklist: `~/disaster_bot_test_logs/detector_eval_20261018_000055/FINAL_DEMO_CHECKLIST.md`

## Scope Statements

- Simulation prototype; never tested on real hardware; not intended for real emergency response.
- Detection and localisation validated for **standing** people in simulation only.
- The priority score is accessibility-based robotic ranking, **not** medical triage.
- Autonomous navigation has not been validated as safe with the perception guard active.

## Documentation Index

| File | Content |
|---|---|
| `ARCHITECTURE.md` | Node-by-node component map, topic flow, parameters |
| `VALIDATION_AND_LIMITATIONS.md` | All measured results, failure cases, known limitations |
| `DEMO_GUIDE.md` | Step-by-step stationary demo procedure |
| `PROJECT_PRESENTATION.md` | College review outline + viva Q&A |
| `RESUME_BULLETS.md` | Resume-ready project bullets |
| `FINAL_STATUS.md` | Audit findings, deliverables, blockers, next steps |

## Licensing

No licence is granted for the original project code. Third-party components keep their own licences
(YOLO11n: AGPL-3.0; YOLO-World v2-s weights: AGPL-3.0 embedded; Standing person mesh: CC0 1.0 per
Gazebo Fuel — upstream provenance unverified). YOLO-World weights are kept outside the workspace and
are for evaluation use only.
