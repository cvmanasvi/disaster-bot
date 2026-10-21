# Disaster Bot — College Project Review Presentation Outline

**Format:** 15–20 minute presentation + 10 minute viva.
All slides/claims must use verified simulation results only. Do not claim real-world performance.

---

## Slide 1 — Title

**Disaster Bot: Autonomous Search and Victim Localisation in Simulated Disaster Environments**

- Name, course, date
- ROS 2 Lyrical · Gazebo Sim 10.4.0 · YOLO11n · LiDAR fusion

---

## Slide 2 — Problem Statement and Motivation

- After structural disasters, first responders need to search damaged buildings rapidly.
- Sending personnel into unstable structures increases risk.
- Goal: a robot that can autonomously enter a building, explore it, and localise people — without operator input during the mission.
- Scope: simulation prototype. Not for real deployment. Not medical triage.

---

## Slide 3 — Objectives

1. Detect and cross the building entrance (doorway detection with LiDAR; IMU-guided alignment).
2. Explore the floor plan with a 2-D occupancy/frontier grid.
3. Detect people with a camera-based YOLO detector.
4. Fuse each detection with 2-D LiDAR to estimate range and bearing.
5. Confirm a track and plan a safe approach; stop ~0.8 m from the person.
6. Filter unreliable detections (debris, wrong surface, stale image) before they reach the navigation node.
7. Demonstrate a stationary perception pipeline with live scene changes.

---

## Slide 4 — System Architecture

**Use the diagram from `ARCHITECTURE.md` §1 (simplified):**

```
Camera ──► YOLO11n detector ──► detection guard (RVC + timing) ──► distance estimator (frozen) ──► navigation
 LiDAR ──────────────────────────────────────────────────────►
```

Key design choices:
- Guard is inserted between the detector and the estimator; the estimator is unchanged.
- No pose/keypoint model in the production pipeline.
- Advisory localizer publishes per-person localizations for observability but nothing in the mission consumes it.

---

## Slide 5 — Implementation: Detection Pipeline

- **YOLO11n** (COCO class 0, conf ≥ 0.50). CPU only; ~8 Hz on i7-1355U.
- Camera: 320×240, wide-angle (burger_cam).
- Box → bearing via a calibrated lens model; nearest LiDAR cluster in that bearing column gives range.
- EMA filter on successive readings (frozen estimator).

**What YOLO11n does well:** standing people at 1–3 m (100 % at 0°/±45°), empty scenes (0 false positives).
**What it misses:** lying people in most views; standing people at the image edge (±80°, ≤ 2.5 m).

---

## Slide 6 — Implementation: Navigation Guard

Problem: the frozen estimator associates a person box with the nearest LiDAR cluster. If debris or a low object is nearby, it measures the debris, not the person.

Solution: `victim_detection_guard` intercepts `/victim_detections` before the estimator.
Withholds a frame if:
- A LiDAR hit projects **below the bottom of the bounding box** (the cluster is on the floor/debris).
- No LiDAR support exists in the **centre column** of the box.
- The LiDAR scan is stale (> 0.5 s).
- The image timestamp is stale (> 1.0 s).
- Any box fails the estimator's required fields.

Validated live (stationary, 8 scenes): debris and lying-proxy scenes → 0 wrong distances. Standing scenes → correct forwarding.

---

## Slide 7 — Implementation: Exploration and Approach

- Coarse occupancy grid; frontier selection by size × inverse-distance × clearance × turn cost.
- Keep-out memory for narrow obstacles encountered while navigating.
- Up to 5 approach headings checked against planned path, keep-out zones, live scan.
- Stop ~0.8 m from person (LiDAR range target); turn to face; publish `/victim_found`.
- Priority score: confidence, distance, accessibility, obstacle risk. Not medical triage.

---

## Slide 8 — Results (Simulation Only)

| What | Result |
|---|---|
| Autonomous missions (disaster site) | 6/6 reached VICTIM_HOLD (25 steps, 0.939 m stand-off) |
| Mission wall-clock time | ~25 minutes |
| Standing detection (Stage A, 0°/±45°) | 100 % at 1–3 m; distance error +1.0..+1.9 cm |
| Guard: debris/lying-proxy scenes | 0 wrong distances (live, 8 scenes) |
| Live stationary distance accuracy | 1.708 m vs validated 1.709 m |
| Automated test suite (final) | 497 passed, 3 known pre-existing failures |

All with one rigid standing person mesh, one world, one victim placement.

---

## Slide 9 — Experimental: YOLO-World v2-s

Motivation: YOLO11n misses most lying views. Larger COCO models do not help. A detector trained on broader caption-grounded data was the next option.

| | YOLO11n | YOLO-World v2-s |
|---|---|---|
| Lying views detected @0.20 | 17/58 | 37/58 |
| Same at navigation gate (0.60) | 5/58 | 5/58 |
| False-positive boxes @0.20 | 0 | 7 (one merged box, close facing pair) |
| CPU latency | 48.6 ms | 163.5 ms |

Decision: **optional, not the default.** Merged-box risk at the nav gate (0.60), 2.3× lower live rate, no validated fix for the merged-box issue.

---

## Slide 10 — Limitations

1. **Simulation only** — one rigid mesh, one world, flat floors, no sensor noise beyond Gazebo defaults.
2. **Postures** — validated for standing. Lying uses proxies (side/oblique only). Sitting/kneeling not evaluated.
3. **Autonomous navigation with the guard** — never run. Safety not established.
4. **Camera/LiDAR not synchronised** — stale readings at scene transitions.
5. **Stale-target behaviour** — navigation node keeps readings for 30 s; unresolved.
6. **Single victim placement** — no generalisation evidence beyond one disaster-site layout.

---

## Slide 11 — Future Work

- Guarded moving-robot mission (one supervised run).
- Stale-target mitigation (e.g. time-stamp images at capture, discard readings > 0.5 s old).
- Lying-person assets with verified licences for sitting/kneeling evaluation.
- Close-pair guard Gazebo validation.
- Merged-box handling for YOLO-World.
- Multi-victim simultaneous tracking (currently ends at first confirmed person).
- Hardware evaluation.

---

## Slide 12 — Summary

- Autonomous simulated robot that enters a building, explores, and localises a standing person.
- 6/6 disaster-site missions reached VICTIM_HOLD with 0 safety interventions.
- A navigation-facing perception guard reduces wrong-surface and wrong-person distances to 0 in stationary validation.
- YOLO-World v2-s improves lying recall offline and live, but cannot be made the default without resolving merged-box and speed issues.
- Honest scope: simulation, standing people, one world. All results clearly labelled.

---

## Viva Questions and Technically Accurate Answers

**Q: Why Gazebo and not a real robot?**
A: Hardware cost and safety constraints. The goal was to validate the perception and navigation pipeline in a reproducible, controllable setting. Real-world deployment would require hardware testing, sensor calibration, and a different safety regime.

**Q: How does the YOLO box give a distance?**
A: The box's horizontal column maps to a bearing via a calibrated wide-angle lens model. The distance estimator then finds the nearest LiDAR cluster whose scan hits project within that column and are supported (cluster height above the floor suggests a person, not the floor). An EMA across frames smooths the estimate.

**Q: Why does the navigation guard help?**
A: The frozen estimator picks the nearest supported LiDAR cluster in the box column. If debris or a low wall is in the same column and closer than the person, it measures the wrong surface. The guard withholds the frame if any LiDAR hit projects below the bottom of the bounding box, which rejects most floor/debris readings. In offline tests this dropped wrong-surface readings from 592 to 0.

**Q: What is the 0.60 confidence threshold?**
A: It is a constant in the frozen `autonomous_search` node (line 946, `VICTIM_MIN_CONFIDENCE = 0.60`). The navigation node ignores any `/victim_distance` message with confidence below that value. This means YOLO-World's improvement in lying detection at 0.20 is mostly invisible to the navigation node at its current threshold.

**Q: Why not use the posture estimator in production?**
A: The posture estimator (YOLO11m-pose) is advisory. It adds no posture classification — only keypoints and orientation hints. Per the constraint, pose estimation is excluded from the final detection pipeline. The advisory localizer uses it for observability, but nothing in the navigation path consumes its output.

**Q: Did the guard slow the robot down?**
A: No, because no guarded moving-robot run was performed. The guard's effect on autonomous navigation speed and correctness is unknown. The stationary demo confirms it behaves correctly at rest with these scene types.

**Q: Could the robot detect someone lying on the floor?**
A: In production (YOLO11n), no — the proxy lying figure is not detected. With the experimental YOLO-World option, side and oblique lying views are detected live, but most readings are below the navigation node's 0.60 confidence bar, so the navigation would still ignore them. Head-on and feet-first views are undetected by both models. Sitting and kneeling have no test evidence.

**Q: Is the priority ranking medical triage?**
A: No. The score ranks by detection confidence, distance, approach path accessibility, and obstacle risk. It has no information about injury, consciousness, or medical urgency, and must not be described or used as medical triage.

**Q: How do you know the robot didn't move in the demo?**
A: Three independent checks: (1) `/cmd_vel` had 0 publishers and 0 messages throughout; (2) the watchdog monitor (`watch.py`) sampled both the publisher count and message count every second; (3) odometry displacement was 0.0 mm (teleports do not change `/odom`, so any wheel-driven motion would appear).

**Q: What are the next most important steps before this could be considered useful?**
A: First, a single supervised guarded moving-robot run to confirm that the guard's timing behaviour does not cause incorrect approaches in motion. Second, stale-target mitigation. Third, sitting/kneeling asset procurement and evaluation. Without at least the first, autonomous navigation safety is an open question.
