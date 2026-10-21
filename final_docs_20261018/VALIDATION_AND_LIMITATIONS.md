# Disaster Bot — Validation and Limitations

All evidence from **Gazebo Sim** on a laptop CPU (i7-1355U, 14 GB RAM, no GPU).
No real-world or hardware testing of any kind was performed.

Evidence paths are under `~/disaster_bot_test_logs/` unless stated otherwise.
Frozen file hashes are in `~/ros2_ws/evidence/SHA256SUMS.frozen` (7 files).

---

## 1. Code version at final audit (2026-10-18)

| File | SHA-256 (first 16 hex) | Role |
|---|---|---|
| `autonomous_search.py` | `2b2b92e1` | Exploration, approach, prioritiser |
| `safety_controller.py` | `ccecad4f` | Speed limits, stale-stop |
| `victim_distance_estimator.py` | `4e1e976c` | Camera–LiDAR range |
| `victim_prioritizer.py` | `145889bb` | Candidate ranking |
| `exploration_grid.py` | `ba84a17c` | Occupancy and frontier grid |
| `disaster_bot_monitor.py` | `f3271236` | Read-only dashboard |
| `disaster_bot_full.launch.py` | `7d61d02b` | Full mission launch |

Non-frozen files changed since 2026-10-14:
- `yolo_detector.py`: added `model_path` and `confidence_threshold` parameters (default behaviour unchanged).
- `test/test_frame_buffer.py`: +1 test for the new parameters.

---

## 2. Automated test suite (latest run: 2026-10-18)

**497 passed, 3 failed, 1 skipped** (`detector_eval_20261018_000055/tests/full_suite.txt`).

The 3 known, pre-existing failures:
- `test_crossing_align::test_alignment_not_improving_early_abandons` (functional; present since 2026-09-24)
- `test_flake8`: 74 issues (1 E501 in the frozen launch file)
- `test_pep257`: 300 issues

Previous run (2026-10-12): 286 passed. Increase is from perception guard and localizer tests added later.

---

## 3. End-to-end autonomous missions (unguarded; pre-guard)

All in the disaster-site world unless noted. `safety_controller` + `autonomous_search` active.

| Run | Build | Result | Steps | Stand-off (GT) | Safety |
|---|---|---|---|---|---|
| Unified launch (`master_launch_20261008`) | 2b2b92e1 | **VICTIM_HOLD t=916.9 s** | 25 | 0.939 m | SC 0/0, tilt 0, FAULT 0 |
| Monitored mission (`monitor_20261008`) | 2b2b92e1 | **VICTIM_HOLD t=877.1 s** | 24 | 0.945 m | SC 0/0 |
| Final regression (harness) | 2b2b92e1 | **VICTIM_HOLD t=875.7 s** | 24 | 0.936 m | SC 0/0 |
| 30-step east route | 3863bd8a | **VICTIM_HOLD t=545.0 s** | 14 | 0.933 m | SC 0/0 |
| 30-step west route | 3863bd8a | **VICTIM_HOLD t=903.3 s** | 24 | 0.921 m | SC 0/0 |
| 15-step baseline | 3863bd8a | INSIDE_HOLD (budget exhausted) | 15 | — | SC 0/0 |

House world (3863bd8a): VICTIM_HOLD in 5/5 runs including an unseen SE-wing placement. Entrance verification 6/6 in Gazebo.

**These runs did not include the navigation guard.** A guarded moving-robot mission has never been run.

---

## 4. Stationary demo rehearsal (2026-10-17)

Evidence: `demo_rehearsal_20261017_234833/`. Verdict: **READY WITH DOCUMENTED LIMITATIONS**.

| Scene | Steady-state result |
|---|---|
| Empty | 0 detections, 0 distances |
| Standing 1.8 m | 117/117 forwarded; median `/victim_distance` **1.708 m** (validated 1.709 m); bearing 0.0–0.2° |
| Behind debris | 99/99 withheld (`lidar_hit_below_box`); 0 distances |
| Standing behind lying proxy | 80/80 withheld (`lidar_hit_below_box`); 0 distances |
| Standing (pose node stopped mid-scene) | 105/105 forwarded; same distance stream |
| Debris (pose node down) | 153/153 withheld; 0 distances |

- 0 `/cmd_vel` publishers and 0 messages throughout.
- Odometry max displacement: 0.0 mm (teleports not seen by `/odom`; cmd_vel/msg counts are the primary evidence).
- Detector rate: 4.0–4.5 Hz.

Transient at scene transitions: previous-scene readings appeared for ~1.7 s after standing→debris. This is a camera/LiDAR timing effect, not a guard failure. See §9.

---

## 5. Final live demo run (2026-10-18, both detectors)

Evidence: `detector_eval_20261018_000055/live/`. Domain 85, no pose node.

### YOLO11n (production)

| Scene | Result |
|---|---|
| Standing 1.8 m | 153 forwarded; median **1.707 m** |
| Behind debris | 140 withheld, 0 distances |
| Standing behind lying proxy | 151 withheld (standing box); 0 distances |
| Lying proxy alone (side) | **0 detections** |
| Two standing, separated 2.6 m | 149 correct distances |

### YOLO-World v2-s (optional, conf 0.20)

| Scene | Result |
|---|---|
| Standing 1.8 m | 67 forwarded; median **1.706 m** |
| Lying proxy alone (side, 1.5 m) | 21 correct distances (conf 0.22–0.54, all < 0.60 nav gate) |
| Lying proxy, oblique (1.8 m) | 64 correct distances; 8 with conf ≥ 0.60 |
| Standing behind lying proxy | lying box forwarded; 67 correct distances |
| Two standing, facing 0.5 m apart | 13 **merged-box** readings (conf 0.67–0.74) |

---

## 6. Standing-person perception baseline (Stage A, 2026-10-14)

Evidence: `stageA_perception_20261014_174311/`.

| Angle | Distance | Detection rate | Distance error |
|---|---|---|---|
| 0°, ±45° | 1–3 m | 100 % (all frames) | +1.0..+1.9 cm |
| ±80° (image edge) | 1.5–2.5 m | partial (blind ≤ 2.5 m) | — |

Consolidated regression (2026-10-14): 7/7 scenes pass after guard integration.

---

## 7. Offline detector comparison (2026-10-18)

2,523 saved frames: 342 lying-proxy frames (A/B/D sets), 981 standing frames (C/D), 234 empty frames.

| Operating point | Model | Lying-proxy views detected / 58 | Standing 0/±45° | FP boxes |
|---|---|---|---|---|
| 0.60 (nav gate) | YOLO11n | 5/58 | 402/402 | 0 |
| 0.60 | YOLO-World v2-s | **5/58** | 402/402 | 4 |
| 0.60 | YOLO-World v2-m | **~25/58** (D-lying-alone 40/84 frames) | 402/402 | ~4 |
| 0.50 (production) | YOLO11n | 7/58 | 402/402 | 0 |
| 0.50 | YOLO-World v2-s | 13/58 | 402/402 | 6 |
| 0.50 | YOLO-World v2-m | **25/58** | 402/402 | **0** |
| 0.20 (calibration) | YOLO11n | 17/58 | 402/402 | 0 |
| **0.20** | **YOLO-World v2-s** | **37/58** | 402/402 | 7 |
| **0.20** | **YOLO-World v2-m** | **40/58** | 402/402 | 11 |

All FP boxes are from **one scene** (two people facing each other 0.5 m apart; YOLO-World draws one merged box).
This merged-box failure disappears at ≥ 0.50 for v2-m (0 FP at 0.50).
Head-on lying views: no detection from any model at any threshold.

Inference: YOLO11n median 48.6 ms, YOLO-World v2-s median 163.5 ms (3.4×), YOLO-World v2-m median 399.6 ms (8.2×).
v2-m live-validated (`v2m_live_20261018_143655`): lying-alone side 8/34 msgs ≥0.60 (23.5%); oblique 34/34 (100%); lying+standing 24/34 (70.6%); live rate 1.65 Hz. Merged box conf 0.216 live (below 0.50 threshold). Standing baseline intact (1.707–1.711 m).

---

## 8. Navigation guard validation

Evidence: `nav_guard_live_20261017_184631/` (live A–H stationary), `nav_guard_resume_20261017_180501/` (offline).

| Test | Result |
|---|---|
| Offline: F2 wrong-person rate | 840 → 0 (guard on) |
| Offline: F2 wrong-surface rate | 592 → 0 (guard on) |
| Live stationary scenes A–H | All standing readings forwarded correctly; debris/lying-proxy 0 wrong distances |
| Close-pair option (offline, F1/F2/IVR) | MIXED readings → 0; correct-reading loss F2 4.1 %, IVR_A_Y 21.3 % |
| Close-pair option (Gazebo live) | **Not validated** |
| Moving-robot test with guard | **Not run** |

---

## 9. Known limitations and failure cases

### 9.1 Camera–LiDAR timing mismatch

The guard pairs each camera frame with the **latest available scan**, not one aligned to the image capture time. The measured image age was 0.43–0.52 s (YOLO11n/YOLO-World) in stationary sim. At scene transitions, previous-scene readings appeared for ~1.7 s. The 10 s demonstration wait is a precaution, not a fix.

### 9.2 Stale targets in the frozen navigation node

`autonomous_search` keeps a `/victim_distance` reading for 30 s and checks message age, not image capture time. If the robot moves after a detection, it will act on a position estimate that is already stale. This is unresolved.

### 9.3 Close-pair association errors

When two people stand within ~0.5 m, the estimator may associate one person's camera box with the other person's LiDAR cluster. The `exclusive_cluster_check` guard option reduces this offline but is default off and has not been Gazebo-validated.

YOLO-World v2-s adds a merged-box failure where it draws one box over both people (conf up to 0.74, above the 0.60 nav gate). This is an additional association risk not present in YOLO11n.

### 9.4 Posture coverage

| Posture | Evidence |
|---|---|
| Standing (0°/±45°) | Validated offline and live in simulation |
| Standing (±80° image edge) | Partial; some views missed |
| Lying | Rigid standing mesh rotated flat (proxy). Side/oblique detected by YOLO-World live. Head-on/feet-first undetected by both models |
| Sitting, kneeling | **No test assets; no evidence; no claim** |
| Multiple people (two standing) | Offline only; 2 live scenes |

### 9.5 Navigation safety

No moving-robot run has been performed with the navigation guard active. It is unknown whether the guard's timing behaviour or the stale-target issue would cause the robot to misbehave during a guarded autonomous mission.

### 9.6 SafetyController never exercised

0 interventions in all missions. Its limiting, stale-command and stale-scan branches have never triggered live and are not unit-tested.

### 9.7 Debris wrong-surface distance (production estimator)

In the independent validation (Experiment B4), 225/237 readings for a person standing beside debris or a beam measured the object rather than the person, because the "nearest supported cluster" rule in the estimator picks whichever cluster is closest. This was fixed in the guard (which withholds such frames), but the underlying estimator logic is frozen and unchanged.

### 9.8 Single world, single victim placement

All 6 disaster-site missions used the same world file, the same spawn pose, and the same victim placement. There is no evidence of generalisation to other victim placements in the disaster site.

---

## 10. What is not implemented or not validated

| Item | Status |
|---|---|
| Real-world deployment | Not attempted |
| Moving-robot test with guard | Not run |
| Sitting/kneeling detection | No assets, no evaluation |
| Close-pair option in Gazebo | Not validated |
| Stale-target mitigation | Not implemented |
| YOLO-World v2-s as production detector | Not made default; merged-box and speed issues open |
| Lying detection via production pipeline (full confidence) | Intermittent at nav gate for side views; consistent for oblique views with v2-m |
| Multi-person approach (two confirmed tracks) | Unit-tested only; never demonstrated live |
| Lying detection via production pipeline | Unresolved (lying proxy missed by YOLO11n; YOLO-World lying boxes mostly below 0.60 nav gate) |

---

## 11. Inconsistencies in earlier documentation corrected here

1. The 2026-10-17 nav-guard live report stated "1,864 guarded msgs = 1,864 `/victim_distance`, all standing CORRECT". This overcounted: it included non-standing frames. Corrected in `final_demo_20261017_232734/REPORT.md` §D.
2. The 2026-10-17 report of `disaster_bot_posture.launch.py nav_guard:=true` as the stationary demo command was **wrong**: that launch file starts `safety_controller` and `autonomous_search` (motion nodes). The demo kit (`demo_stack.launch.py`) was built to correct this. The older command is not safe for a stationary demo.
3. The 2026-10-14 VALIDATION doc (§6.2) references the 78-frame YOLO11m-pose study. This is superseded by the 2,523-frame YOLO-World comparison in §7 above (same saved frames, extended).
4. The `~/ros2_ws/README.md` architecture diagram does not include the navigation guard or the advisory localizer. Those components post-date the README.
