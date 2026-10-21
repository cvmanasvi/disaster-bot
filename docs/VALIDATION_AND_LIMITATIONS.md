# Disaster Bot — Validation Evidence and Limitations

Compiled 2026-10-14 from existing reports. **No experiment was re-run for this document.**

- **All evidence is from Gazebo simulation** with one rigid person mesh. None of it is real-world validation, and the system is **not** ready for real disaster deployment.
- Times `t=` are simulation seconds. "GT" means Gazebo ground truth. "SC 0/0" means the SafetyController zeroed 0 and capped 0 forward commands.
- The primary cross-checked source is [evidence/FINAL_EVIDENCE_AUDIT.md](../evidence/FINAL_EVIDENCE_AUDIT.md) (2026-10-12).
- Detector and multi-person experiments live in the private archive `~/disaster_bot_pose_eval_20261012/`, which is **not** in this repository.

## 1. Code versions
| `autonomous_search.py` | Content | Used by |
|---|---|---|
| `2b2b92e1` (**current**) | approach + prioritizer + hardening | unified-launch mission, monitored mission, final regression |
| `f76fe57c` | prioritizer before hardening | prioritizer Cases A/B/C |
| `3863bd8a` | approach only | all earlier disaster and house runs |

Results on older builds are labelled as such. The hardening diff changed no motion path, but those runs were not repeated on `2b2b92e1`.

## 2. End-to-end missions reaching `VICTIM_HOLD`
**Disaster-site world** (spawn (0, −1.2) outside; victim GT (7.35, 6.65) behind a slab pile in the NE room):

| Run | Build / config | Result | Steps | Approach | GT stand-off | Safety |
|---|---|---|---|---|---|---|
| Unified launch `master_launch_20261008` | 2b2b92e1, `disaster_bot_full.launch.py gui:=false` | **VICTIM_HOLD t=916.9** | 25 | +10° offset | **0.939 m** | SC 0/0, tilt 0, FAULT 0; no orphans after Ctrl+C |
| Monitored `monitor_20261008` | 2b2b92e1, unified launch + monitor | **VICTIM_HOLD t=877.1** | 24 | +10° | **0.945 m** | SC 0/0 |
| Final regression | 2b2b92e1, harness (not the launch file) | **VICTIM_HOLD t=875.7** | 24 | +10° | **0.936 m** | SC 0/0 |
| Case A | f76fe57c | VICTIM_HOLD t=880.1 | 24 | +10° | 0.930 m | SC 0/0 |
| 30-step east route | 3863bd8a, prioritization off | VICTIM_HOLD t=545.0 | 14 | 0° | 0.933 m | SC 0/0 |
| 30-step west route | 3863bd8a | VICTIM_HOLD t=903.3 | 24 | +20° | 0.921 m | SC 0/0 |
| **15-step baseline** | 3863bd8a | **INSIDE_HOLD: budget exhausted, victim never seen** | 15 | — | — | SC 0/0 |

**House world** (`turtlebot3_house`, 3863bd8a, 15 steps): VICTIM_HOLD in V1 ×3 (GT 0.905–0.925 m), V2 (0.956 m), and V3 in a previously unseen
SE-wing placement (0.775 m). Entrance verification succeeded 6/6 in Gazebo (verify_minwidth).

Wall-clock to `VICTIM_HOLD` on the current code was about 24.6–25.1 min. After the hold, the robot moved ≤ 0.004 m over ≥ 30 s.
The victim-position estimate error was 0.06–0.25 m.

## 3. Exploration budget: limited coverage
- 30 moves succeeded in **6/6** single-victim disaster runs, all in **one** world with **one** victim placement and the default spawn.
- 15 moves failed once (west route needs 24–25 moves).
- The route (east 14 moves vs west 24–25) depends on near-tie frontier scores, so it is not predictable in advance.
- n = 6 is not a statistical guarantee. Other layouts, spawn poses and victim placements are untested with the current code.

## 4. Mission Monitor
- Read-only:
  - no publisher in the code;
  - live graph showed only rclpy's internal `/parameter_events`;
  - `/cmd_vel` and `/cmd_vel_raw` each had one publisher (safety_controller / autonomous_search).
- 19/19 unit tests pass, including a wording test that forbids medical-triage language.
- One monitored full mission rendered state, steps, ranking, approach headings and `VICTIM_HOLD` correctly.
- `INSIDE_HOLD`/`FAULT_HOLD` display and multi-candidate ranking display are unit-tested only.

## 5. Automated tests
Run 2026-10-12 (`python3 -m pytest test/`); all 18 test files and fixtures are byte-identical on 2026-10-14:

**286 passed, 3 failed, 1 skipped.** The failures:
- `test_crossing_align::test_alignment_not_improving_early_abandons`: functional, pre-existing since 2026-09-24, unfixed;
- `test_flake8`: 74 issues, including 1 E501 in the validated launch file;
- `test_pep257`: 300 issues.

`test_copyright` is skipped (no headers).

Per-file passes: victim_approach 68, narrow_keepout 67, victim_prioritization 55, verify_inside_width 46, disaster_bot_monitor 19,
exploration_grid 13, victim_distance_gate 9, plan_path_margin 4, crossing_align 3, mypy 1, xmllint 1.

**No tests exist** for `safety_controller.py`, `yolo_detector.py` or the launch file.

The suite has been at 286 passing since 2026-10-08 (earlier counts 212 → 247 → 267 belong to earlier code versions; do not add them up).
The separate archive test suites are not production tests and are not included above: Stage 1 30/30, offline rule 23/23, robustness 20/20, independent validation 11/11.

## 6. Perception limitations

### 6.1 YOLO11n and lying people (production detector)
| Test | Result |
|---|---|
| 19 public photos (exploratory, hand-picked) | standing 5/5, sitting 4/4, crouch/kneel 5/5, **lying 2/5** |
| Isolated Gazebo, rigid mesh rotated to lie | **supine 0/42 frames**; prone only side-on with **head-only** boxes (1.5 m 5/6, 2.5 m 1/6, head-on 0/6) |
| Upright + lying figure together | upright outranked lying in **132/132** messages, so the estimator always measured the upright figure |

**Conclusion:** the production mission is validated only for **upright** people. Lying-person detection is a known, unresolved limitation.

### 6.2 Experimental detector comparison: limited, non-production
On 78 saved simulation frames (13 viewpoints, one mesh, one yard; CPU, 640 px) at the 0.50 threshold:

| Model | Lying frames detected | False positives | ms/frame |
|---|---|---|---|
| YOLO11n | 6/78, all head-only | 0 | 45 |
| YOLO11m | 12/78 | 0 | 253 |
| YOLO11m-pose | 44/78, whole-body | recurring "sky" box (removed by a ≥ 5-keypoint filter) | 357 |

- No model detects end-on (head-first or feet-first) lying views, and in mixed scenes the upright person always scores highest.
- In isolated Gazebo, YOLO11m-pose with the keypoint filter detected 14/17 lying viewpoints with 0 accepted false positives.
- Inference took 1.4–2.6 s per frame under Gazebo load (vs 45 ms for YOLO11n offline). Pinning the detector to performance cores made it about 5× slower.
- **YOLO11m-pose is experimental only and is not the production detector.** These results come from a few viewpoints of one mesh.

### 6.3 One person per frame; multi-person candidate rejected
- The production estimator measures only the highest-confidence box. Ranking of ≥ 2 *confirmed* tracks works in unit tests but was **never demonstrated live**.
  In the two-person Gazebo case (B, f76fe57c) only one track was confirmed. Every mission ends at the first `VICTIM_HOLD`.
- A test-only **Stage 1 per-person estimator** was built and validated offline and in isolated Gazebo. Its **candidate ambiguity rule** then **FAILED**
  pre-registered independent validation (2026-10-13):
  - **2,821 MIXED readings** that blend two people, including 108 from unperturbed synthetic boxes and 673 from real YOLO11n boxes;
  - all of them at 0.40–0.50 m separation with overlapping boxes, caused by "shared" rays being accepted.
- The candidate is **not integrated and not eligible for integration**. No production file references it. Any fix would be a new candidate that needs new validation data.
- Missed lying people can't be detected from one box plus a 2-D scan (an observability limit); this is a detector-stage problem.

### 6.4 Nearby debris or beam: wrong-surface distance
In the independent validation (Experiment B4), readings for an upright person standing beside debris or a beam measured the **object** instead of the person
in **225** published readings (12 synthetic boxes, 213 YOLO11n boxes). The cause is the "nearest supported cluster" rule. With a single box, the
production estimator behaves identically, so **production has this risk**. It is not fixed.

### 6.5 Other perception limits
- No detection closer than about 1 m (the fisheye crops the person); the close-range hold logic exists for this reason.
- At about 2 m in the shadowed room, confidence was only 0.50–0.55.
- `detection_age_s` is not the image age, and readings are placed using the *current* pose.
- "Not rotating" (< 3 °/s) confirmation can happen during straight-line motion.

## 7. Navigation and safety limitations
- **SafetyController:** 0 interventions in every mission, so its limiting, stale-command and stale-scan paths were never exercised live, and none of them is unit-tested.
- **Tilt guard / `FAULT_HOLD`:** never triggered live (maximum pitch 0.81°). The worlds have flat floors, and climbing over rubble or ramps is untested.
- **No contact sensor.** "No collision" is a geometric GT inference where it was computed (min footprint clearance in the disaster world: wall 0.116 m,
  rubble 0.156 m, beam 0.214 m). For the current-code runs only the minimum LiDAR range and SC counts were recorded.
- **Stand-off:** 0.80 m is a LiDAR-range target, not a guaranteed minimum. GT stand-off ranged 0.711–0.956 m, because exploration motion can come closer.
- **Narrow passages:** the 0.70 m passage in the disaster world can't be planned by the grid (the route uses the wide opening). The live narrow-jamb release path was never exercised in Gazebo.
- The robot must move before it can detect the doorway from the spawn pose.

## 8. Simulation and scenario limits
- **Person model:** one static Fuel "Standing person" mesh. It is rigid, so it can only be rotated, not posed; its arms interpenetrate when two copies are ≤ 0.50 m apart.
  There are no partially buried, occluded, moving, small or differently dressed people.
- **Environments:** `turtlebot3_house` (earlier build) and one generated earthquake disaster site, with static boxes, flat floor, no dust, smoke or lighting variation, and no sensor noise beyond Gazebo defaults.
- **Hardware:** never run on a real robot. The detector's speed depends on the CPU, and the stack ran at about 0.6× real time.
- Unified-launch evidence: 1 mission + 1 monitored mission + 1 smoke test. Multi-person scenes were never run through the launch file.

## 9. Robotic prioritization is not triage
The priority score ranks candidates by detection confidence, distance, approach accessibility and obstacle risk. It has **no** information about
injury, consciousness or medical urgency, and must not be described or used as medical triage.
