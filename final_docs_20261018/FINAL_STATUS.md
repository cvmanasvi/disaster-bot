# Disaster Bot — Final Audit and Status (2026-10-18)

---

## 1. Evidence inspected

| File | Purpose |
|---|---|
| `detector_eval_20261018_000055/REPORT.md` | YOLO-World evaluation; final detector decision; live results for both detectors |
| `detector_eval_20261018_000055/DETECTOR_COMPARISON.md` | Full numeric comparison (2,523 frames, all operating points) |
| `detector_eval_20261018_000055/FINAL_DEMO_CHECKLIST.md` | Corrected, live-verified demo checklist (source of truth for demo commands) |
| `demo_rehearsal_20261017_234833/REPORT.md` | Full rehearsal; per-scene results; timing, guard, velocity, cleanup |
| `demo_rehearsal_20261017_234833/REHEARSAL_RESULTS.tsv` | Per-scene steady-state and transient results |
| `demo_rehearsal_20261017_234833/DEMO_CHECKLIST.md` | Rehearsal-corrected checklist (6 corrections from rehearsal) |
| `final_demo_20261017_232734/REPORT.md` | Essential correction: wrong launch command identified; demo kit introduced |
| `~/ros2_ws/README.md` | Existing project overview (written 2026-10-14; missing guard/localizer) |
| `~/ros2_ws/docs/ARCHITECTURE.md` | Existing architecture doc (2026-10-14; pre-guard) |
| `~/ros2_ws/docs/VALIDATION_AND_LIMITATIONS.md` | Existing validation doc (2026-10-14; pre-guard; pre-YOLO-World) |
| `~/ros2_ws/evidence/README.md`, `SHA256SUMS.frozen` | Frozen file hashes; evidence index |
| Source files: `yolo_detector.py`, `victim_detection_guard.py`, `victim_distance_estimator.py`, `victim_localizer.py`, `victim_posture_estimator.py`, `autonomous_search.py` (line 946) | Direct parameter and topic verification |

---

## 2. Audit findings (Task A)

### What is implemented and verified

| Component | Verified level |
|---|---|
| Autonomous building entry (doorway detect, IMU align, cross, verify) | Gazebo missions 6/6 |
| Frontier exploration (occupancy grid, keep-out memory) | Gazebo missions 6/6 |
| Standing-person detection + camera–LiDAR distance (YOLO11n) | Stage A offline 100 %; live stationary 1.708 m |
| Navigation guard (RVC + timing + single-box association) | Offline WP/WS → 0; live 8 scenes, 0 wrong distances |
| YOLO-World v2-s as optional detector (conf 0.20) | Integrated; live stationary validated; not default |
| Per-person advisory localizer (victim_localizer) | Integrated; advisory; nothing in production consumes it |
| Advisory posture estimator (victim_posture_estimator) | Integrated; advisory; not in final demo stack |
| Test suite | 497 passed, 3 known pre-existing failures |

### What YOLO11n does well / fails

Well: standing people at 0°/±45°/1–3 m (100 %); empty scenes (0 FP); fast (~8 Hz CPU).
Fails: lying people (most views undetected); standing at ±80° image edge ≤ 2.5 m (partial); sitting/kneeling (no evidence).

### What YOLO-World v2-s and v2-m improve and their limitations

**v2-s:** Improves lying recall offline (17→37/58 views at 0.20); lying detected live side/oblique; edge-standing 217→443/579 at 0.20.
Limitations: merged-box over two facing people (conf up to 0.74); 3.4× slower; at nav gate (0.60) lying views = 0 improvement (5/58 both); no moving-robot validation.

**v2-m (live-validated 2026-10-18):** Improves lying recall offline (17→40/58 views at 0.20; 7→25/58 at 0.50); **0 FP at 0.50** (vs v2-s 6 FP); D-lying-alone 40/84 at 0.60 (vs v2-s 12/84, 11n 7/84).
Live stationary validation (`v2m_live_20261018_143655`): lying-alone side 8/34 msgs ≥0.60 (23.5%); lying oblique 34/34 msgs ≥0.60 (100%); lying+standing 24/34 msgs ≥0.60 (70.6%); standing baseline intact (1.707–1.711 m). Merged box live: conf 0.216 (well below 0.50 threshold; v2-s was up to 0.74).
**imgsz=480 update (2026-10-18, `posture_improvement_20261018_152105`):** `inference_imgsz` parameter added to `yolo_detector.py` (default 640, backward-compatible); `demo_stack.launch.py` passes `inference_imgsz:=480` for `worldv2m`.
Offline improvement: lying@0.60 +27.5% (51/84 vs 40/84); no new FP (0/234 on N+C0); standing unchanged (50/50).
Live (imgsz=480): lying-alone side **39/84 (46.4%)** above nav gate (vs 8/34 / 23.5% at 640); standing baseline conf 0.912 (unchanged); rate **3.8 Hz** (vs 1.65 Hz, 2.3× faster); cmd_vel 0 throughout. Build: 496/4/1 (4 failures pre-existing).
Limitations: ~2.6× slower than YOLO11n (3.8 Hz vs ~9 Hz); side-view still intermittent (46.4% not 100%); no moving-robot test.

### Camera detections and LiDAR in victim localisation

Production path: YOLO box → bearing column via lens model → nearest LiDAR cluster with support in the column (frozen estimator, EMA, one person per frame). The guard pre-filters frames with wrong-surface or stale signals. The advisory localizer adds per-person RVC checks and uses `/victim_posture` when available, but its output is not consumed by the navigation node.

### Navigation guard and its limitations

The guard withholds a frame if any box fails RVC (LiDAR hit below box bottom, or no support in centre column), scan is > 0.5 s old, image is > 1.0 s old, or required fields are missing. Limitation: camera and LiDAR are not synchronised; the guard pairs each frame with the latest scan, not one aligned to image capture time. The optional close-pair rule reduces MIXED readings offline but is not Gazebo-validated.

### Remaining stale-reading/timing issues

- **Camera/LiDAR timing mismatch:** ~0.43 s image age in stationary sim. After scene transitions, previous-scene readings appear for ~1.7 s.
- **Stale targets:** the frozen `autonomous_search` keeps `/victim_distance` readings for 30 s and checks message age, not image capture age. Unresolved.
- **YOLO-World image age:** 0.52 s (slightly higher due to lower detector rate).

### What is not implemented / not validated

- Guarded moving-robot mission (not run).
- Sitting/kneeling postures (no assets, no evaluation).
- Close-pair option in Gazebo (default off, not validated).
- Stale-target mitigation (not implemented).
- YOLO-World as default (blocked by merged-box risk and speed).
- Multi-person simultaneous tracking (unit-tested only; never demonstrated live).
- Hardware deployment.

### Inconsistencies in existing documentation

1. **`~/ros2_ws/README.md` architecture diagram** omits the navigation guard and advisory localizer (components added after 2026-10-14).
2. **`~/ros2_ws/docs/ARCHITECTURE.md`** is accurate for the pre-guard system; does not cover `victim_detection_guard`, the post-2026-10-18 `model_path`/`confidence_threshold` parameters, or the demo stack.
3. **`~/ros2_ws/docs/VALIDATION_AND_LIMITATIONS.md` §6.2** references the 78-frame YOLO11m-pose study; superseded by the 2,523-frame YOLO-World comparison.
4. **2026-10-17 live report** ("1,864 guarded msgs = 1,864 `/victim_distance`") overcounted. Corrected in `final_demo_20261017_232734/REPORT.md §D`.
5. **2026-10-17 report stated** `disaster_bot_posture.launch.py nav_guard:=true` as the stationary demo command; this is wrong (starts motion nodes). Corrected in the demo kit.

---

## 3. Deliverables created (Task B)

All in `~/ros2_ws/final_docs_20261018/`:

| File | Contents |
|---|---|
| `README.md` | Project overview, objectives, architecture summary, capabilities, demo entry point, scope statements |
| `ARCHITECTURE.md` | Node-by-node reference, Mermaid diagram, topic table, launch file guide, YOLO-World configuration |
| `VALIDATION_AND_LIMITATIONS.md` | All measured results, guard validation, offline comparison, per-limitation detail, inconsistency corrections |
| `DEMO_GUIDE.md` | Full step-by-step stationary demo (source of truth: `FINAL_DEMO_CHECKLIST.md`); Segment A + optional Segment B; cleanup |
| `PROJECT_PRESENTATION.md` | 12-slide outline + 10 viva Q&A with technically accurate answers |
| `RESUME_BULLETS.md` | 3 resume bullet options with usage notes |
| `FINAL_STATUS.md` | This file: audit, deliverables, evidence, blockers, next steps |

---

## 4. Safe checks performed (Task C)

- Read source files: `yolo_detector.py`, `victim_detection_guard.py`, `victim_distance_estimator.py`, `victim_localizer.py`, `victim_posture_estimator.py` (first ~30 lines and targeted grep).
- Grep: `VICTIM_MIN_CONFIDENCE` in `autonomous_search.py` → line 946, confirmed 0.60.
- Grep: parameter names, topic names, guard startup log line.
- `ls` of source files, launch files, evidence directory.
- Verified `~/ros2_ws/final_docs_20261018` did not exist before creation.
- Verified frozen hashes present: `evidence/SHA256SUMS.frozen` readable (7 entries).

No Gazebo or ROS nodes were started. No production code was modified. No existing reports or logs were overwritten.

---

## 5. Known blockers

| Blocker | Impact |
|---|---|
| No guarded moving-robot run | Autonomous navigation safety is unknown |
| Stale-target mitigation not implemented | Navigation could act on 30-s-old position estimates |
| Close-pair option not Gazebo-validated | The option is default off and cannot be recommended as default |
| Lying detection: v2-s weak at nav gate | v2-s shows same 5/58 views as YOLO11n at 0.60; v2-m much better (40/84 D-alone frames @0.60) but not live-validated |
| Sitting/kneeling: no assets | Cannot claim posture-agnostic detection |
| YOLO-World merged-box risk (v2-s @0.20) | Cannot be made default; v2-m clears this at ≥0.50 (0 FP @0.50) |
| v2-m + imgsz=480 side-view still intermittent (46.4%) | Improved from 23.5% but not every frame; oblique remains 100%; head-on/feet-first still 0 |

---

## 6. Recommended next steps

In priority order for the demo and for honest reporting:

1. **Run the demo using `FINAL_DEMO_CHECKLIST.md`** (source of truth). Start with Segment A (YOLO11n). Show Segment B optionally and state its limitations.
2. **Do not claim** autonomous navigation safety, posture-agnostic detection, or real-world capability.
3. If time permits before the review: run a single supervised guarded moving-robot mission (one command: `disaster_bot_full.launch.py` with a `nav_guard:=true` remap) to close the largest gap. Record the outcome honestly.
4. Resolve the YOLO-World merged-box issue (enable close-pair option + Gazebo validation) before considering it as the default.
5. Obtain sitting/kneeling Gazebo assets with verified licences for a future evaluation round.

---

## 7. Final verdict

**READY for a stationary simulation demonstration with documented limitations.**
The perception pipeline correctly forwards standing-person detections, withholds debris, and reports accurate distances. YOLO-World v2-m with imgsz=480 live-validated: lying-proxy figures detected and localized above the 0.60 navigation gate (oblique 100%, side 46.4% / up from 23.5%, lying+standing 70.6%+). Inference rate 3.8 Hz (2.3× faster than v2-m@640). Autonomous navigation with the guard active is an open question.
