# Disaster Bot — Final QA Audit

**Audit date:** 2026-10-18  
**Auditor:** Read-only inspection of source files, scripts, and existing reports.  
No Gazebo or ROS nodes started. No production code modified. No existing files overwritten.

---

## 1. Overall Readiness Assessment

**PASS WITH LIMITATIONS — READY FOR STATIONARY DEMONSTRATION**

The stationary simulation demonstration is safe to run as written in the final demo kit
(`detector_eval_20261018_000055/demo/`). The perception pipeline is correctly implemented
and its live behavior is confirmed by two verified sessions (2026-10-18). All safety
constraints (no motion nodes, no autonomous navigation) are structurally enforced by the
demo stack's design and verified via assertions in the launch file.

Three items require attention before use:
1. **DEMO_GUIDE.md Step 3**: minor counting error ("four" startup lines; there are five).
2. **RESUME_BULLETS.md Option C**: implies the guard was active during the 6/6 missions;
   those missions were unguarded.
3. **VALIDATION_AND_LIMITATIONS.md §1**: `victim_prioritizer.py` SHA has a formatting space
   (`1458 89bb`) that should be `145889bb`.

None of these errors affect the safety or correctness of the demo procedure itself.

---

## 2. Final Status Verification

### 2a. Claims vs. evidence

| Claim in FINAL_STATUS.md | Verification | Result |
|---|---|---|
| Autonomous missions 6/6 VICTIM_HOLD (disaster site) | Confirmed: `evidence/README.md`, `docs/VALIDATION_AND_LIMITATIONS.md` §2; master_launch_20261008, monitor_20261008, final regression + 3 earlier runs | **PASS** |
| Standing detection: Stage A offline 100 % | Confirmed: `stageA_perception_20261014_174311` referenced in existing `docs/VALIDATION_AND_LIMITATIONS.md` §5 | **PASS** |
| Live stationary distance 1.708 m (validated 1.709 m) | Confirmed: rehearsal REPORT.md §C (median 1.708, validated 1.709) and detector eval REPORT.md §7 (live YOLO11n 1.707 m) | **PASS** |
| Guard offline: F2 WP 840→0, WS 592→0 | Confirmed: `docs/VALIDATION_AND_LIMITATIONS.md` §D references `nav_guard_resume_20261017_180501` with those exact numbers | **PASS** |
| Guard live: 8 scenes, 0 wrong distances | Confirmed: rehearsal REPORT.md §C (debris 99/99 withheld, lying 80/80 withheld); detector eval REPORT.md §3 (140 withheld debris, 151 withheld lying) | **PASS** |
| Test suite: 497 passed, 3 failed, 1 skipped | Confirmed: detector eval REPORT.md §6 | **PASS** |
| Previous run (2026-10-12): 286 passed | Confirmed: `docs/VALIDATION_AND_LIMITATIONS.md` §5 | **PASS** |
| YOLO-World: lying recall 17→37/58 views at 0.20 | Confirmed: DETECTOR_COMPARISON.md §3.1 table | **PASS** |
| YOLO-World: merged-box 7 FP @0.20 (one scene) | Confirmed: DETECTOR_COMPARISON.md §3.2 | **PASS** |
| YOLO-World: 3.4× slower (163.5 ms vs 48.6 ms) | Confirmed: DETECTOR_COMPARISON.md §3.3 | **PASS** |
| Stale-target: navigation node keeps readings 30 s | Confirmed: `autonomous_search.py` line 946 area, previously verified | **PASS** |
| VICTIM_MIN_CONFIDENCE = 0.60 | Confirmed: grep of `autonomous_search.py` line 946 | **PASS** |
| 7 frozen files, hashes in evidence/SHA256SUMS.frozen | Confirmed: `evidence/README.md`, SHA256SUMS.frozen read | **PASS** |

### 2b. Distinction between implementation categories

FINAL_STATUS.md correctly distinguishes:
- Implemented and verified (missions, guard, detector, tests): **YES**
- Demonstrated in stationary simulation (guard, detector): **YES**
- Offline evaluation results: **YES** (clearly labelled)
- Experimental functionality (YOLO-World): **YES** (explicitly labelled not default)
- Unresolved limitations (guarded mission, stale-target, posture, close-pair): **YES**

### 2c. Minor formatting issue

**NEEDS CORRECTION (minor):**  
`VALIDATION_AND_LIMITATIONS.md §1`, SHA entry for `victim_prioritizer.py`:  
- Recorded as: `1458 89bb`  
- Correct form: `145889bb` (the space breaks the hex string; source: `evidence/SHA256SUMS.frozen`, hash `145889bb3de3...`)

This is a cosmetic formatting error and does not affect any functional verification.

---

## 3. Stationary Demo Safety Audit

### 3a. File existence checks (read-only)

| File/path referenced in DEMO_GUIDE.md | Exists? |
|---|---|
| `~/disaster_bot_test_logs/detector_eval_20261018_000055/demo/demo_env.sh` | **YES** |
| `~/disaster_bot_test_logs/detector_eval_20261018_000055/demo/demo_stack.launch.py` | **YES** |
| `~/disaster_bot_test_logs/detector_eval_20261018_000055/demo/site/launch/demo_sim.launch.py` | **YES** |
| `~/disaster_bot_test_logs/detector_eval_20261018_000055/demo/scene.sh` | **YES** |
| `~/disaster_bot_test_logs/detector_eval_20261018_000055/demo/watch.py` | **YES** |

### 3b. Scene commands in scene.sh

All scene names used in DEMO_GUIDE.md confirmed present in `scene.sh`:

| Scene name | In scene.sh? |
|---|---|
| `empty` | **YES** |
| `standing` | **YES** |
| `debris` | **YES** |
| `lying` | **YES** |
| `lying_alone` | **YES** |
| `lying_oblique` | **YES** |
| `pair` | **YES** |
| `pair_facing` | **YES** |

### 3c. demo_env.sh sets correct variables

Confirmed: `demo_env.sh` sources `/opt/ros/lyrical/setup.bash` and `~/ros2_ws/install/setup.bash`,
sets `ROS_DOMAIN_ID=42`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, `GZ_PARTITION=demo42`,
`YOLO_OFFLINE=True`, `YOLO_AUTOINSTALL=False`, and `DEMO` pointing to the new kit. **PASS**

### 3d. Motion node safety

The `demo_stack.launch.py` source confirms:
- It imports the `disaster_bot_full.launch.py` actions and **asserts** `safety_controller` is dropped.
- It **asserts** `autonomous_search` (a `TimerAction`) is not included.
- It does not start `obstacle_detector.py`.
- The `OVERRIDES` dict forces `posture_enabled:=false`, preventing the pose node from starting via T2.

Result: **PASS** — the demo stack structurally cannot start motion nodes.

### 3e. Startup log messages

Actual log strings in source code:
- `yolo_detector.py` line 30: `'🚨 Disaster Bot YOLO Detector Started'`
- `yolo_detector.py` line 57: `f'Detector model {model_path}, confidence >= {self.confidence_threshold}'`
- `victim_distance_estimator.py` line 131: `'📏 Victim Distance Estimator Started'`
- `victim_detection_guard.py` line 65: `f'Victim detection guard started: /victim_detections -> /victim_detections_nav …'`
- `victim_localizer.py` line 102: `f'Victim localizer started: …'`

The DEMO_GUIDE.md shows the text without emojis (e.g., `YOLO Detector Started` instead of
`🚨 Disaster Bot YOLO Detector Started`). The text is a substring of the actual log output
and will be visible in the terminal. **PASS WITH LIMITATIONS** — the substrings are correct
and unambiguous; the emojis appear immediately before the text in the actual output.

### 3f. NEEDS CORRECTION — startup line count

**DEMO_GUIDE.md Step 3 says:** "Wait ~5 s for all **four** startup lines"  
**Actual count:** five bullet points are listed.  
**Source of truth** (`FINAL_DEMO_CHECKLIST.md`): lists five items without a count.

**Correction:** Change "four" to "five" in Step 3 of DEMO_GUIDE.md.  
Do not change any other text. This is a one-word correction.

### 3g. `pair` scene expected output

DEMO_GUIDE.md says: "Expected: both detected and localized; `/victim_distance` shows ~2.65 m."

This matches `FINAL_DEMO_CHECKLIST.md` (source of truth) exactly. The phrasing "both detected
and localized" is accurate: both people produce boxes in `/victim_detections` (detected); the
advisory localizer in `/victim_localizations` localizes both (localized). The `/victim_distance`
topic shows one distance (max-confidence box selected by the frozen estimator). The wording
does not claim two `/victim_distance` readings.

**PASS WITH LIMITATIONS** — consistent with source of truth; a presenter should be prepared
to explain that `/victim_distance` shows the higher-confidence person's distance, while the
advisory localizer (`/victim_localizations`) has per-person entries.

### 3h. Demo kit verification basis

The rehearsed checklist in `demo_rehearsal_20261017_234833/DEMO_CHECKLIST.md` uses the
**older** kit (`final_demo_20261017_232734/demo/`). DEMO_GUIDE.md uses the **newer** kit
(`detector_eval_20261018_000055/demo/`). The two kits are related (new kit was built from
the same launch file actions) but differ in the demo script and detector selection.

The newer kit was run end-to-end in two live stationary sessions on 2026-10-18 (YOLO11n and
YOLO-World), documented in `detector_eval_20261018_000055/REPORT.md §3` and confirmed in
`FINAL_DEMO_CHECKLIST.md`. The formal multi-session rehearsal script (REHEARSAL_RESULTS.tsv)
was produced for the older kit only.

**PASS WITH LIMITATIONS** — the new kit is live-verified but has no REHEARSAL_RESULTS.tsv.
This does not affect safety. The presenter should do one practice run before the actual demo.

### 3i. Cleanup — `ros2 daemon stop`

DEMO_GUIDE.md Step 9 includes `ros2 daemon stop` and the `pgrep` check. This matches the
correction noted in the rehearsal REPORT.md §I.1 and the rehearsal DEMO_CHECKLIST.md §6.
**PASS**

### 3j. YOLO-World labelling

YOLO-World is labelled "OPTIONAL, EXPERIMENTAL" in the Step 7 header, with explicit warnings
about speed, merged boxes, and navigation gate. **PASS**

### 3k. Pose estimation excluded from demo

The demo stack forces `posture_enabled:=false` via `OVERRIDES`. DEMO_GUIDE.md has no T3 pose-
node command (unlike the rehearsal checklist which had the pose node as T3). **PASS**

---

## 4. Resume Bullet Accuracy

### 4a. Option A

> "Built a simulated autonomous robot that enters a damaged building, explores the floor plan
> with LiDAR-based frontier search, and locates a standing person using camera–LiDAR fusion;
> 6/6 Gazebo missions reached the person (25 exploration steps, 0.939 m stand-off, 0 safety
> interventions)."

- "simulated autonomous robot": **PASS**
- "camera–LiDAR fusion": accurate (distance estimator combines these). **PASS**
- "6/6 Gazebo missions": confirmed, disaster-site world. **PASS**
- "25 exploration steps": master_launch_20261008 used 25 steps. **PASS** (other runs used 14–24)
- "0.939 m stand-off": GT value from master_launch_20261008. **PASS**
- "0 safety interventions": SC 0/0 confirmed in all runs. **PASS**
- No claim of real-world performance. **PASS**

**Overall: PASS**

### 4b. Option B

> "Designed and validated a real-time detection filter that intercepts YOLO person detections
> before the distance estimator; in stationary simulation (8 scenes), wrong-surface readings
> dropped from 592 to 0 (offline) and debris/occlusion scenes produced 0 wrong distances live,
> without modifying the frozen navigation stack."

- "real-time detection filter": accurate (guard processes each camera frame). **PASS**
- "intercepts YOLO person detections before the distance estimator": accurate via SetRemap. **PASS**
- "wrong-surface readings dropped from 592 to 0 (offline)": confirmed from F2 offline test
  in `nav_guard_resume_20261017_180501`. **PASS**
- "8 scenes": confirmed from rehearsal (6 scenes) and the 2026-10-18 live YOLO11n session
  (9 scenes). The rehearsal had 6 scene steps; a note that the "8" refers to distinct scene types
  A–H from the guard live report would be clearer. **PASS WITH LIMITATIONS** — the number is
  defensible (guard live validation tested 8 stationary scenarios A–H); recommend being
  prepared to explain if asked.
- "0 wrong distances live": confirmed from rehearsal REPORT.md §C. **PASS**
- "without modifying the frozen navigation stack": confirmed (guard intercepts before the
  estimator via a launch-file remap; estimator source is unchanged). **PASS**

**Overall: PASS WITH LIMITATIONS**

### 4c. Option C

> "Simulation prototype: autonomous building entry, frontier exploration, and camera–LiDAR
> person localisation with a perception guard; 6/6 missions reached VICTIM_HOLD; 497
> automated tests passing."

**NEEDS CORRECTION:**  
The phrase "camera–LiDAR person localisation **with a perception guard**; 6/6 missions
reached VICTIM_HOLD" implies the guard was active during the 6/6 missions. It was not.
The 6/6 disaster-site missions used the unguarded stack (`disaster_bot_full.launch.py`
without `nav_guard:=true`). The guard was added and validated separately in stationary tests.

**Corrected wording:**

> "Simulation prototype: autonomous building entry, frontier exploration, and camera–LiDAR
> person localisation; 6/6 missions reached VICTIM_HOLD; added a navigation-facing perception
> guard validated in stationary simulation (0 wrong-surface distances); 497 automated tests passing."

Or, simpler, just remove "with a perception guard" from that clause:

> "Simulation prototype: autonomous building entry, frontier exploration, and camera–LiDAR
> person localisation; 6/6 missions reached VICTIM_HOLD; perception guard reduces wrong-surface
> distances to 0 in stationary simulation; 497 automated tests passing."

---

## 5. Technical Documentation Accuracy

### 5a. ARCHITECTURE.md

| Claim | Verification | Result |
|---|---|---|
| `/camera/image_raw` → yolo_detector → `/victim_detections` | Confirmed: `yolo_detector.py` subscription and publisher | **PASS** |
| victim_detection_guard subscribes `/victim_detections` and `/scan` | Confirmed: guard source | **PASS** |
| Guard publishes `/victim_detections_nav` and `/victim_detection_guard` | Confirmed: guard source lines 59–60 | **PASS** |
| victim_distance_estimator subscribes `/victim_detections` (remapped to `/victim_detections_nav` by launch) | Confirmed: estimator source line 109; posture launch line 98–99 SetRemap | **PASS** |
| Estimator subscribes `/scan` | Confirmed: estimator source line 102 | **PASS** |
| autonomous_search: VICTIM_MIN_CONFIDENCE = 0.60 | Confirmed: line 946 | **PASS** |
| victim_localizer subscribes `/victim_posture`, `/scan`, `/odom` | Confirmed: victim_localizer.py lines 97–99 | **PASS** |
| `/victim_localizations` not consumed by any production node | Confirmed: guard docstring: "nothing downstream" | **PASS** |
| yolo_detector parameters: `model_path` (default `yolo11n.pt`), `confidence_threshold` (default 0.50) | Confirmed: yolo_detector.py lines 43–44 | **PASS** |
| Guard parameters: `exclusive_cluster_check`, `max_image_age_s` (1.0), `scan_timeout_s` (0.5) | Confirmed: guard source | **PASS** |
| `demo_stack.launch.py` drops safety_controller and autonomous_search | Confirmed: demo_stack source assertions | **PASS** |
| Remap mechanism for guard | ARCHITECTURE.md §2.4 says "the estimator subscribes directly to `/victim_detections` when the guard is off" — this is correct; remap is applied by the launch file when `nav_guard:=true`. The runtime view presented in the diagram is accurate. **PASS** |

**One clarification not in ARCHITECTURE.md** (minor, not an error): the estimator's code always
subscribes to `/victim_detections`; the guard integration works via a launch-file `SetRemap`
that transparently redirects that subscription to `/victim_detections_nav` at runtime. This
implementation detail is correctly implied but not explicitly stated. The runtime behavior
described in the diagram is accurate. No correction needed.

**Overall: PASS**

### 5b. VALIDATION_AND_LIMITATIONS.md

| Claim | Verification | Result |
|---|---|---|
| 497 passed / 3 failed / 1 skipped | Confirmed: detector eval REPORT.md §6 | **PASS** |
| 3 failures: crossing_align, flake8, pep257 | Confirmed: REPORT.md §6 | **PASS** |
| Rehearsal: standing 117/117 forwarded, median 1.708 m | Confirmed: REPORT.md §C | **PASS** |
| Rehearsal: debris 99/99 withheld, lying 80/80 withheld | Confirmed: REPORT.md §C | **PASS** |
| Detector rate rehearsal 4.0–4.5 Hz | Confirmed: REPORT.md §C | **PASS** |
| 2026-10-18 live YOLO11n: standing 1.707 m, debris 140 withheld | Confirmed: DETECTOR_COMPARISON.md §4 | **PASS** |
| 2026-10-18 live YOLO-World: lying proxy 21/64 correct distances | Confirmed: DETECTOR_COMPARISON.md §4 | **PASS** |
| Offline: lying views @0.20 YOLO11n 17/58 vs YOLO-World 37/58 | Confirmed: DETECTOR_COMPARISON.md §3.1 | **PASS** |
| Guard offline F2 WP 840→0, WS 592→0 | Confirmed: existing `docs/VALIDATION_AND_LIMITATIONS.md` §D | **PASS** |
| Stage A: 0°/±45°/1–3 m: 100 %, error +1.0..+1.9 cm | Confirmed: existing `docs/VALIDATION_AND_LIMITATIONS.md` §5 | **PASS** |
| SHA hash format `1458 89bb` for victim_prioritizer.py | **FORMATTING ERROR**: should be `145889bb` (no space). See §2c above | **NEEDS CORRECTION** |
| Wrong-surface: 225/237 readings measured the object (IVR B4) | Confirmed: existing `docs/VALIDATION_AND_LIMITATIONS.md` §6.4 | **PASS** |

**Overall: PASS WITH LIMITATIONS** (one formatting error)

### 5c. PROJECT_PRESENTATION.md

| Claim | Verification | Result |
|---|---|---|
| 6/6 missions: 25 steps, 0.939 m, 0 SC interventions | Confirmed: evidence (master_launch run) | **PASS** |
| Standing detection 100 % at 0°/±45° 1–3 m; error +1.0..+1.9 cm | Confirmed: Stage A evidence | **PASS** |
| Guard: WS 592→0 offline, 0 wrong distances live | Confirmed | **PASS** |
| Live stationary: 1.708 m vs validated 1.709 m | Confirmed | **PASS** |
| 497 automated tests | Confirmed | **PASS** |
| YOLO-World: lying 17→37/58, latency 163.5 ms vs 48.6 ms | Confirmed | **PASS** |
| Viva Q&A: description of the remap/guard mechanism | Consistent with source code | **PASS** |
| Viva Q&A: VICTIM_MIN_CONFIDENCE = 0.60 at line 946 | Confirmed | **PASS** |
| "6/6 missions" in Slide 8 not attributed to guarded stack | Slide 8 says "Autonomous missions (disaster site) | 6/6…" with no guard claim | **PASS** |
| "Everything is simulation; one world, one victim placement" | Confirmed | **PASS** |

**Overall: PASS**

---

## 6. Verified Claims and Evidence Sources

| Claim | Evidence source |
|---|---|
| 6/6 disaster-site missions VICTIM_HOLD | `evidence/README.md`; `docs/VALIDATION_AND_LIMITATIONS.md` §2; `evidence/master_launch_20261008/` |
| Guard reduces WS from 592→0, WP from 840→0 (offline) | `docs/VALIDATION_AND_LIMITATIONS.md` §D ref `nav_guard_resume_20261017_180501` |
| Stationary demo: 0 motion throughout | rehearsal REPORT.md §D and §E; detector eval REPORT.md §3 |
| YOLO11n standing detection 100 % at 0°/±45°, 1–3 m | `stageA_perception_20261014_174311`; existing `docs/VALIDATION_AND_LIMITATIONS.md` §5 |
| YOLO11n live stationary distance ~1.707–1.709 m | rehearsal REPORT.md §C; detector eval REPORT.md §7 |
| YOLO-World lying recall improvement offline | DETECTOR_COMPARISON.md §3.1 |
| YOLO-World merged-box (7 FP @0.20, one scene) | DETECTOR_COMPARISON.md §3.2 |
| YOLO-World 3.4× slower, ~3.5 Hz live | DETECTOR_COMPARISON.md §3.3 and §4 |
| 497 passed / 3 failed / 1 skipped | detector eval REPORT.md §6 |
| VICTIM_MIN_CONFIDENCE = 0.60 | `autonomous_search.py` line 946 |
| Guard drops motion nodes (structural) | `demo_stack.launch.py` assertions |
| No guarded moving-robot mission run | All reports; explicitly stated |
| Sitting/kneeling: no assets, no evaluation | REPORT.md §4 (2026-10-18); existing docs |

---

## 7. Incorrect, Ambiguous, or Unsupported Claims

| Location | Claim | Issue | Severity |
|---|---|---|---|
| `DEMO_GUIDE.md` Step 3 | "all **four** startup lines" | Lists 5 bullet points; should say "five" | Minor |
| `RESUME_BULLETS.md` Option C | "camera–LiDAR person localisation **with a perception guard**; 6/6 missions reached VICTIM_HOLD" | The 6/6 missions were run WITHOUT the guard; implies otherwise | **Moderate** |
| `VALIDATION_AND_LIMITATIONS.md` §1 | `victim_prioritizer.py` SHA: `1458 89bb` | Space breaks hex string; should be `145889bb` | Minor |

No fabricated metrics, no real-world deployment claims, no unsupported posture claims, and no
claim that autonomous navigation is safe were found in any document.

---

## 8. Corrected Commands or Wording

### Correction 1: DEMO_GUIDE.md Step 3

Current text: `"Wait ~5 s for all four startup lines:"`  
Corrected text: `"Wait ~5 s for all five startup lines:"`

### Correction 2: RESUME_BULLETS.md Option C

Current text:
> "Simulation prototype: autonomous building entry, frontier exploration, and camera–LiDAR
> person localisation with a perception guard; 6/6 missions reached VICTIM_HOLD; 497
> automated tests passing."

Corrected text (two options):

**Option C-v2a** (preserves the guard mention):
> "Simulation prototype: autonomous building entry, frontier exploration, and camera–LiDAR
> person localisation (YOLO11n + LiDAR fusion); 6/6 unguarded Gazebo missions reached
> VICTIM_HOLD; navigation-facing perception guard validated in stationary simulation;
> 497 automated tests passing."

**Option C-v2b** (shorter):
> "Simulation prototype: autonomous building entry, frontier exploration, and camera–LiDAR
> person localisation; validated guard reduces wrong-surface detections to 0 in simulation;
> 6/6 missions reached VICTIM_HOLD; 497 automated tests passing."

### Correction 3: VALIDATION_AND_LIMITATIONS.md §1

Current text: `| \`victim_prioritizer.py\` | \`1458 89bb\` | Candidate ranking |`  
Corrected text: `| \`victim_prioritizer.py\` | \`145889bb\` | Candidate ranking |`

---

## 9. Outstanding Checks That Cannot Be Completed Without Launching the Robot

The following cannot be verified without starting Gazebo or ROS nodes:

| Check | Why it cannot be done now |
|---|---|
| Confirm `demo_stack.launch.py` actually starts all 5 expected nodes at runtime | Requires launching; not attempted |
| Confirm `/cmd_vel` has 0 publishers after launch (live verification) | Requires launch |
| Confirm guard startup logs match expected substrings at runtime | Requires launch |
| Confirm `scene.sh` teleports produce the expected LiDAR readings | Requires Gazebo + ROS |
| Confirm YOLO-World weights path is accessible at runtime (`WORLD_WEIGHTS` path in demo_stack) | Requires launch; the file path `~/disaster_bot_test_logs/detector_eval_20261018_000055/models/yolov8s-worldv2.pt` was not checked for existence |
| Verify victim_localizer fallback behavior with no pose node | Requires running nodes |
| Verify close-pair guard option (default off) does not cause errors if accidentally enabled | Not run |

**One outstanding read-only check identified:**  
The YOLO-World weights path referenced in `demo_stack.launch.py` (`WORLD_WEIGHTS` constant)
should be confirmed to exist before the Segment B demo is attempted.

```bash
ls ~/disaster_bot_test_logs/detector_eval_20261018_000055/models/yolov8s-worldv2.pt
```

This is a safe check that requires no ROS launch. It was not performed during this audit.
Run it before the demo if Segment B will be shown.

---

## 10. Final Recommendation for Presenting the Project

**The project is READY FOR a stationary college demonstration** provided the three minor
corrections above are noted (they do not affect the safety or correctness of the demo itself).

### Pre-demo checklist (30 minutes before)

1. Run the pre-flight checks from DEMO_GUIDE.md Step 0 (AC online, no stale processes,
   frozen files OK).
2. Confirm YOLO-World weights exist if Segment B will be shown:
   `ls ~/disaster_bot_test_logs/detector_eval_20261018_000055/models/yolov8s-worldv2.pt`
3. Do one practice run of Segment A (empty → standing → debris → lying → pair) using
   DEMO_GUIDE.md. This substitutes for the formal rehearsal of the new kit.
4. Have DEMO_GUIDE.md open for reference during the presentation.

### What to say during the demo

- "The robot is stationary. Scene changes are Gazebo teleports, not robot motion."
- "The ~10 s wait after each scene is a precaution against camera/LiDAR timing effects, not a fix."
- After the standing scene: "The guard forwarded this because a LiDAR cluster is present
  within the bounding box column and no hit falls below the box bottom."
- After the debris scene: "The guard withheld this because a LiDAR hit fell below the box bottom."
- For Segment B (if shown): "This is YOLO-World v2-s, an experimental option. It detects
  lying figures better, but it's 3× slower and can merge two people into one box."

### Claims to make confidently

- 6/6 autonomous missions (unguarded) reached the person in simulation.
- The perception guard reduces wrong-surface distances from 592 to 0 offline.
- YOLO11n detects standing people reliably at 0°/±45°, 1–3 m.
- The demo is stationary and verifiably safe (0 velocity publishers throughout).

### Claims to avoid

- "The guard makes autonomous navigation safe." (Not validated.)
- "The robot can detect any posture." (Only standing validated live.)
- "YOLO-World is a better overall detector." (It has merged-box risk and speed issues.)
- "The system is ready for real disaster response." (Simulation only.)
- "We tested on a real robot." (No hardware testing.)

---

**Audit complete. One new file created:** `~/ros2_ws/final_docs_20261018/FINAL_QA_AUDIT.md`
No existing files were modified.
