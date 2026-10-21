# Disaster Bot — Final Evidence Audit (Phase 1 of packaging)

Audit date: 2026-10-12 (IST). Auditor: Claude Code, read-only audit.
Scope: consolidate existing evidence. No Gazebo run, no mission, no source/launch/config/parameter/model change.
The only file created by this audit is this report. The test suite was re-run once (read-only: `PYTHONDONTWRITEBYTECODE=1`,
`-p no:cacheprovider`, `--basetemp` in a scratch directory outside the project). No project file's mtime changed.

Conventions used below:
- **t=** values are **simulation** seconds (from autonomous_search log lines / `/clock`). Wall-clock figures are labelled "wall".
- **GT** = Gazebo ground truth (model poses). "range" = the node's own LiDAR-based range to its victim estimate.
- **[direct]** = value printed in an existing report/log. **[audit-derived]** = value I extracted from existing raw logs during this
  audit (command reproducible; no new experiment). **[inference]** = my interpretation, not a measurement.
- "SC 0/0" = SafetyController zeroed 0 / capped 0 forward commands (rule: raw forward > 0.01 m/s zeroed or capped on `/cmd_vel`).

---

## 1. Executive Summary

- **Source integrity: unchanged.** Every frozen/validated file hash on disk (and in the colcon install space) equals the last value
  recorded in the 2026-10-08 evidence. No source file has an mtime after 2026-10-08 14:48.
- **Discrepancy 1 — there is no Git repository.** `~/ros2_ws`, `src/`, `src/my_first_robot`, `disaster_site/` and the log tree are not
  Git repositories (`fatal: not a git repository`). Branch/HEAD/working-tree status cannot be reported; integrity rests on SHA-256 hashes
  and per-change backups/diffs in the log tree. Nothing has ever been committed or pushed.
- **Discrepancy 2 — autonomous_search.py hash.** The task brief says it was "most recently recorded as 3863bd8a". That is outdated:
  the evidence chain records 3863bd8a → f76fe57c (prioritizer, 2026-10-08) → **2b2b92e1** (hardening, 2026-10-08). The current file is
  **2b2b92e1**, matching the latest record. Results obtained on 3863bd8a are therefore one or two revisions old (see §12).
- **Best verified end-to-end result (current code, unified launch):** `master_launch_20261008/mission` — one command
  (`disaster_bot_full.launch.py`, defaults), earthquake-disaster world, autonomous entrance → interior verification → 25 exploration steps
  → victim confirmed (22 readings) → +10° offset approach → **VICTIM_HOLD at t=916.9**, GT robot-centre→person **0.939 m**,
  SC 0/0, tilt-guard 0, FAULT_HOLD 0, one publisher per topic, Ctrl+C shutdown with "NO ORPHAN PROCESSES".
  Two further runs on identical code reproduce it (final regression 0.936 m; monitored run 0.945 m).
- **Victim prioritization** is implemented exactly as `0.35·conf + 0.35·dist + 0.30·access − 0.20·risk` (confidence rescaled from
  [0.60, 1] to [0, 1]); it is **robotic accessibility-based ranking, not medical triage**. 55 unit tests pass. In Gazebo it selected the
  nearer/feasible person in two-person scenes, and rejected an infeasible (blocked) one; but **simultaneous multi-victim tracking and
  ranking of more than one *confirmed* track was not demonstrated live** — the estimator publishes only the highest-confidence person per
  frame. Cases A/B/C ran on f76fe57c (pre-hardening), not on the current 2b2b92e1.
- **Mission Monitor** (`disaster_bot_monitor`, f3271236): no publishers in code; graph check during a live mission showed only the
  rclpy-internal `/parameter_events` publisher; 19/19 unit tests pass; one monitored full mission rendered VICTIM_HOLD correctly.
  FAULT_HOLD and INSIDE_HOLD display are unit-tested only.
- **Test suite (re-run today):** 286 passed, 3 failed, 1 skipped. Failures: 1 functional (`test_crossing_align::test_alignment_not_improving_early_abandons`,
  pre-existing since 2026-09-24) + 2 lint (flake8 74, pep257 300). Not "all tests pass".
- **Collisions:** no contact sensor is used anywhere. "0 collisions" claims rest on GT-pose geometric clearance analysis (available for
  some runs) and LiDAR minimum ranges; for the three current-code disaster runs only min-LiDAR range and SC counts were recorded.

---

## 2. Current Repository and Source Integrity

### 2.1 Version control
| Check | Result |
|---|---|
| `git rev-parse` in ~/ros2_ws, src, src/my_first_robot, disaster_site, ~/disaster_bot_test_logs | **not a git repository** (all five) |
| Any `.git` under ~ (depth 5, excl. yolo_env) | only `~/nav2_ws/src/navigation2/.git` (unrelated) |
| Branch / HEAD / status | **Not available** — no repository |
| Commits / pushes made | None possible; none made |

### 2.2 Hashes (SHA-256, measured 2026-10-12; src == install copy for every file listed)
| File | Current hash | Last recorded value (source) | Match |
|---|---|---|---|
| my_first_robot/autonomous_search.py | `2b2b92e13e4b…f09485f` | 2b2b92e1 (master_launch_20261008/hashes_after.txt, monitor_20261008/mission.hash) | ✅ (brief's 3863bd8a is outdated) |
| my_first_robot/safety_controller.py | `ccecad4fa63744f0ed2878176daf7adf093dfdc0ea2128cd3045c5999f94bea7` | identical full hash in brief and hashes_after.txt | ✅ |
| my_first_robot/exploration_grid.py | `ba84a17c1d87…6c2b99` | ba84a17c… | ✅ |
| my_first_robot/disaster_bot_monitor.py | `f3271236bd6a…ee5ce` | f3271236… | ✅ |
| my_first_robot/victim_prioritizer.py | `145889bb3de3…3c1c9e` | 145889bb (hardening/delivered) | ✅ |
| my_first_robot/victim_distance_estimator.py | `4e1e976c93a8…ae13f7` | 4e1e976c | ✅ |
| my_first_robot/yolo_detector.py | `67188d387c57…cdfbe1` | 67188d38 | ✅ |
| my_first_robot/victim_rescue.py (not launched) | `16a724151…ebbe93` | 16a72415 | ✅ |
| launch/disaster_bot_full.launch.py | `7d61d02b507f…a72d08` | 7d61d02b | ✅ |
| disaster_site/launch/disaster_site.launch.py | `636a771c…dbbf21` | 636a771c (hashes_after.txt) | ✅ |
| disaster_site/world/earthquake_disaster.sdf | `6c314f7e…4e52` | 6c314f7e (hashes_after.txt) | ✅ |
| test/test_victim_prioritization.py | `be5e09a5…` | be5e09a5 | ✅ |
| test/test_victim_approach.py | `a26f6991…` | a26f6991 | ✅ |
| test/test_narrow_keepout.py | `e7b3a029…` | e7b3a029 | ✅ |
| test/test_disaster_bot_monitor.py | `fdf5d427…` | (not in hashes_after.txt — file created after it) | n/a |
| setup.py | `9c2cb2c9…` | no recorded hash (memory: +1 entry point on 2026-10-08) | not verifiable |
| ~/ros2_ws/yolo11n.pt (model) | `0ebbc80d4a76…644ee1`, mtime 2026-09-08 | **no hash recorded anywhere in the evidence** | not verifiable (first record) |

Latest mtimes: autonomous_search.py 2026-10-08 12:13, victim_prioritizer.py 12:13, launch 14:02, setup.py 14:40, monitor + its test 14:47.
Hashes were recorded per run (`*.hash`, `hashes_before/after.txt`, preaudit files) and are consistent with the run chronology.

### 2.3 Other files in the package (not part of the validated stack)
`autonomous_search_backup.py`, `mission_control.py(.save)`, `mission_monitor.py` (older monitor, distinct from `disaster_bot_monitor`),
`obstacle_detector.py` (legacy, publishes /cmd_vel directly — explicitly *not* started by the unified launch), `victim_rescue.py`,
`battery_*`, `keyboard_controller.py`, `camera_viewer.py`, `setup.py.save`, legacy `launch/disaster_bot.launch.py`, `launch/robot.launch.py`.
These are packaging-cleanup candidates; none was audited for behaviour.

---

## 3. System Components Audited

| Component | File | Role (verified in code) |
|---|---|---|
| Unified launch | launch/disaster_bot_full.launch.py | Includes disaster_site.launch.py; starts safety_controller, yolo_detector (yolo_env python, cwd ~/ros2_ws), victim_distance_estimator; autonomous_search after a 10 s TimerAction with `victim_approach_enabled=True`, `victim_prioritization_enabled=True`, `explore_max_steps` arg (default 30), `scan_record_dir` arg. Docstring states obstacle_detector, victim_rescue, mission_control, mission_monitor, battery, keyboard are deliberately not started. Node defaults in autonomous_search stay `False` for both victim flags. |
| Autonomous search | autonomous_search.py (5546 lines) | Entrance detection/crossing, VERIFY_INSIDE, grid exploration, narrow keep-out, tilt guard → FAULT_HOLD, victim memory/confirmation/approach/VICTIM_HOLD; publishes `/cmd_vel_raw` and `/victim_found`. |
| Safety controller | safety_controller.py | `/cmd_vel_raw` → `/cmd_vel`. Front ±30°: safe 0.55 m (slow 0.15 m/s), emergency 0.30 m; rear ±30°: 0.45/0.30 m; max fwd 0.60, rev 0.15 m/s. Publishes zero if scan older than 1.5 s or command older than 0.5 s. **No tilt logic** (tilt is in autonomous_search). |
| YOLO detector | yolo_detector.py | Pretrained `yolo11n.pt`, COCO class 0 (person) only, confidence threshold 0.50. Not retrained. |
| Distance estimator | victim_distance_estimator.py | Fisheye pixel→bearing model, associates bbox angular span with LiDAR ranges. `process_detection` takes `max(detections, key=confidence)` — **one person per frame** ("MVP: only the most confident person is tracked"). |
| Prioritizer | victim_prioritizer.py | Pure functions (imports only `math`); see §7. |
| Mission Monitor | disaster_bot_monitor.py | Read-only terminal dashboard; see §9. |
| Grid | exploration_grid.py | Occupancy/frontier grid used by the planner. |

Victim-confirmation constants (autonomous_search.py ~L946–965, params ~L1497–1508): min confidence 0.60, range 0.20–3.5 m, bearing ≤100°,
readings accepted only while IMU yaw rate < 3 °/s ("not rotating" — **not** "stationary"), `victim_confirm_count` 3 within 0.50 m,
`victim_stop_distance_m` 0.80, reached when `range − 0.80 ≤ 0.05` (VICTIM_REACHED_TOL_M), close-range hold if unseen at ≤1.2 m,
max 8 approach moves, headings tried 0, +10, −10, +20, −20°.

---

## 4. Verified Mission Results

### 4.1 Disaster-site world (earthquake_disaster.sdf; spawn (0,−1.2); one world victim at GT (7.35, 6.65) unless noted)

| Run (path) | Code / launch / config | Result | Steps | First accepted victim reading | Confirmed (readings) | Approach | Hold range / GT dist | Post-hold | Safety | Roll/pitch max | Proc/pub uniqueness | Shutdown |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| **Unified launch mission** `master_launch_20261008/mission` (+ `mission.report.txt`, `mission.start.log`) | 2b2b92e1; `ros2 launch my_first_robot disaster_bot_full.launch.py gui:=false record_dir:=…` (defaults: 30 steps, approach+prioritization on) | **VICTIM_HOLD t=916.9** | 25 | t=861.09, 1.59 m, conf 0.82, during EXPLORE_MOVE [audit-derived] | t=886.2 (22 readings) [audit-derived] | 0° corridor_blocked → **+10°**, move 0.591 m | range 0.84 m / **GT 0.939 m**; estimate error 0.189 m | 0.003 m moved after hold to t=952.0 | SC 0/0, cuts 0, refusals 0, escapes 0, tilt guard 0, FAULT_HOLD 0 | 0.01° / 0.81° | pubwatch 96 samples: all 1 (0 only at start/end) | SIGINT to process group; "launch exited after ~4s"; **"NO ORPHAN PROCESSES"** |
| **Monitored mission** `monitor_20261008/mission` | 2b2b92e1; unified launch + `disaster_bot_monitor` | **VICTIM_HOLD t=877.1** | 24 | t=821.63, 1.56 m, conf 0.81 [audit-derived] | t=846.7 (16 readings) | 0° blocked → **+10°**, 0.585 m | range 0.83 / **GT 0.945**; est. err 0.203 | 0.004 m to t=910.9 | SC 0/0, all guards 0, FAULT 0 | 0.01° / 0.79° (from monitor display; not in report.txt) | all 1 | "NO ORPHAN PROCESSES" (helpers INT→TERM→KILL) |
| **Final single-victim regression** `victim_prioritizer_20261008/final_regression` | 2b2b92e1; harness `SEARCH_ARGS` (30 steps, approach+prio true) — **not** the unified launch | **VICTIM_HOLD t=875.7** | 24 | t=819.80, 1.59 m, conf 0.77 [audit-derived] | t=845.0 (23 readings) | +10°, 0.591 m | range 0.84 / **GT 0.936**; est. err 0.189 | 0.003 m | SC 0/0, all 0 | 0.01° / 0.80°; IMU-vs-GT heading err max 1.55° | all 1 | "all stopped" (no orphan scan recorded in this file) |
| Prioritizer Case A `…/runs/caseA` | f76fe57c (pre-hardening) | VICTIM_HOLD t=880.1 | 24 | t=824.48 | t=849.5 (10) | +10°, 0.605 m | 0.83 / GT 0.930 | 0.004 m | SC 0/0, all 0 | 0.01° / 0.79° | all 1 | not recorded |
| East-first, 30 steps `disaster_mission_30_20261007/REPORT.md` | 3863bd8a; approach on, prioritization **off** | VICTIM_HOLD t=545.0 | 14 | t=490.5, 1.40 m, conf 0.53 | t=514.6 (3, **all during EXPLORE_MOVE**, 0 stationary) | 0° ok, 0.51 m | 0.83 / GT 0.933; est. 0.225 m from GT | 0 motion 33 s | SC 0/0, all 0; front-cone min 0.348; footprint min wall 0.175, rubble 0.156, beam 0.214 | 0.01° / 0.79° | 56/58 ticks all 1; est/YOLO ratio 0.982 | not recorded |
| West-first repeat, 30 steps `disaster_mission_30_repeat_20261008/REPORT.md` | 3863bd8a; approach on, prioritization off | VICTIM_HOLD t=903.3 | 24 | t=825.6, 1.41 m, conf 0.60 | t≈874 (8: 7 stationary + 1 translating) after look turn | 0/±10° blocked → **+20°**, 0.55 m | GT 0.921 | 0.000 m over ~33 s after facing turn | SC 0/0, all 0; front-cone min 0.254 (in-place rotation only); footprint min wall 0.116, rubble 0.173, beam 0.223 | 0.01° / 0.79° | 91/93 ticks all 1 | not recorded |
| Baseline, 15 steps `disaster_mission_20261007/REPORT.md` | 3863bd8a | **INSIDE_HOLD (budget exhausted) t=582.5**, victim not found | 15 | none (victim never in view) | — | — | — | cmd 0 | SC 0/0, all 0; footprint min wall 0.199, rubble 0.198 | 0.01° / 0.79° | 65/68 ticks all 1 | not recorded |

Wall-clock (search-node first log → VICTIM_HOLD, [audit-derived]): unified-launch mission 1483.6 s (~24.7 min; launch 14:06:05, SIGINT 14:31:59);
monitored 1504.0 s; final regression 1477.1 s. Simulation-to-wall ratio ≈ 0.6.

Entrance/interior (direct): unified mission ENTERING t=7.9 → INSIDE verified t=70.5; monitored 8.9 → 61.4; final regression 8.7 → 61.4.

**Route variability (direct):** with the same code and budget, the first disaster runs went east (14 steps) or west (24–25 steps); the 15-step
budget is insufficient for the west route. 30 steps succeeded in every recorded 30-step run (6/6 single-world-victim runs: 30_20261007,
30_repeat, Case A, final regression, master launch, monitor). This is n=6, not a statistical guarantee.

### 4.2 House world (turtlebot3_house; spawned static person) — all on 3863bd8a, `explore_max_steps 15`, approach on, prioritization off
| Run | Victim GT | Result | Confirmed (readings) | Approach | GT dist | Safety | Source |
|---|---|---|---|---|---|---|---|
| full_mission run_v1 | (3.4, 0.5) | VICTIM_HOLD t=184.8 ("not seen at close range 0.80 m") | t=125.2 (12 stationary) | 0° 1.00 + 0.35 m | 0.905 | SC 0/0, tilt 0 | full_mission_20261006/REPORT.md |
| V1 #2 / #3 | (3.4, 0.5) | t=184.1 / 181.3 | t=124.3 / 121.9 (10 / 10) | 0°, 1.00+0.35 / 1.00+0.32 | 0.907 / 0.925 | SC 0/0, tilt 0, FAULT 0 | v1_repeat_20261007/REPORT.md |
| Clean V2 | (−1.5, 1.2) | t=190.6 | 10 stationary | +10° 1.00 m, then +20° 0.93 m | 0.956 | SC 0/0; corner centre 0.292 m | clean_v2_20261007/REPORT.md |
| V3 generalization (unseen SE wing) | (5.5, −3.0) | t=452.6 | t=354.3 (84: 64 translating, 20 stationary) | +20°, 0°, −10° | **0.775** | SC 0/0; wall body clearance min 0.200 | generalization_20261007/REPORT.md |
| verify_minwidth flagoff + E1–E5 | — | 6/6 INSIDE on scan 1 | — | — | mailbox 0.101–0.141 m | SC 0/0, tilt 0 | verify_minwidth_20261006/REPORT.md |

Superseded: victim_approach / victim_cue / victim_offsets runs (2026-10-06) ran with duplicate estimator processes (2–56 copies per reading,
full_mission_20261006/REPORT.md "Integration findings") — their confirmation counts are contaminated; only victim_approach V1 and later
runs are clean.

---

## 5. Victim Detection and Distance Estimation

| Claim | Status | Evidence |
|---|---|---|
| YOLO11n pretrained, person class only, conf ≥ 0.50 | Implemented (code) | yolo_detector.py L28–31, L92 |
| Model file unchanged | **Not verifiable** — no prior hash; current `0ebbc80d…`, mtime 2026-09-08 | this audit |
| Estimator fuses bbox bearing with LiDAR | Implemented; unit-tested (test_victim_distance_gate.py, 9 pass) | victim_distance_estimator.py |
| Bearing model accuracy | Simulation: LiDAR vs model median 0.5–0.6 cm (LiDAR itself); estimator detection at ~2 m conf 0.50–0.55, ~2.9 m conf 0.54–0.82 | disaster_site_validation_20261007/REPORT.md |
| Victim-position estimate error (nav-frame → world) | 0.06–0.25 m across runs (e.g. 0.189 / 0.203 / 0.189 m in the three current-code runs) | report.txt files; REPORTs |
| Detection at very close range | Fisheye crops the person at ~0.93 m → no detection (why close-range hold exists) | disaster_site_validation REPORT |
| False positives | 1 recorded (mailbox, outside, house V3 t=35.9) — ignored by node; 0 in disaster baseline | generalization REPORT; disaster_mission REPORT |
| **One person per frame** | Implemented limitation (code: `max(detections, key=confidence)`) | estimator L219 |
| Confirmation readings are "not rotating", not necessarily "stationary" | Confirmed by code (IMU yaw-rate < 3 °/s) and data: east-first disaster run confirmed from 3 readings all during EXPLORE_MOVE | disaster_mission_30_20261007 REPORT; generalization REPORT |

Note on "first detection": reports record the first **accepted /victim_distance reading** (or estimator output), not the first raw YOLO
box. YOLO per-frame detection times are not logged in the unified-launch runs (yolo.log contains only startup lines) → "Not recorded".

---

## 6. Victim Approach and Stopping

- Approach evaluates up to five headings (0, +10, −10, +20, −20°) per decision; each must pass the corridor/straight-path check, remembered
  narrow keep-out, entrance guard, fresh live-scan check and pre-rotation check before any motion. Implemented; unit-tested
  (test_victim_approach.py 68 pass). **Exercised live:** +10° (three current-code runs), +20° (west-first, clean V2), −10° (V3).
- **Stop criterion is a LiDAR range to the victim estimate**, `range − 0.80 m ≤ 0.05 m` → hold at ≤ 0.85 m node range. The VICTIM_HOLD log
  text reads e.g. "range 0.84 m <= 0.80 m"; this omits the 0.05 m tolerance and is cosmetically misleading but consistent with the code.
- **GT robot-centre distances at hold:** current code 0.936–0.945 m; earlier disaster runs 0.921–0.933 m; house 0.775–0.956 m.
- **0.80 m is not a guaranteed minimum.** Case C held at node range 0.58 m / GT 0.711 m because an exploration step (not the approach)
  brought the robot inside the stop distance before confirmation ("+0:reached"); V3 held at GT 0.775 m. The approach never commands a move
  that would end closer than the stop distance, but exploration motion is not victim-aware.
- After VICTIM_HOLD: facing turn (angular only), then all-zero `/cmd_vel_raw` and `/cmd_vel` for ≥30 s and ≤0.008 m GT drift in every run.
- `/victim_found` republished every 5 s while holding (7 messages per 30 s hold in each report).

---

## 7. Victim Prioritization

### 7.1 Formula (verified in victim_prioritizer.py, hash 145889bb)
```
score = 0.35·conf_s + 0.35·dist_s + 0.30·access_s − 0.20·risk
conf_s   = clamp((mean_confidence − 0.60) / 0.40)      # MEAN over a track's readings
dist_s   = clamp(1 − (range − 0.80) / (3.5 − 0.80))    # 1 at stop distance, 0 at 3.5 m
access_s = {0°: 1.0, ±10°: 0.75, ±20°: 0.50}; no passing heading → infeasible (not selectable)
risk     = max over narrow obstacles of clamp(1 − (d_segment − keep_radius)/0.30); person's own legs within the person radius ignored
```
Ranking: feasible first, then score, shorter range, higher confidence, lower id. Switching requires +0.10 score margin and is only possible
before the first approach move (target lock). The brief's formula is correct, with the caveat that `confidence` is the *rescaled mean*,
whereas `/victim_found` and the Monitor's "Max confidence" report the *max* — two deliberately different figures (hardening diff).

### 7.2 What it is and is not
Robotic, accessibility-based ranking from detector confidence, range, approach-heading accessibility and narrow-obstacle proximity.
It does **not** estimate injury severity, consciousness, medical urgency, survival probability, or who medically needs help first.
The module docstring and the Monitor both state this ("Robotic prioritization only - not medical triage; injury severity is unknown").

### 7.3 Evidence levels
| Capability | Implemented | Unit-tested | Sim-tested | End-to-end demonstrated |
|---|---|---|---|---|
| Score/rank/choose/hysteresis/lock | ✅ | ✅ 55 pass (test_victim_prioritization.py) | ✅ | ✅ single confirmed track (current code ×3) |
| Infeasible candidate rejected, alternative chosen | ✅ | ✅ | ✅ Case C (f76fe57c): left person "no safe heading" ×3 (all 5 headings corridor_blocked/narrow_keepout), right person held, GT 0.711 m | Partially — the right person was reached by exploration, not by a ranked approach |
| Two persons in view → nearer/feasible chosen | ✅ | ✅ | ✅ Case B (f76fe57c): `vic_L` chosen, VICTIM_HOLD GT 0.915 m; vic_R min 1.736 m | **Ranking among cue groups only** — live, L had 8 confirming readings, R ~1; only ONE confirmed track was ranked |
| Ranking of ≥2 *confirmed* tracks | ✅ | ✅ | ❌ | **Not validated live** (estimator one-person-per-frame) |
| Simultaneous multi-victim tracking / sequential multi-victim rescue | partly (memory keeps tracks) | partly | ❌ | **Not demonstrated** — every mission ends at the first VICTIM_HOLD |
| M2 hardening fix (keep-out stats restored after scoring) | ✅ | ✅ | not triggerable live (0 narrow detections in the decision scan) | offline replay of the live decision: identical result (final_regression/m2_replay.txt) |

Cases A/B/C ran on f76fe57c; the hardening (→2b2b92e1) changed docstrings, removed an always-False `locked=` argument and restored
diagnostic counters — no motion-path change per the diff (+33/−6 lines, inspected). Only single-victim scenes were re-run on 2b2b92e1, so
Case B/C behaviour on the current build is **inferred**, not re-measured.

---

## 8. Navigation and Safety Validation

| Item | Unit tests | Full-mission / Gazebo evidence | Notes |
|---|---|---|---|
| Entrance detection & crossing | test_verify_inside_width (46), test_crossing_align (3/4 pass) | Every disaster run: entrance confirmed then INSIDE verified (e.g. t=7.9→70.5); house 36/36 archived entries verify on replay; 6/6 Gazebo | Door not detected from the spawn pose itself (grazing 2.73 m) — robot must move first (disaster_site_validation). Entrance sidestep "hunting" up to 3 sidesteps seen at tolerance 0.02 in some house runs. |
| Interior verification | ✅ | ✅ all runs | V2a false-rejection fixed by width-min window (verify_minwidth) |
| Grid/frontier exploration | test_exploration_grid (13), plan_path_margin (4) | 60-step full-house run; disaster 14/24/25-step routes | Route depends on near-tie frontier scores; narrow 0.70 m passage is unplannable by the grid (clearance 0.20 < 0.30) |
| Narrow-obstacle keep-out & release | test_narrow_keepout (67) | Live cuts/refusals/escapes exercised in house runs (release_validation, width_rule); 0 in current-code disaster runs | Live jamb **release path never exercised** in Gazebo (only replay/2-D closed loop) |
| Wall-end clearance | plan_path_margin | wallend_mismatch Gazebo validation (2×60 moves) | — |
| Approach-heading evaluation | test_victim_approach (68) | §6 | — |
| SafetyController front/rear limits | **no unit tests** | SC 0 zeroed / 0 capped in every recorded mission → **the controller never had to intervene**; its limiting behaviour is not exercised in this evidence set | Validation shows the planner stays clear, not that the SC works under stress |
| Stale command / stale scan → zero | **no unit tests**; code inspected (0.5 s / 1.5 s) | **Not directly tested** | — |
| Tilt monitoring → terminal FAULT_HOLD | test_narrow_keepout tilt tests (threshold + node) | Max roll ≤0.02°, pitch ≤0.81° in all runs; tilt guard 0, FAULT_HOLD 0 → **never triggered live** | Disaster world is flat; ramps/rubble climbing not tested |
| Collisions | — | **No contact/bumper sensor.** Some reports compute GT-footprint clearance to modelled obstacles (min wall 0.116 m, rubble 0.156 m, beam 0.214 m in disaster runs; min body clearance 0.090–0.20 m in house). Current-code runs record only min raw LiDAR range (0.218 m, master) and SC counts | "0 collisions" = no geometric overlap found in GT analysis where computed; not proof from a contact sensor |
| Clean shutdown / orphan check | — | Unified launch: SIGINT to process group, "NO ORPHAN PROCESSES" (master_launch, monitor). Earlier harness bug left orphan estimators (fixed in harness, not code) | Shutdown noise: autonomous_search rclpy take_message traceback, parameter_bridge exit −11 (project notes; not in a report file). A `ros2 bag record` leftover from the master-launch phase is recorded only in project notes (not in the logs; the master-launch leftover pattern did not include the bag recorder) |
| Legacy obstacle_detector | — | Not launched by disaster_bot_full; not run in this audit | Still installed as an entry point |

No claim of universal collision avoidance or guaranteed safe operation is supported.

---

## 9. Mission Monitor Validation

| Requirement | Evidence | Status |
|---|---|---|
| Read-only; no motion publisher | Code: no `create_publisher`; Node created with `enable_rosout=False, start_parameter_services=False`. Live graph (monitor_20261008/mission/monitor_graph.txt): Publishers = `/parameter_events` only (rclpy-internal); `/cmd_vel` publisher count 1 (safety_controller), `/cmd_vel_raw` 1 (autonomous_search). Tests `test_module_never_publishes`, `test_node_creates_no_motion_or_application_publishers` pass | ✅ verified |
| Mission state & exploration progress | Live final frame: State VICTIM_HOLD since t=877.1, steps 24/30, last move, nav pose | ✅ live |
| Prioritization info | Live: ranking table (#1 0.588, access 0.75, risk 0.00), "Why selected" | ✅ live (single candidate); multi-candidate & infeasible display unit-tested only |
| Approach status & heading candidates | Live: "+0 deg corridor_blocked, +10 deg ok", move 0.58 m, REACHED | ✅ live |
| Safety interventions via /cmd_vel_raw vs /cmd_vel | Live: 0 zeroed / 0 capped "since monitor start"; rule unit-tested | ✅ (zero case only live) |
| Collision data honesty | Displays "not observable live (no contact topic); see mission report" | ✅ |
| VICTIM_HOLD / INSIDE_HOLD / FAULT_HOLD | VICTIM_HOLD live; INSIDE_HOLD and FAULT_HOLD banners unit-tested only | partial |
| No medical-triage wording | `test_wording_is_robotic_not_medical` passes; live text "not medical triage" | ✅ |

Limitations (from code/tests): state parsed from autonomous_search `/rosout` text (format-coupled); intervention counts start at monitor
start; access/risk terms only with `record_dir`; events before the monitor starts are not recoverable except via `[NAV]` lines
(`test_late_start_from_nav_lines_only`); collisions not observable. The captured final frame was rendered after shutdown (nodes ×0,
"publisher none"), which is expected. Monitor 19/19 tests pass today; one monitored mission only.

---

## 10. Test Suite Results

### 10.1 Re-run during this audit (2026-10-12, code hashes as in §2, `python3 -m pytest test/`)
**286 passed, 3 failed, 1 skipped** (52.9 s).

| File | Pass | Fail | Notes |
|---|---|---|---|
| test_victim_approach.py | 68 | 0 | |
| test_narrow_keepout.py | 67 | 0 | includes tilt guard |
| test_victim_prioritization.py | 55 | 0 | |
| test_verify_inside_width.py | 46 | 0 | |
| test_disaster_bot_monitor.py | 19 | 0 | |
| test_exploration_grid.py | 13 | 0 | |
| test_victim_distance_gate.py | 9 | 0 | |
| test_plan_path_margin.py | 4 | 0 | |
| test_crossing_align.py | 3 | **1** | `test_alignment_not_improving_early_abandons` — **functional**, pre-existing since 2026-09-24 |
| test_flake8.py | 0 | **1** | lint: 74 issues (baseline; +1 E501 from the launch file) |
| test_pep257.py | 0 | **1** | lint: 300 docstring errors (baseline) |
| test_mypy.py, test_xmllint.py | 1 + 1 | 0 | |
| test_copyright.py | — | — | 1 skipped |

### 10.2 History (from project notes / reports — consistent trend, not re-verified individually)
212 pass (verify_minwidth) → 247 (prioritizer) → 267 (hardening) → **286 (monitor, 2026-10-08)**, always with the same 3 failures.
Today's re-run reproduces 286/3/1 exactly on the current files, so the historical figure is **current, not stale**.
There are no tests for safety_controller.py, yolo_detector.py, or the launch file; launch behaviour is evidenced only by Gazebo runs.

---

## 11. Known Limitations and Unverified Claims

1. Single-person-per-frame estimator → live multi-victim confirmation and ranking of multiple confirmed tracks **not demonstrated**.
2. Mission ends at the first VICTIM_HOLD; no multi-victim sequential search.
3. "Stationary confirmation" is actually "not rotating"; confirmation can happen during straight motion.
4. Stop distance is a node-estimated LiDAR range; GT stand-off varied 0.711–0.956 m; exploration can bring the robot closer than 0.80 m.
5. No contact sensor; collision-free status is a geometric GT inference where computed.
6. SafetyController interventions, stale-command/stale-scan zeroing, and the tilt → FAULT_HOLD path were **never triggered in Gazebo**.
7. 30-step budget success observed 6/6 on one world/victim placement; route choice is sensitive to near-tie frontier scores; 15 steps
   fails on the west route.
8. Narrow 0.70 m passage in the disaster world is unplannable by the grid (route uses the wide opening).
9. Victim positions are nav-frame (odometry + IMU yaw) estimates with 0.06–0.25 m error; drift up to ~0.11 m.
10. Detector at the edge of usefulness at ~2 m in the shadowed room (conf 0.50–0.55); no detection at <~1 m.
11. Only one simulated person model (static Fuel "Standing person"); no lying/occluded/partially buried victims, no real hardware.
12. Unified-launch evidence: 1 mission + 1 monitored mission + smoke test; multi-victim scenarios never run through the unified launch.

---

## 12. Evidence Gaps and Conflicting Results

| # | Item | Resolution |
|---|---|---|
| G1 | No Git repository | **Unresolved** — integrity relies on hashes + backups/diffs |
| G2 | Brief: autonomous_search "most recently 3863bd8a" vs actual 2b2b92e1 | Resolved: brief outdated; chain 3863bd8a→f76fe57c→2b2b92e1 documented by backups/diffs |
| G3 | Disaster 14/24-step and house V1/V2/V3 results ran on 3863bd8a; Cases A/B/C on f76fe57c | **Potentially stale** relative to current code. Single-victim behaviour re-confirmed on 2b2b92e1 (3 runs); multi-person cases not re-run |
| G4 | No narrative REPORT.md for victim_prioritizer_20261008, master_launch_20261008, monitor_20261008 | Gap: evidence exists only as machine reports (`*.report.txt`, `*.start.log`, `smoke_checks.txt`) and project notes. Values in this audit for those runs come from those files |
| G5 | Case B report "safety.log STOP/limit lines: 1" vs SC 0/0 | Resolved: line is the startup config message ("Front safe/emergency: 0.55 / 0.30 m"), counted by an earlier filter; current filter yields 0 |
| G6 | Hold log "range 0.84 m <= 0.80 m" | Resolved: code uses a 0.05 m tolerance; log text omits it (cosmetic) |
| G7 | Report column "access 10.0" vs term "access 0.75" (prio_report.py) | Resolved: the column prints the heading offset in degrees, the term is the accessibility score |
| G8 | Shutdown: "NO ORPHAN PROCESSES" (master launch) vs project note that a `ros2 bag record` from that phase kept running ~40 min | Not contradictory for the project stack (pattern covered only stack processes); harness-side leftover documented only in notes. **Partly unresolved** (no log artifact) |
| G9 | yolo11n.pt hash never recorded | Gap: first recorded today (`0ebbc80d…`) |
| G10 | setup.py hash never recorded | Gap: first recorded today (`9c2cb2c9…`) |
| G11 | Max roll/pitch for the monitored run absent from its report.txt | Value taken from the monitor's own display (0.01°/0.79°) |
| G12 | Collision evidence for current-code runs | Only SC counts and min LiDAR range; no GT-footprint clearance computed |
| G13 | Earlier victim runs (2026-10-06) contaminated by duplicate estimators | Superseded by clean runs; must not be cited |

---

## 13. Recommended Next Steps for Documentation (not started)

1. Decide on version control: initialise a Git repository (or archive) and record this audit's hash table as the baseline before any README work.
2. Write narrative REPORT.md summaries for master_launch / monitor / victim_prioritizer from their existing `*.report.txt` (no reruns needed).
3. README "Validated results" section should cite §4.1 rows 1–3 (current code) as primary, with the 3863bd8a disaster/house runs labelled as earlier-build evidence.
4. State limitations §11 verbatim-in-spirit: robotic prioritization not triage; one-person-per-frame; no contact sensor; safety paths not stress-tested.
5. Optional future validation (needs approval, would change state): SafetyController/stale-input unit tests; multi-person Case B/C rerun on 2b2b92e1 via the unified launch.

---

## 14. Source Report Index

| Path (under ~/disaster_bot_test_logs/ unless absolute) | Date | Content |
|---|---|---|
| master_launch_20261008/mission.report.txt, mission.start.log, mission.config, mission.hash, hashes_before/after.txt | 2026-10-08 | Unified-launch full mission, shutdown/orphan check, hashes |
| master_launch_20261008/smoke/smoke_checks.txt | 2026-10-08 | Node list, single publishers per topic, process list, search params |
| master_launch_20261008/mission/search.log | 2026-10-08 | First reading / confirmation times (audit-derived) |
| monitor_20261008/mission.report.txt, mission.start.log, mission/monitor_graph.txt, mission/monitor.out, standalone.out | 2026-10-08 | Monitored mission, monitor graph, live dashboard output |
| victim_prioritizer_20261008/final_regression/run.report.txt, m2_replay.txt, run.hash | 2026-10-08 | Single-victim regression on 2b2b92e1; M2 offline replay |
| victim_prioritizer_20261008/runs/case{A,B,C}.report.txt, .config, .hash | 2026-10-08 | Prioritizer Gazebo cases (f76fe57c) |
| victim_prioritizer_20261008/hardening/*.diff, backup/, delivered/ | 2026-10-08 | Hardening change record |
| disaster_mission_20261007/REPORT.md | 2026-10-07 | 15-step baseline (budget exhausted) |
| disaster_mission_30_20261007/REPORT.md | 2026-10-07 | East-first, 14 steps |
| disaster_mission_30_repeat_20261008/REPORT.md, run1/metrics_posthold.txt, core_hashes_before.txt | 2026-10-08 | West-first, 24 steps |
| disaster_site_validation_20261007/REPORT.md | 2026-10-07 | World/sensor/perception validation (scripted driver) |
| full_mission_20261006/REPORT.md | 2026-10-06/07 | House V1 + flag-off; duplicate-estimator finding |
| v1_repeat_20261007/REPORT.md | 2026-10-07 | House V1 repeatability ×3 |
| clean_v2_20261007/REPORT.md | 2026-10-07 | House V2 clean, offset retries |
| generalization_20261007/REPORT.md | 2026-10-07 | House V3 unseen SE wing |
| verify_minwidth_20261006/REPORT.md | 2026-10-06 | VERIFY_INSIDE fix, 36/36 replay, 6 Gazebo entries |
| victim_offsets_20261006/, victim_cue_20261006/, victim_approach_20261006/ REPORT.md | 2026-10-06 | Development history (contaminated confirmation counts) |
| full_house_60_20261005/, width_rule_validation_20261005/, release_validation_20261005/, wallend_mismatch_20261004/, center_tol_validation_20261006/ REPORT.md | 2026-10-04..06 | Navigation/keep-out/entrance history (house) |
| ~/ros2_ws/src/my_first_robot/{my_first_robot,launch,test}/ | current | Code and tests inspected; hashes in §2 |
| Test re-run output (scratchpad, not retained in project) | 2026-10-12 | 286 passed / 3 failed / 1 skipped |
