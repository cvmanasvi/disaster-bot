# Evidence (curated, redacted copies)

This folder holds a small, curated subset of the Disaster Bot validation records. The full private log archive is about 4.6 GB
(rosbags, scan records, CSV traces, raw node logs) and is **not** published. Each file here is a copy of an original in that archive.
The only change is redaction of absolute home-directory paths (→ `~`) and auditor scratch paths (→ `<auditor-scratch>`).
No measured value, timestamp, hash, or pass/fail outcome was changed. `PROVENANCE.sha256` lists the SHA-256 of each original and of
its redacted copy.

Start with **`FINAL_EVIDENCE_AUDIT.md`** (2026-10-12). It cross-checks these runs, separates direct evidence from inference, and lists
the limitations. Paths in it such as `~/disaster_bot_test_logs/...` refer to the private archive. Only the files listed below are in this repository.

All times `t=` are **simulation** seconds. "GT" = Gazebo ground-truth pose. "SC 0/0" = SafetyController zeroed 0 / capped 0 commands.
There is no contact sensor: "collision" statements are geometric/ground-truth inferences, as explained in the audit.

## Which build each run used
| `autonomous_search.py` | Meaning | Runs |
|---|---|---|
| `2b2b92e1` (**current, frozen**) | prioritizer + hardening | `master_launch_20261008`, `monitor_20261008`, `victim_prioritizer_20261008/final_regression` |
| `f76fe57c` | prioritizer before hardening | `victim_prioritizer_20261008/runs/case{A,B,C}` |
| `3863bd8a` | before the prioritizer (approach only) | all other folders |

`SHA256SUMS.frozen` holds the hashes of the 7 frozen files. From the workspace root, check them with `sha256sum -c evidence/SHA256SUMS.frozen`.

## Contents
| Folder / file | What it substantiates |
|---|---|
| `master_launch_20261008/` (`mission.report.txt`, `mission.config`, `mission.start.log`, `mission.preaudit.txt`, `hashes_after.txt`, `smoke/smoke_checks.txt`) | Single-command unified launch in the disaster-site world: VICTIM_HOLD t=916.9, GT 0.939 m, 25 steps; smoke test (node list, one publisher per topic); clean Ctrl+C shutdown with no orphan processes |
| `monitor_20261008/` (`mission.report.txt`, `mission.config`, `mission.start.log`, `mission.hash`, `mission/monitor_graph.txt`, `mission/monitor_final_frame.txt`) | Same mission with the read-only Mission Monitor: graph shows no motion publisher from the monitor; last dashboard frame (rendered after shutdown, hence "x0" node counts) |
| `victim_prioritizer_20261008/final_regression/` (`run.report.txt`, `run.config`, `run.hash`, `m2_replay.txt`) | Single-victim regression on the frozen build; offline replay of the prioritization decision |
| `victim_prioritizer_20261008/runs/case{A,B,C}.report.txt` + `.config` | Prioritizer Gazebo cases (one world victim; two people in view; one person blocked). Built on `f76fe57c`. Only one confirmed track was ranked live |
| `disaster_mission_20261007/REPORT.md` | 15-step baseline: **budget exhausted, victim not found** (failure case, kept on purpose) |
| `disaster_mission_30_20261007/REPORT.md`, `disaster_mission_30_repeat_20261008/REPORT.md` | 30-step missions, east-first (14 steps) and west-first (24 steps) |
| `disaster_site_validation_20261007/REPORT.md` | Disaster-site world validation (scripted driver, sensors, perception at range) |
| `full_mission_20261006/`, `v1_repeat_20261007/`, `clean_v2_20261007/`, `generalization_20261007/`, `verify_minwidth_20261006/` `REPORT.md` | Earlier TurtleBot3-house validations (approach, repeatability, heading offsets, unseen placement, entrance verification) |

## Known gaps (see the audit, sections 11–12)
- The three 2026-10-08 folders have machine-generated `*.report.txt` files, not narrative reports.
- Some numbers quoted in the audit were derived from raw logs that aren't published here (e.g. confirmation reading counts, wall-clock durations).
- Raw data (rosbags, scan records, CSV traces, node logs) is kept only in the private archive.
