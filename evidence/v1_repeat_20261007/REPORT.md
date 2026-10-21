# V1 repeatability (2026-10-07) — validation only, no code change
Harness unchanged (full_mission_20261006/tools/run_mission.sh), victim static (3.4,0.5), explore_max_steps 15,
victim_approach_enabled true. Per-run tools/preaudit.sh (kill stale, daemon restart, empty table+node list) + pubwatch.
Hashes unchanged: 3863bd8a ccecad4f ba84a17c 4e1e976c 67188d38 16a72415.
Note: first run-3 attempt aborted by preaudit (CONTAMINATED) — false positive: my concurrent summarize.py ->
src/my_first_robot/tools/mailbox_lidar_clearance.py matched the 'my_first_robot/' pattern. Re-run alone: CLEAN.
(run_v1_3.preaudit_aborted_selfmatch.txt)

| | V1 #1 (full_mission run_v1) | V1 #2 | V1 #3 |
|---|---|---|---|
| inside verified | t=78.6 scan 1 | t=77.7 scan 1, 1 sidestep | t=75.3 scan 1, 1 sidestep |
| cues before confirm / stationary confirm readings | -/12 | 149/10 | 146/10 |
| confirmed | t=125.2 | t=124.3 | t=121.9 |
| approach moves (0 deg) | 1.00 + 0.35 | 1.00 + 0.35 | 1.00 + 0.32 |
| VICTIM_HOLD | t=184.8 | t=184.1 | t=181.3 |
| GT robot->person | 0.905 | 0.907 | 0.925 |
| /victim_found msgs | 7 | 7 | 7 |
| mailbox / jambs W/E | - | 0.132 / 0.374/0.348 | 0.141 / 0.385/0.337 |
| nav drift max/final | - | 0.091/0.085 | 0.077/0.076 |
| post-hold motion | 0 for 32 s | 0 for 32.5 s | 0 for 33.2 s |
SC 0/0, tilt 0 (pitch <=0.80), FAULT 0, keep-out cuts/refusals/escapes 0 in all. Estimator/YOLO ratio 0.966/0.975, 0 same-instant pairs.
