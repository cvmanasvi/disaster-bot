# Clean-process V2 victim mission (2026-10-07) — validation only, no code change

Harness: unchanged full_mission_20261006/tools/run_mission.sh (via tools/go.sh), SEARCH_ARGS
"-p explore_max_steps:=15 -p victim_approach_enabled:=true", victim (-1.5, 1.2), sim limit 650, 30 s post-hold.
Added (validation tooling only): tools/pubwatch.sh (process/node/publisher counts every ~20 s -> run_v2.pubwatch.csv),
tools/dupe_timing.py, tools/metrics.py. Pre/post audits: pre_run_audit.txt, post_run_audit.txt. Hashes identical pre/post.

RESULT: PASS — VICTIM_HOLD t=190.6; heading-offset retry genuinely exercised (+10 deg move 1, +20 deg move 2).
- Clean table: 0 processes/nodes before; 1 each of yolo/estimator/search/safety; 1 publisher on /camera/image_raw,
  /victim_detections, /victim_distance, /cmd_vel_raw, /cmd_vel, /victim_found in all 50 running watcher ticks.
- Estimator/YOLO rosout ratio 0.964, 0 same-instant pairs (identical-text repeats spaced >=0.138 s = detection period,
  concentrated in stationary hold; same pattern in clean run_v1) -> no duplicates.
- Candidates t=125.1 (nav 3.05,0.93, range 2.56): 0 deg corridor_blocked (free 0.29); +10 ok (free 1.00, live free 9.50,
  live keep-out 1.00) -> move 1.00. t=158.1 (range 1.61): 0 corridor_blocked (free 0.00); +10 already_used;
  -10 corridor_blocked (0.00); +20 ok (free 1.00, live 9.50, live keep-out 0.93) -> move 0.93. -20 not evaluated.
  Pre-rotation keep-out check passed for both offsets (no refusal).
- Confirmation: 10 stationary readings (STOP_AND_SCAN, IMU rate 0.00 deg/s); 112 turning cues before confirmation not counted;
  look turn 99.7-103.8 lin cmd 0 (GT 0.012 m pivot offset).
- Hold: GT centre->person 0.956 m (min 0.941), heading err -5.6 deg GT; estimate world (-1.482,1.021), 0.180 m from origin;
  /victim_found 7 msgs; facing turn to t=192.9 (ang only), then cmd_vel_raw/cmd_vel all zero 30.5 s, GT motion 0.000 m.
- Safety: SC 0/0, tilt 0, FAULT 0, keep-out cuts/refusals/escapes 0, path-blocked 0; roll 0.02 / pitch 0.81 deg;
  mailbox 0.117, jambs 0.360/0.362; corner (0.54,0.78) centre 0.292 m; plate clearance 0.451 m; min LiDAR ray inside 0.258;
  nav drift max 0.084 / final 0.064 m.
Details: run_v2/victim_summary.txt, run_v2/metrics.txt, run_v2/scan_record/victim_events.jsonl.
