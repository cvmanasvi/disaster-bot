# Full end-to-end mission validation (2026-10-06/07) — validation only, no source change

Code: autonomous_search.py 3863bd8a (frozen); all frozen files unchanged (final_hashes.txt; run_*.hash = installed copies).
Harness: tools/run_mission.sh (= run_one.sh + probe.sh topology/param dump + 30 s post-VICTIM_HOLD observation),
tools/drive.sh. Exact parameters: run_*/params_autonomous_search.yaml; topology: run_*/topology.txt; run args: run_*.config.
Victim: static Fuel "Standing person" (tools/spawn_victim.sh) at world (3.400, 0.500), = victim_approach V1 / offsets V1 placement.

## run_flag_off  (victim_approach_enabled false, default explore_max_steps 8, no victim)
INSIDE verified scan 1 (t=94.1, 3 sidesteps — door-centre estimate jitter at the 0.02 tolerance, max sidesteps reached);
8 moves all target_reached, 9 scans, final INSIDE_HOLD (explore_steps_done) t=383.8. SC 0/0, keep-out cuts 0, refusals 0,
escapes 0, tilt 0, FAULT 0, max roll 0.02 / pitch 0.80 deg; mailbox 0.145 m, jambs 0.385/0.336 m.
Route went WEST at step 1 (earlier flag-offs went east): frontier tie 0.984 vs 0.960 flipped by a few cm of entry pose.

## run_v1  (victim_approach_enabled true, explore_max_steps 15)
see run_v1/victim_summary.txt. INSIDE scan 1 t=78.6; first estimator output t=91.5; cues t=91.9..; look t=99.9 (turn done 103.9);
stationary readings 104.4..104.9; CONFIRMED t=125.2 (12 stationary readings); move 1 (0 deg, 1.00 m) t=125.2; move 2 (0 deg,
0.35 m) t=158.7; VICTIM_HOLD t=184.8 ("not seen at close range 0.80 m"); /victim_found 7 msgs (5 s republish) over 32 s hold.
GT robot->person 0.905 m, heading error +5.0 deg; estimate error 0.250 m; person base-plate body clearance 0.524 m.
Post-hold: cmd_vel_raw and cmd_vel exactly 0, GT motion 0.000 m / 0.00 deg for 32 s.

## Integration findings
1. Orphan yolo_detector / victim_distance_estimator / ros2 topic echo processes from earlier sessions (run_one.sh killall_sim
   awk syntax error) were alive during run_flag_off (7 estimators, 2 YOLO). Killed by run_mission.sh; run_v1 had exactly one each.
   tools/estimator_dupes.py: earlier victim runs had N copies of every /victim_distance reading (victim_approach V2 2-4,
   cue V1 3-6, cue V2 4-16, offsets V1 5-30, offsets V2 7-56, V2a 6-24); only victim_approach V1 and this run_v1 were clean.
2. 7 readings accepted at the start of the approach turn (t=125.25-125.57, GT yaw rate 2.7-12.6 deg/s): 1.0 s IMU-rate window
   lags at turn onset. After confirmation; shifted target y 0.873->0.814, corrected by move readings within 3 s.
