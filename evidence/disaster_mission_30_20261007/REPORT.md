# Disaster-site mission rerun, explore_max_steps 30 (2026-10-07) — only change vs the 15-step baseline
Harness ~/ros2_ws/disaster_site/tools/mission/ (unchanged); world/models/launch/layout hashes unchanged; core hashes unchanged.
Audit CLEAN; 56/58 pubwatch ticks all 1 (2 = startup/shutdown); est/YOLO ratio 0.982, 0 same-instant pairs.

RESULT: SUCCESS — VICTIM_HOLD t=545.0 after 14 exploration steps (+1 approach move). GT robot->person 0.933 m.
NOTE: 14 steps < 15: this run would also have fit the old budget. Difference vs baseline = frontier choice at step 2
(east lobby frontier 0.41 here vs west B frontier 0.445 in the baseline), not the budget.
Route: lobby east (1-5) -> WIDE opening (6-8) -> beam east-end gap (9-10) -> north lane (11-12) -> D door (13) -> D (14).
B/C dead end never visited. Narrow passage not used (pocket behind it clearance 0.20 at scan 13; 0.40 at scan 8 before beam mapped).
Grid scan 13: wide opening 0.45, beam gap 0.40, north lane 0.40 (single row), D door 0.45 — all traversable+reachable.
Perception: first /victim_distance t=490.5 (1.40 m, conf 0.53); YOLO 106 detections conf 0.51-0.87 (mean 0.67).
Confirmation t=514.6 from 3 readings t 492.46-492.95, ALL in EXPLORE_MOVE (GT 0.037-0.049 m/s, yaw rate 0) — 0 stationary
readings; 36 rotating cues before confirmation not counted; no look turn.
Approach: 1 candidate evaluated, 0 deg ok (free 0.84, keep-out 1.00) -> 0.51 m. VICTIM_HOLD "range 0.83 <= 0.80" (sic, node text).
/victim_found 7 msgs, nav (7.247,7.650) -> world (7.247,6.450), 0.225 m from GT origin. Heading err +2.2 deg GT.
Post-hold: facing turn to 546.5, then cmd_vel_raw/cmd_vel all zero 33 s, GT motion 0.000 m.
Safety: SC 0/0, cuts/refusals/escapes/tilt guard/FAULT 0, roll 0.01 / pitch 0.79 deg, front cone min 0.348, footprint min wall 0.175
(lobby_n_3), rubble/debris min 0.156 (pile_1, during scan rotation), beam 0.214, plate 0.477; drift max 0.108 / final 0.107 m.
(victim_report.py's 2.16 m error / 2.08 m drift use the HOUSE spawn offset — wrong for this world; corrected values above.)
