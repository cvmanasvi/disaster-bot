# Disaster-site repeatability run, explore_max_steps 30 (2026-10-08) — frozen code/world, same harness
Audit CLEAN; 91/93 pubwatch ticks all 1 (startup/shutdown); est/YOLO ratio 0.989, 0 same-instant pairs; 1 victim (world).
Core + world hashes unchanged.

RESULT: SUCCESS after the unfavourable WEST-first route. VICTIM_HOLD t=903.3, GT robot->person 0.921 m, 24 exploration steps.
Phases: step 1 east; 2-4 target B (robot reaches B at step 4); 5-6 target C (ends at C doorway y 4.66, C not entered);
7-12 backtrack (back in lobby at step 9); 13-15 east lobby; 16 first east-hall target (wide opening); 17 enters E;
18-20 beam east-end gap; 21-22 north lane; 23 D doorway; 24 inside D. Steps 13-24 mirror steps 3-14 of the east-first run.
Victim: first /victim_distance t=825.6 (1.41 m, conf 0.60, during step 24 move); 187 YOLO detections conf 0.50-0.86 (mean 0.69);
look turn t 849.7-852.7 (36 agreeing cues, lin cmd 0, GT 0.008 m pivot); CONFIRMED t ~874 from 8 readings: 7 STATIONARY
(STOP_AND_SCAN, GT 0.0000 m/s, yaw 0) + 1 translating (EXPLORE_MOVE 0.10 m/s); 103 rotating cues not counted.
Approach t=874.0 range 1.296: 0/+10/-10 corridor_blocked (plan free 0.00), +20 ok (plan 1.00, live free 1.05, live keep-out 0.55,
pre-rotation passed) -> 0.55 m. /victim_found 7 msgs, world (7.227,6.480), 0.210 m from origin; heading err +1.1 deg.
Post-hold: facing turn to 905.3, then all-zero cmd 33 s, GT motion 0.000 m.
Safety: SC 0/0, cuts/refusals/escapes/tilt/FAULT 0; roll 0.01 / pitch 0.79 deg; front cone min 0.254 m only during in-place
scan rotation at the C doorway (0 forward cmds while < 0.30); footprint min wall 0.116 (mid_n_1, scan rotation at C door),
rubble/debris 0.173 (pile_1), beam 0.223, plate 0.488; drift max 0.105 / final 0.101 m. Final grid V/F/O/U 180/920/363/763.
