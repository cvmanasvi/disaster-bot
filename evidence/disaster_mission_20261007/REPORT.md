# First autonomous disaster-site mission (2026-10-07) — baseline, frozen code, no parameter change
Harness (test tooling only): ~/ros2_ws/disaster_site/tools/mission/ (start_run/run_mission/stop_run = house harness with
disaster_site.launch.py gui:=false and NO victim spawn; preaudit + pubwatch + probe). SEARCH_ARGS explore_max_steps 15,
victim_approach_enabled true. Victim from world (7.35,6.65) — one victim model. Hashes unchanged (3863bd8a ccecad4f ba84a17c
4e1e976c 67188d38 16a72415). Audit CLEAN; 65/68 pubwatch ticks all 1 (others startup/shutdown).

RESULT: "Mission budget exhausted before victim discovery." INSIDE_HOLD (explore_steps_done) t=582.5 at GT (5.62,2.02), east lobby.
- Entrance: confirmed t=10.7 (robot moved 0.23 m), real door only (blocked door never a candidate); 1 sidestep; CROSS t=45.4;
  INSIDE verified scan 1 t=63.7 (lateral +0.012).
- 15 moves (12.2 m commanded, 18.3 m GT incl. entry), all target_reached, 0 live cuts. Route: step1 east toward island; steps 2-6 west
  into B and to the B->C doorway (C scanned from y 4.61, not entered); steps 7-12 backtrack via B to lobby; 13-15 east lobby.
- Final grid V/F/O/U 107/684/240/1195, frontier 90 cells/11 clusters. Final candidates: C0 lobby south (4.90,0.92) 0.571;
  C1 just inside WIDE opening (7.20,2.80) 0.418 reachable; C3 E pocket behind NARROW passage (6.27,3.13) UNREACHABLE.
- Grid replay (analysis/grid_traversability.py): narrow passage cell clearance 0.40 (traversable) but the 0.72 m pocket between lobby
  wall and fallen beam has clearance 0.20 (< grid_min_clear_m 0.30) -> narrow-passage route not plannable by the frozen grid.
- Perception: 0 YOLO detections, 0 estimator outputs (victim never in view — consistent with design), 0 false positives.
- Safety: SC 0 zeroed/0 capped (entry_report rule); naive matcher's 12 flags = 1-cycle onset relay lag (front >= 0.87 m).
  cuts/refusals/escapes/tilt/FAULT 0; roll 0.01 / pitch 0.79 deg; front cone min 0.371; footprint min wall 0.199, rubble 0.198;
  drift max 0.082 / final 0.074 m. After INSIDE_HOLD cmd 0.
