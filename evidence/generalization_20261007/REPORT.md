# Generalization run V3 (unseen victim, SE wing) 2026-10-07 — validation only, no code change
Victim static Fuel person at GT (5.500,-3.000,0) (SE wing: separate room behind doorway x~5.8-6.7,y=-0.2; never
explored deeper than y -1.89 before; not visible from hall/V1/V2). Harness unchanged; SEARCH_ARGS explore_max_steps 15,
victim_approach_enabled true; sim cap 700. preaudit CLEAN; pubwatch 105/107 ticks all 1 (others = startup/shutdown).
Hashes identical before/after (hashes_before/after.txt). Map: housemap_60.png.
RESULT: PASS. Route east at step 1 (tie-break), NE room steps 1-6, toward SE doorway steps 7-8; first accepted reading
t=329.8 (during step-8 forward move), confirmed t=354.3 after 84 readings (64 translating in EXPLORE_MOVE at 0.02-0.10 m/s,
yaw 0; 20 stationary in STOP_AND_SCAN; stationary-only mean 0.055 m from GT, same as all). No look turn (readings accepted
before any cue needed); 202 rotating cues before confirmation, none counted.
Approach: move1 +20 deg 1.00 m (0/+10/-10 corridor_blocked by SE doorway wall), move2 0 deg 1.00 m, move3 -10 deg 0.61 m
(0/+10 narrow_keepout = person's legs detected narrow). VICTIM_HOLD t=452.6, GT centre->person 0.775 m, heading err -1.4,
estimate 0.060 m from GT, /victim_found 7 msgs, 0 motion 32 s after facing turn.
Safety: SC 0/0, tilt 0 (roll 0.02/pitch 0.79), FAULT 0, cuts/refusals/escapes/path-blocked 0; wall body clearance min 0.200
(Wall_106); cafe_table_0 col centre 0.656 (>=0.63); marble A 0.738; person plate 0.561; mailbox 0.130; jambs 0.372/0.349;
nav drift max 0.102/final 0.090. One YOLO false positive (mailbox, outside, t=35.9) ignored by node.
