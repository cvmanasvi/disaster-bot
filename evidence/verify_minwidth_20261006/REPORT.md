# VERIFY_INSIDE minimum-width window — implementation + validation (2026-10-06)

Change: autonomous_search.py 58438b6a… -> 3863bd8a… (+31/-4 lines, autonomous_search.diff).
- confirmation track + _entry_refine keep e['width_min'] = min of matched, unoccluded single-scan widths
  (detect_entrances-valid only); e['width'] smoothing untouched.
- _entry_verify_width(): min(e['width'], max(entry_min_width_m, width_min)); fallback e['width'].
- handle_verify_inside: half = 0.5 * _entry_verify_width(); log adds 'width avg/min/used'.
New tests: test/test_verify_inside_width.py (46), fixture test/data/verify_inside_replay.json.
Suite: 212 passed, 1 skipped, 3 known failures (crossing_align early-abandon, flake8 73, pep257 257 = baseline).

historical_replay.csv: 36 archived entries through the modified node: 36/36 INSIDE on scan 1
(V2a old 5 gap -> new 0). Min east margin -0.008 -> +0.042 (others >= +0.058); min west +0.098.

Gazebo (summary.txt): flagoff + E1..E5 all INSIDE on scan 1, gap 0, SC 0/0, tilt 0, pitch <= 0.80,
mailbox 0.101..0.141 m.
run|avg width|min width|used|gap|L/R|east/west margin|centre err (GT, door line)|sidesteps|mailbox|SC zero/cap|min jamb W/E|tilt guard, max pitch|verified scan|final
flagoff|0.981|0.894|0.894|0|40/41|+0.076/+0.130|+0.011|1|0.129|0/0|0.371/0.350|0, 0.79|1 (INSIDE) of 1|INSIDE_HOLD (explore_steps_done) t=360.70
E1|0.998|0.888|0.888|0|40/42|+0.063/+0.149|+0.024|1|0.139|0/0|0.385/0.337|0, 0.79|1 (INSIDE) of 1|INSIDE_HOLD (explore_steps_done) t=132.70
E2|0.994|0.895|0.895|0|41/41|+0.084/+0.121|+0.001|1|0.119|0/0|0.362/0.360|0, 0.80|1 (INSIDE) of 1|INSIDE_HOLD (explore_steps_done) t=131.80
E3|0.968|0.883|0.883|0|42/40|+0.106/+0.111|-0.012|1|0.101|0/0|0.348/0.373|0, 0.80|1 (INSIDE) of 1|INSIDE_HOLD (explore_steps_done) t=132.90
E4|0.978|0.889|0.889|0|40/41|+0.083/+0.128|+0.003|1|0.118|0/0|0.364/0.358|0, 0.80|1 (INSIDE) of 1|INSIDE_HOLD (explore_steps_done) t=131.70
E5|0.955|0.889|0.889|0|40/42|+0.069/+0.142|+0.024|1|0.141|0/0|0.385/0.337|0, 0.79|1 (INSIDE) of 1|INSIDE_HOLD (explore_steps_done) t=133.10
