#!/bin/bash
# usage: preaudit.sh <out_file> -- kill stale sim/robot processes, restart ROS daemon, verify empty table + node list.
# exit 0 = clean, 1 = contaminated (do not run).
O=$1; source /opt/ros/lyrical/setup.bash; export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
PAT="gz-sim|gz sim|ruby.*gz|my_first_robot/|yolo_detector|victim_distance_est|autonomous_sea|safety_contr|parameter_bridge|robot_state_publisher|image_bridge|rosbag2|recorder\.py|gt_reader\.py|topic echo /victim"
list() { ps -eo pid,args | grep -E "$PAT" | grep -vE "grep|preaudit" ; }
{ echo "== pre-run audit $(date -Is)"; echo "-- before cleanup:"; list || echo "(none)"
  for p in $(list | awk '{print $1}'); do kill -9 $p 2>/dev/null; done; sleep 3
  ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1
  echo "-- after cleanup:"; list || echo "(none)"
  echo "-- ros2 node list:"; timeout 15 ros2 node list; echo "(end)"
  echo "-- hashes:"; sha256sum ~/ros2_ws/install/my_first_robot/lib/python3.14/site-packages/my_first_robot/{autonomous_search,safety_controller,exploration_grid,victim_distance_estimator,yolo_detector,victim_rescue}.py
} > $O 2>&1
[ -z "$(list)" ] && [ -z "$(timeout 15 ros2 node list 2>/dev/null)" ] && { echo CLEAN >> $O; exit 0; }
echo CONTAMINATED >> $O; exit 1
