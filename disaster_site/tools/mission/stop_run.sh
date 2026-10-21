#!/bin/bash
# usage: stop_run.sh -- stop search first (zeroes cmd), then the rest; disaster-site launch pattern.
me=$$
k() { for p in $(pgrep -f "$1"); do [ "$p" != "$me" ] && kill -${2:-TERM} $p 2>/dev/null; done; }
k "lib/my_first_robot/autonomous_search" INT; timeout 2 tail -f /dev/null
k "lib/my_first_robot/safety_controller" INT; k "yolo_detector.py" INT; k "lib/my_first_robot/victim_distance_estimator" INT
k "topic echo /victim_found"
k "ros2 bag record" TERM; k "rosbag2" TERM; timeout 3 tail -f /dev/null
k "mission/recorder.py"; k "mission/gt_reader.py"; k "gz topic -e -t /world/default/dynamic_pose"
k "disaster_site.launch.py" INT; timeout 5 tail -f /dev/null
k "gz-sim-main"; k "parameter_bridge"; k "robot_state_publisher"; k "image_bridge"
timeout 3 tail -f /dev/null; k "gz-sim-main" KILL
ps -eo pid,args | grep -E "gz-sim|my_first_robot|yolo_detector|parameter_bridge|rosbag2|mission/recorder|mission/gt_reader" | grep -vE "grep|stop_run" || echo "all stopped"
