#!/bin/bash
# usage: probe.sh <run_dir> -- once the search node is up: node list, publishers/subscribers of the
# velocity topics, /victim_* topics, and the full autonomous_search parameter dump.
R=$1; source /opt/ros/lyrical/setup.bash; source ~/ros2_ws/install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
for i in $(seq 1 60); do ros2 node list 2>/dev/null | grep -q autonomous_search && break; sleep 2; done
sleep 5
{ echo "== nodes"; ros2 node list
  for t in /cmd_vel /cmd_vel_raw /cmd_vel_search /victim_distance /victim_found /camera/image_raw /victim_detections; do
    echo "== $t"; ros2 topic info -v $t 2>&1 | grep -E "Type:|Node name|Endpoint type|Publisher count|Subscription count"; done
} > $R/topology.txt 2>&1
ros2 param dump /autonomous_search > $R/params_autonomous_search.yaml 2>&1
ros2 param dump /safety_controller > $R/params_safety_controller.yaml 2>&1
