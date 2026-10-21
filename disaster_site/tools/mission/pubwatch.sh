#!/bin/bash
# usage: pubwatch.sh <run_dir> -- every ~10 s until <run_dir>.start.log has END: process counts,
# node-name counts and publisher counts of the perception/velocity topics -> <run_dir>.pubwatch.csv
R=$1; source /opt/ros/lyrical/setup.bash; source ~/ros2_ws/install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
O=$R.pubwatch.csv
echo "wall,sim,p_yolo,p_est,p_search,p_safety,n_yolo,n_est,n_search,n_safety,pub_camera,pub_det,pub_dist,pub_cmdraw,pub_cmd,pub_found" > $O
pc() { ps -eo args | grep -cE "$1"; }
pub() { ros2 topic info $1 2>/dev/null | awk '/Publisher count/ {print $3}'; }
until grep -q "^END" $R.start.log 2>/dev/null; do
  s=$(tail -n1 $R/gt.csv 2>/dev/null | cut -d, -f1)
  a=$(pc "^$HOME/ros2_ws/yolo_env/bin/python src/my_first_robot/my_first_robot/yolo_detector")
  b=$(pc "^/usr/bin/python3 $HOME/ros2_ws/install/my_first_robot/lib/my_first_robot/victim_distance_est")
  c=$(pc "^/usr/bin/python3 $HOME/ros2_ws/install/my_first_robot/lib/my_first_robot/autonomous_sea")
  d=$(pc "^/usr/bin/python3 $HOME/ros2_ws/install/my_first_robot/lib/my_first_robot/safety_contr")
  nl=$(timeout 10 ros2 node list 2>/dev/null)
  n1=$(grep -c "yolo" <<<"$nl"); n2=$(grep -c "distance" <<<"$nl"); n3=$(grep -c "autonomous_search" <<<"$nl"); n4=$(grep -c "safety" <<<"$nl")
  echo "$(date +%T),$s,$a,$b,$c,$d,$n1,$n2,$n3,$n4,$(pub /camera/image_raw),$(pub /victim_detections),$(pub /victim_distance),$(pub /cmd_vel_raw),$(pub /cmd_vel),$(pub /victim_found)" >> $O
  sleep 8
done
echo "watch ended $(date +%T)" >> $R.pubwatch.log
