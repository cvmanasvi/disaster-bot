#!/bin/bash
# usage: start_run.sh <run_dir>  -- disaster-site variant of full_mission_20261006/tools/start_run.sh (test tooling only).
# Differences vs the house harness: launches disaster_site.launch.py (gui:=false) and NEVER spawns a victim
# (the static victim is part of earthquake_disaster.sdf). Logging, recorder, bag topics, node order unchanged.
R=$1; mkdir -p $R; T=$(cd $(dirname $0) && pwd)
source /opt/ros/lyrical/setup.bash; source ~/ros2_ws/install/setup.bash; export TURTLEBOT3_MODEL=burger_cam
export GZ_IP=127.0.0.1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
if pgrep -f gz-sim-main >/dev/null; then echo "gz-sim-main still running -- abort"; exit 1; fi
nohup ros2 launch ~/ros2_ws/disaster_site/launch/disaster_site.launch.py gui:=false > $R/gazebo.log 2>&1 &
for i in $(seq 1 60); do timeout 5 ros2 topic echo --once /scan --field header.stamp >/dev/null 2>&1 && { echo scan-ready; break; }; done
echo "spawn pose:"; gz model -m burger_cam -p 2>/dev/null | tail -n 2 | tee $R/spawn_pose.txt
echo "victim pose (from world, not spawned):"; gz model -m victim -p 2>/dev/null | tail -n 2 | tee $R/victim_pose.txt
gz model --list > $R/model_list.txt 2>&1
nohup python3 $T/gt_reader.py $R > $R/gt_reader.log 2>&1 &
nohup python3 $T/recorder.py $R > $R/recorder.log 2>&1 &
nohup ros2 bag record -o $R/bag --topics /scan /imu /odom /cmd_vel_raw /cmd_vel /rosout > $R/bag.log 2>&1 &
for i in $(seq 1 30); do [ -s $R/gt.csv ] && [ -s $R/imu.csv ] && [ -d $R/bag ] && break; timeout 1 tail -f /dev/null; done
nohup ros2 run my_first_robot safety_controller --ros-args -p use_sim_time:=true > $R/safety.log 2>&1 &
for i in $(seq 1 15); do grep -q "Safety Controller Started" $R/safety.log && break; timeout 1 tail -f /dev/null; done
# perception: same commands as spawn_victim.sh, minus the victim spawn
cd ~/ros2_ws
nohup ~/ros2_ws/yolo_env/bin/python src/my_first_robot/my_first_robot/yolo_detector.py --ros-args -p use_sim_time:=true > $R/yolo.log 2>&1 &
nohup ros2 run my_first_robot victim_distance_estimator --ros-args -p use_sim_time:=true > $R/distance.log 2>&1 &
nohup ros2 topic echo /victim_found std_msgs/msg/String --qos-durability transient_local > $R/victim_found_echo.txt 2>&1 &
for i in $(seq 1 60); do grep -q "Listening to /camera" $R/yolo.log 2>/dev/null && break; timeout 1 tail -f /dev/null; done
echo "search start at sim $(tail -n1 $R/gt.csv | cut -d, -f1)" | tee $R/search_start.txt
nohup ros2 run my_first_robot autonomous_search --ros-args -p use_sim_time:=true -p scan_record_dir:=$R/scan_record $SEARCH_ARGS > $R/search.log 2>&1 &
echo started
