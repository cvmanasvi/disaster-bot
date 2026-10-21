#!/bin/bash
# usage: run_mission.sh <run_dir> <max_sim_sec>  -- disaster-site variant of full_mission run_mission.sh (no victim spawn).
# preaudit gate -> start_run -> probe + pubwatch -> wait for INSIDE_HOLD / FAULT / tilt / VICTIM_HOLD(+POST) / sim limit -> stop.
R=$1; LIMIT=$2; T=$(cd $(dirname $0) && pwd); POST=${POST_HOLD_SEC:-30}
source /opt/ros/lyrical/setup.bash; source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger_cam GZ_IP=127.0.0.1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
bash $T/preaudit.sh $R.preaudit.txt || { echo "ABORTED: process table contaminated" | tee $R.start.log; exit 1; }
sha256sum ~/ros2_ws/install/my_first_robot/lib/python3.14/site-packages/my_first_robot/*.py > $R.hash 2>/dev/null
echo "SEARCH_ARGS=$SEARCH_ARGS world=earthquake_disaster.sdf limit=$LIMIT post_hold=$POST (victim from world, not spawned)" > $R.config
bash $T/start_run.sh $R > $R.start.log 2>&1
bash $T/probe.sh $R > $R.probe.log 2>&1 &
bash $T/pubwatch.sh $R > $R.pubwatch.log 2>&1 &
sim() { tail -n1 $R/gt.csv 2>/dev/null | cut -d, -f1 | cut -d. -f1; }
until grep -qE "TILT_GUARD|-> INSIDE_HOLD|-> VICTIM_HOLD|-> FAULT_HOLD" $R/search.log 2>/dev/null || [ "$(sim)" -ge $LIMIT ] 2>/dev/null; do sleep 10; done
if grep -q -- "-> VICTIM_HOLD" $R/search.log; then
  th=$(grep -oE "t=[0-9.]+ \w+ -> VICTIM_HOLD" $R/search.log | head -1 | sed 's/t=\([0-9]*\).*/\1/')
  until [ "$(sim)" -ge $((th + POST)) ] 2>/dev/null; do sleep 10; done
fi
sleep 5
bash $T/stop_run.sh >> $R.start.log 2>&1; sleep 5
echo "END $(date +%T) $(grep -oE '\-> (INSIDE_HOLD|VICTIM_HOLD|FAULT_HOLD)[^)]*\)' $R/search.log | tail -1) sim=$(tail -n1 $R/gt.csv | cut -d, -f1)" >> $R.start.log
