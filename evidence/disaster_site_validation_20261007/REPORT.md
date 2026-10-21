# Disaster-site world: environment validation (2026-10-07). No project code changed; full mission NOT run.
World: ~/ros2_ws/disaster_site/world/earthquake_disaster.sdf (generated from tools/layout.py). Launch:
TURTLEBOT3_MODEL=burger_cam ros2 launch ~/ros2_ws/disaster_site/launch/disaster_site.launch.py [gui:=false]
Headless run (gui:=false), GZ_IP=127.0.0.1, ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST, RTF ~0.57.
- Loads: 0 errors; warnings = robot-SDF gz_frame_id deprecation (as house), <gui><camera>, Ogre2 mask x3 (my evidence cams).
- Spawn GT (-0.0004,-1.200,0.010) pitch 0.17 deg; nearest obstacle 1.09 m. Victim GT (7.350,6.650,0.000); unchanged at end.
- Topics (wall Hz @RTF 0.57): scan 2.86, image 17.0, camera_info 16.8, imu 110.6, odom 28.1; TF odom->base_footprint->base_link->
  {base_scan, imu_link, camera_link->camera_rgb_frame->optical, wheels, caster}.
- LiDAR vs model: median 0.5-0.6 cm, 95% <= 2.1 cm (4 scans).
- Drives (scripted GT waypoint driver -> /cmd_vel; not Disaster Bot): R1 spawn->door->lobby->NARROW 0.70->beam pocket->around beam
  east end->D door->victim approach (15.3 m, completed, min front 0.384, pitch <= 0.72 = rest baseline). R2 D->E->WIDE opening->lobby
  (aborted by my driver guard at r_lobby_corr: bad waypoint line, no contact; then zero-cmd creep 8 cm to 2 mm from the block — same
  robot-physics creep as the house). R2b lobby->B->C dead end->back (10.7 m, completed).
- Blocked: corridor stop y 2.867 (target 4.5); facade door stop y -0.78 (from yard); beam west end stop (4.60,3.19). Dead end C: 360 scan.
- Frozen detect_entrances on real yard scans: real door found from 11/16 poses (width 0.92-1.02, centre +-5 cm), NOT from spawn
  (2.73 m grazing) nor (0.8,-1.2), (2.45,-2.0); blocked door never detected (0 false entrances).
- Perception chain (unmodified yolo_detector + victim_distance_estimator): 0.93 m: no detection (fisheye crops person, as house hold);
  ~2.0 m: conf 0.50-0.55, 1.86 m; ~2.9 m: 144 readings conf 0.54-0.82 mean 0.75, 2.75 m.
- Hashes: core 6 files identical (src+install) to frozen set; no file in turtlebot3_gazebo or src modified.
