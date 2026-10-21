# Disaster Bot — Setup and Execution Guide

Every command is labelled:
- **[validated]**: used in recorded runs on the development machine (evidence cited);
- **[inspected]**: derived from source or config, but not run as written;
- **[untested]**: never run. This includes every clean-machine step.

**Reproduction on a fresh machine has never been attempted.** Treat sections 1–4 as a best-effort recipe, not a verified procedure.

## 1. Platform (the only configuration ever used)

| Item | Version on the development machine |
|---|---|
| OS | Ubuntu 26.04 LTS ("Resolute Raccoon") |
| ROS 2 | Lyrical (`/opt/ros/lyrical`), e.g. rclpy 10.0.10, cv_bridge 4.1.0 |
| Gazebo | Gazebo Sim 10.4.0 via `ros-lyrical-gz-sim-vendor` 0.4.5; `ros_gz` 3.0.9 |
| TurtleBot3 | `ros-lyrical-turtlebot3-gazebo` 2.3.7 (model `burger_cam`, unmodified) |
| Python | 3.14.4 (system and venv) |
| Compute | CPU-only inference (no NVIDIA driver; `torch.cuda.is_available() == False`), Intel i7-1355U laptop |
| Network | Needed on the **first** launch: the world includes Fuel *Ground Plane* and *Sun* by URL (then cached in `~/.gz/fuel`) |

Older ROS distributions, other Gazebo versions and GPU inference are **untested**.

## 2. ROS packages [untested on a clean machine]
`package.xml` declares the ROS packages the stack uses (dependency declarations added 2026-10-14; the package was rebuilt with them the same day). `rosdep install --from-paths src --ignore-src` from a clean machine has **not** been tried, and the YOLO venv and weights are outside rosdep in any case.
You also need at least the following (apt names inferred from the ROS package names):
```bash
sudo apt install ros-lyrical-desktop ros-lyrical-turtlebot3-gazebo ros-lyrical-ros-gz \
                 ros-lyrical-cv-bridge ros-lyrical-robot-state-publisher python3-pytest
```

## 3. Workspace layout and YOLO environment
The workspace root must hold these, side by side:
```
<ws>/src/my_first_robot   <ws>/disaster_site   <ws>/yolo_env   <ws>/yolo11n.pt
```
The launch file finds the workspace as four levels above `install/my_first_robot/share/my_first_robot`. All validated runs used `<ws> = ~/ros2_ws`.
The harness scripts under `disaster_site/tools/mission/` hard-code `~/ros2_ws`.

**YOLO venv** [inspected; the validated venv was created before these files were written]:
```bash
cd <ws>
python3 -m venv yolo_env                    # validated venv: include-system-site-packages = false
yolo_env/bin/pip install -r requirements-yolo.txt          # 7 direct pins
#   exact 65-package set recorded from the validated venv:  requirements-yolo.lock.txt
touch yolo_env/COLCON_IGNORE                # present in the validated workspace, so colcon skips the venv
```
- `rclpy`, `sensor_msgs`, `std_msgs` and `cv_bridge` are **not** pip packages. They come from ROS through `PYTHONPATH` after sourcing ROS.
- The default Linux `torch` wheel pulls in NVIDIA CUDA wheels (proprietary licence) even though inference runs on the CPU. A CPU-only torch build is **untested**.
- Importing `ultralytics` creates `~/.config/Ultralytics/settings.json` unless `YOLO_CONFIG_DIR` points elsewhere.

**Weights:** `yolo11n.pt` is not in the repository. Obtain it and check its SHA-256 as described in [YOLO_MODEL.md](YOLO_MODEL.md):
```bash
cd <ws> && echo "0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1  yolo11n.pt" | sha256sum -c -
```
The launch file does **not** download the weights. If they are missing, it stops with `RuntimeError: ... required file not found: <ws>/yolo11n.pt`.

## 4. Build [validated]
The last builds on record used this exact command, with no symlink install: `log/build_2026-10-08_14-48-08` (validated missions) and
`log/build_2026-10-14_12-41-57` (rebuild after the metadata edits; exit 0, only the existing `tests_require` setuptools warning):
```bash
source /opt/ros/lyrical/setup.bash
cd ~/ros2_ws
colcon build --packages-select my_first_robot
```
After any source edit you must rebuild, because without `--symlink-install` the nodes run from `install/`.

**What the checksum file covers.** `sha256sum -c evidence/SHA256SUMS.frozen`, run from the workspace root, checks only these seven **source** files
against their recorded hashes:
- `src/my_first_robot/my_first_robot/` `autonomous_search.py`, `safety_controller.py`, `victim_distance_estimator.py`, `victim_prioritizer.py`,
  `exploration_grid.py` and `disaster_bot_monitor.py`;
- `src/my_first_robot/launch/disaster_bot_full.launch.py`.

It does **not** check the build, the `install/` tree, `yolo_detector.py`, the world/models, the weights or the venv. A passing check means "these
source files are unchanged". It does not mean "the installed package is current". To compare one installed module with its source by hand [inspected]:
```bash
cmp src/my_first_robot/my_first_robot/autonomous_search.py \
    install/my_first_robot/lib/python3.14/site-packages/my_first_robot/autonomous_search.py && echo same
```
**Rebuild of 2026-10-14.** The source `package.xml` (dependency declarations, description, licence, maintainer) and the `setup.py` metadata
(licence, maintainer email) were edited on 2026-10-14, and the package was then rebuilt with the command above. Checks after the rebuild:
- the installed `share/my_first_robot/package.xml` is identical to the source `package.xml`;
- the installed package metadata (`PKG-INFO`) shows the maintainer `Manasvi`, the GitHub noreply address and `All rights reserved`;
- all 17 installed Python modules and 3 launch files are identical to their source;
- only three installed files changed (`package.xml`, `PKG-INFO`, colcon's package index);
- the seven frozen source files and all 95 `src/` + `disaster_site/` files are byte-identical to before the rebuild.

The rebuild did not change any code or launch behaviour. It has **not** been followed by a new Gazebo mission run.

## 5. Run the mission
Open a terminal with a graphical session (the detector opens an OpenCV window):
```bash
source /opt/ros/lyrical/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_first_robot disaster_bot_full.launch.py gui:=false
```
- **[validated]** `gui:=false` with all other defaults: unified-launch mission and monitored mission, 2026-10-08 (`evidence/master_launch_20261008`,
  `evidence/monitor_20261008`). The validation harness also exported `GZ_IP=127.0.0.1` and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`,
  and ran inside a desktop session that had a `DISPLAY`.
- **[inspected]** The default `gui:=true`, i.e. plain `ros2 launch my_first_robot disaster_bot_full.launch.py`, additionally starts the Gazebo GUI client.
  That variant does **not** appear in any recorded mission log and has **not been validated**. The GUI also uses extra CPU, which may lower the real-time factor.
- Optional arguments: `explore_max_steps:=30` (default; 15 was too few on the west route), `record_dir:=/abs/path` (writes `autonomous_search`'s
  JSONL decision records), `x_pose`, `y_pose`, `yaw`. Only the default spawn pose has been validated.
- What to expect:
  - the entrance is found and the robot is `INSIDE` after about 60–70 s of sim time;
  - `VICTIM_HOLD` after about 875–920 s of sim time, which is **about 25 min wall-clock** (sim/wall ≈ 0.6);
  - the log line `[VICTIM] REACHED ... /victim_found {...}`.

Watch the result [inspected; similar `ros2 topic echo` command used by the harness]:
```bash
ros2 topic echo /victim_found std_msgs/msg/String --qos-durability transient_local
```

## 6. Mission Monitor (optional, read-only)
In a second terminal, after the same two `source` lines:
```bash
ros2 run my_first_robot disaster_bot_monitor                                   # [validated: standalone start]
ros2 run my_first_robot disaster_bot_monitor --ros-args -p record_dir:=<same path as the launch record_dir>   # [validated: monitored mission]
```
Without `record_dir`, the accessibility and risk terms of the ranking are not shown. The monitor never publishes commands. Stop it with Ctrl+C.

## 7. Tests [validated 2026-10-14; full suite run after the rebuild]
```bash
source /opt/ros/lyrical/setup.bash && source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/my_first_robot
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest test/ -p no:cacheprovider
```
The 2026-10-14 run additionally isolated ROS and temporary files:
```bash
ROS_DOMAIN_ID=97 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR=<temp dir outside the project> PYTHONDONTWRITEBYTECODE=1 \
  python3 -m pytest test/ -p no:cacheprovider -rfEs --basetemp=<temp dir outside the project>
```
- Latest full run (2026-10-14, after the rebuild): **286 passed, 3 failed, 0 errors, 1 skipped**. This is the same result as the 2026-10-12 baseline.
  The suite is **not** clean. The 3 failures are known and unfixed:
  - `test_crossing_align.py::test_alignment_not_improving_early_abandons`: **functional**. The node stays in `CROSSING_OPENING` instead of falling back to
    `FOLLOWING_BOUNDARY`. The identical assertion is recorded in logs from 2026-10-03 (pre-existing since 2026-09-24);
  - `test_flake8`: 74 existing style issues;
  - `test_pep257`: 300 existing docstring issues.
- The run changed no project files and left no processes running.
- `test_copyright` is skipped because no file has a copyright header.
- Several tests call `rclpy.init()` and create in-process nodes. Running them while a mission is live on the same ROS domain is not recommended
  (precaution; not observed to cause problems).
- `colcon test` has **not** been used for the recorded results [untested].

## 8. Shut down safely [validated]
- Press **Ctrl+C once** in the launch terminal and wait a few seconds. The recorded runs sent SIGINT to the launch process group.
  The launch exited in 3–4 s and the leftover check reported "NO ORPHAN PROCESSES".
- Expected shutdown noise (harmless in the recorded runs):
  - `rcl_trigger_guard_condition() failed` from rclpy nodes;
  - an `autonomous_search` take_message traceback;
  - `image_bridge`/`parameter_bridge` exiting with codes −6/−11.
- Check for leftovers before the next launch:
  ```bash
  pgrep -af "gz-sim-main|gz sim|yolo_detector.py|lib/my_first_robot/|parameter_bridge|image_bridge|robot_state_publisher"
  ```
  A surviving `gz-sim-main` keeps the old world alive, and the next "fresh" launch then attaches to it. Stop it explicitly (`kill <pid>`)
  if it is left over. Killing only the `gz sim` Ruby wrapper does not stop it.

## 9. Troubleshooting
| Symptom | Likely cause / action |
|---|---|
| `RuntimeError: disaster_bot_full.launch.py: required file not found: ...` | `yolo11n.pt`, `yolo_env/bin/python`, `disaster_site/` or the detector script is not at the workspace root. Check the layout in §3 |
| `Package 'my_first_robot' not found` | The workspace isn't built or `install/setup.bash` isn't sourced |
| `ModuleNotFoundError: ultralytics` | The detector was started with system `python3`, e.g. `ros2 run my_first_robot yolo_detector`. That entry point has a `/usr/bin/python3` shebang and **cannot** work. Use the launch file, which runs the venv's python |
| `ModuleNotFoundError: rclpy` / `em` / `lark` inside the venv | ROS not sourced before launch, or `empy`/`lark` missing from the venv (both are pinned in `requirements-yolo.txt`) |
| Detector crashes at start with a Qt/xcb error | No display. The venv's OpenCV has only the xcb backend, and `QT_QPA_PLATFORM=offscreen` crashed it in testing. Run from a desktop session (`DISPLAY`, `XAUTHORITY`) |
| Gazebo hangs at start / missing ground or sun | First launch needs internet for the Fuel Ground Plane/Sun; afterwards they come from `~/.gz/fuel` |
| Old world or robot positions reappear | An orphan `gz-sim-main` from a previous run (§8) |
| `ros2 topic list` lacks `/scan`, `/camera/image_raw` | The bridges haven't started yet or have died. Check the launch output for `parameter_bridge`/`image_bridge` |
| No `/victim_detections` | No person in view, or confidence < 0.50 (detection depends on posture and range; see VALIDATION_AND_LIMITATIONS.md §6). YOLO warm-up takes several seconds |
| `/victim_detections` but no `/victim_distance` | Estimator warnings: box touching the image edge, no fresh scan, or no LiDAR cluster beyond the size gate |
| Readings but robot does not approach | Readings are used only once `INSIDE`, while not rotating, with confidence ≥ 0.60 and range ≤ 3.5 m; 3 agreeing readings are needed. Check `[VICTIM]` log lines |
| Mission ends in `INSIDE_HOLD` | Exploration budget used up (15 steps failed on the west route). Use the default 30 |
| Robot stopped in `FAULT_HOLD` | Stale scan/odom (recovers when fresh), recovery exhausted or tilt ≥ 10° (terminal, restart) |
| Monitor shows `WAITING` | `autonomous_search` hasn't started yet (it starts 10 s after launch), or the monitor is on a different ROS domain |
| Two publishers on `/cmd_vel` | A second stack, or the legacy `obstacle_detector`, is running. `obstacle_detector` publishes a zero-velocity `geometry_msgs/Twist` on `/cmd_vel` whenever its closest LiDAR reading is < 0.8 m; it is an unvalidated extra publisher whose type conflicts with the bridge's `TwistStamped`. Stop everything and relaunch without it |
