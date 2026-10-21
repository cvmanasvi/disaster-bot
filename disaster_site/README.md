# Earthquake disaster site (Gazebo) for Disaster Bot

An independent Gazebo world that tests the frozen Disaster Bot stack in an environment that is substantially different from `turtlebot3_house`. Nothing in the house world or in `src/my_first_robot` is used or modified, apart from launching the unchanged nodes.

## Launch

```bash
source /opt/ros/lyrical/setup.bash && source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger_cam
ros2 launch ~/ros2_ws/disaster_site/launch/disaster_site.launch.py            # with GUI
ros2 launch ~/ros2_ws/disaster_site/launch/disaster_site.launch.py gui:=false # headless
#   optional: x_pose:=0.0 y_pose:=-1.2 yaw:=0.0   (default spawn: outside, facing +x)
```

The launch file starts the same nodes as `turtlebot3_house.launch.py`:

- `gz sim` server and an optional GUI client
- `robot_state_publisher`
- `ros_gz_sim create` (burger_cam)
- the `turtlebot3_burger_cam_bridge.yaml` parameter bridge
- the `/camera/image_raw` image bridge

Only the world and the spawn pose differ. The world name is `default`, so the existing tooling keeps working: `/world/default/create`, `/world/default/set_pose` and `/world/default/dynamic_pose/info`.

## Files

```
disaster_site/
├── world/earthquake_disaster.sdf      generated world (physics/plugins identical to the house world)
├── models/
│   ├── damaged_room/     building shell: outer walls, partitions, jagged/broken openings (25 boxes)
│   ├── collapsed_wall/   rubble choking the corridor + blocked facade doorway + fallen wall chunk (10)
│   ├── fallen_beam/      2.67 m x 0.30 m x 0.40 m beam across the east hall (1)
│   ├── debris/           lobby slab island, 1.8/1.4 m slab pile hiding the victim, room slabs (7)
│   ├── rubble/           small blocks inside + in the yard (16)
│   └── victim/           static standing person (cached Fuel mesh + textures; same geometry as the house tests)
├── launch/disaster_site.launch.py
├── tools/layout.py           single source of truth for every box (world frame)
├── tools/generate_world.py   writes models/* and world/*.sdf from layout.py
├── tools/check_geometry.py   reachability / bottlenecks / dead ends / victim visibility / clearances
└── evidence/                 renders, robot camera frames, layout map, geometry report, validation overlay
```

To change the layout, edit `tools/layout.py`, then run `python3 tools/generate_world.py && python3 tools/check_geometry.py`.

## Layout (world frame, metres)

The building is 8.0 × 7.2 m (x 0..8, y 0..7.2). Walls are 0.15 m thick and 2.0 m tall; broken sections are 0.5–1.4 m tall. The robot spawns **outside** at (0.0, −1.2), yaw 0, in the south yard, just as in the house tests.

| Area | Extent | Notes |
|---|---|---|
| Yard | y < 0 | 4 rubble blocks; the facade doorway (x 2.0–2.9, 0.90 m) is the only way in |
| A lobby | y 0–2.6 | Slab island (1.0 × 0.55 m) plus rubble; blocked facade doorway (x 5.6–6.5) filled flush, with piles inside and outside |
| B west room | x 0–3.3, y 2.6–4.8 | Entered through a 1.0 m broken opening; slab debris |
| C collapsed room | x 0–3.3, y 4.8–7.2 | **Dead end**: only exit is the B→C doorway (0.90 m); collapsed 0.5–0.9 m wall section toward B |
| Corridor | x 3.375–4.125, y 2.6–4.8 | **Blocked passage**: four touching rubble pieces wall-to-wall at y≈3.0–4.1; dead-end stubs from both ends |
| E east hall | x 4.2–8, y 2.6–4.8 | Entered through the **narrow passage** (x 4.6–5.3, 0.70 m) or the wide opening (x 6.6–7.6, 1.0 m). The **fallen beam** runs from the corridor wall (no gap) to (6.85, 3.80), so the only route north is **around its east end** (1.06 m) |
| D victim room | x 3.3–8, y 4.8–7.2 | Entered through the D doorway (x 5.8–6.7, 0.90 m); slab pile at (6.5–7.4, 5.3–6.1) |
| Victim | (7.35, 6.65, yaw 0) | Static, in the NE corner behind the pile |

All obstacles are static boxes extruded from the floor, rotated about z only. Every non-wall piece is at least 0.25 m wide and 0.35 m tall, so the LiDAR plane (z 0.171 m) cuts each obstacle at its full footprint. Nothing hides below the scan, and nothing overhangs the robot. The dust floor is a visual-only plate 2 mm above the ground plane and has no collision.

## Validation (2026-10-07)

Logs are in `~/disaster_bot_test_logs/disaster_site_validation_20261007/`.
