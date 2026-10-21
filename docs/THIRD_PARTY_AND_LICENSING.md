# Third-party components and licensing status

This file is not legal advice. Third-party facts were checked on 2026-10-14 against the sources named below.

## 1. The project's own code: no licence granted
- The original project code is published for **viewing only**. **No licence is granted**: there is no permission to reuse, modify or redistribute it,
  and the project is **not** released under an open-source licence. There is deliberately no `LICENSE` file.
- The owner's licensing decision is **still open**. It will be revisited once the third-party review (in particular Ultralytics, §3) is complete.
- Package metadata says `All rights reserved` (`package.xml` `<license>`, `setup.py` `license=`). This is a plain statement, not an SPDX licence identifier.
- Public visibility does not by itself prevent copying. GitHub's Terms of Service allow other users to view and fork public repositories.
- Original project code: `src/my_first_robot` (nodes, launch files, tests, tools) and `disaster_site/`. The latter covers the world generator, generated world,
  the box-built models `damaged_room`, `collapsed_wall`, `fallen_beam`, `debris` and `rubble`, the victim wrapper `model.sdf`/`model.config`,
  the launch file and the harness scripts.

## 2. Third-party material inside this repository
| Path | Origin | Licence | Notes |
|---|---|---|---|
| `src/my_first_robot/test/test_copyright.py`, `test_flake8.py`, `test_mypy.py`, `test_pep257.py`, `test_xmllint.py` | ROS 2 `ament` package templates, "Copyright … Open Source Robotics Foundation, Inc." | Apache License 2.0 (stated in each file header) | Third-party template code under its own licence; keep the headers. This does not license the rest of the project |
| `disaster_site/models/victim/meshes/standing.dae` and `materials/textures/*.png` (11 files) | Byte-identical copy of Gazebo Fuel **OpenRobotics / "Standing person", version 3**. Mesh author Marina Kollmitz; "Model created with Makehuman" | **Listed as CC0 1.0 Universal on the official Gazebo Fuel page** (<https://app.gazebosim.org/OpenRobotics/fuel/models/Standing%20person>, checked by the project owner on 2026-10-14; the Fuel API record <https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing%20person>, read the same day, agrees) | The Fuel listing is the licence evidence. The upstream provenance of the individual mesh and textures (e.g. the original author's terms, the texture sources) has **not** been independently verified, and no upstream source was found. 9 of the 11 textures are referenced by the mesh |

**Attribution:** "Standing person" by Marina Kollmitz, published by OpenRobotics on Gazebo Fuel
(<https://app.gazebosim.org/OpenRobotics/fuel/models/Standing%20person>, version 3). Created with MakeHuman (<http://www.makehuman.org/>).

## 3. Third-party material NOT in this repository (installed or fetched separately)
| Component | Used by | Licence (source) | Notes |
|---|---|---|---|
| `yolo11n.pt` weights | `yolo_detector.py` | AGPL-3.0 (embedded metadata; Ultralytics docs: "AGPL-3.0 and Enterprise licenses") | **Excluded** from the repository. The local file matches the official `ultralytics/assets` v8.3.0 asset (same size and MD5). See `docs/YOLO_MODEL.md` |
| `ultralytics` 8.4.137 (+ `ultralytics-thop`, `ultralytics-platform`) | `yolo_detector.py` (`from ultralytics import YOLO`) | AGPL-3.0 (package metadata, repository LICENSE) | Installed in the separate YOLO venv |
| `torch` 2.13.0, `torchvision` 0.28.0 | ultralytics | BSD-style / Apache-2.0 family | venv |
| NVIDIA CUDA runtime wheels (`nvidia-*`, `cuda-*`) | pulled in by the default Linux `torch` wheel | NVIDIA proprietary | Not used for inference (CPU). Never redistribute |
| `opencv-python` 5.0.0.93, `numpy` 2.5.2 | yolo_detector | Apache-2.0; BSD-3-Clause and others | venv |
| ROS 2 Lyrical packages (rclpy, launch, launch_ros, messages, cv_bridge, …) | all nodes | Apache-2.0 (cv_bridge: Apache-2.0 + BSD) | System install |
| `turtlebot3_gazebo` 2.3.7 (burger_cam model, bridge config), `ros_gz_*` 3.0.9 | `disaster_site.launch.py` | Apache-2.0 | System install, unmodified |
| Gazebo Fuel `OpenRobotics/Ground Plane` (v5), `OpenRobotics/Sun` (v3) | world SDF, `<include>` by URL | Listed as CC0 1.0 on Gazebo Fuel | Downloaded by Gazebo at first launch, not redistributed |

## 4. Ultralytics / AGPL-3.0: unresolved
- The detector imports Ultralytics and loads YOLO11n weights. Both are licensed AGPL-3.0, and Ultralytics also offers Enterprise licences.
- Ultralytics states that software using its code or models should be released under AGPL-3.0 unless covered by an Enterprise licence (<https://www.ultralytics.com/license>).
- **Whether the view-only, no-licence approach for this repository is compatible with Ultralytics' terms is unresolved and requires further review.**
  Excluding the weights from the repository does not settle the question. Anyone running, modifying or redistributing the detector must comply with Ultralytics' terms.
