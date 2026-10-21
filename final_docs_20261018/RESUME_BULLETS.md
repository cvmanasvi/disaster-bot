# Disaster Bot — Resume Bullets

These bullets describe only what is verified by simulation evidence.
Do not expand them to claim real-world performance, hardware testing, or safe autonomous navigation.

---

## Option A — Emphasising the full system

**Disaster Bot** (ROS 2 Lyrical, Gazebo Sim, YOLO11n, Python)
Built a simulated autonomous robot that enters a damaged building, explores the floor plan with LiDAR-based frontier search, and locates a standing person using camera–LiDAR fusion; 6/6 Gazebo missions reached the person (25 exploration steps, 0.939 m stand-off, 0 safety interventions).

---

## Option B — Emphasising the perception guard

**Navigation-facing perception guard** (ROS 2, Python, LiDAR)
Designed and validated a real-time detection filter that intercepts YOLO person detections before the distance estimator; in stationary simulation (8 scenes), wrong-surface readings dropped from 592 to 0 (offline) and debris/occlusion scenes produced 0 wrong distances live, without modifying the frozen navigation stack.

---

## Option C — Brief, versatile

**Autonomous Disaster-Search Robot** — ROS 2 · Gazebo · YOLO11n · LiDAR
Simulation prototype: autonomous building entry, frontier exploration, and camera–LiDAR person localisation; 6/6 unguarded Gazebo missions reached VICTIM_HOLD; navigation-facing perception guard validated separately in stationary simulation (0 wrong-surface distances live); 497 automated tests passing.

---

## Usage notes

- Specify *simulation* in every version; do not imply hardware deployment.
- The guard bullet is appropriate if applying for perception/sensor-fusion roles.
- The full-system bullet suits robotics or autonomy roles.
- If a recruiter asks about posture coverage: only standing people are validated live; lying uses rigid proxies; sitting/kneeling is not evaluated.
- If asked about navigation safety: autonomous navigation has not been validated with the guard active; the stationary demo confirms perception behaviour only.
