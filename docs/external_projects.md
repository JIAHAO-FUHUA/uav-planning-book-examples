# External system and learning projects

The currently implemented cases are listed in `book_sections.md`. The pinned Fast-Planner and EGO-Planner-v2 workspaces have now been compiled and run using the documented common bridge. This verifies planner operation, with recorded rejection conditions; it does not establish physical flight safety. EGO-Swarm and learning integrations remain pending.

- [EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2): pinned `9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010`; main_ws uses quintic minimum-jerk MINCO. One-UAV common bridge executed.
- [EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-swarm): multi-UAV planning.
- [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner): pinned `41be219fe4ecc43bf0e0c2b42a523f8755ccc0bd`; kinodynamic search, ESDF and cubic B-splines. Common bridge executed; frozen record rejected for altitude/position-step consistency.
- [Agile Autonomy](https://github.com/uzh-rpg/agile_autonomy): a learning-based flight
  reference used by the book's introductory illustration.

Figure 4.3 in the manuscript uses an EGO-Planner-v2 video frame from
`swarm-playground/main_ws/WatchMe_main.mp4` at 160 s, and an Agile Autonomy animation
frame from `planner_learning/img/animation_medium.gif` at zero-based index 37.
The manuscript arranges these full frames side by side, with EGO-Planner-v2 on
the left and Agile Autonomy on the right. Their English headings and legends
remain editable in the locally generated SVG.
The code repository references those original project media rather than packaging
the frames or composite image. The figure driver skips Figure 4.3 if the two
locally supplied source frames are absent.

Each future integration should record its upstream commit, dependencies, simulator,
launch commands and reproduced result. Deep-learning, reinforcement-learning and
VLA examples will require separate environment checks. Language/VLA task proposals
remain inputs to constrained motion planning and validation.


See `examples/chapter04/Chapter_4.3_Code/ros_cases/README.md` and
`SOURCES_SYSTEM_CASES.md` for environment, source-file hashes and execution scope.
Native default simulators differ: Fast includes quadrotor/controller dynamics;
the selected v2 XML enables ideal command-to-odometry. ROS Noetic is end of life;
the observed source dates do not establish active maintenance or ROS2 support.
