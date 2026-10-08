# External system and learning projects

The currently implemented cases are listed in `book_sections.md`. These links name
projects intended for later system and learning sections; they do not claim that a
ROS, flight or learning integration has already been reproduced in this repository.

- [EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2): single-UAV online
  trajectory planning and its published obstacle-scene demonstrations.
- [EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-swarm): multi-UAV planning.
- [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner): mapping-based
  kinodynamic search and trajectory planning.
- [Agile Autonomy](https://github.com/uzh-rpg/agile_autonomy): a learning-based flight
  reference used by the book's introductory illustration.

Figure 4.3 in the manuscript uses an EGO-Planner-v2 video frame from
`swarm-playground/main_ws/WatchMe_main.mp4` at 160 s, and an Agile Autonomy animation
frame from `planner_learning/img/animation_medium.gif` at zero-based index 37.
The code repository references those original project media rather than packaging
the frames or composite image. The figure driver skips Figure 4.3 if the two
locally supplied source frames are absent.

Each future integration should record its upstream commit, dependencies, simulator,
launch commands and reproduced result. Deep-learning, reinforcement-learning and
VLA examples will require separate environment checks. Language/VLA task proposals
remain inputs to constrained motion planning and validation.
