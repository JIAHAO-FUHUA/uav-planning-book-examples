# Book sections and executable examples

Paths are relative to `examples/chapter04/`.

| Section | Implemented material | Source or driver |
| --- | --- | --- |
| 4.2.1 | Grid construction, obstacle inflation and collision rules | `Chapter_4.2_Code/planning.py`, `visual_chapter_figures.py` |
| 4.2.2 | Dijkstra and A* | `Chapter_4.2_Code/planning.py`, `chapter_examples.py` |
| 4.2.3 | Weighted A*, Theta* and shortcutting | `Chapter_4.2_Code/planning.py`, `algorithm_exposition_figures.py` |
| 4.2.4 | D* Lite and its map-update examples | `Chapter_4.2_Code/planning.py`, `chapter_examples.py` |
| 4.2.5 | PRM, RRT and RRT* | `Chapter_4.2_Code/sampling_dynamics.py`, `run_sampling.py` |
| 4.2.6 | Double-integrator search and motion primitives | `Chapter_4.2_Code/sampling_dynamics.py`, `run_sampling.py` |
| 4.2.7 | Execution, map updates and incremental repair | `Chapter_4.2_Code/simulations.py`, `run_simulations.py` |
| 4.2.8 | 3D RRT/RRT* with whole-segment box checks | `Chapter_4.2_Code/simulations.py`, `run_simulations.py` |
| 4.3.1 | Timed paths, derivative limits and time dilation | `Chapter_4.3_Code/trajectory_generation.py`, `algorithm_figures.py` |
| 4.3.2 | Polynomial interpolation and fixed-time minimum snap | `Chapter_4.3_Code/trajectory_generation.py`, `run_trajectory.py` |

The three core Section 4.2 modules and the Section 4.3 solver have corresponding
C++17 programs. JPS has a conceptual description in the book but no implementation
in this repository. System cases and learning methods belong to later additions.
