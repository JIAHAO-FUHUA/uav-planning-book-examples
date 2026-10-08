# Reproducibility

The first code snapshot accompanies the revised combined manuscripts through
Section 4.3.2 dated 8 October 2026. Algorithm sources and existing numerical tests
are carried over unchanged; the repository adds documentation, build automation,
and an optional-media check in the figure driver.

## Environment and checks

- Recorded Python environment: Python 3.12.14, NumPy 2.5.3, Matplotlib 3.11.2.
- Root `requirements.txt` pins the numerical and plotting packages; use Python 3.12+.
- C++ programs require C++17. CMake builds all four and CTest runs their self-tests.
- `python tools/run_checks.py` runs four existing Python check programs. Assertions
  must remain enabled; do not use `python -O`.
- Figure generation requires an installed Times New Roman font. Fonts are not
  bundled; PNG and SVG assets are provided.

## Fixed examples

- Grid: eight-neighbor movement with no corner cutting; Dijkstra and A* both obtain
  length 50.284 in the static example. Processing counts include valid queue removals,
  including the goal; they are not wall-clock timing measurements.
- 2D sampling: seed 17, fixed 14×10 m scene, 250 PRM samples, a first-route RRT case,
  and 600 RRT* draws. Stopping rules differ, so this example is not a runtime ranking.
- Dynamics: an exact endpoint lattice retains both position and velocity. The
  documented optimal objective is 10.4 s, with duration 8 s and effort 12 m²/s³.
- Moving replanning: D* Lite's remaining graph cost is checked against fresh A*.
  A disconnected map causes holding; an old route is not executed after invalidation.
- 3D: paired RRT/RRT* accepted coordinates, 2000 draws and seeds 3, 17, 41, 73, 101.
  Collision checks cover the complete segment against closed boxes. Finite-budget
  failure in general does not establish that a route is impossible.
- Trajectories: three normalized septic segments with durations 2, 2, 2 s;
  shared velocity, acceleration and jerk; zero endpoint derivatives. The fixed-time
  joint snap objective is approximately 1227.477435 m²/s⁷. Dilation can satisfy
  derivative bounds while preserving a collision, as the printed example demonstrates.

JSON, CSV and C++ output records remain with each chapter. Figure sources and the
`SOURCES_*.md` notes give exact scene definitions and bibliographic provenance.
Results are for the modeled geometry and translational dynamics; full vehicle
attitude, sensors, actuators and feedback control are outside these teaching cases.

## Updating the book and repository

Keep complete Python drivers synchronized with the book's appendices. When a case
changes, rerun its checks, record its scene and seed, then regenerate its figures.
Preserve English figure text, legends and editable SVG files. New chapters can be
added under `examples/chapterXX/` without changing the current examples' paths.
