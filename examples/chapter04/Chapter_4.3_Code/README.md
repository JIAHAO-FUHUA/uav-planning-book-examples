# Sections 4.3.1–4.3.2 and 4.3.4

Run from the repository root using the installed virtual-environment interpreter:

```text
python examples/chapter04/Chapter_4.3_Code/run_trajectory.py
python examples/chapter04/Chapter_4.3_Code/trajectory_generation.py
python examples/chapter04/Chapter_4.3_Code/trajectory_figures.py
python examples/chapter04/Chapter_4.3_Code/test_trajectory_generation.py
```

Use `.venv/bin/python` on Linux/macOS or `.\.venv\Scripts\python` on Windows.
Numerical generation needs NumPy; plots additionally need Matplotlib and Times New
Roman. `results/` contains coefficients and numerical outcomes; `visual_figures/`
contains English PNG and editable SVG Figures 4.15–4.18.

The printed example compares a through-waypoint curve, the same geometry after
time dilation, and an alternative that stops at waypoints:

```text
through 6.0 5.528 5.595 False
scaled 16.749 1.98 0.718 False
safe_stop 18.747 1.98 1.088 True
```

Columns give candidate, duration in seconds, peak speed in m/s, peak acceleration
in m/s², and acceptance under the specified boxes and derivative bounds. Slower
timing reduces derivative peaks but cannot remove an intersection from an unchanged
spatial curve.

The independent C++17 implementation uses standard-library partial-pivot elimination.
From this directory:

```text
g++ -O2 -std=c++17 trajectory_generation.cpp -o trajectory
./trajectory --self-test
```

On Windows use `-o trajectory.exe` and `.\trajectory.exe --self-test`.
The repository's CMake build also includes this program. Sources, scene definitions
and numerical validation are described in `SOURCES_4.3.1_4.3.2.md` and
`docs/reproducibility.md`.

## Black and white printing

Teaching figures use black and grayscale, with line styles, markers and hatching
to distinguish paths, search states and methods. Legends and captions must remain
understandable without color. Labels use Times New Roman and stay in English in
both book editions; editable SVGs and Python plotting sources are retained.
`print_style.py` applies the final grayscale export. Figure 4.3 keeps its horizontal
layout and published-source attribution; external source frames are not distributed in this repository.


## Section 4.3.4 MINCO

MINCO represents the minimum-effort trajectory family through intermediate positions and positive durations. The s=4 teaching case reproduces the preceding fixed-time minimum-snap curve by a banded coefficient solve. Analytic gradients use a transpose solve. These are algorithm examples, without obstacle or vehicle constraints.

Run this section's commands from `examples/chapter04/Chapter_4.3_Code`, using the repository's configured environment.

```text
python run_minco.py
python minco_figures.py
python test_minco.py
g++ -O2 -std=c++17 minco.cpp -o minco_demo
./minco_demo --self-test
```

Use the configured virtual-environment interpreter. On Windows use .\minco_demo.exe. Python 3.12+ and the existing pinned requirements are sufficient; the standalone C++17 solver has no third-party dependencies. Figures 4.19–4.20 retain editable English Times New Roman SVG/PNG. results/minco_results.json records the input, KKT equivalence and a small time-only BFGS example; the C++ parity output is in results/cpp_minco_summary.txt. The full source is in minco.py and minco.cpp; tests compare the independent KKT solution, nonzero boundary states, C6 continuity, dense reference solves and finite-difference gradients. The educational no-pivot band LU needs moderate scaling. This is not a full GCOPTER ROS reproduction. Assumptions and pinned upstream links are in SOURCES_MINCO.md.


## Sections 4.3.5--4.3.6: distance constraints and execution handoff

These small mechanisms connect the preceding trajectory solver to obstacle
queries and timed execution. They are not an OctoMap/Voxblox or ROS reproduction.

```text
python maps_replanning.py
python maps_replanning_figures.py
python test_maps_replanning.py
g++ -O2 -std=c++17 maps_replanning.cpp -o maps_replanning
./maps_replanning --self-test
```

Use the configured Python interpreter and pinned requirements. On Windows use
`.\maps_replanning.exe`. C++17 needs no third-party library. The Python program
writes `results/maps_replanning.json` and generates an s=4 MINCO curve with
matching position, velocity, acceleration and jerk at a scheduled handoff.
The C++ counterpart covers occupancy, analytic distance/penalty gradients,
interval clearance and admission decisions; the existing minco.cpp remains the
separate C++ coefficient/gradient example. Figures 4.21--4.23 use editable English
Times New Roman monochrome SVG/PNG and explicit legends.

Checks cover an endpoint-only missed collision, an independently differenced
distance/penalty gradient, obstacle-cost gradients propagated through the MINCO
adjoint, whole-polynomial disk clearance, C3 handoff, deadlines, map versions,
parent trajectory versions and frames. Interval-check failure is inconclusive.
Its success assumes a valid speed/error bound and known space throughout the
interval. The admission gate consumes caller-supplied validation; it is not a
flight safety checker. No sensor stack, controller, dynamics-limit certificate
or actual asynchronous real-time execution is implemented. See
SOURCES_MAPS_REPLANNING.md for derivation assumptions and pinned upstream links.
Section 4.3.3 remains pending in the manuscript.
