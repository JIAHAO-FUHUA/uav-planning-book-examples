# Chapter 4.3 polynomial trajectory examples

This folder accompanies Sections4.3.1–4.3.2. Unzip beside Chapter_4.2_Code.
Use the existing Python environment described in AppendixA1; the algorithms require
NumPy and plots require Matplotlib. No ROS or flight simulator is needed.

From this folder in Windows PowerShell:

```powershell
..\Chapter_4.2_Code\.venv\Scripts\python trajectory_generation.py
..\Chapter_4.2_Code\.venv\Scripts\python trajectory_figures.py
..\Chapter_4.2_Code\.venv\Scripts\python test_trajectory_generation.py
..\Chapter_4.2_Code\.venv\Scripts\python run_trajectory.py
```

On Linux/macOS replace the interpreter by `../Chapter_4.2_Code/.venv/bin/python`.
For a standalone copy, create a Python environment and install `requirements.txt`.
The recorded environment is Python3.12.14, NumPy2.5.3 and Matplotlib3.11.2.
The pinned requirements need Python3.12 or later. The NumPy/Matplotlib versions match the existing Chapter4.2 plotting environment;
core numerical and C++ results are also supplied if only the figures are needed.

```powershell
g++ -O2 -std=c++17 trajectory_generation.cpp -o trajectory.exe
.\trajectory.exe --self-test
```

C++17 uses only its standard library. The verified compilation used Zig's C++17
driver on Windows. `results/cpp_trajectory_summary.txt` records the actual run,
and `results/cpp_coefficients.csv` contains all24×3 joint-QP coefficients.
Figure text uses installed Times New Roman; `visual_figures/*.svg` retains text,
curves, axes, legends and obstacle shapes as editable objects.

The complete copied Word example is `run_trajectory.py`. Its columns are candidate,
seconds, peak speed m/s, peak acceleration m/s², and acceptance for the specified
closed boxes and derivative limits:

```text
through 6.0 5.528 5.595 False
scaled 16.749 1.98 0.718 False
safe_stop 18.747 1.98 1.088 True
```

The first candidate exceeds bounds and intersects the upper box. Uniform dilation
fixes the derivative peaks but keeps the intersection. The stopped alternative
passes the boxes and bounds after its own dilation; it is not minimum-time.

`trajectory_generation.py` uses ascending coefficients in normalized local s∈[0,1].
The fixed-time septic QP has cost1/2 cᵀQc with Q twice the integrated-squared-snap
matrix. Each segment interpolates two endpoint positions; velocity, acceleration
and jerk are shared across knots, with all three zero at the overall start/end.
The C++ system is assembled independently and uses partial-pivot elimination.
Both programs scale the objective and constraint rows, without changing the optimum.
Coefficients and displayed quantities agree within1e-7 in the supplied case.

For the joint case, waypoints are(1,3,1),(4,7,1),(9,7,1),(13,3,1), durations2,2,2s.
Snap costs at the same6s:1227.477434997... joint vs64575 stopped, in m²/s⁷.
Boxes already include clearance: A=[6,7]×[0,6]×[0.5,1.5]m;
B=[5.8,6.6]×[9.1,9.7]×[0.5,1.5]m. Speed bound2m/s, acceleration bound2.5m/s².
Choose dilation max(1,peak_speed/2,sqrt(peak_acceleration/2.5))×1.01.
Zero endpoint derivatives are preserved. Nonzero required endpoint derivatives
would need to be rescaled or re-solved; do not silently change a measured initial state.

For derivative extrema, roots of the derivative of the squared norm are evaluated
along with endpoints. NumPy's polynomial-root routine is used in Python; C++
recursively isolates real roots by derivative-root partitions and bisection.
Polynomial-box checks partition [0,1] at every real coordinate-boundary root,
test each interval and boundary, and treat contact as collision (tolerance1e-8).
This is whole-segment numerical checking with floating arithmetic, not formal
exact-arithmetic certification. Drawing samples are not safety evidence.

Tests include known quintic/septic coefficients and single snap integral100800,
nonzero derivative interpolation, independent nullspace perturbations of the QP,
C3 continuity, derivative extrema, uniform time scaling, thin boxes and tangency.
Change one waypoint or positive duration, rerun generation and tests, then plot.
Severely uneven times can cause poor conditioning; long problems require a more
appropriate sparse/endpoint-derivative formulation.

All examples fix yaw and model only translational polynomial motion. They contain
no controller, attitude/actuator model, sensors or real-flight validation. Acceptance
means passing exactly the modeled static boxes and translational bounds. Later
trajectory and system sections remain responsible for full vehicle feasibility.
Sources, figure provenance and exact scene details are in SOURCES_4.3.1_4.3.2.md.


Algorithm exposition revision: Figures 4.15–4.17 explain corner handling, endpoint equation assembly and shared junction derivatives; Figure 4.18 is the single main-text numerical comparison. trajectory_figures.py calls algorithm_figures.py. The numerical solver and complete results are unchanged. The main text now provides separate single-segment interpolation and multi-segment minimum-snap algorithms.

## Black and white printing

Teaching figures use black and grayscale, with line styles, markers and hatching
to distinguish paths, search states and methods. Legends and captions must remain
understandable without color. Labels use Times New Roman and stay in English in
both book editions; editable SVGs and Python plotting sources are retained.
`print_style.py` applies the final grayscale export. Figure 4.3 keeps its horizontal
layout and published-source attribution; the archive retains untouched source
frames, while the book shows them in grayscale.


## Section 4.3.4 MINCO

MINCO represents the minimum-effort trajectory family through intermediate positions and positive durations. The s=4 teaching case reproduces the preceding fixed-time minimum-snap curve by a banded coefficient solve. Analytic gradients use a transpose solve. These are algorithm examples, without obstacle or vehicle constraints.

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


## Sections 4.3.7–4.3.9: ROS system cases

See [ros_cases/README.md](ros_cases/README.md) for the separate Ubuntu20.04 /
ROS Noetic environment, pinned source retrieval, build and launch instructions.
Both official planner/server systems compiled and ran locally with an ideal
command-to-odometry bridge. Saved results include rejection examples: Fast's
position jump/altitude violation and EGO-v2's failure of a raised clearance rule.
They are not accepted physical flights or a statistical ranking.

Offline assessment and figures need only the numerical Python environment:
`python ros_cases/assess_cases.py`, `python system_case_figures.py`, and
`python test_system_cases.py`. Figures 4.24–4.28 have editable English monochrome
SVGs. Version and evidence details are in `SOURCES_SYSTEM_CASES.md` and
`results/system_case_verification.json`. Section 4.3.3 remains pending.
