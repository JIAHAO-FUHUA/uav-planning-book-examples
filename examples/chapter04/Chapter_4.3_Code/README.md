# Sections 4.3.1–4.3.2

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
