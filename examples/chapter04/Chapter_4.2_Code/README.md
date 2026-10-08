# Sections 4.2.1–4.2.8

Use the repository-root Python environment and dependencies. Commands below are
run from the **repository root**.

```text
python examples/chapter04/Chapter_4.2_Code/chapter_examples.py
python examples/chapter04/Chapter_4.2_Code/run_sampling.py
python examples/chapter04/Chapter_4.2_Code/run_simulations.py
```

Replace `python` with `.venv/bin/python` on Linux/macOS or
`.\.venv\Scripts\python` on Windows. The programs print lengths, queue processing
counts and modeled trajectory properties. Complete scenes and outputs are in `results/`.

## Generate the English figures

```text
python examples/chapter04/Chapter_4.2_Code/visual_chapter_figures.py
python examples/chapter04/Chapter_4.2_Code/sampling_dynamics.py
python examples/chapter04/Chapter_4.2_Code/sampling_dynamics_figures.py
python examples/chapter04/Chapter_4.2_Code/simulations.py --case all
python examples/chapter04/Chapter_4.2_Code/simulation_figures.py
```

Times New Roman must be installed. Output PNG/SVG files retain the book's figure
numbers: 4.1–4.2 and 4.4–4.14. Figure 4.3 is a composite of external project
demonstrations; its media is referenced in `docs/external_projects.md`.
The first command skips that figure when the external frames are absent.
`algorithm_exposition_figures.py` can regenerate Figures 4.6–4.7 alone.

## Inspect algorithms

- `planning.py` / `planning.cpp`: grid search, shortcutting and D* Lite.
- `sampling_dynamics.py` / `.cpp`: PRM, RRT, RRT* and double-integrator state search.
- `simulations.py` / `.cpp`: moving-agent grid replanning and 3D RRT/RRT*.
- `grid_planning_core.hpp`: shared C++ grid implementation used by the simulations.
- `test_planning.py`, `test_sampling_dynamics.py`, `test_simulations.py`: numerical checks.

From this directory with a C++17 compiler:

```text
g++ -O2 -std=c++17 planning.cpp -o planning
g++ -O2 -std=c++17 sampling_dynamics.cpp -o sampling
g++ -O2 -std=c++17 simulations.cpp -o simulations
```

Append `.exe` to output names on Windows and run each executable with `--self-test`.
Alternatively use the repository's CMake build.

## Change a case

```text
python examples/chapter04/Chapter_4.2_Code/simulations.py --case 3d --seed 17 --budget 1000
```

This replaces the recorded 3D JSON. Restore the default five-seed case using
`simulations.py --case all` before generating the comparison figures.
See the `SOURCES_*.md` files and `docs/reproducibility.md` for constraints,
references and interpretation of the toy cases.
