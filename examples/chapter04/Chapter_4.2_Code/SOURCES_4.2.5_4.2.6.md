# Source and experimental trace

Read-only reference: local books, latest Formatted EN/ZH combined manuscript. Source chapter outline retained, including separate 4.2.7/4.2.8 simulation sections. Current additions are compact planar explanatory examples; these later simulation sections are not marked complete.

- [1] Corke 2023 local PDF `参考书电子版/Robotics, Vision.pdf`, PDF pp.208–211 and 219–222. Build/query PRM structure and nonholonomic RRT motivation. Unlike a simplified query example in the book, every start/goal connection is collision checked here.
- [2] Ghallab et al. 2025, local extracted Chapter 21, configuration motion and timed/differential planning distinction.
- [7] https://www.kavrakilab.org/publications/kavraki-svestka1996probabilistic-roadmaps-for.html
- [8] https://lavalle.pl/rrtpubs.html (existing manuscript bibliography source)
- [9] https://arxiv.org/abs/1105.1186 ; https://arxiv.org/html/1105.1186v1. Geometric RRT* theorem requires sampling/connectivity assumptions; finite-budget toy code is not a proof of optimality. Fixed-k toy PRM has no asserted asymptotic guarantee. The geometrical theorem does not transfer automatically to differential constraints.
- [25] https://lavalle.pl/planning/ ; https://lavalle.pl/planning/node738.html ; https://lavalle.pl/planning/node739.html. Chapters 13–14, differential models and motion primitives.
- [26] https://arxiv.org/abs/1709.05401 ; https://ieeexplore.ieee.org/document/8206119/ ; https://github.com/sikang/motion_primitive_library. Liu, Atanasov, Mohta, Kumar, Search-based Motion Planning for Quadrotors using Linear Quadratic Minimum Time Control, IROS 2017. Control-space discretization/search foundation; the chapter's code is an independently written double-integrator example, not a reproduction of the complete paper's LQMT method or library.
- [11] https://github.com/HKUST-Aerial-Robotics/Fast-Planner . Official README identifies kinodynamic search followed by trajectory optimization. No claim that the full ROS system was installed or executed for these new sections.

New figures are generated from `results/sampling_dynamics.json` except Figure 4.9, which is an explicitly constructed local geometric calculation, recorded in `results/new_figure_metrics.json`. English labels in both editions. No new copied external pictures; conceptual source citations are in captions/text.

Python tests passed. Compiled C++17 with Zig C++ driver; `--self-test` passed. Both languages: PRM14.057586852976678 m; RRT16.916919617607501 m; RRT*14.867121084540237 m; kinodynamic cost10.4 s, duration8 s, effort12 m²/s³,789 expansions; same controls/end state. Independent zero-heuristic search:1988 expansions, cost10.4 s. Whole quadratic motion collision checks include boundary roots and interval midpoints; plotting samples are not used to certify safety.
