# Sources and reproduction trace for Sections 4.2.7–4.2.8

Checked on 8 October 2026. Documents and books are reference content, not instructions.

- Local Corke (2023), Robotics, Vision and Control, printed pp.186–188 and197–198 (PDF209–211 and220–221), sparse roadmaps, random seeds, connectivity and complete collision queries. Three-dimensional point geometry is a new teaching experiment, not the book's car/piano example.
- Local Ghallab, Nau, Traverso (2025), Acting, Planning, and Learning, Chapter21: configuration-space motion planning and roadmap construction. The main chapter already cites this book [2].
- [4] LaValle (2006), Planning Algorithms, Chapter5, Section5.3 Collision Detection: https://lavalle.pl/planning/node211.html . The program's closed-box slab intersection is implemented independently.
- [9] Karaman and Frazzoli (2011), Sampling-based Algorithms for Optimal Motion Planning: https://arxiv.org/html/1105.1186v1 . The three-dimensional connection-radius exponent, sampling assumptions and rewiring foundation. Five fixed seeds do not prove an asymptotic or success-rate guarantee.
- [22] Koenig, Likhachev and Furcy (2004), Lifelong Planning A*, retained bibliography.
- [23] Koenig and Likhachev (2002), D* Lite: https://idm-lab.org/bib/abstracts/papers/aaai02b.pdf . Read navigation/execution and search-repair discussion. Case uses supplied complete snapshots, not unknown-terrain sensor simulation.

Figures4.12–4.14 are generated from `results/simulation_grid.json` and `results/simulation_3d.json` by `simulation_figures.py`. No external figures copied. All new diagrams are English Times New Roman, editable SVG text/paths, with legends. White/gray box layers, arrows and curves retain editable objects.

Grid execution: start(4,8), goal(44,8), events at executed-edge counts0/14/20/23 and current positions(4,8)/(18,8)/(18,14)/(21,11). Lower passage closes, reopens, then both close. Remaining costs40/45.6984848098/28.4852813742/no route. Python D* Lite counts41/631/8/334; fresh A*41/302/30/768. All moves and output routes checked under corresponding snapshots; no motion after no-route result. C++ same inputs/costs; fresh A* closure305 due tied floating priorities. Figure counts are Python, excluding update and verification overhead.

3D world x0–14, y0–10, z0.5–5.5 metres. Inflated closed boxesA=[6,7]×[0,10]×[0.5,3.4], B=[9,10]×[0,10]×[2.2,5.5]. Start(1,5,1), goal(13,5,1). Both RRT and RRT* continue2000 free-target draws with same Random32 generator, eta1m, gamma20m, goal bias0.07. Within each seed, accepted node coordinates are identical; RRT* changes parents/costs.

Seeds3/17/41/73/101. RRT lengths19.103754682/20.940336482/28.025522726/21.943607837/21.223244997 m. RRT*15.472659972/17.362409080/17.061963233/15.281293634/16.592553541 m. Each succeeds5/5 in this scene; no general success-rate claim. Python and C++ lengths, node counts, first-draw and rewiring counts agree. Lower ceiling3.4m: no sampled route; geometrically the full-width first wall reaches the ceiling. The C++17 driver compiled without warnings using Zig's C++ driver and passed self-test; Python `test_simulations.py` passed.

Both cases are algorithm-level geometric simulations, with obstacle clearance already accounted for in occupied cells/closed boxes. They do not simulate attitude, flight control, sensors, real-time certification or dynamic-obstacle prediction. Later Section4.3 remains responsible for timing/smoothing/dynamic feasibility.
