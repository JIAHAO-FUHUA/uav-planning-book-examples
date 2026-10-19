# Teaching revision sources and scope

Revision checked on 19 October 2026.

- JPS original paper: Daniel Harabor and Alban Grastien, Online Graph Pruning for Pathfinding on Grid Maps, AAAI 2011. https://ojs.aaai.org/index.php/AAAI/article/view/7994 ; author-hosted paper https://pathfinding.ai/pdf/harabor-grastien-aaai11.pdf . Figure 4.6 is a separately constructed local illustration. Movement and pruning rules must use the same corner policy; this illustration forbids corner cutting. It does not implement or benchmark full JPS.
- D* Lite original paper: Sven Koenig and Maxim Likhachev, D* Lite, AAAI 2002. https://idm-lab.org/bib/abstracts/papers/aaai02b.pdf . Figure 4.7 is a separately constructed directed graph with zero heuristic, showing predecessor propagation after a closure. Local costs, corner legality and the numerical repair states are checked in algorithm_exposition_figures.py; results/algorithm_illustrations.json records those values.
- Polynomial/minimum-snap exposition retains the manuscript's references [1,10,27] and native equations (4.12)–(4.20). Cubic interpolation is introduced before the general septic endpoint system. Free internal derivatives are jointly optimized. Fixed-time minimum snap alone does not enforce collision or full quadrotor feasibility.
- Weighted A* cites manuscript reference [24]; JPS cites [20]. Prior numerical Python/C++ implementations, benchmark JSON and source frames are unchanged.
