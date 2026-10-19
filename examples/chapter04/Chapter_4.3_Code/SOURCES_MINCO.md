# MINCO sources and reproduction scope

Section 4.3.4 checked on 19 October 2026. The manuscript follows the existing outline: 4.3.3 B-spline/Bézier remains pending; 4.3.4 MINCO is now added.

- [12] Z. Wang, X. Zhou, C. Xu, F. Gao. Geometrically Constrained Trajectory Optimization for Multicopters, IEEE Transactions on Robotics 38(5), 3259–3278, 2022. DOI https://doi.org/10.1109/TRO.2022.3160022 . Author manuscript https://zhepeiwang.github.io/pubs/tro_2022_gcopter.pdf ; accessible HTML https://arxiv.org/html/2103.00190 . Sections IV-B–IV-D, Theorem 2 and equations (45)–(68) support the optimality conditions, banded reconstruction and adjoint. PDF pages 7–8 were visually inspected. The scalar integration-by-parts explanation and pedagogical notation connect these results to the preceding chapter model.
- [28] Official GCOPTER repository https://github.com/ZJU-FAST-Lab/GCOPTER . Pinned commit e0444f6d47b84f972ced91746b05feb36ce1fd4f, dated 6 June 2023. Source https://github.com/ZJU-FAST-Lab/GCOPTER/blob/e0444f6d47b84f972ced91746b05feb36ce1fd4f/gcopter/include/gcopter/minco.hpp . MINCO_S4NU, BandedSystem and propogateGrad were inspected. The repository was accessible and this was the main-branch head observed on the check date. Accessibility does not establish active maintenance or a successful full ROS reproduction; the ROS system was not run here.
- [1] Local reference: Peter Corke, Robotics, Vision and Control, 3rd edition, Chapter 3.3, printed pp.98–102 (PDF pp.122–126). Retained interpolation/time-scaling context; it is not a source for the later MINCO method. [27] Richter et al. supplies the preceding independent KKT/endpoint-derivative context.

## Teaching implementation

minco.py and minco.cpp implement the stated s=4, position-only internal-knot equations in physical local time. Both use fixed overall position, velocity, acceleration and jerk, reconstruct coefficients through order seven, and match internal derivatives through order six. They implement fixed-width no-pivot band LU and a transpose solve. Moderate scaling is required. A positive duration guarantees mathematical nonsingularity, not numerical conditioning; use robust pivoted band solvers or rescaling for difficult inputs. No upstream header is redistributed.

Python also demonstrates a small BFGS optimization of log durations with internal points fixed. It optimizes snap plus a time cost and has no collision, speed, attitude or actuator constraints. The example bounds log durations to avoid extreme scales. Its optimizer is not the full GCOPTER optimizer. Dense KKT is used only as an independent reference in tests, not by the MINCO reconstruction path.

Tests compare coefficients after conversion to the normalized basis of 4.3.2, check nonzero endpoint derivatives and C6 continuity, compare band/transpose solves with independent dense solves, and verify point/time gradients by centered finite differences. Python/C++ numerical comparisons were run locally. No runtime ranking, ROS flight reproduction or continuous-time safety claim is made.

## Figure provenance

- Figure 4.19: two 3D segments through (0,0,1), (1,0.8,1), (2,0,1) m; times 1.2, 1.8 s; zero velocity/acceleration/jerk at overall endpoints. KKT and MINCO curves coincide. The other curves move internal y to 1.2 m or swap durations. These are input deformations, not optimized obstacle-avoidance trajectories. The plots show x–y projections at z=1 m. Filled squares are specified points; the open square is the moved point; open circles are KKT samples. For a free 3D internal point there are five outer parameters; mandatory coordinates remain fixed.
- Figure 4.20: nonzero pattern of the assembled 24×24 system for three segments. Four initial rows, two eight-row internal blocks and four final rows. The flow shows coefficient reconstruction, objective partials, adjoint and reduced-gradient gathering. Fixed polynomial order and dimension give linear solve/propagation complexity, excluding objective/constraint evaluation and the number of outer iterations.

Both figures use English Times New Roman, black/grayscale, line styles and explicit legends. Editable SVG and minco_figures.py are supplied. No paper images were copied.
