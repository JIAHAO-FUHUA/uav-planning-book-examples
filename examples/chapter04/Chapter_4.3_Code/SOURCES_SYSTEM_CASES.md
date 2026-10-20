# Sections 4.3.7–4.3.9: source and execution provenance

Checked 20 October 2026. Official source files and papers are reference
material. Only the book's wrappers, editable diagrams and recorded traces are
distributed here; no upstream planning source, paper PDF, video or proprietary
library is bundled.

## Primary sources

- [Fast-Planner repository](https://github.com/HKUST-Aerial-Robotics/Fast-Planner),
  pinned [41be219fe4ecc43bf0e0c2b42a523f8755ccc0bd](https://github.com/HKUST-Aerial-Robotics/Fast-Planner/tree/41be219fe4ecc43bf0e0c2b42a523f8755ccc0bd),
  latest default-branch commit observed at the check date, dated 24 October 2024.
  Inspected `plan_manage/src/kino_replan_fsm.cpp`, `planner_manager.cpp`,
  `traj_server.cpp`, `path_searching/src/kinodynamic_astar.cpp`,
  `bspline_opt/src/bspline_optimizer.cpp`, `bspline/src/non_uniform_bspline.cpp`,
  `plan_env/src/sdf_map.cpp`, launch parameter files and simulator XML.
- [Zhou et al., Robust and Efficient Quadrotor Trajectory Generation for Fast
  Autonomous Flight](https://arxiv.org/abs/1907.01531), IEEE RA-L 4(4),
  3529–3536, 2019. Book reference [11]. The source's componentwise feasibility
  tests and time reallocation were checked directly, not inferred from a name.
- [EGO-Planner-v2 repository](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2),
  pinned [9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2/tree/9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010),
  latest default-branch commit observed at the check date, dated 17 March 2023.
  Inspected `swarm-playground/main_ws/src/planner` trajectory optimizer,
  `MinJerkOpt`, manager, FSM, grid map, server, native messages and launch XML.
  This **main_ws is quintic minimum-jerk MINCO**, not the earlier EGO B-spline
  system. Constraints stored in `cps_` refer to sampled curve points.
- [Zhou et al., Swarm of micro flying robots in the wild](https://zhepeiwang.github.io/pubs/sr_2022_swarm.pdf),
  Science Robotics 7, eabm5954, 2022; official Swarm Playground tutorial PDF in
  the pinned repository. Book reference [13]. This chapter uses one UAV;
  multi-UAV terms belong in the later multi-UAV section.
- [Open Robotics: ROS Noetic End-of-Life, May 31, 2025](https://discourse.ros.org/t/ros-noetic-end-of-life-may-31-2025/43160),
  book reference [31]. The legacy environment was checked, with no assertion
  of current maintenance or automatic ROS 2 compatibility.
- Local reference: Peter Corke, *Robotics, Vision and Control*, Chapter 5,
  supplied as `参考书电子版/Robotics, Vision.pdf`; used for planning context.

## Build and run scope

Both pinned projects' complete required catkin workspaces compiled with
Noetic on WSL2 Ubuntu 20.04.6, GCC 9.4.0, Eigen 3.3.7, Armadillo 9.8 and
NLopt 2.7.1. NLopt was built under the task's Linux home; Fast-Planner's two
hardcoded dependency paths were changed to that prefix. No planning algorithm
code was changed. NLopt archive SHA-256:
`db88232fa5cef0ff6e39943fc63ab6074208831dc0031cf1545f6ecd31ae2a1a`.

Each official planner and trajectory server was actually launched and reached
the goal in the common headless case. The adapter follows commands ideally;
it publishes static range-limited point clouds without ray occlusion or noise.
There is no actual quadrotor/controller integration or physical flight test.
The official default demo launch graphs were inspected, not executed here.
The portable setup script passed an idempotent rerun against the compiled,
hash-verified source trees; no clean operating-system dependency installation
is asserted. `pinned_sources.json` lists all 1,909 retrieved source-file hashes.

## Evidence and limitations

Figures 4.24 and 4.26 are source-based mechanism diagrams with editable SVG
text. Figures 4.25, 4.27 and 4.28 are reconstructed from the supplied actual
ROS JSON traces. All figures use English Times New Roman, grayscale and
legends. Python source regenerates them; no trajectory was drawn by hand to
pretend that an unexecuted simulation succeeded.

Fast replay: 15.624 m length including a 2.104 m command position step,
0.501 m box clearance, minimum altitude −0.00887 m. It fails a 0.30 m altitude
rule and the external position-step consistency rule. Its reported command
speed peak 1.525 m/s does **not** describe the jump's position difference.
The trace alone does not identify the jump's unique root cause.

EGO-v2 replay: 15.150 m, 0.344 m box clearance, 1.507 m/s command speed peak,
minimum altitude about 1 m, consistent recorded position steps. It passes the
0.30 m box-distance rule but fails a revised 0.40 m requirement without
replanning. These different rejection examples are teaching cases, not a
benchmark ranking. Parameter equality does not equalize margin/limit semantics.

Distances apply to the recorded polyline, not continuously to the original
B-spline/MINCO curves or an actual vehicle envelope. Raw online summaries use
100 Hz samples; offline figures use every second retained sample. Numerical
results can vary with scheduling and optimizer restarts. Tests verify evidence
handling and known rejection conditions; they do not certify the records safe.
