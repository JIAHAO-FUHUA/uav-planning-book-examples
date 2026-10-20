# ROS system cases: Sections 4.3.7–4.3.9

These wrappers run the **official compiled planners and trajectory servers** in
one static box scene. They relay position commands ideally to odometry and
publish unoccluded spherical-range point clouds. They do not model quadrotor
dynamics, tracking error, ray occlusion, noise, or verified free-space carving.
The saved Fast-Planner run is a **rejected execution example**, not a successful
physical flight: it has a position-command jump and insufficient altitude.
The EGO-Planner-v2 replay fails a subsequently raised 0.40 m clearance rule.
Arrival must not be interpreted as acceptance.

## Environment actually used

WSL2 Ubuntu 20.04.6, ROS Noetic (rosversion 1.17.4), native `/usr/bin/python3`
3.8, GCC 9.4.0, CMake, Eigen 3.3.7, Armadillo 9.8 and local NLopt 2.7.1.
ROS Python modules must come from this environment, not the numerical Python
3.12 environment used by the book's offline examples. ROS Noetic reached end
of life on 31 May 2025. ROS 2 is not a drop-in replacement.

Prerequisites: an existing Noetic installation with catkin/roscpp/rospy,
roslaunch, message generation, standard geometry/nav/sensor/visualization
messages, message_filters, tf, pcl_ros/pcl_conversions, cv_bridge and
image_transport; system build-essential, cmake, python3-numpy, Eigen3,
Armadillo, PCL, OpenCV and Boost development libraries. The setup script does
not install system packages or use sudo. Use a Linux filesystem for build
workspaces (the default is under your Linux home), even when these wrappers
are in a Windows-mounted directory. The headless common case needs no GPU,
joystick or RViz window.

## Prepare and build

From this `ros_cases` directory in Ubuntu:

```bash
source /opt/ros/noetic/setup.bash
bash setup_ros_cases.sh
```

The script verifies 1,909 pinned source-file hashes, downloads missing files,
builds NLopt under `~/uav_book_cases_20261020/deps` and builds two **separate**
catkin workspaces. Existing changed source is rejected instead of overwritten.
Only the hardcoded Fast-Planner NLopt include/library paths are adjusted.
Planning algorithms are unchanged. The NLopt archive is SHA-256 verified.
Build logs and environment helpers are saved in that Linux root. Source files
are downloaded from upstream under their own licenses; they are not bundled
with these examples. `pinned_sources.json` records each URL and digest.

An optional `UAV_BOOK_CASE_ROOT` changes the build root. Pass the same root to
`make_launch.py --root /absolute/path` if generating launch files separately.
Both complete workspaces compiled locally; the distributable setup script was
also rerun successfully against the verified cached sources. A clean-machine
system-package installation is not claimed.

## Run one planner per fresh terminal

Remain in this `ros_cases` directory. Start with Fast-Planner:

```bash
source ~/uav_book_cases_20261020/case_fast_env.sh
export ROS_MASTER_URI=http://127.0.0.1:11321
export ROS_IP=127.0.0.1
python3 run_case.py --planner fast --output ../results/ros_fast_new
```

After it exits, use another fresh terminal and the same working directory:

```bash
source ~/uav_book_cases_20261020/case_ego_env.sh
export ROS_MASTER_URI=http://127.0.0.1:11321
export ROS_IP=127.0.0.1
python3 run_case.py --planner ego --output ../results/ros_ego_new
```

The helpers also set the local NLopt library path. Do not source both catkin
workspaces in one shell: package names overlap. Choose an unused master port
if 11321 is occupied. The runner starts its own roslaunch process group and
stops that group when done. New output directories above preserve the frozen
book records. No timeout or failed run is silently turned into arrival.

Edit `scene.json`, then run `python3 make_launch.py` to regenerate parameters.
The common scene uses three surface-sampled boxes, 0.1 m points/voxels, 5 m
spherical sensing at 10 Hz, 100 Hz odometry, start (−7,0,1) and goal (7,0,1).
The goal is submitted after at least 3 s. Native interfaces are different:
Fast `plan_manage/Bspline` and `nav_msgs/Path`; v2 `traj_utils/PolyTraj` and
`quadrotor_msgs/GoalSet`, drone 0. v2 requires the polynomial-server heartbeat.
See `make_launch.py` for all parameter overrides; copied upstream map bounds
are not a physical floor guarantee.

## Offline checks and frozen evidence

From `Chapter_4.3_Code` in the numerical environment with NumPy/Matplotlib:

```bash
python ros_cases/assess_cases.py
python system_case_figures.py
python test_system_cases.py
```

These commands read the frozen `results/ros_fast` and `results/ros_ego` JSONs,
not `_new` runs. To assess your own record, call `assess(json.load(...))` from
`assess_cases.py`, or deliberately replace a copy of the result directory.

The validator computes exact segment-to-closed-box distances for the recorded
**piecewise-linear replay**, not the original continuous upstream curves.
It separately checks a 0.30 m altitude floor and recorded position-step
consistency (`step <= nominal_speed * elapsed_gap + 0.05 m`). This is an
external diagnostic rule; passing it is not a continuous tracking guarantee.
Recorded command velocity/acceleration fields are not derivatives reconstructed
from the replay positions. The Fast record shows why that distinction matters.
The test is successful when known failure conditions are correctly detected.

## Official default simulations

Launch graphs were inspected; local execution in this task used the common
bridge above. In separate terminals sourced for the appropriate workspace:

```bash
# Fast-Planner: RViz in one terminal, demo in another
roslaunch plan_manage rviz.launch
roslaunch plan_manage kino_replan.launch
# Optional distinct topological pipeline: topo_replan.launch

# EGO-Planner-v2 (package remains named ego_planner)
roslaunch ego_planner rviz.launch
roslaunch ego_planner single_drone_interactive.launch
```

Fast's default simulation includes a quadrotor/controller loop. The selected
v2 simulator XML enables an ideal `poscmd_2_odom` adapter; its physical-controller
blocks are commented out. v2's interactive goal converter is `manual_take_over`.
Their default simulations therefore do not support a direct tracking-error
ranking. Use the pinned official README/tutorial for UI interaction.

Versions, links, scope and observed metrics: `../SOURCES_SYSTEM_CASES.md` and
`../results/system_case_verification.json`.
