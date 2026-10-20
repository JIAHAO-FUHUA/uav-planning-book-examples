# 无人机规划书籍样例代码

[English](README.md)

总仓库维护者请看：[第 4 章接入与更新命令](docs/integration.md)。

本仓库按书籍章节组织 Python 与 C++ 样例。目前包含 **4.2.1–4.2.8** 和
**4.3.1–4.3.2、4.3.4–4.3.9**：栅格搜索、增量重规划、采样规划、动力学搜索及多项式轨迹生成。

## 先运行一个例子

固定依赖需要 **Python 3.12 或以上版本**。在仓库根目录打开 PowerShell：

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python -m pip install -r requirements.txt
.\.venv\Scripts\python examples/chapter04/Chapter_4.2_Code/chapter_examples.py
.\.venv\Scripts\python examples/chapter04/Chapter_4.3_Code/run_trajectory.py
.\.venv\Scripts\python tools/run_checks.py
```

Linux/macOS 使用 `python3 -m venv .venv` 创建环境，后续解释器改为
`.venv/bin/python`。栅格、采样和重规划算法只依赖标准库；轨迹计算使用 NumPy，
绘图使用 Matplotlib。数值例子与离线回放检查无需 ROS；新增系统案例单独使用 Ubuntu 20.04 / Noetic / Python 3.8，见[运行说明](examples/chapter04/Chapter_4.3_Code/ros_cases/README.md)。

## 按章节查找

| 章节 | 内容 | 样例入口，相对于 `examples/chapter04/` |
| --- | --- | --- |
| 4.2.1–4.2.4 | Dijkstra、A*、加权 A*、Theta*、捷径化与 D* Lite | `Chapter_4.2_Code/chapter_examples.py` |
| 4.2.5–4.2.6 | PRM、RRT、RRT*、动力学 A* | `Chapter_4.2_Code/run_sampling.py` |
| 4.2.7–4.2.8 | 行进中的地图更新、三维采样 | `Chapter_4.2_Code/run_simulations.py` |
| 4.3.1–4.3.2 | 多项式插值与最小 snap | `Chapter_4.3_Code/run_trajectory.py` |
| 4.3.4 | MINCO带状系数求解与伴随梯度 | `Chapter_4.3_Code/run_minco.py` |
| 4.3.5–4.3.6 | 距离约束与预约接管 | `Chapter_4.3_Code/maps_replanning.py` |
| 4.3.7–4.3.9 | Fast-Planner / EGO-Planner-v2 ROS案例与失败诊断 | `Chapter_4.3_Code/ros_cases/README.md` |

每个目录均提供对应的 C++17 实现、英文实验图、可编辑 SVG、数值结果及检查程序。
目前 JPS 只有书中原理介绍，尚无实现。详见[章节与文件对应表](docs/book_sections.md)。

## C++ 编译

安装 CMake 3.16 或以上版本和 C++17 编译器，在仓库根目录执行：

```text
cmake -S . -B build
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
```

生成 `grid_planning`、`sampling_dynamics`、`simulations` 和
`trajectory_generation`、`minco_demo`、`maps_replanning` 六个程序。Visual Studio 的可执行文件在
`build/Release/`，单配置生成器的文件在 `build/`。GitHub Actions
会在 Windows 和 Linux 上运行已有的 Python 检查与 C++ 自检。

## 查看结果与复现

各章节的 `results/` 保存数值记录，`visual_figures/` 保存 PNG 和可编辑 SVG。
绘图命令见章节目录说明。重新生成出版用图需要安装 Times New Roman 字体；
直接查看已生成 PNG 不需要安装字体。

场景、随机种子及结果含义见[复现说明](docs/reproducibility.md)。Fast-Planner 与 EGO-Planner-v2 已编译并运行共同桥接案例。Fast记录到达目标但违反高度/指令连续性，EGO-v2回放不满足提高后的间距准则，均不代表实飞安全或算法排名。EGO-Swarm与学习案例仍待核实，见[外部项目说明](docs/external_projects.md)。

图片按**黑白印刷**设计，使用线型、标记和斜线表达区别，不依赖颜色。
SVG 保留可编辑的英文 Times New Roman 文字与图形。
