# 第4.3节 多项式轨迹示例

本目录配套4.3.1—4.3.2节，解压后与Chapter_4.2_Code并列。
复用附录A1的Python环境，算法需要NumPy，绘图需要Matplotlib。
在本目录内执行：

```powershell
..\Chapter_4.2_Code\.venv\Scripts\python trajectory_generation.py
..\Chapter_4.2_Code\.venv\Scripts\python trajectory_figures.py
..\Chapter_4.2_Code\.venv\Scripts\python test_trajectory_generation.py
..\Chapter_4.2_Code\.venv\Scripts\python run_trajectory.py
g++ -O2 -std=c++17 trajectory_generation.cpp -o trajectory.exe
.\trajectory.exe --self-test
```

Linux/macOS解释器路径为`../Chapter_4.2_Code/.venv/bin/python`。
实际验证环境为Python3.12.14、NumPy2.5.3、Matplotlib3.11.2。
固定版本的requirements.txt需要Python3.12及以上；单独使用本目录时，可创建该环境并安装依赖。
四张图均为英文Times New Roman，SVG保留可编辑文字与几何元素。
C++仅用标准库；已用Windows上的Zig C++17驱动编译并通过自检。

run_trajectory.py是Word中的完整代码，输出候选名称、时长、最大速度、最大加速度及是否接受：

```text
through 6.0 5.528 5.595 False
scaled 16.749 1.98 0.718 False
safe_stop 18.747 1.98 1.088 True
```

联合曲线超出导数限制并撞入上方盒体。拉长时间后导数达标，但碰撞仍存在；
停点方案保留折线几何，重新定时后通过本例盒体与导数检查。它不是最短时间解。
固定时间下，联合与停点方案snap代价分别为1227.477434997和64575 m²/s⁷。

系数采用归一化局部坐标s∈[0,1]的升幂排列。QP的目标为1/2 cᵀQc，Q是积分矩阵的两倍。
两端位置、总体起终点速度/加速度/jerk以及内部C3连续构成等式；中间导数自由。
全局四个路点为(1,3,1)、(4,7,1)、(9,7,1)、(13,3,1)，三段各2 s。
盒体A=[6,7]×[0,6]×[0.5,1.5] m，B=[5.8,6.6]×[9.1,9.7]×[0.5,1.5] m，已包含间隙。
上限为2 m/s与2.5 m/s²；放缩因子为max(1,最大速度/2,sqrt(最大加速度/2.5))×1.01。
时间放缩会改变非零端点导数，不能默默改变测量初始状态，应按要求重新求解。

JSON保存系数、时长、代价、残差及检查结果。C++独立组装系统并用主元消元；
results/cpp_coefficients.csv与cpp_trajectory_summary.txt保存实际系数和运行输出。
两种语言系数与展示结果在1e-7内一致。

导数峰值检查端点与平方范数导数的实根；盒体检查按坐标边界根划分区间，并检查区间与接触。
Python使用NumPy多项式求根，C++采用导数根划分单调区间及二分隔离。
默认接触容差1e-8；这是覆盖完整模型线段的浮点数值检查，并非形式化精确证明。
测试还包含解析系数、已知snap积分100800、非零端点导数、QP零空间扰动、连续性、放缩、薄障碍与切触。

这些例子固定偏航，仅模拟平移多项式，不包含ROS、控制器、姿态/执行器、感知或真机验证。
接受仅指通过这里的静态盒体与导数约束。完整飞行可行性属于后续轨迹与系统章节。
详细技术说明见README_EN.md，参考书页码与来源见SOURCES_4.3.1_4.3.2.md。


算法讲解修订：图4.15—4.17解释拐角处理、端点方程构造和共享衔接导数；图4.18保留一个正文数值对比。trajectory_figures.py调用algorithm_figures.py。数值求解器与完整结果不变，正文分别给出单段插值和多段最小snap算法。

## 黑白印刷

教学图优先使用黑色与灰度，以线型、标记、填充和斜线区分路径、搜索状态与方法。
图例、图注不依赖颜色。两版书稿共用英文 Times New Roman 标注，保留可编辑 SVG
和 Python 绘图源文件。print_style.py 负责最终灰度导出。图4.3保留左右布局和来源，
书稿使用灰度画面，源文件包仍保留未经修改的引用原图。


## 4.3.4 MINCO

MINCO用中间位置与正时长表示最小控制努力轨迹族。s=4教学程序通过带状系数求解重现上一节的固定时间最小snap曲线，并用转置求解传播解析梯度。运行run_minco.py生成例子，minco_figures.py生成图4.19—4.20，test_minco.py执行独立KKT、非零边界、六阶连续性和有限差分检查。C++17对应程序为minco.cpp，编译及运行命令见README_EN.md。沿用Python 3.12+与固定依赖；C++不需要第三方库。

run_minco.py还演示固定路点下的小型BFGS时间优化，结果位于results/minco_results.json。图稿保持英文黑白Times New Roman，SVG文字可编辑。例子未加入障碍或机体限制，小型无主元带状LU要求适度尺度，也不代表完整GCOPTER ROS系统已经复现。假设、官方固定版本与来源见SOURCES_MINCO.md。4.3.3的B样条与Bézier部分仍待撰写。


## 4.3.5--4.3.6 距离约束与执行衔接

运行maps_replanning.py生成距离惩罚、区间检查与MINCO接管例子；
maps_replanning_figures.py生成图4.21--4.23；test_maps_replanning.py检查
距离与系数梯度、连续曲线漏检、接管导数及过期结果拒绝。C++17对应程序
maps_replanning.cpp实现占据、解析距离、梯度与准入逻辑，编译命令见英文说明。
沿用前述Python环境与固定依赖。图中文字为英文Times New Roman，黑白SVG可编辑。

例子使用二维解析圆盘，不实现栅格ESDF或完整在线规划器。区间检查需真实成立的
速度/距离误差界及整段已知空间，未通过仅表示不能确认安全。准入函数使用外部
验证结果；没有实现感知、控制器或实际异步实时执行，也不宣称整机可飞。
来源、假设与固定版本见SOURCES_MAPS_REPLANNING.md。4.3.3仍待撰写。


## 4.3.7—4.3.9：ROS系统案例

固定源码版本、独立 Ubuntu20.04 / ROS Noetic 环境、构建及运行命令见
[ros_cases/README.md](ros_cases/README.md)。两套官方规划器/服务器已实际编译运行，
采用理想指令—里程计桥接。保存记录保留拒绝案例：Fast的指令跳变/高度越界，
以及EGO-v2在提高间距要求后的失败。不能把这些记录当成实飞成功或统计排名。

数值Python环境可离线运行 `python ros_cases/assess_cases.py`、
`python system_case_figures.py`、`python test_system_cases.py`。
图4.24—4.28为英文黑白图，附可编辑SVG；来源与执行范围见
`SOURCES_SYSTEM_CASES.md`和`results/system_case_verification.json`。4.3.3仍待编写。
