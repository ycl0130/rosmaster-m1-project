# Rosmaster M1 双算法导航仿真

当前 `main` 同时提供 Nav2 MPPI、默认 Imperative 和 localized Imperative 三套 Gazebo 仿真入口。
它们共享 M1 世界和传感器/里程计适配层，但同一时刻只能启动一个控制器。完整的节点、Topic、Action、Service、TF、参数与控制链说明见
[系统架构文档](docs/system_architecture.md)。

## 构建

```bash
cd /path/to/rosmaster-m1-project
source /opt/ros/humble/setup.bash

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-numpy python3-matplotlib python3-pytest \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-mppi-controller \
  ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces \
  ros-humble-robot-state-publisher ros-humble-rviz2 ros-humble-xacro

unset PYTHONPATH

/usr/bin/colcon build --packages-select \
  yahboomcar_description m1_nav2_support m1_nav2_bringup imperative_navigation \
  --symlink-install
source install/setup.bash
```

## Python 与 PyTorch 运行环境

ROS 2 Humble 的 `rclpy` 使用系统 Python 3.10；Imperative 的张量计算使用 `pendulum-rl` 环境中的
PyTorch 和 NumPy。不要直接用 Python 3.12 的 Conda 环境加载 ROS 2 节点。

```bash
conda activate pendulum-rl
source /opt/ros/humble/setup.bash
source install/setup.bash

/usr/bin/python3 -c "import torch, numpy, rclpy; print('torch:', torch.__version__); print('numpy:', numpy.__version__); print('rclpy: ok')"
```

构建使用 `/usr/bin/colcon`，运行 ROS 节点使用 `/usr/bin/python3`；仅通过 `PYTHONPATH` 引入
`pendulum-rl` 的 `torch`，这样不会把系统 `rclpy` 与 Conda 的 Python 版本混用。

## 三套 Gazebo 入口

Nav2 MPPI（支持 RViz **2D Goal Pose** 与 Nav2 Action）：

```bash
ros2 launch m1_nav2_bringup nav2_m1_gazebo.launch.py \
  gui:=true rviz:=true dynamic_obstacles:=true software_lidar:=false
```

默认 Imperative（目标仅由启动参数 `goal_x/goal_y` 给定）：

```bash
ros2 launch imperative_navigation imperative_m1_gazebo.launch.py \
  gui:=true rviz:=true dynamic_obstacles:=true software_lidar:=false
```

localized Imperative（AMCL + map 目标；默认干运行）：

```bash
ros2 launch imperative_navigation imperative_m1_localized_gazebo.launch.py \
  gui:=true enabled:=false
```

本文档只保证并说明仿真路径；实机入口需要单独的硬件驱动与安全验收。
