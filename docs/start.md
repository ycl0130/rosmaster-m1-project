# Nav2 MPPI 仿真启动

```bash
cd /path/to/rosmaster-m1-project
source /opt/ros/humble/setup.bash
colcon build --packages-select yahboomcar_description m1_nav2_support m1_nav2_bringup --symlink-install
source install/setup.bash
ros2 launch m1_nav2_bringup nav2_m1_gazebo.launch.py gui:=true rviz:=true dynamic_obstacles:=true
```

在 RViz 选择 **2D Goal Pose** 设置导航目标。不要单独启动 `m1_nav2_support`：该包只提供 Gazebo 适配、软件激光雷达、动态障碍物场景与最终速度 watchdog，导航由 Nav2 启动文件统一管理。三套仿真入口及完整接口说明见 [system_architecture.md](system_architecture.md)。
