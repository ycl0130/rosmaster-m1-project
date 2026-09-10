"""Launch M1 Gazebo Sim resources used by the Nav2 MPPI bringup."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_gazebo(context):
    package_share = get_package_share_directory("m1_nav2_support")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    server_only = "-s " if LaunchConfiguration("gui").perform(context).lower() == "false" else ""
    render_engine = LaunchConfiguration("render_engine").perform(context)
    if render_engine not in {"ogre", "ogre2"}:
        raise ValueError("render_engine must be 'ogre' or 'ogre2'")
    world_name = "m1_ogre.sdf" if render_engine == "ogre" else "m1_ogre2_regression.sdf"
    world = os.path.join(package_share, "worlds", world_name)

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            ros_gz_sim_share, "launch", "gz_sim.launch.py")),
        # OGRE2 is required for the GPU LiDAR's full 360-degree field of view.
        launch_arguments={
            "gz_args": (
                f"{server_only}-r -v 4 --render-engine "
                f"{render_engine} {world}")}.items(),
    )]


def generate_launch_description():
    description_share = get_package_share_directory("yahboomcar_description")
    description_resource_root = os.path.dirname(description_share)
    model_file = os.path.join(
        description_share, "urdf", "yahboomcar_M1_gazebo.urdf.xacro")
    # Feed Gazebo the expanded XML directly. This avoids the unreliable
    # transient-local /robot_description topic handshake in ros_gz_sim 0.244.
    robot_description_xml = Command([
        FindExecutable(name="xacro"), " ", model_file,
        " enable_gpu_lidar:=",
        PythonExpression([
            "'false' if '", LaunchConfiguration("software_lidar"),
            "' == 'true' else 'true'"]),
        " gpu_lidar_min_angle:=", LaunchConfiguration("gpu_lidar_min_angle"),
        " gpu_lidar_max_angle:=", LaunchConfiguration("gpu_lidar_max_angle"),
        " dual_gpu_lidar:=", LaunchConfiguration("dual_gpu_lidar"),
    ])
    robot_description = ParameterValue(robot_description_xml, value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "robot_description": robot_description,
        }],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_m1",
        output="screen",
        arguments=[
            "-string", robot_description_xml,
            "-name", "m1",
            "-x", "-2.5",
            "-y", "-1.5",
            "-z", "0.01",
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="m1_gazebo_bridge_core",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/world/m1/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/ground_truth/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/model/moving_obstacle_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/model/moving_obstacle_2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/model/moving_obstacle_3/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/world/m1/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/world/m1/set_pose@ros_gz_interfaces/srv/SetEntityPose",
        ],
        remappings=[
            ("/world/m1/clock", "/clock"),
            ("/world/m1/dynamic_pose/info", "/m1/gazebo_dynamic_tf"),
        ],
        output="screen",
    )
    single_scan_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="m1_gazebo_bridge_scan",
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("software_lidar"), "' == 'false' and '",
            LaunchConfiguration("dual_gpu_lidar"), "' == 'false'",
        ])),
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        output="screen",
    )
    dual_scan_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="m1_gazebo_bridge_dual_scan",
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("software_lidar"), "' == 'false' and '",
            LaunchConfiguration("dual_gpu_lidar"), "' == 'true'",
        ])),
        arguments=[
            "/scan_front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/scan_rear@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        output="screen",
    )
    dual_laser_merger = Node(
        package="dual_laser_merger",
        executable="dual_laser_merger_node",
        name="m1_dual_laser_merger",
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("software_lidar"), "' == 'false' and '",
            LaunchConfiguration("dual_gpu_lidar"), "' == 'true'",
        ])),
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "laser_1_topic": "/scan_front",
            "laser_2_topic": "/scan_rear",
            "merged_scan_topic": "/scan",
            "merged_cloud_topic": "/scan_merged_cloud",
            "target_frame": "laser_scan_link",
            "angle_min": -3.14159265359,
            "angle_max": 3.14159265359,
            "angle_increment": 0.009420067926806,
            "scan_time": 0.083333333333333,
            "range_min": 0.05,
            "range_max": 12.0,
            "min_height": -0.05,
            "max_height": 0.05,
            "use_inf": True,
            "enable_calibration": False,
            "enable_shadow_filter": False,
            "enable_average_filter": False,
        }],
    )

    dynamic_obstacle_mover = Node(
        package="m1_nav2_support",
        executable="dynamic_obstacle_mover",
        name="dynamic_obstacle_mover",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "enabled": LaunchConfiguration("dynamic_obstacles"),
            "random_seed": LaunchConfiguration("dynamic_seed"),
            "motion_mode": LaunchConfiguration("dynamic_motion_mode"),
            "test_sim_time_trigger": LaunchConfiguration("dynamic_test_sim_time_trigger"),
        }],
    )

    software_lidar = Node(
        package="m1_nav2_support",
        executable="software_lidar",
        name="software_lidar",
        condition=IfCondition(LaunchConfiguration("software_lidar")),
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "pose_topic": "/ground_truth/odom",
        }],
    )
    odom_slip_simulator = Node(
        package="m1_nav2_support",
        executable="odom_slip_simulator",
        name="odom_slip_simulator",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "enabled": LaunchConfiguration("slip_enabled"),
            "profile": LaunchConfiguration("slip_profile"),
            "x_scale": LaunchConfiguration("odom_x_scale"),
            "y_scale": LaunchConfiguration("odom_y_scale"),
            "yaw_scale": LaunchConfiguration("odom_yaw_scale"),
            "x_bias_per_second": LaunchConfiguration("odom_x_bias"),
            "y_bias_per_second": LaunchConfiguration("odom_y_bias"),
            "yaw_bias_per_second": LaunchConfiguration("odom_yaw_bias"),
            "x_noise_std": LaunchConfiguration("odom_x_noise_std"),
            "y_noise_std": LaunchConfiguration("odom_y_noise_std"),
            "yaw_noise_std": LaunchConfiguration("odom_yaw_noise_std"),
            "random_walk_xy_std": LaunchConfiguration("odom_random_walk_xy_std"),
            "random_walk_yaw_std": LaunchConfiguration("odom_random_walk_yaw_std"),
            "random_seed": LaunchConfiguration("slip_seed"),
            "burst_enabled": LaunchConfiguration("burst_enabled"),
            "burst_start": LaunchConfiguration("burst_start"),
            "burst_duration": LaunchConfiguration("burst_duration"),
            "burst_x_scale": LaunchConfiguration("burst_x_scale"),
            "burst_y_scale": LaunchConfiguration("burst_y_scale"),
            "burst_yaw_scale": LaunchConfiguration("burst_yaw_scale"),
        }],
    )

    # Gazebo Sim resolves package:// mesh URIs as model:// package paths.  The
    # parent of the ament share directory is therefore part of its resource
    # search path so the copied M1 STL meshes can be loaded after spawning.
    ignition_resource_path = os.pathsep.join(filter(
        None, [description_resource_root,
               os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")]))
    gz_resource_path = os.pathsep.join(filter(
        None, [description_resource_root,
               os.environ.get("GZ_SIM_RESOURCE_PATH", "")]))

    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz", default_value="false",
            description="Unused adapter compatibility argument."),
        DeclareLaunchArgument("gui", default_value="true", description="Open the Gazebo GUI."),
        DeclareLaunchArgument(
            "render_engine", default_value="ogre",
            description="Gazebo rendering engine used by GPU LiDAR A/B tests."),
        DeclareLaunchArgument(
            "dual_gpu_lidar", default_value="true",
            description="Use coincident front/rear 180-degree GPU LiDAR sensors."),
        DeclareLaunchArgument(
            "gpu_lidar_min_angle", default_value="-3.14159265359",
            description="GPU LiDAR horizontal minimum angle in radians."),
        DeclareLaunchArgument(
            "gpu_lidar_max_angle", default_value="3.14159265359",
            description="GPU LiDAR horizontal maximum angle in radians."),
        DeclareLaunchArgument(
            "software_lidar", default_value="false",
            description=(
                "Use the software fallback LaserScan. Set true if the GPU lidar "
                "is unavailable on this WSLg installation.")),
        DeclareLaunchArgument(
            "dynamic_obstacles", default_value="true",
            description="Move dynamic obstacles; false parks them outside the room."),
        DeclareLaunchArgument(
            "dynamic_seed", default_value="20260814",
            description="Deterministic seed for the moving-obstacle scenario."),
        DeclareLaunchArgument(
            "dynamic_motion_mode", default_value="continuous",
            description=(
                "Obstacle motion: continuous smooth-random motion or "
                "random_waypoint for waypoint patrol.")),
        DeclareLaunchArgument("dynamic_test_sim_time_trigger", default_value="false",
            description="Test-only crossing trigger from causality T0 and /clock."),
        DeclareLaunchArgument("slip_enabled", default_value="true"),
        DeclareLaunchArgument("slip_profile", default_value="none"),
        DeclareLaunchArgument("slip_seed", default_value="20260902"),
        DeclareLaunchArgument("odom_x_scale", default_value="1.0"),
        DeclareLaunchArgument("odom_y_scale", default_value="1.0"),
        DeclareLaunchArgument("odom_yaw_scale", default_value="1.0"),
        DeclareLaunchArgument("odom_x_bias", default_value="0.0"),
        DeclareLaunchArgument("odom_y_bias", default_value="0.0"),
        DeclareLaunchArgument("odom_yaw_bias", default_value="0.0"),
        DeclareLaunchArgument("odom_x_noise_std", default_value="0.0"),
        DeclareLaunchArgument("odom_y_noise_std", default_value="0.0"),
        DeclareLaunchArgument("odom_yaw_noise_std", default_value="0.0"),
        DeclareLaunchArgument("odom_random_walk_xy_std", default_value="0.0"),
        DeclareLaunchArgument("odom_random_walk_yaw_std", default_value="0.0"),
        DeclareLaunchArgument("burst_enabled", default_value="false"),
        DeclareLaunchArgument("burst_start", default_value="0.0"),
        DeclareLaunchArgument("burst_duration", default_value="0.0"),
        DeclareLaunchArgument("burst_x_scale", default_value="1.0"),
        DeclareLaunchArgument("burst_y_scale", default_value="1.0"),
        DeclareLaunchArgument("burst_yaw_scale", default_value="1.0"),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", ignition_resource_path),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_path),
        OpaqueFunction(function=launch_gazebo),
        robot_state_publisher,
        TimerAction(period=2.0, actions=[spawn_robot]),
        bridge,
        single_scan_bridge,
        dual_scan_bridge,
        dual_laser_merger,
        dynamic_obstacle_mover,
        odom_slip_simulator,
        software_lidar,
    ])
