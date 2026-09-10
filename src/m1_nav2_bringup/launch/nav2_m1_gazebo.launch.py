"""Run the M1 Gazebo world with the Nav2 localization and navigation stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
    TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def _configured_nav2_params(
        params_path, namespace, use_sim_time, scope_enabled, planner_mode,
        fast2d_bt_xml, local_fast2d_mode=None):
    planner_plugin = PythonExpression([
        "'nav2_navfn_planner/NavfnPlanner' if '", planner_mode,
        "' == 'navfn' else 'm1_fast_planner::Fast2DPlanner'",
    ])
    nav_to_pose_tree = PythonExpression([
        "'/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/"
        "navigate_to_pose_w_replanning_and_recovery.xml' if '", planner_mode,
        "' == 'navfn' else '", fast2d_bt_xml, "'",
    ])
    controller_plugin = PythonExpression([
        "'nav2_mppi_controller::MPPIController' if '", local_fast2d_mode or "off",
        "' == 'off' else 'm1_local_fast2d::HybridController'",
    ])
    rewrites = {
                "use_sim_time": use_sim_time,
                (
                    "local_costmap.local_costmap.ros__parameters."
                    "scope_layer.enabled"
                ): scope_enabled,
                "planner_server.ros__parameters.GridBased.plugin": planner_plugin,
                # The Fast2D-specific tree retains Nav2's stock recovery
                # behavior, but asks for the global kinodynamic path at 2 Hz.
                # NavFn keeps Humble's stock 1 Hz tree unchanged.
                "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml": nav_to_pose_tree,
    }
    if local_fast2d_mode is not None:
        # Off uses native MPPI exactly; shadow/active select the thin
        # controller wrapper while preserving all MPPI parameters.
        rewrites["controller_server.ros__parameters.FollowPath.plugin"] = controller_plugin
        rewrites["controller_server.ros__parameters.FollowPath.local_fast2d.mode"] = local_fast2d_mode
    return ParameterFile(
        RewrittenYaml(
            source_file=params_path,
            root_key=namespace,
            param_rewrites=rewrites,
            convert_types=True,
        ),
        allow_substs=True,
    )


def _validate_planner_mode(context):
    mode = LaunchConfiguration("planner_mode").perform(context)
    if mode not in {"navfn", "fast2d"}:
        raise RuntimeError(
            "planner_mode must be either 'navfn' or 'fast2d'; got %r" % mode)
    return []


def _validate_local_fast2d_mode(context):
    mode = LaunchConfiguration("local_fast2d_mode").perform(context)
    if mode not in {"off", "shadow", "active"}:
        raise RuntimeError("local_fast2d_mode must be off, shadow, or active; got %r" % mode)
    return []


def generate_launch_description():
    bringup_share = get_package_share_directory("m1_nav2_bringup")
    support_share = get_package_share_directory("m1_nav2_support")
    scope_share = get_package_share_directory("m1_scope_predictor")
    gazebo_launch = os.path.join(
        support_share, "launch", "m1_gazebo.launch.py")
    params_file = os.path.join(bringup_share, "config", "nav2_params.yaml")
    default_map = os.path.join(bringup_share, "maps", "m1_baseline.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    namespace = LaunchConfiguration("namespace")
    params_path = LaunchConfiguration("params_file")

    configured_params = _configured_nav2_params(
        params_path,
        namespace,
        use_sim_time,
        LaunchConfiguration("scope_enabled"),
        LaunchConfiguration("planner_mode"),
        os.path.join(
            bringup_share, "behavior_trees",
            "navigate_to_pose_fast2d_replanning.xml"),
        LaunchConfiguration("local_fast2d_mode"),
    )
    localization_params = ParameterFile(
        RewrittenYaml(
            source_file=params_path,
            root_key=namespace,
            param_rewrites={
                "use_sim_time": use_sim_time,
                "yaml_filename": LaunchConfiguration("map"),
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Keep the legacy Gazebo launch's arguments local.  In particular, its
    # rviz=false must not disable Nav2's RViz node below.
    gazebo = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    "rviz": "false",
                    "software_lidar": LaunchConfiguration("software_lidar"),
                    "dynamic_obstacles": LaunchConfiguration("dynamic_obstacles"),
                    "dynamic_seed": LaunchConfiguration("dynamic_seed"),
                    "dynamic_motion_mode": LaunchConfiguration("dynamic_motion_mode"),
                    "dynamic_test_sim_time_trigger": LaunchConfiguration("dynamic_test_sim_time_trigger"),
                    "render_engine": LaunchConfiguration("render_engine"),
                    "gpu_lidar_min_angle": LaunchConfiguration("gpu_lidar_min_angle"),
                    "gpu_lidar_max_angle": LaunchConfiguration("gpu_lidar_max_angle"),
                    "dual_gpu_lidar": LaunchConfiguration("dual_gpu_lidar"),
                }.items(),
            ),
        ],
    )

    scope_observer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            scope_share, "launch", "scope_online.launch.py")),
        condition=IfCondition(LaunchConfiguration("scope_enabled")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "model_path": LaunchConfiguration("scope_model_path"),
            "device": LaunchConfiguration("scope_device"),
            "num_samples": LaunchConfiguration("scope_num_samples"),
            "evaluator_enabled": LaunchConfiguration("scope_evaluator_enabled"),
            "rviz": "false",
        }.items(),
    )

    scan_relay = Node(
        package="m1_nav2_bringup",
        executable="scan_relay",
        name="m1_scan_relay",
        condition=IfCondition(LaunchConfiguration("software_lidar")),
        parameters=[{
            # Launch CLI values such as ``scan_dropout_start:=5`` are parsed
            # as integers unless the parameter type is constrained here.
            # ScanRelay declares both values as doubles, so keep the
            # diagnostic gate robust to either integer- or decimal-looking
            # launch arguments.
            "dropout_start_seconds": ParameterValue(
                LaunchConfiguration("scan_dropout_start"), value_type=float),
            "dropout_duration_seconds": ParameterValue(
                LaunchConfiguration("scan_dropout_duration"), value_type=float),
        }],
        output="screen",
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[localization_params],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[localization_params],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )
    localization_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "node_names": ["map_server", "amcl"],
        }],
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[configured_params],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("cmd_vel", "/cmd_vel_nav"),
        ],
    )
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[configured_params],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[configured_params],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("cmd_vel", "/cmd_vel_nav"),
        ],
    )
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[configured_params],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )
    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[configured_params],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[configured_params],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("cmd_vel", "/cmd_vel_nav"),
            ("cmd_vel_smoothed", "/cmd_vel_smoothed"),
        ],
    )
    navigation_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "node_names": [
                "controller_server", "planner_server", "behavior_server",
                "bt_navigator", "waypoint_follower",
            ],
        }],
    )

    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[configured_params],
    )
    safety_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_safety",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "node_names": ["velocity_smoother", "collision_monitor"],
        }],
    )
    watchdog = Node(
        package="m1_nav2_support",
        executable="m1_cmd_watchdog",
        name="m1_cmd_watchdog",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic": "/m1/cmd_vel_raw",
            "output_topic": "/cmd_vel",
            "watchdog_timeout": 0.40,
            "publish_rate": 20.0,
        }],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="nav2_rviz",
        # The legacy Gazebo and SCOPE includes both expose an ``rviz``
        # argument and are deliberately invoked with ``rviz:=false``.  Do
        # not share that argument with this top-level Nav2 visualizer: an
        # included launch can otherwise make this condition false before the
        # delayed action is evaluated.
        condition=IfCondition(LaunchConfiguration("nav2_rviz")),
        arguments=["-d", os.path.join(bringup_share, "rviz", "m1_nav2.rviz")],
        output="screen",
    )

    arguments = [
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "nav2_rviz", default_value="true",
            description="Start the top-level Nav2 RViz visualizer."),
        DeclareLaunchArgument(
            "rviz_start_delay",
            default_value="30.0",
            description=(
                "Delay RViz until Gazebo, localization, Nav2, and the "
                "safety lifecycle nodes have had time to become active.")),
        DeclareLaunchArgument(
            "navigation_start_delay",
            default_value="35.0",
            description=(
                "Wait for Gazebo odom and AMCL map->odom TF before configuring "
                "the global/local costmaps.")),
        DeclareLaunchArgument(
            "scan_dropout_start",
            default_value="-1.0",
            description=(
                "Diagnostic-only software-lidar relay dropout start in seconds "
                "after relay startup; negative disables it.")),
        DeclareLaunchArgument(
            "scan_dropout_duration",
            default_value="0.0",
            description=(
                "Diagnostic-only software-lidar relay dropout duration in "
                "seconds; zero disables it.")),
        DeclareLaunchArgument(
            "software_lidar", default_value="false",
            description="Use deterministic software LaserScan for Gazebo Sim 6 / WSLg."),
        DeclareLaunchArgument(
            "render_engine", default_value="ogre",
            description="Gazebo rendering engine used by GPU LiDAR backend A/B tests."),
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
            "dynamic_obstacles", default_value="true",
            description="Enable moving obstacles for navigation tests."),
        DeclareLaunchArgument(
            "dynamic_seed", default_value="20260814",
            description="Deterministic seed for the moving-obstacle scenario."),
        DeclareLaunchArgument(
            "dynamic_motion_mode", default_value="continuous",
            description="Obstacle motion: continuous or random_waypoint."),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("scope_enabled", default_value="false"),
        DeclareLaunchArgument(
            "scope_start_delay", default_value="28.0",
            description=(
                "Delay observer-only SCOPE startup until the velocity "
                "smoother and collision monitor lifecycle transitions have "
                "completed.")),
        DeclareLaunchArgument(
            "planner_mode", default_value="navfn",
            description=(
                "GridBased implementation: navfn for the frozen baseline or "
                "fast2d for the Phase 1A smoke planner.")),
        DeclareLaunchArgument(
            "local_fast2d_mode", default_value="off",
            description="Experimental local guide: off (native MPPI), shadow, or active."),
        DeclareLaunchArgument("dynamic_test_sim_time_trigger", default_value="false"),
        DeclareLaunchArgument(
            "scope_model_path",
            default_value=(
                "/home/xinlei/Data/SCOPE-repro/reference/scope/"
                "model/scope_model.pth")),
        DeclareLaunchArgument("scope_device", default_value="cuda"),
        DeclareLaunchArgument("scope_num_samples", default_value="4"),
        DeclareLaunchArgument("scope_evaluator_enabled", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("params_file", default_value=params_file),
        DeclareLaunchArgument("map", default_value=default_map),
    ]

    return LaunchDescription(arguments + [
        OpaqueFunction(function=_validate_planner_mode),
        OpaqueFunction(function=_validate_local_fast2d_mode),
        gazebo,
        # SCOPE is strictly an observer, but loading its CUDA model can delay
        # executor scheduling long enough for Humble's lifecycle transition
        # service client to abandon velocity_smoother/configure. Bring the
        # safety chain up first; the scope layer accepts a late first grid.
        # This is ordering only: it does not alter SCOPE inputs or Nav2
        # command/safety wiring.
        TimerAction(
            period=LaunchConfiguration("scope_start_delay"),
            actions=[scope_observer]),
        scan_relay,
        map_server,
        amcl,
        # Let Gazebo spawn M1 and publish odom->base_footprint before AMCL
        # consumes its YAML initial_pose and starts broadcasting map->odom.
        TimerAction(period=5.0, actions=[localization_lifecycle]),
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        # Gazebo needs time to spawn M1 and start /odom plus odom->base TF.
        # Starting controller_server earlier leaves the local controller inactive.
        TimerAction(
            period=LaunchConfiguration("navigation_start_delay"),
            actions=[navigation_lifecycle]),
        collision_monitor,
        TimerAction(period=24.0, actions=[safety_lifecycle]),
        watchdog,
        # RViz is visualization only.  Starting it while Gazebo is creating
        # the OGRE2 GPU-LiDAR render context can starve the lifecycle service
        # callback and make controller_server/change_state time out.  Delay
        # it until after Nav2 and the safety chain are normally active.
        TimerAction(
            period=LaunchConfiguration("rviz_start_delay"),
            actions=[rviz]),
    ])
