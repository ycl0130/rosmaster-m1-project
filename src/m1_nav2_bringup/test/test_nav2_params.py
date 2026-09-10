"""Regression tests for the M1 Nav2 configuration and launch wiring."""

import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration


PACKAGE_ROOT = Path(__file__).parents[1]
PARAMS = PACKAGE_ROOT / "config" / "nav2_params.yaml"
GAZEBO_LAUNCH = PACKAGE_ROOT / "launch" / "nav2_m1_gazebo.launch.py"
RVIZ_CONFIG = Path(__file__).parents[1] / "rviz" / "m1_nav2.rviz"
PACKAGE_XML = PACKAGE_ROOT / "package.xml"


def load_gazebo_launch_module():
    spec = importlib.util.spec_from_file_location(
        "nav2_m1_gazebo", GAZEBO_LAUNCH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def costmap_scan_source(costmap_name):
    parameters = yaml.safe_load(PARAMS.read_text())[costmap_name][costmap_name][
        "ros__parameters"
    ]
    return parameters["obstacle_layer"]["scan"]


def controller_follow_path():
    parameters = yaml.safe_load(PARAMS.read_text())["controller_server"][
        "ros__parameters"
    ]
    return parameters["FollowPath"]


def velocity_smoother_parameters():
    return yaml.safe_load(PARAMS.read_text())["velocity_smoother"][
        "ros__parameters"
    ]


def controller_parameters():
    return yaml.safe_load(PARAMS.read_text())["controller_server"][
        "ros__parameters"
    ]


def planner_parameters():
    return yaml.safe_load(PARAMS.read_text())["planner_server"][
        "ros__parameters"
    ]


def collision_monitor_parameters():
    return yaml.safe_load(PARAMS.read_text())["collision_monitor"][
        "ros__parameters"
    ]


def costmap_parameters(costmap_name):
    return yaml.safe_load(PARAMS.read_text())[costmap_name][costmap_name][
        "ros__parameters"
    ]


def test_gpu_lidar_scan_has_an_explicit_observation_height_window():
    """Laser returns transformed above z=0 must not be filtered by defaults."""
    for costmap_name in ("local_costmap", "global_costmap"):
        scan = costmap_scan_source(costmap_name)
        assert scan["min_obstacle_height"] == 0.0
        assert scan["max_obstacle_height"] >= 0.175


def test_nav2_gazebo_defaults_to_native_gpu_lidar():
    launch_source = GAZEBO_LAUNCH.read_text()
    assert '"software_lidar", default_value="false"' in launch_source


def test_nav2_gazebo_exposes_the_support_render_engine_for_backend_ab():
    launch_source = GAZEBO_LAUNCH.read_text()

    assert '"render_engine": LaunchConfiguration("render_engine")' in launch_source
    assert '"render_engine", default_value="ogre"' in launch_source
    assert '"dual_gpu_lidar": LaunchConfiguration("dual_gpu_lidar")' in launch_source
    assert '"dual_gpu_lidar", default_value="true"' in launch_source


def test_nav2_gazebo_forwards_gpu_lidar_fov_for_the_cubemap_ab():
    launch_source = GAZEBO_LAUNCH.read_text()

    assert '"gpu_lidar_min_angle": LaunchConfiguration("gpu_lidar_min_angle")' in launch_source
    assert '"gpu_lidar_max_angle": LaunchConfiguration("gpu_lidar_max_angle")' in launch_source
    assert '"gpu_lidar_min_angle", default_value="-3.14159265359"' in launch_source
    assert '"gpu_lidar_max_angle", default_value="3.14159265359"' in launch_source


def test_scope_observer_is_opt_in_and_receives_no_navigation_output_topics():
    launch_source = GAZEBO_LAUNCH.read_text()
    assert '"scope_enabled", default_value="false"' in launch_source
    assert 'condition=IfCondition(LaunchConfiguration("scope_enabled"))' in launch_source
    assert '"scope_model_path"' in launch_source
    assert '"/scope/' not in launch_source
    package_xml = (GAZEBO_LAUNCH.parents[1] / "package.xml").read_text()
    assert "<exec_depend>m1_scope_predictor</exec_depend>" in package_xml


def test_scope_costmap_layer_is_local_only_and_precedes_inflation():
    local = costmap_parameters("local_costmap")
    global_costmap = costmap_parameters("global_costmap")

    assert local["plugins"] == [
        "obstacle_layer", "scope_layer", "inflation_layer"
    ]
    assert local["inflation_layer"]["inflation_radius"] == 0.4
    assert local["scope_layer"] == {
        "plugin": "m1_scope_costmap_layer::ScopeLayer",
        "enabled": False,
        "prediction_topic": "/scope/prediction",
        "uncertainty_topic": "/scope/uncertainty",
        "low_threshold": 0.45,
        "lethal_threshold": 0.85,
        "uncertainty_gain": 0.5,
        "uncertainty_encoding_scale": 0.5,
        "medium_cost": 200,
        "stale_timeout": 0.5,
    }
    assert "scope_layer" not in global_costmap["plugins"]
    assert "scope_layer" not in global_costmap


def test_scope_enabled_drives_predictor_and_only_the_local_scope_layer(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros-log"))
    launch_module = load_gazebo_launch_module()
    package_root = GAZEBO_LAUNCH.parents[1]
    monkeypatch.setattr(
        launch_module,
        "get_package_share_directory",
        lambda package_name: str(package_root / package_name),
    )

    launch_description = launch_module.generate_launch_description()
    scope_timers = [
        action for action in launch_description.entities
        if isinstance(action, TimerAction)
        and any(isinstance(child, IncludeLaunchDescription) for child in action.actions)
    ]
    assert len(scope_timers) == 1
    scope_observer = next(
        child for child in scope_timers[0].actions
        if isinstance(child, IncludeLaunchDescription))

    original = yaml.safe_load(PARAMS.read_text())
    for enabled in ("false", "true"):
        context = LaunchContext()
        context.launch_configurations.update({
            "namespace": "",
            "params_file": str(PARAMS),
            "scope_enabled": enabled,
            "planner_mode": "navfn",
            "use_sim_time": "true",
        })
        configured = launch_module._configured_nav2_params(
            LaunchConfiguration("params_file"),
            LaunchConfiguration("namespace"),
            LaunchConfiguration("use_sim_time"),
            LaunchConfiguration("scope_enabled"),
            LaunchConfiguration("planner_mode"),
            "/tmp/navigate_to_pose_fast2d_replanning.xml",
        )
        rewritten_path = configured.evaluate(context)
        rewritten = yaml.safe_load(rewritten_path.read_text())
        configured.cleanup()

        assert scope_observer.condition.evaluate(context) is (
            enabled == "true"
        )
        scope_layer = rewritten["local_costmap"]["local_costmap"][
            "ros__parameters"
        ]["scope_layer"]
        assert scope_layer["enabled"] is (enabled == "true")

        expected = yaml.safe_load(PARAMS.read_text())
        expected["local_costmap"]["local_costmap"]["ros__parameters"][
            "scope_layer"
        ]["enabled"] = enabled == "true"
        expected["planner_server"]["ros__parameters"]["GridBased"][
            "plugin"
        ] = "nav2_navfn_planner/NavfnPlanner"
        expected["bt_navigator"]["ros__parameters"][
            "default_nav_to_pose_bt_xml"
        ] = (
            "/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/"
            "navigate_to_pose_w_replanning_and_recovery.xml"
        )
        for node in expected.values():
            if isinstance(node, dict) and "ros__parameters" in node:
                node["ros__parameters"]["use_sim_time"] = True
            elif isinstance(node, dict):
                for nested in node.values():
                    if (
                        isinstance(nested, dict)
                        and "ros__parameters" in nested
                    ):
                        nested["ros__parameters"]["use_sim_time"] = True
        assert rewritten == expected

    assert original["local_costmap"]["local_costmap"]["ros__parameters"][
        "scope_layer"
    ]["enabled"] is False


def test_planner_mode_rewrites_only_gridbased_plugin(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros-log"))
    launch_module = load_gazebo_launch_module()
    original = yaml.safe_load(PARAMS.read_text())
    for mode, plugin in (
        ("navfn", "nav2_navfn_planner/NavfnPlanner"),
        ("fast2d", "m1_fast_planner::Fast2DPlanner"),
    ):
        context = LaunchContext()
        context.launch_configurations.update({
            "namespace": "",
            "params_file": str(PARAMS),
            "scope_enabled": "false",
            "planner_mode": mode,
            "use_sim_time": "true",
        })
        configured = launch_module._configured_nav2_params(
            LaunchConfiguration("params_file"),
            LaunchConfiguration("namespace"),
            LaunchConfiguration("use_sim_time"),
            LaunchConfiguration("scope_enabled"),
            LaunchConfiguration("planner_mode"),
            "/tmp/navigate_to_pose_fast2d_replanning.xml",
        )
        rewritten_path = configured.evaluate(context)
        rewritten = yaml.safe_load(rewritten_path.read_text())
        configured.cleanup()
        assert rewritten["planner_server"]["ros__parameters"]["GridBased"][
            "plugin"
        ] == plugin
        expected_tree = (
            "/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/"
            "navigate_to_pose_w_replanning_and_recovery.xml"
            if mode == "navfn" else "/tmp/navigate_to_pose_fast2d_replanning.xml"
        )
        assert rewritten["bt_navigator"]["ros__parameters"][
            "default_nav_to_pose_bt_xml"
        ] == expected_tree
        assert rewritten["controller_server"]["ros__parameters"]["FollowPath"] == (
            original["controller_server"]["ros__parameters"]["FollowPath"])
        assert rewritten["local_costmap"]["local_costmap"]["ros__parameters"][
            "scope_layer"
        ]["enabled"] is False


def test_invalid_planner_mode_is_rejected_early():
    launch_module = load_gazebo_launch_module()
    context = LaunchContext()
    context.launch_configurations["planner_mode"] = "invalid"
    with pytest.raises(RuntimeError, match="planner_mode"):
        launch_module._validate_planner_mode(context)


def test_local_fast2d_mode_keeps_native_mppi_off_and_selects_wrapper(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros-log"))
    launch_module = load_gazebo_launch_module()
    for mode, plugin in (
        ("off", "nav2_mppi_controller::MPPIController"),
        ("shadow", "m1_local_fast2d::HybridController"),
        ("active", "m1_local_fast2d::HybridController"),
    ):
        context = LaunchContext()
        context.launch_configurations.update({
            "namespace": "", "params_file": str(PARAMS), "scope_enabled": "false",
            "planner_mode": "fast2d", "local_fast2d_mode": mode,
            "use_sim_time": "true",
        })
        configured = launch_module._configured_nav2_params(
            LaunchConfiguration("params_file"), LaunchConfiguration("namespace"),
            LaunchConfiguration("use_sim_time"), LaunchConfiguration("scope_enabled"),
            LaunchConfiguration("planner_mode"), "/tmp/tree.xml",
            LaunchConfiguration("local_fast2d_mode"))
        rewritten = yaml.safe_load(configured.evaluate(context).read_text())
        configured.cleanup()
        follow_path = rewritten["controller_server"]["ros__parameters"]["FollowPath"]
        assert follow_path["plugin"] == plugin
        assert follow_path["local_fast2d"]["mode"] == mode


def test_invalid_local_fast2d_mode_is_rejected_early():
    launch_module = load_gazebo_launch_module()
    context = LaunchContext()
    context.launch_configurations["local_fast2d_mode"] = "unsafe"
    with pytest.raises(RuntimeError, match="local_fast2d_mode"):
        launch_module._validate_local_fast2d_mode(context)


def test_fast2d_kinodynamic_defaults_stay_within_downstream_envelope():
    params = yaml.safe_load(PARAMS.read_text())
    grid_based = params["planner_server"]["ros__parameters"]["GridBased"]
    assert grid_based["max_velocity_x"] == 0.45
    assert grid_based["max_velocity_y"] == 0.45
    assert grid_based["max_accel_x"] == 0.60
    assert grid_based["max_accel_y"] == 0.60
    assert grid_based["primitive_duration"] == 0.25
    assert grid_based["collision_check_dt"] <= 0.05
    assert grid_based["search_timeout_ms"] == 200
    assert grid_based["max_expansions"] == 100000
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow_path["motion_model"] == "Omni"
    assert follow_path["vx_max"] == 0.5
    assert follow_path["vy_max"] == 0.5


def test_scope_costmap_layer_is_a_runtime_dependency():
    package_xml = PACKAGE_XML.read_text()
    assert "<exec_depend>m1_scope_costmap_layer</exec_depend>" in package_xml


def test_gpu_lidar_scan_marks_and_clears_both_costmaps():
    for costmap_name in ("local_costmap", "global_costmap"):
        scan = costmap_scan_source(costmap_name)
        assert scan["data_type"] == "LaserScan"
        assert scan["marking"] is True
        assert scan["clearing"] is True


def test_costmap_observation_buffers_are_freshness_gated():
    for costmap_name in ("local_costmap", "global_costmap"):
        scan = costmap_scan_source(costmap_name)
        assert scan["observation_persistence"] == 0.0
        assert 0.0 < scan["expected_update_rate"] <= 0.30
        assert scan["inf_is_valid"] is False


def test_local_costmap_requests_complete_diagnostic_snapshots_at_20_hz():
    parameters = costmap_parameters("local_costmap")

    assert parameters["update_frequency"] == 10.0
    assert parameters["publish_frequency"] == 20.0
    assert parameters["always_send_full_costmap"] is True


def test_safety_timeout_hierarchy_is_ordered_after_observation_timeout():
    local_scan = costmap_scan_source("local_costmap")
    smoother = velocity_smoother_parameters()
    collision = collision_monitor_parameters()

    assert local_scan["expected_update_rate"] < collision["source_timeout"]
    assert collision["source_timeout"] <= smoother["velocity_timeout"]
    assert smoother["velocity_timeout"] <= 0.40


def test_gazebo_launch_waits_for_localization_before_navigation_lifecycle():
    launch_source = GAZEBO_LAUNCH.read_text()

    assert '"navigation_start_delay"' in launch_source
    assert 'default_value="35.0"' in launch_source
    assert 'period=LaunchConfiguration("navigation_start_delay")' in launch_source
    assert 'actions=[navigation_lifecycle]' in launch_source


def test_behavior_server_routes_recovery_commands_through_safety_chain():
    launch_source = GAZEBO_LAUNCH.read_text()
    start = launch_source.index("behavior_server = Node")
    end = launch_source.index("bt_navigator = Node")
    behavior_block = launch_source[start:end]
    assert '("cmd_vel", "/cmd_vel_nav")' in behavior_block


def test_nav2_gazebo_forwards_dynamic_scenario_controls():
    launch_source = GAZEBO_LAUNCH.read_text()
    for name, default in (("dynamic_seed", "20260814"), ("dynamic_motion_mode", "continuous")):
        assert f'"{name}": LaunchConfiguration("{name}")' in launch_source
        assert f'"{name}", default_value="{default}"' in launch_source


def test_scan_dropout_gate_is_opt_in_and_only_applies_to_software_lidar():
    launch_source = GAZEBO_LAUNCH.read_text()

    assert '"scan_dropout_start"' in launch_source
    assert 'default_value="-1.0"' in launch_source
    assert '"scan_dropout_duration"' in launch_source
    assert 'default_value="0.0"' in launch_source
    assert 'condition=IfCondition(LaunchConfiguration("software_lidar"))' in launch_source
    assert 'ParameterValue(' in launch_source
    assert 'value_type=float' in launch_source


def test_local_costmap_has_a_dynamic_obstacle_lookahead_window():
    parameters = costmap_parameters("local_costmap")
    scan = parameters["obstacle_layer"]["scan"]

    assert parameters["rolling_window"] is True
    assert parameters["width"] == 5
    assert parameters["height"] == 5
    assert scan["obstacle_max_range"] == 4.0
    assert scan["raytrace_max_range"] == 4.5
    assert parameters["inflation_layer"]["inflation_radius"] == 0.4
    assert parameters["always_send_full_costmap"] is True


def test_mppi_uses_supported_omni_velocity_constraints():
    follow_path = controller_follow_path()
    controller = controller_parameters()

    assert follow_path["plugin"] == "nav2_mppi_controller::MPPIController"
    assert follow_path["motion_model"] == "Omni"
    assert controller["controller_frequency"] == 20.0
    assert follow_path["time_steps"] == 40
    assert follow_path["model_dt"] == 0.05
    assert follow_path["batch_size"] == 500
    assert follow_path["iteration_count"] == 1
    assert follow_path["vx_std"] == 0.3
    assert follow_path["vy_std"] == 0.3
    assert follow_path["wz_std"] == 0.5
    assert follow_path["vx_max"] == 0.5
    assert follow_path["vx_min"] == -0.5
    assert follow_path["vy_max"] == 0.5
    assert follow_path["wz_max"] == 0.8

    for unsupported_parameter in (
        "vy_min",
        "ax_max",
        "ax_min",
        "ay_max",
        "ay_min",
        "az_max",
    ):
        assert unsupported_parameter not in follow_path


def test_velocity_smoother_matches_mppi_limits_and_acceleration_profile():
    smoother = velocity_smoother_parameters()

    assert smoother["smoothing_frequency"] == 20.0
    assert smoother["scale_velocities"] is True
    assert smoother["feedback"] == "CLOSED_LOOP"
    assert smoother["max_velocity"] == [0.25, 0.25, 0.4]
    assert smoother["min_velocity"] == [-0.25, -0.25, -0.4]
    assert smoother["max_accel"] == [0.4, 0.4, 0.7]
    assert smoother["max_decel"] == [-0.5, -0.5, -0.8]


def test_mppi_path_alignment_and_forward_preference_weights():
    follow_path = controller_follow_path()

    assert follow_path["PathAlignCritic"]["cost_weight"] == 4.0
    assert follow_path["PathAngleCritic"]["cost_weight"] == 4.0
    assert follow_path["PreferForwardCritic"]["enabled"] is True
    assert follow_path["PreferForwardCritic"]["cost_weight"] == 2.0
    assert follow_path["GoalCritic"]["cost_weight"] == 8.0
    assert follow_path["GoalAngleCritic"]["cost_weight"] == 3.0
    assert follow_path["TwirlingCritic"]["cost_weight"] == 10.0
    assert follow_path["motion_model"] == "Omni"


def test_collision_monitor_uses_only_the_stop_polygon():
    parameters = collision_monitor_parameters()
    stop = parameters["PolygonStop"]

    assert parameters["polygons"] == ["PolygonStop"]
    assert "PolygonSlow" not in parameters
    assert stop["points"] == [0.30, 0.25, 0.30, -0.25, -0.30, -0.25, -0.30, 0.25]
    assert stop["action_type"] == "stop"
    assert stop["max_points"] == 4


def test_rviz_has_no_collision_slowdown_polygon_display():
    rviz_config = RVIZ_CONFIG.read_text()

    assert "Collision Slowdown Polygon" not in rviz_config
    assert "/polygon_slowdown" not in rviz_config


def test_terminal_goal_and_planner_tolerances_are_consistent_for_near_obstacles():
    controller = controller_parameters()
    planner = planner_parameters()
    goal_checker = controller["general_goal_checker"]

    assert goal_checker["xy_goal_tolerance"] == 0.12
    assert goal_checker["yaw_goal_tolerance"] == 0.40
    assert planner["GridBased"]["tolerance"] <= goal_checker["xy_goal_tolerance"]
