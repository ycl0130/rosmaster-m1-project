"""Launch the observer-only M1 SCOPE predictor and optional RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("m1_scope_predictor")
    config = os.path.join(share, "config", "scope_online.yaml")
    rviz_config = os.path.join(share, "rviz", "scope_online.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "model_path",
            default_value=EnvironmentVariable(
                "M1_SCOPE_MODEL_PATH", default_value="scope_model.pth")),
        DeclareLaunchArgument("device", default_value="cuda"),
        DeclareLaunchArgument("num_samples", default_value="4"),
        DeclareLaunchArgument("sequence_horizon_steps", default_value="2"),
        DeclareLaunchArgument("evaluator_enabled", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        Node(
            package="m1_scope_predictor", executable="scope_predictor",
            # The ROS Humble console-script wrapper is generated with the
            # Select a Python with PyTorch via M1_SCOPE_PYTHON. The portable
            # fallback is the current system python3 executable.
            prefix=EnvironmentVariable("M1_SCOPE_PYTHON", default_value="python3"),
            name="scope_predictor", output="screen",
            parameters=[config, {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool),
                "model_path": ParameterValue(
                    LaunchConfiguration("model_path"), value_type=str),
                "device": ParameterValue(
                    LaunchConfiguration("device"), value_type=str),
                "num_samples": ParameterValue(
                    LaunchConfiguration("num_samples"), value_type=int),
                "sequence_horizon_steps": ParameterValue(
                    LaunchConfiguration("sequence_horizon_steps"), value_type=int),
                "evaluator_enabled": ParameterValue(
                    LaunchConfiguration("evaluator_enabled"), value_type=bool),
            }],
        ),
        Node(
            package="rviz2", executable="rviz2", name="scope_rviz",
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", rviz_config], output="screen"),
    ])
