"""Test-only finite harness; Nav2/Gazebo must already be running."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scope_enabled", default_value="false"), DeclareLaunchArgument("record", default_value="false"), DeclareLaunchArgument("bag_uri", default_value="phase3b_causality"),
        Node(package="m1_local_fast2d", executable="m1_local_fast2d_causality_harness", output="screen", parameters=[{"use_sim_time": True, "scope_enabled": LaunchConfiguration("scope_enabled"), "wait_for_recorder": LaunchConfiguration("record")}]),
        Node(package="m1_nav2_bringup", executable="m1_phase3_recorder", output="screen", condition=IfCondition(LaunchConfiguration("record")), parameters=[{"use_sim_time": True, "scope_enabled": LaunchConfiguration("scope_enabled"), "bag_uri": LaunchConfiguration("bag_uri")}]),
    ])
