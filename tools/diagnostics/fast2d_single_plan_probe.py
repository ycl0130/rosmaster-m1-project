#!/usr/bin/env python3
"""Readiness-gated fixed-path or NavigateToPose diagnostic, with persisted evidence."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

STATUS = {GoalStatus.STATUS_UNKNOWN: "UNKNOWN", GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
          GoalStatus.STATUS_EXECUTING: "EXECUTING", GoalStatus.STATUS_CANCELING: "CANCELING",
          GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_CANCELED: "CANCELED",
          GoalStatus.STATUS_ABORTED: "ABORTED"}


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class Probe(Node):
    def __init__(self, args):
        super().__init__("fast2d_single_plan_probe")
        self.args, self.amcl, self.odom, self.scan = args, None, None, None
        self.clock_samples, self.clock_backwards, self.feedback, self.odom_trace = [], False, [], []
        self.cmd = {topic: {"samples": 0, "nonzero_samples": 0, "max_abs_linear_x": 0.0,
                             "max_abs_linear_y": 0.0, "max_abs_angular_z": 0.0}
                    for topic in ["/cmd_vel_nav", "/cmd_vel_smoothed", "/m1/cmd_vel_raw", "/cmd_vel"]}
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 30)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        from rosgraph_msgs.msg import Clock
        self.create_subscription(Clock, "/clock", self.on_clock, 30)
        for topic in self.cmd:
            self.create_subscription(Twist, topic, lambda msg, t=topic: self.on_cmd(t, msg), 30)
        self.compute = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.follow = ActionClient(self, FollowPath, "/follow_path")
        self.navigate = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_amcl(self, msg):
        self.amcl = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw(msg.pose.pose.orientation))

    def on_odom(self, msg):
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw(msg.pose.pose.orientation))
        self.odom_trace.append((time.monotonic(), *self.odom))

    def on_scan(self, msg):
        self.scan = msg

    def on_clock(self, msg):
        value = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.clock_samples and value + 1e-9 < self.clock_samples[-1]:
            self.clock_backwards = True
        self.clock_samples.append(value)

    def on_cmd(self, topic, msg):
        item = self.cmd[topic]
        item["samples"] += 1
        item["max_abs_linear_x"] = max(item["max_abs_linear_x"], abs(msg.linear.x))
        item["max_abs_linear_y"] = max(item["max_abs_linear_y"], abs(msg.linear.y))
        item["max_abs_angular_z"] = max(item["max_abs_angular_z"], abs(msg.angular.z))
        if max(abs(msg.linear.x), abs(msg.linear.y), abs(msg.angular.z)) > 1e-6:
            item["nonzero_samples"] += 1

    def spin_until(self, future, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()

    def pose(self):
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            return (transform.transform.translation.x, transform.transform.translation.y,
                    yaw(transform.transform.rotation))
        except Exception:
            return self.amcl

    def scan_valid(self):
        if self.scan is None or len(self.scan.ranges) < 100:
            return False
        finite = [value for value in self.scan.ranges if math.isfinite(value)]
        return len(finite) >= 100 and not all(abs(value - 0.05) < 1e-3 for value in finite)

    def lifecycle_active(self, name):
        client = self.create_client(GetState, f"/{name}/get_state")
        if not client.wait_for_service(timeout_sec=0.1):
            self.destroy_client(client)
            return False
        future = client.call_async(GetState.Request())
        done = self.spin_until(future, 1.0)
        self.destroy_client(client)
        return bool(done and future.result() and future.result().current_state.id == 3)

    def polygon_stop_enabled(self):
        client = self.create_client(GetParameters, "/collision_monitor/get_parameters")
        if not client.wait_for_service(timeout_sec=0.1):
            self.destroy_client(client)
            return False
        request = GetParameters.Request()
        request.names = ["PolygonStop.enabled"]
        future = client.call_async(request)
        done = self.spin_until(future, 1.0)
        self.destroy_client(client)
        return bool(done and future.result() and future.result().values and
                    future.result().values[0].bool_value)

    def wait_ready(self):
        deadline = time.monotonic() + self.args.ready_timeout
        last = {}
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            last = {"clock_samples": len(self.clock_samples), "clock_monotonic": not self.clock_backwards,
                    "scan_valid": self.scan_valid(), "map_to_base": self.pose() is not None,
                    "planner_active": self.lifecycle_active("planner_server"),
                    "controller_active": self.lifecycle_active("controller_server"),
                    "bt_active": self.lifecycle_active("bt_navigator"),
                    "polygon_stop_enabled": self.polygon_stop_enabled(),
                    "compute_server": self.compute.server_is_ready(),
                    "follow_server": self.follow.server_is_ready(),
                    "navigate_server": self.navigate.server_is_ready()}
            if self.clock_backwards:
                return False, "ENVIRONMENT_INVALID", last
            if len(self.clock_samples) >= 8 and all(last.values()):
                return True, "READY", last
        return False, "READINESS_TIMEOUT", last

    def goal(self):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x, pose.pose.position.y = self.args.x, self.args.y
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(self.args.yaw / 2), math.cos(self.args.yaw / 2)
        return pose

    def send_action(self, client, goal, timeout, feedback=None):
        future = client.send_goal_async(goal, feedback_callback=feedback)
        if not self.spin_until(future, 15):
            return {"accepted": False, "status": "ACCEPT_TIMEOUT"}, None
        handle = future.result()
        if not handle or not handle.accepted:
            return {"accepted": False, "status": "REJECTED"}, None
        future = handle.get_result_async()
        started = time.monotonic()
        if not self.spin_until(future, timeout):
            handle.cancel_goal_async()
            return {"accepted": True, "status": "TIMEOUT", "wall_time_s": time.monotonic() - started}, None
        wrapped = future.result()
        return {"accepted": True, "status": STATUS.get(wrapped.status, str(wrapped.status)),
                "wall_time_s": time.monotonic() - started}, wrapped.result

    def final_metrics(self, report):
        time.sleep(0.3)
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.05)
        final = self.pose()
        report.update({"final_map_pose": final, "final_amcl": self.amcl, "final_odom": self.odom,
                       "cmd_vel_stats": self.cmd, "clock_backwards": self.clock_backwards})
        if final:
            report["final_position_error"] = distance(final, (self.args.x, self.args.y))
            report["final_yaw_error"] = abs(math.atan2(math.sin(final[2] - self.args.yaw), math.cos(final[2] - self.args.yaw)))

    def run(self):
        report = {"goal": [self.args.x, self.args.y, self.args.yaw], "action": self.args.action}
        ready, status, evidence = self.wait_ready()
        report["readiness"] = evidence
        if not ready:
            report["environment_status"] = status
            return report
        report["start"] = self.pose()
        if self.args.action == "navigate":
            goal = NavigateToPose.Goal()
            goal.pose = self.goal()
            result, _ = self.send_action(self.navigate, goal, self.args.timeout,
                                         lambda msg: self.feedback.append({"distance_to_goal": msg.feedback.distance_remaining}))
            report["navigate_to_pose"] = result
        else:
            compute_goal = ComputePathToPose.Goal()
            compute_goal.goal, compute_goal.planner_id, compute_goal.use_start = self.goal(), self.args.planner_id, False
            result, response = self.send_action(self.compute, compute_goal, 35)
            report["compute_path"] = result
            if response is not None and result["status"] == "SUCCEEDED":
                path = response.path
                points = [{"x": p.pose.position.x, "y": p.pose.position.y, "yaw": yaw(p.pose.orientation)} for p in path.poses]
                length = sum(distance((points[i]["x"], points[i]["y"]), (points[i - 1]["x"], points[i - 1]["y"])) for i in range(1, len(points)))
                report["compute_path"].update({"planning_time_s": response.planning_time.sec + response.planning_time.nanosec * 1e-9,
                                                "path_points": len(points), "path_length": length})
                Path(self.args.path_output).write_text(json.dumps({"header_frame": path.header.frame_id, "poses": points}, indent=2))
                follow_goal = FollowPath.Goal()
                follow_goal.path, follow_goal.controller_id, follow_goal.goal_checker_id = path, "FollowPath", "general_goal_checker"
                report["follow_path"], _ = self.send_action(
                    self.follow, follow_goal, self.args.timeout,
                    lambda msg: self.feedback.append({"distance_to_goal": msg.feedback.distance_to_goal, "speed": msg.feedback.speed}))
            report["follow_path_feedback"] = self.feedback
        self.final_metrics(report)
        return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--planner-id", default="GridBased")
    parser.add_argument("--x", type=float, default=2.5)
    parser.add_argument("--y", type=float, default=1.5)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--timeout", type=float, default=100.0)
    parser.add_argument("--ready-timeout", type=float, default=90.0)
    parser.add_argument("--action", choices=["fixed", "navigate"], default="fixed")
    parser.add_argument("--output", default="/tmp/fast2d_single_plan_result.json")
    parser.add_argument("--path-output", default="/tmp/fast2d_single_plan_path.json")
    args = parser.parse_args()
    rclpy.init()
    node = Probe(args)
    try:
        result = node.run()
        Path(args.output).write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
