#!/usr/bin/env python3
"""Readiness-gated fixed-path or NavigateToPose diagnostic, with persisted evidence."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseArray, PoseStamped, Twist, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav2_msgs.srv import GetCostmap
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry, Path as NavPath
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
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
        self.args, self.amcl, self.odom, self.scan, self.sim_scan = args, None, None, None, None
        self.clock_samples, self.clock_backwards, self.feedback, self.odom_trace = [], False, [], []
        self.dynamic_center_min_distance, self.dynamic_samples = math.inf, 0
        self.plan_snapshots = []
        self.scope = {"prediction_messages": 0, "uncertainty_messages": 0,
                      "prediction_nonempty": 0, "uncertainty_nonempty": 0,
                      "diagnostics": []}
        self.restricted_mppi_trace = []
        self.layer_snapshots = {}
        self.first_no_feasible_layer_snapshots = None
        self.first_full_collision_layer_snapshots = None
        self.active_path = []
        self.tracking_start_wall_time = None
        self.tracking_end_wall_time = None
        self.cmd = {topic: {"samples": 0, "nonzero_samples": 0, "max_abs_linear_x": 0.0,
                             "max_abs_linear_y": 0.0, "max_abs_angular_z": 0.0}
                    for topic in ["/cmd_vel_nav", "/cmd_vel_smoothed", "/m1/cmd_vel_raw", "/cmd_vel"]}
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 30)
        self.create_subscription(NavPath, "/plan", self.on_plan, 10)
        self.create_subscription(OccupancyGrid, "/scope/prediction", self.on_scope_prediction, 10)
        self.create_subscription(OccupancyGrid, "/scope/uncertainty", self.on_scope_uncertainty, 10)
        self.create_subscription(DiagnosticArray, "/scope/diagnostics", self.on_scope_diagnostics, 10)
        self.create_subscription(DiagnosticArray, "/local_fast2d/diagnostics", self.on_local_diagnostics, 50)
        for name in ("after_obstacle", "after_scope", "after_inflation"):
            self.create_subscription(
                Costmap, f"/local_costmap/debug/{name}",
                lambda msg, n=name: self.on_layer_snapshot(n, msg), 5)
        # scan_relay intentionally publishes sensor-data QoS (best effort).
        # A default reliable probe subscription is incompatible and silently
        # receives no beams, producing a false readiness failure.
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/sim_scan", self.on_sim_scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap_raw", self.on_local_costmap, 10)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap_raw", self.on_global_costmap, 10)
        self.local_costmap = self.global_costmap = None
        # Evaluation only: this truth topic is never sent to Nav2 or Fast2D.
        self.create_subscription(PoseArray, "/m1/dynamic_obstacles", self.on_dynamic_obstacles, 10)
        from rosgraph_msgs.msg import Clock
        # Simulation clock can be published hundreds of times per second.
        # Retaining a deep queue starves readiness service responses in a
        # single-threaded diagnostic node.
        self.create_subscription(Clock, "/clock", self.on_clock, QoSProfile(depth=1))
        for topic in self.cmd:
            self.create_subscription(Twist, topic, lambda msg, t=topic: self.on_cmd(t, msg), 30)
        self.compute = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.follow = ActionClient(self, FollowPath, "/follow_path")
        self.navigate = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lifecycle_cache, self.costmap_cache = {}, {}
        self.last_readiness_poll = 0.0

    def on_amcl(self, msg):
        self.amcl = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw(msg.pose.pose.orientation))

    def on_odom(self, msg):
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw(msg.pose.pose.orientation))
        self.odom_trace.append((time.monotonic(), *self.odom))

    def on_plan(self, msg):
        points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        length = sum(math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1])
                     for i in range(1, len(points)))
        snapshot = {"wall_time_s": time.monotonic(), "points": len(points), "length_m": length,
                    "start": points[0] if points else None, "end": points[-1] if points else None}
        if not self.plan_snapshots or snapshot["points"] != self.plan_snapshots[-1]["points"] or abs(snapshot["length_m"] - self.plan_snapshots[-1]["length_m"]) > 0.02:
            self.plan_snapshots.append(snapshot)

    def on_scope_prediction(self, msg):
        self.scope["prediction_messages"] += 1
        if any(value > 0 for value in msg.data):
            self.scope["prediction_nonempty"] += 1

    def on_scope_uncertainty(self, msg):
        self.scope["uncertainty_messages"] += 1
        if any(value > 0 for value in msg.data):
            self.scope["uncertainty_nonempty"] += 1

    def on_scope_diagnostics(self, msg):
        for status in msg.status:
            values = {str(item.key): str(item.value) for item in status.values}
            level = status.level[0] if isinstance(status.level, bytes) else status.level
            self.scope["diagnostics"].append({"level": int(level), "message": str(status.message),
                                              "values": values})

    def on_local_diagnostics(self, msg):
        for status in msg.status:
            if status.name != "restricted_mppi":
                continue
            values = {str(item.key): str(item.value) for item in status.values}
            self.restricted_mppi_trace.append({"wall_time_s": time.monotonic(),
                                               "status": str(status.message), "values": values})
            if (status.message == "NO_FEASIBLE_CONTROL" and
                    self.first_no_feasible_layer_snapshots is None):
                self.first_no_feasible_layer_snapshots = {
                    "captured_wall_time_s": time.monotonic(),
                    "snapshots": dict(self.layer_snapshots)}
            if (values.get("collision_full_mask") == "1" and
                    self.first_full_collision_layer_snapshots is None):
                self.first_full_collision_layer_snapshots = {
                    "captured_wall_time_s": time.monotonic(),
                    "collision_full_mask_stamp_ns": values.get("collision_full_mask_stamp_ns"),
                    "snapshots": dict(self.layer_snapshots)}

    def on_layer_snapshot(self, name, msg):
        self.layer_snapshots[name] = {
            "stamp_ns": msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
            "frame": msg.header.frame_id,
            "resolution": msg.metadata.resolution,
            "width": msg.metadata.size_x,
            "height": msg.metadata.size_y,
            "origin_x": msg.metadata.origin.position.x,
            "origin_y": msg.metadata.origin.position.y,
            "data": list(msg.data),
        }

    def on_scan(self, msg):
        self.scan = msg

    def on_sim_scan(self, msg):
        self.sim_scan = msg

    def on_local_costmap(self, msg):
        self.local_costmap = msg

    def on_global_costmap(self, msg):
        self.global_costmap = msg

    def on_dynamic_obstacles(self, msg):
        pose = self.pose()
        if pose is None:
            return
        self.dynamic_samples += len(msg.poses)
        for obstacle in msg.poses:
            self.dynamic_center_min_distance = min(
                self.dynamic_center_min_distance,
                math.hypot(pose[0] - obstacle.position.x, pose[1] - obstacle.position.y))

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

    def tf_valid(self):
        try:
            self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            return True
        except Exception:
            return False

    def unique_graph(self):
        topics = ["/clock", "/odom", "/tf", "/tf_static"]
        publishers = {topic: len(self.get_publishers_info_by_topic(topic)) for topic in topics}
        nodes = self.get_node_names_and_namespaces()
        duplicate_nodes = sorted({name for name, namespace in nodes
                                  if sum(1 for n, ns in nodes if (n, ns) == (name, namespace)) > 1})
        return {"publishers": publishers, "duplicate_node_names": duplicate_nodes,
                "valid": publishers["/clock"] == 1 and publishers["/odom"] == 1
                and publishers["/tf"] >= 1 and publishers["/tf_static"] >= 1
                and not duplicate_nodes}

    def scan_valid(self):
        if self.scan is None or len(self.scan.ranges) < 100:
            return False
        finite = [value for value in self.scan.ranges if math.isfinite(value)]
        return len(finite) >= 100 and not all(abs(value - 0.05) < 1e-3 for value in finite)

    def lifecycle_active(self, name):
        # Lifecycle RPC discovery is considerably more expensive than a topic
        # callback. Poll at 2 Hz; creating a service client on every 100 ms
        # loop can starve the very startup callbacks this gate is observing.
        now = time.monotonic()
        cached = self.lifecycle_cache.get(name)
        if cached and now - cached[0] < 0.5:
            return cached[1]
        client = self.create_client(GetState, f"/{name}/get_state")
        if not client.wait_for_service(timeout_sec=0.1):
            self.destroy_client(client)
            self.lifecycle_cache[name] = (now, False); return False
        future = client.call_async(GetState.Request())
        done = self.spin_until(future, 1.0)
        self.destroy_client(client)
        active = bool(done and future.result() and future.result().current_state.id == 3)
        self.lifecycle_cache[name] = (now, active)
        return active

    def costmap_ready(self, name, observed):
        if observed is not None and bool(observed.data):
            return True
        now = time.monotonic()
        cached = self.costmap_cache.get(name)
        if cached and now - cached[0] < 0.5:
            return cached[1]
        client = self.create_client(GetCostmap, f"/{name}/get_costmap")
        if not client.wait_for_service(timeout_sec=0.1):
            self.destroy_client(client); self.costmap_cache[name] = (now, False); return False
        future = client.call_async(GetCostmap.Request())
        done = self.spin_until(future, 1.0)
        self.destroy_client(client)
        ready = bool(done and future.result() and future.result().map.data)
        self.costmap_cache[name] = (now, ready)
        return ready

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
                    "sim_scan_valid": self.sim_scan is not None and len(self.sim_scan.ranges) >= 100,
                    "scan_valid": self.scan_valid(), "map_to_base_tf": self.tf_valid(),
                    "planner_active": self.lifecycle_active("planner_server"),
                    "controller_active": self.lifecycle_active("controller_server"),
                    "bt_active": self.lifecycle_active("bt_navigator"),
                    "amcl_active": self.lifecycle_active("amcl"),
                    "velocity_smoother_active": self.lifecycle_active("velocity_smoother"),
                    "collision_monitor_active": self.lifecycle_active("collision_monitor"),
                    "local_costmap_ready": self.costmap_ready("local_costmap", self.local_costmap),
                    "global_costmap_ready": self.costmap_ready("global_costmap", self.global_costmap),
                    "polygon_stop_enabled": self.polygon_stop_enabled(),
                    "compute_server": self.compute.server_is_ready(),
                    "follow_server": self.follow.server_is_ready(),
                    "navigate_server": self.navigate.server_is_ready()}
            graph = self.unique_graph()
            last["publisher_graph"] = graph
            if self.clock_backwards:
                return False, "ENVIRONMENT_INVALID", last
            gate = {key: value for key, value in last.items() if key != "publisher_graph"}
            if len(self.clock_samples) >= 8 and all(gate.values()) and graph["valid"]:
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
                       "cmd_vel_stats": self.cmd, "clock_backwards": self.clock_backwards,
                       "replan_snapshots": self.plan_snapshots,
                       "scope_evaluation": self.scope,
                       "restricted_mppi_trace": self.restricted_mppi_trace,
                       "first_no_feasible_layer_snapshots": self.first_no_feasible_layer_snapshots,
                       "first_full_collision_layer_snapshots": self.first_full_collision_layer_snapshots})
        if self.dynamic_samples:
            report["dynamic_truth_evaluation"] = {
                "samples": self.dynamic_samples,
                "minimum_center_distance_m": self.dynamic_center_min_distance}
        if final:
            report["final_position_error"] = distance(final, (self.args.x, self.args.y))
            report["final_yaw_error"] = abs(math.atan2(math.sin(final[2] - self.args.yaw), math.cos(final[2] - self.args.yaw)))
        if self.active_path and self.tracking_start_wall_time is not None:
            samples = [item for item in self.odom_trace
                       if item[0] >= self.tracking_start_wall_time and
                       (self.tracking_end_wall_time is None or item[0] <= self.tracking_end_wall_time)]
            if samples:
                # This is a measurement-only nearest-path deviation; it never
                # feeds Nav2, MPPI, the tube, or any safety decision.
                errors = [min(math.hypot(x - point[0], y - point[1])
                              for point in self.active_path) for _, x, y, _ in samples]
                report["tracking_quality"] = {
                    "sample_count": len(errors),
                    "average_path_deviation_m": sum(errors) / len(errors),
                    "maximum_path_deviation_m": max(errors),
                }

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
                self.active_path = [(point["x"], point["y"]) for point in points]
                length = sum(distance((points[i]["x"], points[i]["y"]), (points[i - 1]["x"], points[i - 1]["y"])) for i in range(1, len(points)))
                report["compute_path"].update({"planning_time_s": response.planning_time.sec + response.planning_time.nanosec * 1e-9,
                                                "path_points": len(points), "path_length": length})
                Path(self.args.path_output).write_text(json.dumps({"header_frame": path.header.frame_id, "poses": points}, indent=2))
                follow_goal = FollowPath.Goal()
                follow_goal.path, follow_goal.controller_id, follow_goal.goal_checker_id = path, "FollowPath", "general_goal_checker"
                self.tracking_start_wall_time = time.monotonic()
                report["follow_path"], _ = self.send_action(
                    self.follow, follow_goal, self.args.timeout,
                    lambda msg: self.feedback.append({"distance_to_goal": msg.feedback.distance_to_goal, "speed": msg.feedback.speed}))
                self.tracking_end_wall_time = time.monotonic()
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
