#!/usr/bin/env python3
"""Scenario 3: observer-only, single-session Gazebo/Nav2 stress harness.

This program deliberately contains no planner or controller configuration
changes.  It owns one launch process, alternates NavigateToPose goals for the
requested wall-clock duration, and writes raw JSON plus the required Markdown
report before tearing its process group down.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from collections import Counter, deque
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from m1_scope_msgs.msg import ScopePredictionSequence
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node

ROOT = Path(__file__).resolve().parents[2]
ACTIVE = 3
SUCCEEDED = GoalStatus.STATUS_SUCCEEDED
REQUIRED_LIFECYCLES = ("controller_server", "planner_server", "bt_navigator",
                       "velocity_smoother", "collision_monitor")
REQUIRED_NODES = ("controller_server", "planner_server", "bt_navigator",
                  "scope_predictor", "velocity_smoother", "collision_monitor",
                  "m1_cmd_watchdog")


def stop_group(process):
    if not process or process.poll() is not None:
        return
    for sig, grace in ((signal.SIGINT, 15), (signal.SIGTERM, 10), (signal.SIGKILL, 3)):
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=grace)
            return
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass


def number(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


class Stress(Node):
    def __init__(self, args, launch):
        super().__init__("final_scenario_3_stress")
        self.args, self.launch = args, launch
        self.navigate = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.scope_sequence = self.scope_prediction = self.scope_raster = 0
        self.scope_failures = 0
        self.scope_states = Counter()
        self.mppi_cycles = self.no_feasible = 0
        self.first_no_feasible = None
        self.mppi_values = {"feasible_sample_count": 0, "rollout_count": 0,
                            "tube_rejected_count": 0, "collision_rejected_count": 0}
        self.mppi_times = deque(maxlen=100000)
        self.watchdog_events = 0
        self.crashes = []
        self.liveness_failures = []
        self.lifecycle_failures = []
        self.samples = []
        self.goal_results = []
        self.last_scope_wall = 0.0
        self.last_mppi_wall = 0.0
        self.create_subscription(DiagnosticArray, "/local_fast2d/diagnostics", self.on_mppi, 100)
        self.create_subscription(DiagnosticArray, "/scope/diagnostics", self.on_scope_diag, 20)
        self.create_subscription(ScopePredictionSequence, "/scope/prediction_sequence",
                                 self.on_sequence, 20)
        self.create_subscription(OccupancyGrid, "/scope/prediction", self.on_prediction, 20)
        self.create_subscription(OccupancyGrid, "/scope/current_ogm", self.on_raster, 20)

    def on_sequence(self, _msg):
        self.scope_sequence += 1; self.last_scope_wall = time.monotonic()

    def on_prediction(self, _msg):
        self.scope_prediction += 1; self.last_scope_wall = time.monotonic()

    def on_raster(self, _msg):
        self.scope_raster += 1; self.last_scope_wall = time.monotonic()

    def on_scope_diag(self, message):
        for status in message.status:
            values = {item.key: item.value for item in status.values}
            state = values.get("input_state", status.message)
            self.scope_states[state] += 1
            level = status.level[0] if isinstance(status.level, bytes) else status.level
            if level >= 2 or "error" in status.message.lower() or "stopped" in status.message.lower():
                self.scope_failures += 1

    def on_mppi(self, message):
        for status in message.status:
            if status.name != "restricted_mppi":
                continue
            values = {item.key: item.value for item in status.values}
            self.mppi_cycles += 1; self.last_mppi_wall = time.monotonic()
            if status.message == "NO_FEASIBLE_CONTROL": self.no_feasible += 1
            if status.message == "NO_FEASIBLE_CONTROL" and self.first_no_feasible is None:
                self.first_no_feasible = {"wall_time_s": time.monotonic(), "values": values}
            for key in self.mppi_values:
                value = number(values.get(key))
                if value is not None: self.mppi_values[key] += value
            value = number(values.get("controller_total_ms"))
            if value is not None: self.mppi_times.append(value)

    def spin_until(self, future, timeout):
        end = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
        return future.done()

    def lifecycle(self, name):
        client = self.create_client(GetState, "/%s/get_state" % name)
        try:
            if not client.wait_for_service(timeout_sec=0.5): return False
            future = client.call_async(GetState.Request())
            return bool(self.spin_until(future, 2.0) and future.result() and
                        future.result().current_state.id == ACTIVE)
        finally:
            self.destroy_client(client)

    def graph_nodes(self):
        return {name for name, namespace in self.get_node_names_and_namespaces()}

    def health(self, elapsed):
        nodes = self.graph_nodes()
        missing = sorted(set(REQUIRED_NODES) - nodes)
        inactive = [name for name in REQUIRED_LIFECYCLES if not self.lifecycle(name)]
        now = time.monotonic()
        # Scope and MPPI may be idle only prior to their respective warmups.
        stale = []
        if elapsed > 45 and now - self.last_scope_wall > 15: stale.append("scope activity")
        # MPPI diagnostics are emitted while FollowPath is actually executing;
        # before the first goal there is intentionally no controller cycle.
        if self.mppi_cycles and now - self.last_mppi_wall > 15:
            stale.append("MPPI diagnostics")
        if missing: self.liveness_failures.append({"elapsed_s": elapsed, "missing": missing})
        if inactive: self.lifecycle_failures.append({"elapsed_s": elapsed, "inactive": inactive})
        self.samples.append({"elapsed_s": round(elapsed, 2), "missing_nodes": missing,
                             "inactive_lifecycles": inactive, "stale": stale,
                             "launch_returncode": self.launch.poll()})
        return missing, inactive, stale

    def pose(self, x, y, yaw):
        result = PoseStamped(); result.header.frame_id = "map"
        result.pose.position.x, result.pose.position.y = x, y
        result.pose.orientation.z, result.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return result

    def run_goal(self, target, remaining):
        goal = NavigateToPose.Goal(); goal.pose = self.pose(*target)
        future = self.navigate.send_goal_async(goal)
        if not self.spin_until(future, min(15, remaining)):
            return "SEND_TIMEOUT", 0.0
        handle = future.result()
        if not handle or not handle.accepted: return "REJECTED", 0.0
        started = time.monotonic(); future = handle.get_result_async()
        if not self.spin_until(future, min(self.args.goal_timeout, remaining)):
            handle.cancel_goal_async(); return "GOAL_TIMEOUT", time.monotonic() - started
        wrapped = future.result()
        return ("SUCCEEDED" if wrapped.status == SUCCEEDED else "STATUS_%d" % wrapped.status,
                time.monotonic() - started)


def percentile(values, p):
    if not values: return None
    data = sorted(values); return data[min(len(data) - 1, int((len(data) - 1) * p))]


def markdown(report):
    m = report["mppi"]; s = report["scope"]
    return f"""# FINAL SCENARIO 3 STRESS TEST REPORT

## A. Harness

One Gazebo/Nav2 launch session was retained while this observer-only harness alternated NavigateToPose goals. It monitored ROS graph/lifecycle state and existing MPPI/SCOPE diagnostic topics; no planner, controller, constraint, critic, SCOPE model, or safety component was modified.

## B. Frozen configuration

`batch_size: 200`, `time_steps: 30`, `model_dt: 0.067`; Fast2D planner, active local Fast2D controller, Restricted MPPI enabled, and SCOPE enabled.

## C. Continuous runtime

{report['duration_s']:.1f} s requested {report['requested_duration_s']} s. Completion reason: `{report['reason']}`.

## D. Navigation cycles

Completed: {report['navigation']['succeeded']}; attempted: {report['navigation']['attempted']}; failures: {report['navigation']['failed']}.

## E. MPPI statistics

Controller diagnostic cycles: {m['cycles']}; NO_FEASIBLE_CONTROL: {m['no_feasible_control']}; feasible samples: {m['feasible_samples']}; rollouts: {m['rollouts']}; feasible ratio: {m['feasible_ratio']}; tube rejections: {m['tube_rejections']}; collision rejections: {m['collision_rejections']}; controller timing ms (p50/p95/max): {m['timing_ms']}.

## F. SCOPE statistics

Prediction sequences: {s['prediction_sequences']}; prediction grids: {s['prediction_grids']}; raster updates: {s['raster_updates']}; inference failures: {s['inference_failures']}; diagnostic states: `{s['states']}`.

## G. Safety events

Collision Monitor active failures: {report['safety']['collision_monitor_inactive']}; velocity smoother active failures: {report['safety']['velocity_smoother_inactive']}; watchdog missing samples: {report['safety']['watchdog_missing']}; watchdog triggers in logs: {report['safety']['watchdog_triggers']}.

## H. Crash count

{report['crash_count']} (`{report['crashes']}`). Lifecycle failures: {len(report['lifecycle_failures'])}; liveness failures: {len(report['liveness_failures'])}.

## I. Final result

**{report['result']}**
"""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1800.0)
    parser.add_argument("--goal-timeout", type=float, default=120.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scope-model-path", required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy(); env.setdefault("ROS_LOCALHOST_ONLY", "1")
    env["ROS_LOG_DIR"] = str(args.output / "roslog"); Path(env["ROS_LOG_DIR"]).mkdir(exist_ok=True)
    launch_cmd = ["ros2", "launch", "m1_nav2_bringup", "nav2_m1_gazebo.launch.py",
                  "gui:=false", "rviz:=false", "software_lidar:=true", "dynamic_obstacles:=true",
                  "dynamic_motion_mode:=continuous", "planner_mode:=fast2d", "scope_enabled:=true",
                  "local_fast2d_mode:=active", "restricted_mppi_enabled:=true",
                  "scope_model_path:=" + args.scope_model_path]
    with (args.output / "launch.log").open("w") as log:
        launch = subprocess.Popen(launch_cmd, cwd=ROOT, env=env, stdout=log,
                                  stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init(); node = Stress(args, launch); start = time.monotonic(); reason = "DURATION_COMPLETE"
        try:
            wait_end = start + 180
            while time.monotonic() < wait_end and not node.navigate.server_is_ready() and launch.poll() is None:
                rclpy.spin_once(node, timeout_sec=0.2)
            # Do not issue lifecycle GetState RPCs while Nav2's own lifecycle
            # managers are transitioning: Humble can time out a transition if
            # startup service traffic is saturated.  The launch deliberately
            # schedules safety at 24 s, SCOPE at 28 s, and navigation later;
            # wait passively through that window, then begin normal sampling.
            while time.monotonic() < wait_end and launch.poll() is None:
                ready_scope = node.scope_sequence > 0
                ready_window = time.monotonic() - start >= 60.0
                if node.navigate.server_is_ready() and ready_scope and ready_window:
                    break
                rclpy.spin_once(node, timeout_sec=0.2)
            if not (node.navigate.server_is_ready() and node.scope_sequence > 0 and
                    time.monotonic() - start >= 60.0):
                reason = "UNRECOVERABLE_READINESS_FAILURE"
            # Keep the stress route in the already validated x-axis heading
            # convention (yaw=0).  The harness tests session continuity; it
            # must not turn an unvalidated lateral-reference convention into
            # a controller-alignment experiment.
            goals = ((-1.5, -1.5, 0.0), (-2.5, -1.5, 0.0))
            index, next_health, consecutive_navigation_failures = 0, start, 0
            while reason == "DURATION_COMPLETE" and time.monotonic() - start < args.duration:
                elapsed = time.monotonic() - start
                if launch.poll() is not None:
                    reason = "UNRECOVERABLE_LAUNCH_EXIT"; break
                if time.monotonic() >= next_health:
                    missing, inactive, stale = node.health(elapsed); next_health += 5
                    if missing or inactive or stale:
                        reason = "UNRECOVERABLE_RUNTIME_FAILURE"; break
                status, wall = node.run_goal(goals[index % len(goals)], args.duration - elapsed)
                node.goal_results.append({"index": index + 1, "status": status, "wall_time_s": round(wall, 3)})
                index += 1
                if status == "SUCCEEDED":
                    consecutive_navigation_failures = 0
                else:
                    consecutive_navigation_failures += 1
                    # A single action result can be a normal Nav2 recovery
                    # outcome.  Preserve the session and issue the next goal;
                    # three full action timeouts/aborts in a row is the
                    # bounded, reproducible unrecoverable-navigation rule.
                    if consecutive_navigation_failures >= 3:
                        reason = "UNRECOVERABLE_NAVIGATION_FAILURE"
        finally:
            elapsed = time.monotonic() - start
            stop_group(launch)
            try: node.destroy_node()
            finally: rclpy.shutdown()
    log_text = (args.output / "launch.log").read_text(errors="replace").lower()
    crashes = [word for word in ("sigsegv", "segmentation fault", "exit code -11", "exit -11") if word in log_text]
    watchdog_triggers = log_text.count("watchdog trigger") + log_text.count("watchdog timeout")
    feasible = node.mppi_values["feasible_sample_count"]; rollouts = node.mppi_values["rollout_count"]
    report = {"requested_duration_s": args.duration, "duration_s": elapsed, "reason": reason,
              "configuration": {"batch_size": 200, "time_steps": 30, "model_dt": 0.067},
              "navigation": {"attempted": len(node.goal_results), "succeeded": sum(x['status'] == 'SUCCEEDED' for x in node.goal_results), "failed": sum(x['status'] != 'SUCCEEDED' for x in node.goal_results), "goals": node.goal_results},
              "mppi": {"cycles": node.mppi_cycles, "no_feasible_control": node.no_feasible, "feasible_samples": feasible, "rollouts": rollouts, "feasible_ratio": feasible / rollouts if rollouts else None, "tube_rejections": node.mppi_values['tube_rejected_count'], "collision_rejections": node.mppi_values['collision_rejected_count'], "timing_ms": {"p50": percentile(node.mppi_times, .5), "p95": percentile(node.mppi_times, .95), "max": max(node.mppi_times) if node.mppi_times else None}},
              "first_no_feasible_control": node.first_no_feasible,
              "scope": {"prediction_sequences": node.scope_sequence, "prediction_grids": node.scope_prediction, "raster_updates": node.scope_raster, "inference_failures": node.scope_failures, "states": dict(node.scope_states)},
              "safety": {"collision_monitor_inactive": sum('collision_monitor' in x['inactive'] for x in node.lifecycle_failures), "velocity_smoother_inactive": sum('velocity_smoother' in x['inactive'] for x in node.lifecycle_failures), "watchdog_missing": sum('m1_cmd_watchdog' in x['missing'] for x in node.liveness_failures), "watchdog_triggers": watchdog_triggers},
              "crashes": crashes, "crash_count": len(crashes), "liveness_failures": node.liveness_failures, "lifecycle_failures": node.lifecycle_failures, "monitor_samples": node.samples}
    report["result"] = "PASS" if reason == "DURATION_COMPLETE" and elapsed >= args.duration and not crashes and not node.lifecycle_failures and not node.liveness_failures and node.mppi_cycles and node.scope_sequence else "FAIL"
    (args.output / "stress_evidence.json").write_text(json.dumps(report, indent=2, sort_keys=True))
    (args.output / "FINAL_SCENARIO_3_STRESS_TEST_REPORT.md").write_text(markdown(report))
    print(json.dumps({"result": report["result"], "report": str(args.output / 'FINAL_SCENARIO_3_STRESS_TEST_REPORT.md')}, indent=2))
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__": sys.exit(main())
