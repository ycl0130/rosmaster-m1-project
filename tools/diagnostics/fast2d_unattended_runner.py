#!/usr/bin/env python3
"""Bounded, isolated Gazebo/Nav2 experiment runner."""
import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = Path("/tmp/fast2d_final_autofix")
DOMAIN_BASE = 71


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True))


def sourced(command):
    return ["bash", "-lc", "source /opt/ros/humble/setup.bash && source install/setup.bash && exec " + command]


def stop_process_group(process):
    if process.poll() is not None:
        return
    for sig, grace in ((signal.SIGINT, 15), (signal.SIGTERM, 10), (signal.SIGKILL, 3)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(grace)
            return
        except subprocess.TimeoutExpired:
            pass


def state(stage, **extra):
    OUT.mkdir(parents=True, exist_ok=True)
    write_json(OUT / "STATE.json", {"current_stage": stage, "updated_unix_s": time.time(), **extra})
    (OUT / "CONTINUE.md").write_text(
        "Resume from STATE.json and the most recent case result. Each retry gets a fresh "
        "ROS_DOMAIN_ID and IGN_PARTITION; only its own process group is terminated.\n")


def environment(index, name):
    domain = DOMAIN_BASE + (index % 80)
    partition = f"fast2d_run_{int(time.time())}_{index}_{name}".replace("/", "_")
    env = os.environ.copy()
    env.update({"ROS_DOMAIN_ID": str(domain), "IGN_PARTITION": partition, "GZ_PARTITION": partition})
    return env, domain, partition


def one(mode, name, index, dynamic=False, seed=None, action="fixed", retries=3,
        x=2.5, y=1.5, yaw=0.0, motion_mode="continuous", scope_enabled=False,
        scope_model_path=None):
    case_dir = OUT / name
    case_dir.mkdir(parents=True, exist_ok=True)
    for attempt in range(retries):
        env, domain, partition = environment(index * retries + attempt, name)
        run_dir = case_dir / f"attempt_{attempt + 1}"
        run_dir.mkdir(parents=True, exist_ok=True)
        # Codex runs with a read-only home directory.  Keep all ROS launch and
        # rclpy logs with the run evidence instead of falling back to ~/.ros.
        ros_log_dir = run_dir / "roslog"
        ros_log_dir.mkdir(exist_ok=True)
        env["ROS_LOG_DIR"] = str(ros_log_dir)
        # Ignition also writes runtime state under $HOME by default.  The
        # workspace sandbox mounts the real home read-only, so give every run
        # an isolated writable home alongside its evidence.
        run_home = run_dir / "home"
        run_home.mkdir(exist_ok=True)
        env["HOME"] = str(run_home)
        metadata = {"name": name, "attempt": attempt + 1, "planner_mode": mode,
                    "dynamic_obstacles": dynamic, "dynamic_seed": seed, "action": action,
                    "goal": [x, y, yaw],
                    "ROS_DOMAIN_ID": domain, "IGN_PARTITION": partition, "GZ_PARTITION": partition,
                    "ROS_LOG_DIR": str(ros_log_dir), "HOME": str(run_home)}
        write_json(run_dir / "environment.json", metadata)
        launch = ["ros2", "launch", "m1_nav2_bringup", "nav2_m1_gazebo.launch.py",
                  "gui:=false", "rviz:=false", "software_lidar:=true",
                  f"dynamic_obstacles:={str(dynamic).lower()}",
                  f"scope_enabled:={str(scope_enabled).lower()}",
                  f"planner_mode:={mode}", f"dynamic_motion_mode:={motion_mode}"]
        if scope_model_path:
            launch.append(f"scope_model_path:={scope_model_path}")
        if seed is not None:
            launch.append(f"dynamic_seed:={seed}")
        launch_command = " ".join(subprocess.list2cmdline([item]) for item in launch)
        state("launching", current_case=name, ROS_DOMAIN_ID=domain, IGN_PARTITION=partition)
        with (run_dir / "launch.log").open("w") as log:
            process = subprocess.Popen(sourced(launch_command), cwd=ROOT, env=env, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            try:
                probe = ["python3", "tools/diagnostics/fast2d_single_plan_probe.py",
                         "--output", str(run_dir / "result.json"),
                         "--path-output", str(run_dir / "path.json"), "--action", action,
                         "--x", str(x), "--y", str(y), "--yaw", str(yaw)]
                command = " ".join(subprocess.list2cmdline([item]) for item in probe)
                # The probe can legitimately spend ready_timeout plus its
                # action timeout in one foreground invocation.  Keep this
                # outer guard larger than their sum and preserve a formal
                # result if the guard itself fires.
                completed = subprocess.run(sourced(command), cwd=ROOT, env=env, text=True,
                                           capture_output=True, timeout=230)
                (run_dir / "probe.stdout").write_text(completed.stdout)
                (run_dir / "probe.stderr").write_text(completed.stderr)
            except subprocess.TimeoutExpired as error:
                (run_dir / "runner_error.txt").write_text(repr(error))
                write_json(run_dir / "result.json", {
                    "environment_status": "RUNNER_TIMEOUT",
                    "error": "probe exceeded runner timeout"})
            except Exception as error:
                (run_dir / "runner_error.txt").write_text(repr(error))
            finally:
                stop_process_group(process)
        result_path = run_dir / "result.json"
        report = json.loads(result_path.read_text()) if result_path.exists() else {"error": "no result"}
        report["environment"] = metadata
        write_json(run_dir / "result.json", report)
        write_json(case_dir / "result.json", report)
        if report.get("environment_status") == "ENVIRONMENT_INVALID" and attempt + 1 < retries:
            continue
        state("case_complete", last_completed_test=name, last_result=report,
              next_test="caller queue", ROS_DOMAIN_ID=domain, IGN_PARTITION=partition)
        return report
    return {"error": "all attempts invalid"}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["fast2d", "navfn"], required=True)
    parser.add_argument("--name", required=True)
    parser.add_argument("--dynamic", action="store_true")
    parser.add_argument("--seed", type=int)
    parser.add_argument("--action", choices=["fixed", "navigate"], default="fixed")
    parser.add_argument("--index", type=int, default=0)
    parser.add_argument("--x", type=float, default=2.5)
    parser.add_argument("--y", type=float, default=1.5)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--motion-mode", default="continuous")
    parser.add_argument("--scope-enabled", action="store_true")
    parser.add_argument("--scope-model-path")
    args = parser.parse_args()
    print(json.dumps(one(args.mode, args.name, args.index, args.dynamic, args.seed, args.action,
                         x=args.x, y=args.y, yaw=args.yaw,
                         motion_mode=args.motion_mode,
                         scope_enabled=args.scope_enabled,
                         scope_model_path=args.scope_model_path), indent=2))


if __name__ == "__main__":
    main()
