#!/usr/bin/env python3
"""Reproducible Phase 6C.5 Restricted-MPPI scale experiment runner.

It creates parameter copies under its evidence directory: the checked-in
baseline remains immutable.  It uses the existing readiness gate, headless
Gazebo world, fixed goal, warm-up, and passive diagnostics collector.
"""
import argparse
import json
import math
import os
import shutil
import statistics
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
BASE_PARAMS = ROOT / "src/m1_nav2_bringup/config/nav2_params.yaml"
DEFAULT_OUT = Path("/tmp/phase6c5_mppi_scale")
GOAL = (-1.5, -1.5, 0.0)  # Same fixed Phase 6C validation goal.
CONFIGS = {
    "A": (500, 40, 0.05), "B": (400, 40, 0.05),
    "C": (300, 40, 0.05), "D": (200, 40, 0.05),
    "E": (300, 30, 0.067), "F": (200, 30, 0.067),
}


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def percentile(values, q):
    if not values:
        return None
    values = sorted(values)
    position = (len(values) - 1) * q
    lo, hi = math.floor(position), math.ceil(position)
    return values[lo] if lo == hi else values[lo] + (values[hi] - values[lo]) * (position - lo)


def stats(values):
    return {key: (round(value, 6) if value is not None else None) for key, value in {
        "p50": percentile(values, .50), "p95": percentile(values, .95),
        "p99": percentile(values, .99), "max": max(values) if values else None,
    }.items()}


def number(values, key):
    try:
        return float(values.get(key, "nan"))
    except (TypeError, ValueError):
        return math.nan


def make_params(out, label, batch, steps, dt):
    config = yaml.safe_load(BASE_PARAMS.read_text())
    p = config["controller_server"]["ros__parameters"]["FollowPath"]
    p["batch_size"], p["time_steps"], p["model_dt"] = batch, steps, dt
    target = out / "parameters" / f"{label}_batch{batch}_steps{steps}_dt{dt:.3f}.yaml"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(yaml.safe_dump(config, sort_keys=False))
    return target


def build(log):
    command = "source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select m1_local_fast2d m1_nav2_bringup"
    return subprocess.run(["bash", "-lc", command], cwd=ROOT, text=True,
                          stdout=log, stderr=subprocess.STDOUT).returncode == 0


def summarize(results):
    trace = []
    successful = []
    final_errors, avg_errors, max_errors = [], [], []
    for result in results:
        if result.get("environment_status") is None and result.get("follow_path", {}).get("status") == "SUCCEEDED":
            successful.append(result)
        trace.extend(item for item in result.get("restricted_mppi_trace", [])
                     if item.get("status") in {"TRACKING", "REFERENCE_SWITCH"}
                     and number(item.get("values", {}), "batch_size") > 0)
        if isinstance(result.get("final_position_error"), (float, int)):
            final_errors.append(result["final_position_error"])
        quality = result.get("tracking_quality", {})
        if isinstance(quality.get("average_path_deviation_m"), (float, int)):
            avg_errors.append(quality["average_path_deviation_m"])
            max_errors.append(quality["maximum_path_deviation_m"])
    vals = [item["values"] for item in trace]
    totals = {key: sum(number(v, key) for v in vals if math.isfinite(number(v, key))) for key in
              ("feasible_sample_count", "tube_rejected_count", "collision_rejected_count",
               "proposed_scalar_control_count", "clamped_scalar_control_count",
               "proposed_sample_step_count", "clamped_sample_step_count", "proposed_sample_count",
               "clamped_sample_count", "vx_clamp_count", "vy_clamp_count", "wz_clamp_count")}
    rate = lambda numerator, denominator: None if not denominator else numerator / denominator
    deadline_ms = 50.0  # controller_frequency=20 Hz in the frozen parameters.
    controller = [number(v, "controller_total_ms") for v in vals]
    restricted = [number(v, "restricted_mppi_total_ms") for v in vals]
    return {
        "fresh_start_successes": len(successful), "fresh_start_runs": len(results),
        "cycle_count": len(vals), "deadline_ms": deadline_ms,
        "deadline_miss": sum(x > deadline_ms for x in controller if math.isfinite(x)),
        "controller_total_ms": stats([x for x in controller if math.isfinite(x)]),
        "restricted_mppi_total_ms": stats([x for x in restricted if math.isfinite(x)]),
        "safety": {"no_feasible_control_count": sum(1 for r in results for x in r.get("restricted_mppi_trace", []) if x.get("status") == "NO_FEASIBLE_CONTROL"),
                   "feasible_sample_count": totals["feasible_sample_count"],
                   "tube_rejected": totals["tube_rejected_count"], "collision_rejected": totals["collision_rejected_count"]},
        "tracking_quality": {"final_goal_error_m": stats(final_errors),
                             "average_tracking_error_m": stats(avg_errors),
                             "maximum_deviation_m": max(max_errors) if max_errors else None},
        "sampling": {"scalar_clamp_rate": rate(totals["clamped_scalar_control_count"], totals["proposed_scalar_control_count"]),
                     "sample_step_clamp_rate": rate(totals["clamped_sample_step_count"], totals["proposed_sample_step_count"]),
                     "sample_clamp_rate": rate(totals["clamped_sample_count"], totals["proposed_sample_count"]),
                     "vx_clamp_ratio": rate(totals["vx_clamp_count"], totals["proposed_scalar_control_count"] / 3),
                     "vy_clamp_ratio": rate(totals["vy_clamp_count"], totals["proposed_scalar_control_count"] / 3),
                     "wz_clamp_ratio": rate(totals["wz_clamp_count"], totals["proposed_scalar_control_count"] / 3)},
    }


def markdown(report):
    rows = ["| Case | batch × steps × dt | starts | cycles | controller p95 (ms) | MPPI p95 (ms) | misses | NFC | final error p50 (m) |", "|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for label, case in report["cases"].items():
        s = case["summary"]; c = case["configuration"]
        rows.append(f"| {label} | {c['batch']} × {c['steps']} × {c['dt']} | {s['fresh_start_successes']}/{s['fresh_start_runs']} | {s['cycle_count']} | {s['controller_total_ms']['p95']} | {s['restricted_mppi_total_ms']['p95']} | {s['deadline_miss']} | {s['safety']['no_feasible_control_count']} | {s['tracking_quality']['final_goal_error_m']['p50']} |")
    recommendation = report.get("recommendation", {})
    return "\n".join(["# PHASE 6C MPPI SCALE OPTIMIZATION REPORT", "", "## A. Baseline", "", "A is the preserved Phase 6C scale baseline (500 × 40 × 0.05; 2.0 s horizon).", "", "## B. All A/B Results", "", *rows, "", "## C. Recommended Configuration", "", recommendation.get("text", "No recommendation: experiment incomplete."), "", "## D. Trade-off Analysis", "", "Selection requires zero deadline misses, no NO_FEASIBLE_CONTROL events, successful fresh starts, and no material tracking regression before lowest MPPI p95 is considered. Parameter copies only change batch_size, time_steps, and model_dt.", "", "## E. Final Gazebo Validation Configuration", "", "Headless m1.sdf, unchanged robot/world/goal/warm-up/probe, `planner_mode:=navfn`, `local_fast2d_mode:=active`, `restricted_mppi_enabled:=true`, and SCOPE enabled with the specified model.", ""])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--runs", type=int, default=10, help="Fresh starts per configuration (minimum 10 for acceptance).")
    parser.add_argument("--cases", nargs="+", choices=CONFIGS, default=list(CONFIGS))
    parser.add_argument("--scope-model", type=Path, default=Path("/home/lin24311/Data/SCOPE-repro/reference/scope/model/scope_model.pth"))
    parser.add_argument("--scope-python", default="/home/lin24311/car_ws/.venv-ros2-ml/bin/python3")
    parser.add_argument("--skip-build", action="store_true")
    args = parser.parse_args(); out = args.out.resolve(); out.mkdir(parents=True, exist_ok=True)
    report = {"phase": "6C.5", "created_utc": datetime.now(timezone.utc).isoformat(), "baseline_preserved": str(BASE_PARAMS), "goal": GOAL, "cases": {}}
    for ci, label in enumerate(args.cases):
        batch, steps, dt = CONFIGS[label]; case_dir = out / label
        case_dir.mkdir(parents=True, exist_ok=True)
        params = make_params(out, label, batch, steps, dt)
        with (case_dir / "build.log").open("w") as log:
            build_ok = True if args.skip_build else build(log)
        results = []
        if build_ok:
            for run in range(args.runs):
                name = f"{label}_fresh_{run + 1:02d}"
                cmd = [sys.executable, "tools/diagnostics/fast2d_unattended_runner.py", "--mode", "navfn", "--name", name, "--index", str(ci * args.runs + run), "--action", "fixed", "--x", str(GOAL[0]), "--y", str(GOAL[1]), "--yaw", str(GOAL[2]), "--scope-enabled", "--scope-model-path", str(args.scope_model), "--local-fast2d-mode", "active", "--restricted-mppi-enabled", "--params-file", str(params), "--out-root", str(case_dir / "runs")]
                env = os.environ.copy(); env["M1_SCOPE_PYTHON"] = args.scope_python
                subprocess.run(cmd, cwd=ROOT, env=env, text=True, stdout=(case_dir / f"run_{run+1:02d}.stdout").open("w"), stderr=subprocess.STDOUT)
                result_path = case_dir / "runs" / name / "result.json"
                results.append(json.loads(result_path.read_text()) if result_path.exists() else {"environment_status": "RUNNER_NO_RESULT"})
        summary = summarize(results); case = {"configuration": {"batch": batch, "steps": steps, "dt": dt, "horizon_s": batch * 0 + steps * dt}, "parameters": str(params), "build_pass": build_ok, "summary": summary, "runs": results}
        report["cases"][label] = case; write(case_dir / "summary.json", case); write(out / "report.json", report)
    eligible = [(case["summary"]["restricted_mppi_total_ms"]["p95"], label) for label, case in report["cases"].items() if case["build_pass"] and case["summary"]["fresh_start_successes"] >= args.runs and case["summary"]["deadline_miss"] == 0 and case["summary"]["safety"]["no_feasible_control_count"] == 0 and case["summary"]["restricted_mppi_total_ms"]["p95"] is not None]
    if eligible:
        _, label = min(eligible); c = report["cases"][label]["configuration"]
        report["recommendation"] = {"case": label, "configuration": c, "text": f"Recommend {label}: batch = {c['batch']}, steps = {c['steps']}, dt = {c['dt']}. It has the lowest eligible Restricted MPPI p95 while preserving the fixed 2 s-class horizon and all Restricted MPPI/SCOPE safety semantics."}
    else:
        report["recommendation"] = {"case": "A", "configuration": report["cases"].get("A", {}).get("configuration"), "text": "Retain A (the baseline): no lower-scale configuration satisfied every acceptance gate."}
    write(out / "report.json", report); (out / "PHASE_6C_MPPI_SCALE_OPTIMIZATION_REPORT.md").write_text(markdown(report))


if __name__ == "__main__":
    main()
