import sys
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "m1_scope_bridge"))
sys.path.insert(0, str(ROOT / "m1_scope_predictor"))

from m1_scope_predictor.streaming import (  # noqa: E402
    OdomSample,
    ScanSample,
    build_job,
)


def scan(stamp_ns, distance=2.0):
    return ScanSample(
        stamp_ns=stamp_ns,
        ranges=np.array([distance], dtype=np.float32),
        angle_min=0.0,
        angle_increment=1.0,
        range_min=0.05,
        range_max=12.0,
        frame_id="laser_scan_link",
    )


def odom(stamp_ns):
    seconds = stamp_ns / 1e9
    return OdomSample(
        stamp_ns=stamp_ns,
        pose=np.array([seconds, 0.0, 0.0]),
        twist=np.array([1.0, 0.0, 0.0]),
    )


def test_build_job_selects_ten_unique_scans_and_interpolates_odom():
    scans = [scan(int(index * 1e9 / 12.0)) for index in range(14)]
    odometry = [odom(int(index * 1e9 / 30.0)) for index in range(40)]
    job = build_job(
        scans, odometry, np.eye(3), anchor_target_ns=900_000_000,
        horizon_seconds=0.5, tolerance_ns=50_000_000)
    assert job.input_ogm.shape == (10, 1, 64, 64)
    assert job.input_generation == 0
    assert len(np.unique(job.history_stamp_ns)) == 10
    assert abs(job.anchor_stamp_ns - 900_000_000) <= 50_000_000
    assert job.target_stamp_ns == job.anchor_stamp_ns + 500_000_000
    assert np.allclose(job.future_base_pose[0], job.anchor_stamp_ns / 1e9 + 0.5)


def test_build_job_rejects_history_outside_scan_tolerance():
    scans = [scan(index * 200_000_000) for index in range(10)]
    odometry = [odom(index * 50_000_000) for index in range(50)]
    with pytest.raises(ValueError, match="history"):
        build_job(
            scans, odometry, np.eye(3), anchor_target_ns=900_000_000,
            horizon_seconds=0.5, tolerance_ns=50_000_000)


def test_build_job_rejects_odom_extrapolation():
    scans = [scan(index * 100_000_000) for index in range(10)]
    odometry = [odom(index * 100_000_000) for index in range(9)]
    with pytest.raises(ValueError, match="coverage"):
        build_job(
            scans, odometry, np.eye(3), anchor_target_ns=900_000_000,
            horizon_seconds=0.5, tolerance_ns=50_000_000)
