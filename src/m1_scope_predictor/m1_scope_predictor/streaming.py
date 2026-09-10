"""Pure streaming-window assembly shared by ROS callbacks and replay tests."""

from dataclasses import dataclass

import numpy as np

from m1_scope_bridge.scope_ogm import (
    HISTORY_PERIOD_NS,
    SEQ_LEN,
    build_scope_input_window,
    find_nearest_indices,
    points_to_grid,
    scan_to_points,
)
from m1_scope_bridge.se2 import matrix_to_pose, pose_to_matrix
from m1_scope_bridge.synchronization import interpolate_odometry


@dataclass(frozen=True)
class ScanSample:
    stamp_ns: int
    ranges: np.ndarray
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    frame_id: str


@dataclass(frozen=True)
class OdomSample:
    stamp_ns: int
    pose: np.ndarray
    twist: np.ndarray


@dataclass(frozen=True)
class InferenceJob:
    input_generation: int
    anchor_stamp_ns: int
    target_stamp_ns: int
    history_stamp_ns: np.ndarray
    input_ogm: np.ndarray
    current_ogm: np.ndarray
    current_laser_pose: np.ndarray
    future_base_pose: np.ndarray
    future_laser_pose: np.ndarray


def _points(sample):
    return scan_to_points(
        sample.ranges, sample.angle_min, sample.angle_increment,
        sample.range_min, sample.range_max)


def build_job(scans, odometry, base_to_laser, anchor_target_ns,
              horizon_seconds=0.5, tolerance_ns=50_000_000, input_generation=0):
    """Select one deterministic history and build a latest inference job."""
    if len(scans) < SEQ_LEN:
        raise ValueError("history does not yet contain 10 scans")
    scan_stamps = np.asarray([item.stamp_ns for item in scans], dtype=np.int64)
    targets = int(anchor_target_ns) - np.arange(
        SEQ_LEN - 1, -1, -1, dtype=np.int64) * HISTORY_PERIOD_NS
    indices, _ = find_nearest_indices(scan_stamps, targets, tolerance_ns)
    if np.any(indices < 0):
        raise ValueError("history scan is outside tolerance")
    if np.any(np.diff(indices) <= 0):
        raise ValueError("history requires ten unique scans")
    selected = [scans[int(index)] for index in indices]
    frame_ids = {item.frame_id for item in selected}
    if len(frame_ids) != 1:
        raise ValueError("history scan frame changed")

    odom_stamps = np.asarray([item.stamp_ns for item in odometry], dtype=np.int64)
    odom_poses = np.asarray([item.pose for item in odometry], dtype=np.float64)
    odom_twists = np.asarray([item.twist for item in odometry], dtype=np.float64)
    history_stamps = scan_stamps[indices]
    try:
        past_poses, _ = interpolate_odometry(
            odom_stamps, odom_poses, odom_twists, history_stamps)
        current_pose, current_twist = interpolate_odometry(
            odom_stamps, odom_poses, odom_twists,
            np.asarray([history_stamps[-1]], dtype=np.int64))
    except ValueError as error:
        raise ValueError("odometry coverage does not bracket scan history") from error

    point_sequences = [_points(item) for item in selected]
    input_ogm, future_pose, future_laser = build_scope_input_window(
        point_sequences, past_poses, current_pose[0], current_twist[0],
        base_to_laser, horizon_seconds)
    current_laser = pose_to_matrix(current_pose[0]).dot(base_to_laser)
    current_grid = points_to_grid(point_sequences[-1])
    anchor_stamp = int(history_stamps[-1])
    return InferenceJob(
        input_generation=int(input_generation),
        anchor_stamp_ns=anchor_stamp,
        target_stamp_ns=anchor_stamp + int(round(float(horizon_seconds) * 1e9)),
        history_stamp_ns=history_stamps.copy(),
        input_ogm=input_ogm.astype(np.float32),
        current_ogm=current_grid,
        current_laser_pose=matrix_to_pose(current_laser),
        future_base_pose=future_pose,
        future_laser_pose=matrix_to_pose(future_laser),
    )
