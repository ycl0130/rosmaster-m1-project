"""Observer-only ROS 2 node for online M1 SCOPE prediction."""

from collections import deque
import math
import threading
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid, Odometry
from m1_scope_msgs.msg import ScopePredictionSequence, ScopePredictionSlice
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from m1_scope_bridge.scope_ogm import (
    compensate_points, points_to_grid, scan_to_points)
from m1_scope_bridge.se2 import pose_to_matrix
from m1_scope_bridge.synchronization import interpolate_odometry, quaternion_to_yaw

from .evaluator import PendingEvaluationQueue, endpoint_metrics
from .inference_worker import InferenceWorker
from .latest_mailbox import LatestMailbox
from .occupancy_grid import (
    endpoint_data, grid_origin_pose, probability_data, uncertainty_data)
from .runtime_backend import ScopeRuntimeBackend
from .streaming import OdomSample, ScanSample, build_job


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _set_stamp(stamp, stamp_ns):
    stamp.sec = int(stamp_ns // 1_000_000_000)
    stamp.nanosec = int(stamp_ns % 1_000_000_000)


def _pose_from_odom(message):
    pose = message.pose.pose
    return np.array([
        pose.position.x, pose.position.y,
        quaternion_to_yaw(
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w),
    ], dtype=np.float64)


def _twist_from_odom(message):
    twist = message.twist.twist
    return np.array(
        [twist.linear.x, twist.linear.y, twist.angular.z], dtype=np.float64)


class ScopePredictorNode(Node):
    def __init__(self):
        super().__init__("scope_predictor")
        defaults = (
            ("scan_topic", "/scan"), ("odom_topic", "/odom"),
            ("ground_truth_topic", "/ground_truth/odom"),
            ("base_frame", "base_footprint"), ("odom_frame", "odom"),
            ("model_path", "scope_model.pth"),
            ("device", "cuda"), ("seed", 1337),
            ("buffer_duration", 3.0), ("scheduler_rate", 10.0),
            ("scan_tolerance", 0.05), ("prediction_dt", 0.1),
            ("legacy_horizon_steps", 2), ("sequence_horizon_steps", 2),
            ("num_samples", 4), ("max_prediction_age", 1.0),
            ("evaluator_enabled", False), ("occupancy_threshold", 0.5),
        )
        for name, value in defaults:
            self.declare_parameter(name, value)
        self.buffer_ns = int(float(self._parameter("buffer_duration")) * 1e9)
        self.tolerance_ns = int(float(self._parameter("scan_tolerance")) * 1e9)
        self.prediction_dt = float(self._parameter("prediction_dt"))
        self.legacy_horizon_steps = int(self._parameter("legacy_horizon_steps"))
        self.sequence_horizon_steps = int(self._parameter("sequence_horizon_steps"))
        if self.prediction_dt != 0.1 or self.legacy_horizon_steps != 2 or not 2 <= self.sequence_horizon_steps <= 20:
            raise ValueError("prediction_dt must be 0.1, legacy_horizon_steps must be 2, and sequence_horizon_steps must be in [2,20]")
        self.horizon_seconds = self.legacy_horizon_steps * self.prediction_dt
        self.scans = deque()
        self.odometry = deque()
        self.ground_truth = deque()
        self.buffer_lock = threading.Lock()
        self.next_anchor_ns = None
        self.input_generation = 0
        self.base_to_laser = None
        self.scan_frame = None
        self.output = LatestMailbox()
        self.worker = None
        self.terminal_error = None
        self.last_input_state = "WARMUP"
        self.last_result = None
        self.last_publish_wall = None
        self.output_intervals = deque(maxlen=30)
        self.jobs_submitted = 0
        self.prediction_id = 0
        self.pending_evaluations = PendingEvaluationQueue(maximum=32)
        self.last_metrics = None

        self.current_publisher = self.create_publisher(
            OccupancyGrid, "/scope/current_ogm", 1)
        self.prediction_publisher = self.create_publisher(
            OccupancyGrid, "/scope/prediction", 1)
        self.uncertainty_publisher = self.create_publisher(
            OccupancyGrid, "/scope/uncertainty", 1)
        self.sequence_publisher = self.create_publisher(
            ScopePredictionSequence, "/scope/prediction_sequence", 1)
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "/scope/diagnostics", 1)
        self.create_subscription(
            LaserScan, str(self._parameter("scan_topic")),
            self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, str(self._parameter("odom_topic")), self._odom_callback, 20)
        if bool(self._parameter("evaluator_enabled")):
            self.create_subscription(
                Odometry, str(self._parameter("ground_truth_topic")),
                self._ground_truth_callback, 20)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        try:
            backend = ScopeRuntimeBackend(
                self._parameter("model_path"), self._parameter("device"),
                self._parameter("seed"))
            self.worker = InferenceWorker(
                backend, self.output,
                self.sequence_horizon_steps,
                self._parameter("num_samples"))
            self.worker.start()
        except Exception as error:
            self.terminal_error = error
            self.get_logger().error("SCOPE runtime unavailable: %s" % error)
        self.create_timer(1.0 / float(self._parameter("scheduler_rate")), self._schedule)
        self.create_timer(0.02, self._publish_result)
        self.create_timer(1.0, self._publish_diagnostics)

    def _parameter(self, name):
        return self.get_parameter(name).value

    def _trim(self, values, newest_stamp):
        lower = int(newest_stamp) - self.buffer_ns
        while values and values[0].stamp_ns < lower:
            values.popleft()

    def _reset_on_time_reversal(self, values, stamp_ns):
        if values and stamp_ns <= values[-1].stamp_ns:
            values.clear()
            self.next_anchor_ns = None
            self.input_generation += 1
            self.last_input_state = "TIME_RESET"

    def _scan_callback(self, message):
        stamp_ns = _stamp_ns(message.header.stamp)
        sample = ScanSample(
            stamp_ns, np.asarray(message.ranges, dtype=np.float32),
            message.angle_min, message.angle_increment,
            message.range_min, message.range_max, message.header.frame_id)
        with self.buffer_lock:
            self._reset_on_time_reversal(self.scans, stamp_ns)
            self.scans.append(sample)
            self._trim(self.scans, stamp_ns)
            if self.scan_frame is None:
                self.scan_frame = sample.frame_id

    def _odom_callback(self, message):
        stamp_ns = _stamp_ns(message.header.stamp)
        sample = OdomSample(stamp_ns, _pose_from_odom(message), _twist_from_odom(message))
        with self.buffer_lock:
            self._reset_on_time_reversal(self.odometry, stamp_ns)
            self.odometry.append(sample)
            self._trim(self.odometry, stamp_ns)

    def _ground_truth_callback(self, message):
        stamp_ns = _stamp_ns(message.header.stamp)
        sample = OdomSample(stamp_ns, _pose_from_odom(message), _twist_from_odom(message))
        with self.buffer_lock:
            self._reset_on_time_reversal(self.ground_truth, stamp_ns)
            self.ground_truth.append(sample)
            self._trim(self.ground_truth, stamp_ns)

    def _resolve_transform(self):
        if self.base_to_laser is not None or not self.scan_frame:
            return self.base_to_laser is not None
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self._parameter("base_frame")), self.scan_frame, Time())
        except TransformException:
            self.last_input_state = "TF_MISSING"
            return False
        value = transform.transform
        yaw = quaternion_to_yaw(
            value.rotation.x, value.rotation.y,
            value.rotation.z, value.rotation.w)
        self.base_to_laser = pose_to_matrix(
            [value.translation.x, value.translation.y, yaw])
        return True

    def _schedule(self):
        if self.worker is None or self.terminal_error is not None:
            return
        if not self._resolve_transform():
            return
        with self.buffer_lock:
            scans = list(self.scans)
            odometry = list(self.odometry)
        if len(scans) < 10 or len(odometry) < 2:
            self.last_input_state = "WARMUP"
            return
        if self.next_anchor_ns is None:
            self.next_anchor_ns = scans[0].stamp_ns + 900_000_000
        if scans[-1].stamp_ns + self.tolerance_ns < self.next_anchor_ns:
            self.last_input_state = "WAITING"
            return
        steps = max(0, (scans[-1].stamp_ns - self.next_anchor_ns) // 100_000_000)
        anchor_target = self.next_anchor_ns + steps * 100_000_000
        self.next_anchor_ns = anchor_target + 100_000_000
        try:
            job = build_job(
                scans, odometry, self.base_to_laser, anchor_target,
                self.horizon_seconds, self.tolerance_ns, self.input_generation)
        except ValueError as error:
            self.last_input_state = str(error)
            return
        self.worker.submit(job)
        self.jobs_submitted += 1
        self.last_input_state = "READY"

    def _grid_message(self, grid, laser_pose, stamp_ns, conversion):
        message = OccupancyGrid()
        message.header.frame_id = str(self._parameter("odom_frame"))
        _set_stamp(message.header.stamp, stamp_ns)
        message.info.resolution = 0.1
        message.info.width = 64
        message.info.height = 64
        x, y, yaw = grid_origin_pose(laser_pose)
        message.info.origin.position.x = x
        message.info.origin.position.y = y
        message.info.origin.orientation.z = math.sin(yaw / 2.0)
        message.info.origin.orientation.w = math.cos(yaw / 2.0)
        message.data = conversion(grid)
        return message

    def _sequence_message(self, result):
        message = ScopePredictionSequence()
        message.header.frame_id = str(self._parameter("odom_frame"))
        _set_stamp(message.header.stamp, result.job.anchor_stamp_ns)
        self.prediction_id += 1
        message.prediction_id = self.prediction_id
        for index, (mean, standard_deviation) in enumerate(
                zip(result.means, result.standard_deviations), start=1):
            slice_message = ScopePredictionSlice()
            offset_ns = int(round(index * self.prediction_dt * 1e9))
            slice_message.time_from_start.sec = offset_ns // 1_000_000_000
            slice_message.time_from_start.nanosec = offset_ns % 1_000_000_000
            stamp_ns = result.job.anchor_stamp_ns + offset_ns
            slice_message.probability = self._grid_message(
                mean, result.job.future_laser_pose, stamp_ns, probability_data)
            slice_message.uncertainty = self._grid_message(
                standard_deviation, result.job.future_laser_pose, stamp_ns, uncertainty_data)
            message.slices.append(slice_message)
        return message

    def _publish_result(self):
        result = self.output.take()
        if result is None:
            self._try_evaluate()
            return
        if result.error is not None:
            self.terminal_error = result.error
            self.get_logger().error("SCOPE inference stopped: %s" % result.error)
            return
        if result.job.input_generation != self.input_generation:
            self.last_input_state = "TIME_RESET"
            return
        age = (self.get_clock().now().nanoseconds - result.job.anchor_stamp_ns) / 1e9
        if age > float(self._parameter("max_prediction_age")):
            self.last_input_state = "STALE_RESULT"
            return
        self.current_publisher.publish(self._grid_message(
            result.job.current_ogm, result.job.current_laser_pose,
            result.job.anchor_stamp_ns, endpoint_data))
        sequence = self._sequence_message(result)
        self.sequence_publisher.publish(sequence)
        legacy_index = self.legacy_horizon_steps - 1
        legacy_slice = sequence.slices[legacy_index]
        self.prediction_publisher.publish(legacy_slice.probability)
        self.uncertainty_publisher.publish(legacy_slice.uncertainty)
        now = time.monotonic()
        if self.last_publish_wall is not None:
            self.output_intervals.append(now - self.last_publish_wall)
        self.last_publish_wall = now
        self.last_result = result
        if bool(self._parameter("evaluator_enabled")):
            self.pending_evaluations.add(result)
        self._try_evaluate()

    def _try_evaluate(self):
        if not len(self.pending_evaluations) or self.base_to_laser is None:
            return
        with self.buffer_lock:
            scans = list(self.scans)
            ground_truth = list(self.ground_truth)
        if not scans or len(ground_truth) < 2:
            return
        match = self.pending_evaluations.pop_match(
            [item.stamp_ns for item in scans], self.tolerance_ns)
        if match is None:
            return
        result, scan_index = match
        sample = scans[scan_index]
        gt_stamps = np.asarray([item.stamp_ns for item in ground_truth], dtype=np.int64)
        try:
            pose, _ = interpolate_odometry(
                gt_stamps,
                np.asarray([item.pose for item in ground_truth]),
                np.asarray([item.twist for item in ground_truth]),
                np.asarray([sample.stamp_ns], dtype=np.int64))
        except ValueError:
            self.pending_evaluations.restore_front(result)
            return
        actual_laser = pose_to_matrix(pose[0]).dot(self.base_to_laser)
        points = scan_to_points(
            sample.ranges, sample.angle_min, sample.angle_increment,
            sample.range_min, sample.range_max)
        target = points_to_grid(compensate_points(
            points, actual_laser, pose_to_matrix(result.job.future_laser_pose)))
        self.last_metrics = endpoint_metrics(
            result.means[self.legacy_horizon_steps - 1], target,
            self._parameter("occupancy_threshold"))

    def _publish_diagnostics(self):
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "SCOPE online predictor"
        status.hardware_id = str(self._parameter("device"))
        if self.terminal_error is not None:
            status.level = DiagnosticStatus.ERROR
            status.message = str(self.terminal_error)
        elif self.last_input_state != "READY":
            status.level = DiagnosticStatus.WARN
            status.message = self.last_input_state
        else:
            status.level = DiagnosticStatus.OK
            status.message = "online"
        result = self.last_result
        rate = (1.0 / (sum(self.output_intervals) / len(self.output_intervals))
                if self.output_intervals else 0.0)
        values = {
            "model_variant": "full_scope",
            "device": str(self._parameter("device")),
            "buffer_size": "%d scans / %d odom" % (
                len(self.scans), len(self.odometry)),
            "input_state": self.last_input_state,
            "inference_latency_ms": ("%.3f" % (result.latency_seconds * 1000.0)
                                     if result else "nan"),
            "preprocess_latency_ms": ("%.3f" % (result.preprocess_seconds * 1000.0)
                                        if result else "nan"),
            "model_inference_latency_ms": ("%.3f" % (result.inference_seconds * 1000.0)
                                              if result else "nan"),
            "postprocess_latency_ms": ("%.3f" % (result.postprocess_seconds * 1000.0)
                                         if result else "nan"),
            "prediction_age_ms": ("%.3f" % (
                (self.get_clock().now().nanoseconds
                 - result.job.anchor_stamp_ns) / 1e6)
                                  if result else "nan"),
            "output_rate_hz": "%.3f" % rate,
            "jobs_submitted": str(self.jobs_submitted),
            "jobs_dropped": str(self.worker.dropped if self.worker else 0),
            "cuda_memory_bytes": str(result.memory_bytes if result else 0),
            "sequence_horizon_steps": str(self.sequence_horizon_steps),
            "legacy_horizon_steps": str(self.legacy_horizon_steps),
        }
        if self.last_metrics:
            for key in ("mae", "occupied_iou", "f1"):
                values["evaluator_" + key] = str(self.last_metrics[key])
        status.values = [
            KeyValue(key=key, value=value) for key, value in values.items()]
        message.status = [status]
        self.diagnostic_publisher.publish(message)

    def destroy_node(self):
        if self.worker is not None:
            self.worker.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ScopePredictorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
