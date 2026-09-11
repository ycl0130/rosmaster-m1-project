"""Independent command watchdog for the physical Rosmaster M1.

This node is deliberately separate from the planner process.  It is the only
project publisher for the hardware ``/cmd_vel`` topic and continuously emits a
zero command whenever the planner's raw command stream is absent or stale.
"""

import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def copy_twist(command):
    """Copy all Twist fields without changing any requested velocity value."""
    copied = Twist()
    copied.linear.x = command.linear.x
    copied.linear.y = command.linear.y
    copied.linear.z = command.linear.z
    copied.angular.x = command.angular.x
    copied.angular.y = command.angular.y
    copied.angular.z = command.angular.z
    return copied


class CommandWatchdogState:
    """Deterministic raw-command freshness gate, independent of ROS runtime."""

    def __init__(self, timeout_seconds):
        if timeout_seconds <= 0.0:
            raise ValueError("watchdog_timeout must be greater than zero")
        self.timeout_ns = int(timeout_seconds * 1_000_000_000)
        self.last_command = None
        self.last_received_ns = None

    @property
    def has_received_command(self):
        return self.last_command is not None

    def receive(self, command, received_ns):
        self.last_command = copy_twist(command)
        self.last_received_ns = int(received_ns)

    def is_stale(self, now_ns):
        return (self.last_received_ns is None or
                int(now_ns) - self.last_received_ns > self.timeout_ns)

    def command_for_time(self, now_ns):
        """Return raw command while fresh; otherwise return a zero Twist."""
        if self.is_stale(now_ns):
            return Twist()
        return copy_twist(self.last_command)


class M1CmdWatchdog(Node):
    """Forward fresh raw commands and continuously override stale ones to zero."""

    def __init__(self):
        super().__init__("m1_cmd_watchdog")
        self.declare_parameter("input_topic", "/m1/cmd_vel_raw")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("watchdog_timeout", 0.40)
        self.declare_parameter("publish_rate", 20.0)

        timeout = float(self.get_parameter("watchdog_timeout").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        if publish_rate <= 0.0:
            raise ValueError("publish_rate must be greater than zero")

        self.state = CommandWatchdogState(timeout)
        self.state_lock = threading.Lock()
        # Start in stale mode: before the first raw command the base receives
        # a zero Twist at publish_rate, rather than waiting for a timeout.
        self.output_is_stale = True
        self.output_publisher = self.create_publisher(
            Twist, self.get_parameter("output_topic").value, 10)
        self.create_subscription(
            Twist, self.get_parameter("input_topic").value, self.raw_command_callback, 10)
        self.create_timer(1.0 / publish_rate, self.publish_timer_callback)

        self.get_logger().info(
            "Watchdog active. input=%s output=%s timeout=%.2f s rate=%.1f Hz" % (
                self.get_parameter("input_topic").value,
                self.get_parameter("output_topic").value, timeout, publish_rate))

    def raw_command_callback(self, command):
        received_ns = time.monotonic_ns()
        with self.state_lock:
            had_command = self.state.has_received_command
            was_stale = self.state.is_stale(received_ns)
            self.state.receive(command, received_ns)
            self.output_is_stale = False

        if not had_command:
            self.get_logger().info("Raw command received.")
        elif was_stale:
            self.get_logger().info("Raw command restored.")

    def publish_timer_callback(self):
        now_ns = time.monotonic_ns()
        with self.state_lock:
            stale = self.state.is_stale(now_ns)
            command = self.state.command_for_time(now_ns)
            became_stale = stale and not self.output_is_stale
            self.output_is_stale = stale

        if became_stale:
            age_s = (now_ns - self.state.last_received_ns) / 1e9 if self.state.has_received_command else float("inf")
            self.get_logger().warn(
                "Command timeout; forcing zero velocity: raw_input=/m1/cmd_vel_raw "
                "age=%.3fs timeout=%.3fs received=%s.",
                age_s, self.state.timeout_ns / 1e9, self.state.has_received_command)
        # This is intentionally published every timer cycle in both modes.
        # In stale mode it continually overwrites any latched driver command.
        self.output_publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = M1CmdWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
