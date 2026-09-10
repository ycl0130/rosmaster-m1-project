"""Finite test-only rosbag recorder keyed by the causality harness state."""
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Phase3Recorder(Node):
    def __init__(self):
        super().__init__("m1_phase3_recorder")
        self.declare_parameter("bag_uri", "phase3b_causality")
        self.declare_parameter("scope_enabled", False)
        self.process = None
        self.ready_publisher = self.create_publisher(String, "/local_fast2d_test/recorder_ready", 1)
        self.ready_timer = None
        self.create_subscription(String, "/local_fast2d_test/state", self.state, 10)
    def state(self, message):
        if self.process is None and message.data.startswith("TEST_READY"):
            topics = ["/clock", "/scan", "/odom", "/local_costmap/costmap_raw", "/local_fast2d_test/reference_path", "/local_fast2d_test/path", "/local_fast2d_test/local_goal", "/local_fast2d_test/diagnostics", "/local_fast2d_test/state", "/m1/dynamic_obstacles"]
            if self.get_parameter("scope_enabled").value:
                topics += ["/scope/prediction", "/scope/uncertainty", "/scope/diagnostics"]
            self.process = subprocess.Popen(["ros2", "bag", "record", "-o", self.get_parameter("bag_uri").value, *topics])
            # rosbag discovery is asynchronous. The harness holds TEST_READY
            # until this bounded setup period has elapsed, so T0 is recorded.
            self.ready_timer = self.create_timer(1.0, self.publish_ready)
        elif message.data.startswith("TEST_COMPLETE") and self.process is not None:
            self.process.terminate(); self.process.wait(timeout=10); rclpy.shutdown()
    def publish_ready(self):
        self.ready_publisher.publish(String(data="ready"))
        self.ready_timer.cancel()
def main(args=None):
    rclpy.init(args=args); node = Phase3Recorder()
    try: rclpy.spin(node)
    finally:
        if node.process is not None and node.process.poll() is None: node.process.terminate()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
