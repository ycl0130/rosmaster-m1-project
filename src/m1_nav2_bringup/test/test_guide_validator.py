import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from m1_local_fast2d.msg import TimedTrajectory, TimedTrajectoryPoint
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Path

from m1_nav2_bringup.guide_validator import Validator


def test_accepted_trajectory_pairs_with_path_costmap_and_id():
    rclpy.init()
    validator = Validator()
    try:
        path = Path()
        path.header.frame_id, path.header.stamp.sec = 'map', 42
        for index in range(2):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(index)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        costmap = Costmap()
        costmap.header = path.header
        costmap.metadata.resolution = 1.0
        costmap.metadata.size_x, costmap.metadata.size_y = 2, 1
        costmap.data = [0, 0]

        trajectory = TimedTrajectory()
        trajectory.header = path.header
        trajectory.planning_result_id = 7
        trajectory.input_generation = 11
        for index in range(2):
            point = TimedTrajectoryPoint()
            point.point_index = index
            point.x, point.y, point.yaw = float(index), 0.0, 0.0
            point.time_from_start = float(index)
            trajectory.points.append(point)

        diagnostic = DiagnosticArray()
        diagnostic.header = path.header
        status = DiagnosticStatus(name='local_fast2d_accepted_snapshot')
        status.values = [KeyValue(key='snapshot_stamp_ns', value=str(42_000_000_000)),
                         KeyValue(key='accepted_guide_id', value='7'),
                         KeyValue(key='accepted_trajectory_id', value='7')]
        diagnostic.status = [status]

        validator.path_cb(path)
        validator.map_cb(costmap)
        validator.trajectory_cb(trajectory)
        validator.diag_cb(diagnostic)
        assert validator.trajectory_stats['accepted_trajectories'] == 1
        assert validator.trajectory_stats['point_count'] == 2
        assert validator.trajectory_stats['id_coupled']
        assert validator.errors == []
    finally:
        validator.destroy_node()
        rclpy.shutdown()
