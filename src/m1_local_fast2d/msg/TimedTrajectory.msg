# The header is the exact planning-snapshot stamp/frame, matching
# /local_fast2d/accepted_path and /local_fast2d/accepted_costmap.
std_msgs/Header header
uint64 planning_result_id
uint64 input_generation
TimedTrajectoryPoint[] points
