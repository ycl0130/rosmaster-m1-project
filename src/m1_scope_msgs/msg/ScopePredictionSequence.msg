# Header stamp is the observation/inference anchor time and frame is the
# world frame shared by every slice (currently odom).
std_msgs/Header header
uint64 prediction_id
ScopePredictionSlice[] slices
