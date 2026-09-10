# State sampled directly from a Kinodynamic A* result.  Acceleration is the
# incoming primitive control used to propagate to this sample; the start point
# has no incoming primitive and therefore carries zero acceleration.
uint32 point_index
float64 x
float64 y
float64 yaw
float64 time_from_start
float64 vx
float64 vy
float64 ax
float64 ay
