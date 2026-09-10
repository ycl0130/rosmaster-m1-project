# Probability and uncertainty use the existing SCOPE OccupancyGrid encoding.
# probability: 0..100 maps to P in [0,1].
# uncertainty: 0..100 maps to sigma / 0.5, clipped to [0,1].
builtin_interfaces/Duration time_from_start
nav_msgs/OccupancyGrid probability
nav_msgs/OccupancyGrid uncertainty
