# Message that contains a trajectory for mobile robot navigation

# Similar to nav_msgs/Odometry, the pose in a TrajectoryPoint should
# be specified in the coordinate frame given by header.frame_id while
# the twist in a TrajectoryPoint should be specified in the coordinate
# frame given by the child_frame_id

# default header, including the frame id for the poses
std_msgs/Header header

# frame id for Twists (velocity and acceleration)
string child_frame_id

# the trajectory points
fub_trajectory_msgs/TrajectoryPoint[] trajectory
