# This is a simplified version of nav_msgs/Odometry:
#  * does not indicate covariance
#  * does not indicate values which are always 0 for a diff drive platform
#  * uses float32 instead of float64
#  * does not specify the child_frame_id (always "base_link")
#
#  TODO(damonkohler): We could further simplify this by changing Header.

# This represents an estimate of a position and velocity in free space.
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame "base_link".
Header header
float32 position_x
float32 position_y
float32 orientation_y
float32 orientation_w
float32 linear_x
float32 linear_y
float32 angular_z
