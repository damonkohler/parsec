float32 min_angle  # Minimum angle, the value must be between -pi/2 and +pi/2
float32 max_angle  # Maximum angle, the value must be between -pi/2 and +pi/2

# TODO(moesenle): Use type duration here. This doesn't work at the
# moment because rosserial's serialization/deserialization code for
# durations seems to be broken.
#
# 0 on both duration parameters indicates that the tilt should be disabled
float32 increasing_duration # indicates the duration for turning from
                            # min_angle to max_angle
float32 decreasing_duration # indicates the duration for turning from
                            # max_angle to min_angle
