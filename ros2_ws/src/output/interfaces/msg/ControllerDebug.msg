# Everything inside the heading PID on one tick. Published on /controller/debug.
# This is the topic to plot when tuning.

std_msgs/Header header

bool enabled             # false = brain is not asking for heading hold

float32 target           # heading we want (deg, 0 to 360)
float32 measurement      # heading we have (deg, 0 to 360)
float32 error            # shortest way from measurement to target (deg, -180 to 180)

float32 p_term           # the three parts of the output, before limiting
float32 i_term
float32 d_term

float32 output           # yaw effort sent on, -1 to +1 (+ = turn right)
