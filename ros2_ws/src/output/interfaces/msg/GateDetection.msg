# What a gate detector saw in one camera frame.
# The colour detector and the yolo detector both publish this, on /qually/gate,
# so the brain never has to know which one is running.

std_msgs/Header header

bool found               # true = two poles seen in this frame

float32 centre_offset    # where the middle of the gate is, left to right:
                         #   -1 = left edge of image, 0 = dead centre, +1 = right edge

float32 separation       # gap between the two poles as a fraction of image width
                         #   0.1 = far away, 0.8 = very close

float32 confidence       # 0 to 1, how sure the detector is

string source            # "colour" or "yolo"
