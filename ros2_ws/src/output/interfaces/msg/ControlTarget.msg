# What the brain wants the vehicle to do right now.
# Published by the brain on /controller/target, read by the heading controller.
# Efforts are -1 to +1 (a fraction of full stick), not m/s.

bool hold_heading        # true = run the heading PID towards heading_deg
                         # false = no yaw command at all

float32 heading_deg      # compass heading to hold, 0 to 360

float32 forward          # +1 = full forward, -1 = full reverse
float32 lateral          # +1 = full right,   -1 = full left
float32 vertical         # +1 = full up,      -1 = full down
                         #  0 in ALT_HOLD means "hold this depth"
