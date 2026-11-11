"""
Every ip / port / number the checks need, in ONE place.
If the network changes, change it here, not in 10 files.

The values below are BlueOS defaults, CONFIRM them on the real rov!
"""

# ---- network ----
PI_IP = "192.168.2.2"
JETSON_IP = "192.168.2.1"      # the ip the pi sends mavlink TO
ETH_IFACE = "eth0"             # run `ip link` on the jetson to get the real name

# Only one program can listen on a udp port. If MAVROS is running it owns 14550,
# so either stop MAVROS before the pymavlink checks, or add a second mavlink
# endpoint in BlueOS (e.g. 14551) just for these checks and put it here.
MAVLINK_PORT = 14550
MAVLINK_URL = f"udpin:0.0.0.0:{MAVLINK_PORT}"

DVL_IP = None                  # set once the dvl is on the switch, e.g. "192.168.2.95"

# ---- thrusters ----
NUM_THRUSTERS = 8
MOTOR_TEST_THROTTLE = 10       # percent, keep it LOW
MOTOR_TEST_SECONDS = 1.5       # short! thrusters don't like running dry
