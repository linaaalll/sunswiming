"""
A simple dashboard that does a simple checking job

The sanity check time:
- Link      /mavros/state
- Sensors:  /mavros/imu/data
            /mavros/vfr_hud
            /mavros/imu/static_pressure
            /mavros/battery
- Outputs:  /mavros/rc/out
            /mavros/rc/override

Colour:
    green = good and working
    yellow = a bit slow but works
    red = nope getting nothing here

How to run:
    ros2 run output/monitor basic_monitor
"""

 
import math
import sys
import time
from collections import deque
 
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
 
from mavros_msgs.msg import State, VfrHud, RCOut, OverrideRCIn
from sensor_msgs.msg import Imu, BatteryState, FluidPressure
 
# ---- terminal colours ----
GREEN = "\033[32m"
YELLOW = "\033[33m"
RED = "\033[31m"
BOLD = "\033[1m"
DIM = "\033[2m"
RESET = "\033[0m"
CLEAR = "\033[2J\033[H"

class basicMonitor(Node):
    def __init__(self):
        super().__init__("basic_monitor")


def main(args=None):
    rclpy.init(args=args)
    node = basicMonitor()

    # Yo let's get the baby spinning