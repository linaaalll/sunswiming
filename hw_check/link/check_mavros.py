"""
Layer 4 of the link: is the MAVROS bridge itself connected to ardusub?
(layer 5, "are all the topics coming in at the right rate", is
output/monitor/basic_monitor.py's job, this just checks the bridge)

Run this one WITH MAVROS running:
    python3 hw_check/link/check_mavros.py

If check_link.py passes and this fails -> the network is fine, look at
MAVROS's fcu_url (port has to match config.MAVLINK_PORT).
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config
from common import PASS, WARN, FAIL, header, report, run, finish


def check_port_owner():
    """
    Who is listening on the mavlink port? Should be MAVROS and nothing else.

    TODO (you):
      1. ok, out = run(["ss", "-ulpn"])   lists every udp port + the program on it
      2. keep only the lines with f":{config.MAVLINK_PORT}" in them
      3. no lines        -> FAIL nobody is listening, MAVROS isn't really up
         "mavros" in it  -> PASS
         anything else   -> FAIL and print the line, something stole the port
         (heads up: you might need sudo to see other users' program names)
    """
    report(WARN, "mavlink port owner", "not written yet")


def check_state():
    """
    Read /mavros/state once and look at .connected

    TODO (you), this is your first rclpy subscriber outside a package:
      1. import rclpy, from mavros_msgs.msg import State
      2. rclpy.init(), node = rclpy.create_node("hw_check_mavros")
      3. node.create_subscription(State, "/mavros/state", callback, 10)
         where callback saves the msg somewhere (a list works: got.append(msg))
      4. spin for up to ~3 s: rclpy.spin_once(node, timeout_sec=0.1) in a loop
         until you got a msg or ran out of time
      5. no msg at all       -> FAIL MAVROS node isn't running / wrong ROS_DOMAIN_ID
         msg.connected False -> FAIL MAVROS up but not hearing ardusub (fcu_url!)
         msg.connected True  -> PASS, also show msg.mode and msg.armed
      6. node.destroy_node(), rclpy.shutdown()
    """
    report(WARN, "mavros state", "not written yet")


def main():
    header("link: mavros bridge")
    check_port_owner()
    check_state()
    finish()


if __name__ == "__main__":
    main()
