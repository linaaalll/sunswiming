"""
Layers 1-3 of the jetson <-> pi link (see hw_check/README.md)

    1. cable    is the ethernet interface up?
    2. ip       does the jetson have the right ip, and can it ping the pi?
    3. mavlink  are ardusub heartbeats actually arriving on the port?

Each layer only matters if the one below works, so it stops at the first FAIL.

Layers 1 and 2 are written as worked examples, copy the pattern for layer 3 :)

How to run (MAVROS off, it hogs the port):
    python3 hw_check/link/check_link.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # so we can import common + config

import config
from common import PASS, WARN, FAIL, header, report, run, finish


def check_cable():
    ok, out = run(["ip", "link", "show", config.ETH_IFACE])
    if not ok:
        report(FAIL, "cable", f"no interface called {config.ETH_IFACE}, check `ip link` and fix config.py")
        return False
    if "state UP" in out:
        report(PASS, "cable", f"{config.ETH_IFACE} is up")
        return True
    report(FAIL, "cable", f"{config.ETH_IFACE} is down, check the cable / switch power")
    return False


def check_ip():
    # the pi sends mavlink to ONE fixed ip, if ours changed nothing arrives
    ok, out = run(["ip", "-4", "addr", "show", config.ETH_IFACE])
    if f"inet {config.JETSON_IP}/" not in out:
        report(FAIL, "jetson ip", f"expected {config.JETSON_IP}, the pi is sending mavlink somewhere else")
        return False
    report(PASS, "jetson ip", config.JETSON_IP)

    ok, out = run(["ping", "-c", "3", "-W", "1", config.PI_IP], timeout=6)
    if not ok:
        report(FAIL, "ping pi", f"{config.PI_IP} not answering")
        return False

    # last line looks like: rtt min/avg/max/mdev = 0.412/0.501/0.623/0.081 ms
    try:
        avg_ms = float(out.splitlines()[-1].split("=")[1].split("/")[1])
    except (IndexError, ValueError):
        report(WARN, "ping pi", "answered but couldn't read the time")
        return True

    if avg_ms > 10:
        report(WARN, "ping pi", f"{avg_ms:.1f} ms, that's slow for a cable")
    else:
        report(PASS, "ping pi", f"{avg_ms:.1f} ms")
    return True


def check_heartbeat():
    """
    Layer 3. Listen on the mavlink port WITHOUT ros and wait for a heartbeat.
    If this passes but MAVROS still says disconnected, the network is fine and
    the problem is MAVROS's fcu_url.

    TODO (you):
      1. pip install pymavlink   (if it's not there already)
      2. from pymavlink import mavutil
         conn = mavutil.mavlink_connection(config.MAVLINK_URL)
      3. msg = conn.wait_heartbeat(timeout=5)   -> gives None if nothing came
      4. None -> FAIL "no heartbeat on port ..., is MAVROS / QGC already on it?"
         else -> PASS, show conn.target_system so you know who answered
      5. bonus: count heartbeats for 3 s with conn.recv_match(type="HEARTBEAT",
         blocking=True, timeout=1), should be ~1 per second. way less -> WARN
    """
    report(WARN, "mavlink heartbeat", "not written yet")
    return True


def main():
    header("link: jetson <-> pi")
    check_cable() and check_ip() and check_heartbeat()
    finish()


if __name__ == "__main__":
    main()
