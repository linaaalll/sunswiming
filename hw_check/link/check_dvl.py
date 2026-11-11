"""
DVL: on the network, and actually seeing the bottom?
Skips itself until config.DVL_IP is set.

    python3 hw_check/link/check_dvl.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config
from common import PASS, WARN, FAIL, header, report, run, finish


def main():
    header("link: dvl")
    if config.DVL_IP is None:
        report(WARN, "dvl", "no DVL_IP in config.py, skipping")
        finish()

    # TODO (you):
    #   1. ping config.DVL_IP, same as check_ip() in check_link.py
    #   2. read one velocity report. Depends on the dvl! e.g. Water Linked A50:
    #        import socket, json
    #        s = socket.create_connection((config.DVL_IP, 16171), timeout=3)
    #        read lines until one has "type": "velocity", json.loads() it
    #   3. report["velocity_valid"] True  -> PASS (it has bottom lock)
    #                                False -> WARN (alive, but can't see the bottom,
    #                                         normal on the bench / out of water)
    report(WARN, "dvl", "not written yet")
    finish()


if __name__ == "__main__":
    main()
