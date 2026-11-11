"""
Jetson itself: is the box healthy enough to run everything?
Good first one to write, it's all plain python, no robot needed.

    python3 hw_check/jetson/check_system.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common import PASS, WARN, FAIL, header, report, run, finish


def check_disk():
    """
    TODO (you):
      1. import shutil, total, used, free = shutil.disk_usage("/")
      2. free is in bytes, turn it into GB (divide by 1024 three times)
      3. < 2 GB -> FAIL (rosbags will die), < 10 GB -> WARN, else PASS
    """
    report(WARN, "disk space", "not written yet")


def check_temperature():
    """
    TODO (you):
      1. every file matching /sys/class/thermal/thermal_zone*/temp holds a
         temperature in milli-degrees, e.g. "45500" = 45.5 C
         (Path("/sys/class/thermal").glob("thermal_zone*/temp") finds them)
      2. read each with .read_text(), int() it, / 1000
      3. take the hottest. > 85 C -> FAIL, > 70 C -> WARN, else PASS
    """
    report(WARN, "temperature", "not written yet")


def check_zed_driver():
    """
    The ZED X Mini is GMSL, not usb, so it needs its driver/daemon running.

    TODO (you):
      1. find the real service name first, by hand:
             systemctl list-units | grep -i zed
      2. ok, out = run(["systemctl", "is-active", "<that name>"])
      3. out == "active" -> PASS, else FAIL "zed driver not running"
    """
    report(WARN, "zed driver", "not written yet")


def main():
    header("jetson: system")
    check_disk()
    check_temperature()
    check_zed_driver()
    finish()


if __name__ == "__main__":
    main()
