"""
ZED X Mini: detected, opens, and actually gives frames.

Run with the zed wrapper OFF, only one program can open the camera.
    python3 hw_check/jetson/check_zed.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common import PASS, WARN, FAIL, header, report, run, finish


def main():
    header("jetson: zed camera")

    # TODO (you), all from `import pyzed.sl as sl` (comes with the ZED SDK):
    #   1. devices = sl.Camera.get_device_list()
    #      empty -> FAIL "no zed found", check the GMSL cable + check_system.py's driver check
    #   2. cam = sl.Camera()
    #      err = cam.open(sl.InitParameters())
    #      err != sl.ERROR_CODE.SUCCESS -> FAIL and print err (it says why)
    #   3. grab 30 frames: cam.grab() == sl.ERROR_CODE.SUCCESS for each one
    #      time it with time.monotonic() to get fps
    #      some grabs failing -> WARN (loose cable shows up like this!)
    #   4. PASS with cam.get_camera_information().serial_number and the fps
    #   5. ALWAYS cam.close() at the end (try/finally), or the wrapper can't open it later
    report(WARN, "zed camera", "not written yet")
    finish()


if __name__ == "__main__":
    main()
