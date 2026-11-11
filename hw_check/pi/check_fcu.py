"""
ArduSub on the pi: are its sensors healthy?
(gyro / accel / mag / pressure / battery)

MAVROS off (or a second port in config.py):
    python3 hw_check/pi/check_fcu.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config
from common import PASS, WARN, FAIL, header, report, run, finish


def main():
    header("pi: ardusub sensors")

    # TODO (you), start the same way as check_heartbeat() in check_link.py:
    #   conn = mavutil.mavlink_connection(config.MAVLINK_URL); conn.wait_heartbeat(timeout=5)
    #
    # 1. sensor health
    #    msg = conn.recv_match(type="SYS_STATUS", blocking=True, timeout=3)
    #    msg.onboard_control_sensors_present / _enabled / _health are BITMASKS,
    #    one bit per sensor. e.g. for the gyro:
    #        bit = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO
    #        present = msg.onboard_control_sensors_present & bit
    #        healthy = msg.onboard_control_sensors_health & bit
    #    do it for 3D_GYRO, 3D_ACCEL, 3D_MAG, ABSOLUTE_PRESSURE (put them in a dict
    #    {"gyro": bit, ...} and loop, way nicer than copy pasting 4 times)
    #    present but not healthy -> FAIL
    #
    # 2. depth sensor (bar30 is usually SCALED_PRESSURE2, the internal one is SCALED_PRESSURE)
    #    msg = conn.recv_match(type="SCALED_PRESSURE2", blocking=True, timeout=3)
    #    None -> FAIL "no external pressure sensor", check its i2c cable
    #    msg.press_abs is in hPa, ~1013 in air. way off in air -> WARN
    #
    # 3. battery, from the SYS_STATUS msg above:
    #    msg.voltage_battery is mV, msg.current_battery is cA (centi-amps, /100 for A)
    #    current == -1 -> WARN "no current sensing", motor_test.py needs it!
    #    voltage low for your pack -> WARN
    report(WARN, "ardusub sensors", "not written yet")
    finish()


if __name__ == "__main__":
    main()
