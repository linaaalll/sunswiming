"""
ArduSub outputs: are all 8 thrusters mapped, and sitting at neutral?
Nothing spins here, it only reads. (motor_test.py is the one that spins.)

MAVROS off (or a second port in config.py), rov DISARMED:
    python3 hw_check/pi/check_outputs.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config
from common import PASS, WARN, FAIL, header, report, run, finish

MOTOR_FUNCTION_FIRST = 33  # SERVOx_FUNCTION 33 = Motor1, 34 = Motor2 ... 40 = Motor8


def main():
    header("pi: thruster outputs")

    # TODO (you):
    # 1. which output pin drives which motor?
    #    for each n in 1..16, read param f"SERVO{n}_FUNCTION":
    #        conn.param_fetch_one(f"SERVO{n}_FUNCTION")
    #        msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=3)
    #        int(msg.param_value)
    #    build a dict {motor_number: output_pin}
    #    any motor 1..config.NUM_THRUSTERS missing -> FAIL "motor X not mapped to any output"
    #    print the table, it's handy to compare against the wiring
    #
    # 2. frame type, param "FRAME_CONFIG" should match your 8 thruster layout
    #    (look up what number yours is in the ArduSub docs, then hardcode the check)
    #
    # 3. what is it actually sending right now?
    #    msg = conn.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=3)
    #    getattr(msg, f"servo{pin}_raw") gives the PWM on that pin
    #    disarmed every motor should sit at ~1500 (neutral, confirm on yours)
    #    not ~1500 -> WARN
    report(WARN, "thruster outputs", "not written yet")
    finish()


if __name__ == "__main__":
    main()
