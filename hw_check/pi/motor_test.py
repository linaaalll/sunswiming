"""
Spins ONE thruster at a time and measures how much extra current it pulls.
This is the one that catches loose crimps.

    all 8 about the same extra current -> fine
    ~0 extra                           -> that thruster isn't running (power / signal / phase wire)
    current jumping around             -> bad phase wire or loose crimp
    way off from the others            -> fouled prop, bad wire, or wrong motor mapping

Wiggle mode: spins one thruster over and over while you wiggle its cable and
connectors. If the current drops out when you touch a spot, that's your crimp.

!! PROPS SPIN. hands clear, rov secured, ideally in water. !!
Not in run_all.sh on purpose, you run this yourself.

    python3 hw_check/pi/motor_test.py
    python3 hw_check/pi/motor_test.py --wiggle 3     # wiggle mode on motor 3

The safety part (confirm prompt + always disarm on exit) is written already,
the measuring is yours.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config
from common import PASS, WARN, FAIL, header, report, run, finish


def connect():
    """
    TODO (you): same as check_link.py's heartbeat, give back the conn.
    """
    raise NotImplementedError("connect() not written yet")


def read_current(conn):
    """
    TODO (you): battery current in amps right now.
      SYS_STATUS .current_battery is in cA (/100), -1 means no current sensor.
      One reading is noisy, average ~5 of them.
    """
    raise NotImplementedError("read_current() not written yet")


def spin_one(conn, motor):
    """
    Spin one motor for config.MOTOR_TEST_SECONDS, give back the list of
    current readings taken while it was spinning.

    TODO (you):
      1. conn.mav.command_long_send(
             conn.target_system, conn.target_component,
             mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
             motor,                                   # param1: motor number, starts at 1
             mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT,
             config.MOTOR_TEST_THROTTLE,              # param3: throttle %
             config.MOTOR_TEST_SECONDS,               # param4: it stops by itself after this
             0, 0, 0)
      2. wait for the COMMAND_ACK, result != MAV_RESULT_ACCEPTED -> it refused, print why
         (ArduSub may want to be armed first, QGC's motor sliders work the same way)
      3. while it spins, keep calling read_current() and keep every value
    """
    raise NotImplementedError("spin_one() not written yet")


def judge(results):
    """
    results = {motor: [current readings]}

    TODO (you):
      1. extra current per motor = average of its readings - the baseline
      2. compare each motor with the middle (median) of all of them
      3. report() one line per motor: ~0 -> FAIL, readings spread out a lot -> WARN,
         much higher/lower than median -> WARN, else PASS
    """
    report(WARN, "motor results", "judge() not written yet")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wiggle", type=int, metavar="MOTOR", help="loop one motor for the wiggle test")
    a = ap.parse_args()

    header("pi: motor test")
    print("!! this SPINS thrusters. hands clear, rov secured. ctrl-c stops everything. !!")
    if input("type SPIN to continue: ").strip() != "SPIN":
        print("ok, not spinning anything")
        return

    conn = None
    try:
        conn = connect()
        # TODO (you): baseline = read_current(conn) before anything spins

        if a.wiggle:
            # TODO (you): loop spin_one(conn, a.wiggle) forever, print each reading
            # live so you can see it drop while you wiggle. ctrl-c gets you out.
            pass
        else:
            results = {}
            for motor in range(1, config.NUM_THRUSTERS + 1):
                input(f"motor {motor}: enter to spin")
                results[motor] = spin_one(conn, motor)
            judge(results)
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        # whatever happened above (crash, ctrl-c, done), leave the rov disarmed
        if conn is not None:
            conn.arducopter_disarm()
    finish()


if __name__ == "__main__":
    main()
