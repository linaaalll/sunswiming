"""
Qualifier dashboard: one terminal screen that shows what the robot is thinking.

    ros2 run monitor qually_monitor

It only listens, it never commands anything, so it is always safe to run.

Colours on the left-hand dot:
    green  = data arriving
    yellow = data is getting old
    red    = nothing arriving
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float64, String
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State, RCOut

from interfaces.msg import ControllerDebug, ControlTarget, GateDetection

GREEN = '\033[32m'
YELLOW = '\033[33m'
RED = '\033[31m'
BOLD = '\033[1m'
DIM = '\033[2m'
RESET = '\033[0m'
HOME = '\033[H'            # cursor to top-left
CLEAR_DOWN = '\033[J'      # wipe from cursor to end of screen
CLEAR_LINE = '\033[K'


class Latest:
    """Remembers the newest message on a topic and when it arrived."""

    def __init__(self):
        self.msg = None
        self.stamp = 0.0

    def set(self, msg):
        self.msg = msg
        self.stamp = time.monotonic()

    def age(self):
        return time.monotonic() - self.stamp if self.msg is not None else None

    def dot(self, ok_sec=1.0, stale_sec=3.0):
        age = self.age()
        if age is None or age > stale_sec:
            return f'{RED}●{RESET}'
        if age > ok_sec:
            return f'{YELLOW}●{RESET}'
        return f'{GREEN}●{RESET}'


def bar(value, width=21):
    """Draw -1..+1 as a little gauge: [-----|--#--]."""
    value = max(-1.0, min(1.0, value))
    cells = ['-'] * width
    cells[width // 2] = '|'
    cells[int(round((value + 1.0) / 2.0 * (width - 1)))] = '#'
    return '[' + ''.join(cells) + ']'


class QuallyMonitor(Node):

    def __init__(self):
        super().__init__('qually_monitor')

        self.fsm_state = Latest()
        self.mavros = Latest()
        self.depth = Latest()
        self.battery = Latest()
        self.target = Latest()
        self.debug = Latest()
        self.gate = Latest()
        self.rc_out = Latest()

        self.state_since = time.monotonic()
        self.last_state_name = None

        self.create_subscription(String, '/qually/state', self.on_fsm_state, 10)
        self.create_subscription(State, '/mavros/state', self.mavros.set, 10)
        self.create_subscription(
            Float64, '/mavros/global_position/rel_alt', self.depth.set,
            qos_profile_sensor_data)
        self.create_subscription(
            BatteryState, '/mavros/battery', self.battery.set, qos_profile_sensor_data)
        self.create_subscription(ControlTarget, '/controller/target', self.target.set, 10)
        self.create_subscription(ControllerDebug, '/controller/debug', self.debug.set, 10)
        self.create_subscription(GateDetection, '/qually/gate', self.gate.set, 10)
        self.create_subscription(
            RCOut, '/mavros/rc/out', self.rc_out.set, qos_profile_sensor_data)

        print('\033[2J', end='')              # clear the screen once
        self.create_timer(0.2, self.draw)     # redraw 5 times a second

    def on_fsm_state(self, msg):
        if msg.data != self.last_state_name:
            self.last_state_name = msg.data
            self.state_since = time.monotonic()
        self.fsm_state.set(msg)

    def draw(self):
        lines = []
        add = lines.append

        add(f'{BOLD}QUALLY{RESET}   {DIM}{time.strftime("%H:%M:%S")}{RESET}')
        add('')

        # ---- brain ----
        if self.fsm_state.msg is None:
            add(f'{self.fsm_state.dot()} brain     {DIM}not running{RESET}')
        else:
            name = self.fsm_state.msg.data
            colour = RED if name == 'ABORT' else GREEN if name == 'DONE' else BOLD
            add(f'{self.fsm_state.dot()} brain     {colour}{name}{RESET}'
                f'   for {time.monotonic() - self.state_since:.1f}s')

        # ---- vehicle ----
        m = self.mavros.msg
        if m is None:
            add(f'{self.mavros.dot()} vehicle   {DIM}no /mavros/state{RESET}')
        else:
            armed = f'{RED}ARMED{RESET}' if m.armed else 'disarmed'
            link = 'connected' if m.connected else f'{RED}NOT CONNECTED{RESET}'
            add(f'{self.mavros.dot()} vehicle   {armed}   mode {m.mode or "?"}   {link}')

        d = self.depth.msg
        depth_text = f'{-d.data:5.2f} m' if d is not None else f'{DIM}  ?{RESET}'
        add(f'{self.depth.dot(2.0, 5.0)} depth     {depth_text}')

        b = self.battery.msg
        battery_text = f'{b.voltage:5.2f} V' if b is not None else f'{DIM}  ?{RESET}'
        add(f'{self.battery.dot(3.0, 6.0)} battery   {battery_text}')
        add('')

        # ---- heading controller ----
        c = self.debug.msg
        if c is None:
            add(f'{self.debug.dot()} heading   {DIM}controller not running{RESET}')
        elif not c.enabled:
            add(f'{self.debug.dot()} heading   {c.measurement:6.1f}°   {DIM}hold off{RESET}')
        else:
            add(f'{self.debug.dot()} heading   {c.measurement:6.1f}°   '
                f'target {c.target:6.1f}°   error {c.error:+6.1f}°')
            add(f'            yaw out {bar(c.output)} {c.output:+.2f}'
                f'   {DIM}P {c.p_term:+.2f}  I {c.i_term:+.2f}  D {c.d_term:+.2f}{RESET}')

        t = self.target.msg
        if t is not None:
            add(f'{self.target.dot()} command   forward {t.forward:+.2f}   '
                f'lateral {t.lateral:+.2f}   vertical {t.vertical:+.2f}')
        else:
            add(f'{self.target.dot()} command   {DIM}brain not commanding{RESET}')
        add('')

        # ---- gate ----
        g = self.gate.msg
        if g is None:
            add(f'{self.gate.dot()} gate      {DIM}detector not running{RESET}')
        elif not g.found:
            add(f'{self.gate.dot()} gate      not seen   {DIM}({g.source}){RESET}')
        else:
            add(f'{self.gate.dot()} gate      {GREEN}SEEN{RESET}   {bar(g.centre_offset)} '
                f'{g.centre_offset:+.2f}   width {g.separation:.2f}   '
                f'conf {g.confidence:.2f}   {DIM}({g.source}){RESET}')
        add('')

        # ---- thrusters ----
        # TODO(lina): show the 8 thruster outputs.
        #
        # self.rc_out.msg.channels is a list of PWM values, the first 8 are
        # the thrusters. 1500 = stopped, 1100 = full one way, 1900 = full the
        # other. bar() above takes -1..+1, so (pwm - 1500) / 400 fits it.
        # One line per thruster, or all 8 side by side, whatever reads best.
        add(f'{self.rc_out.dot(2.0, 5.0)} thrusters {DIM}(panel not written yet){RESET}')

        print(HOME + '\n'.join(line + CLEAR_LINE for line in lines) + CLEAR_DOWN,
              end='', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = QuallyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(RESET)
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
