import argparse
import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from gz.msgs10.double_pb2 import Double
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

#--- Tuning of the forces
MAX_FORCE = 40.0
MAX_TURN = 6.0 
BUOYANCY_FF = -2.94
DEPTH_KP, DEPTH_KI, DEPTH_KD = 60.0, 10.0, 30.0 
YAW_KP, YAW_KD = 6.0, 2.5
RATE = 50.0 # Hz, control loop rate

# ---------------- thruster geometry (from sunswim.urdf.xacro) ----------------
# Horizontal thrusters: push direction (x, y) and position (x, y) on the body.
S = math.sqrt(0.5)
H = {  # name: (dir_x, dir_y, pos_x, pos_y)
    "FL": (S, -S, 0.25, 0.20),
    "FR": (-S, -S, 0.25, -0.20),
    "RL": (S, S, -0.25, 0.20),
    "RR": (-S, S, -0.25, -0.20),
}
V = ["VFL", "VRL", "VFR", "VRR"]  # vertical thrusters, + pushes up

# Each column = what 1 N on that thruster does to the body: [Fx, Fy, Mz]
# (Mz = x*Fy - y*Fx, the turning effect about the centre)
A = np.array([[dx for dx, dy, px, py in H.values()],
              [dy for dx, dy, px, py in H.values()],
              [px * dy - py * dx for dx, dy, px, py in H.values()]])
A_INV = np.linalg.pinv(A)   # body force wanted -> 4 thruster forces

# rov is always turning the short way round.
def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

class RovDriver(Node):
    def __init__(self, world, model, max_thrust):
        super().__init__("rov_driver")
        self.model, self.max_thrust = model, max_thrust
        self.cmd = Twist()
        self.lock = threading.Lock()
        self.z = self.yaw = None
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)

        self.gz = GzNode()
        self.gz.subscribe(Pose_V, f"/world/{world}/dynamic_pose/info", self.on_pose)
        self.pubs = {n: self.gz.advertise(f"/model/{model}/joint/thruster_{n}_joint/cmd_thrust",
                        Double) for n in list(H) + V}

        self.z_sp = self.yaw_sp = None
        self.z_int = 0.0
        self.prev = None
        self.vz = self.r = 0.0
        self.create_timer(1.0 / RATE, self.step)
        self.get_logger().info("rov_driver ready: send /cmd_vel")

    def on_cmd(self, msg):
        with self.lock:
            self.cmd = msg

    def on_pose(self, msg):
        for p in msg.pose:
            if p.name == self.model:
                q = p.orientation
                with self.lock:
                    self.z = p.position.z
                    self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 
                                         1 - 2 * (q.y * q.y + q.z * q.z))

    def send(self, name, force):
        m = Double()
        m.data = float(np.clip(force, -self.max_thrust, self.max_thrust))
        self.pubs[name].publish(m)

    def step(self):
        with self.lock:
            cmd, z, yaw = self.cmd, self.z, self.yaw

        if z is None:
            return

        now = time.monotonic()

        # let's try to smooth the move out
        if self.prev is not None:
            dt = max(now - self.prev[0], 1e-3)
            self.vz = 0.7 * self.vz + 0.3 * (z - self.prev[1]) / dt
            self.r = 0.7 * self.r + 0.3 * wrap(yaw - self.prev[2]) / dt

        self.prev = (now, z, yaw)
        dt = 1.0 / RATE

        u = lambda v: float(np.clip(v, -1.0, 1.0))

        if abs(cmd.linear.z) > 0.05 or self.z_sp is None:
            fz = u(cmd.linear.z) * MAX_FORCE + BUOYANCY_FF
            self.z_sp, self.z_int = z, 0.0
        else:
            err = self.z_sp - z
            self.z_int = float(np.clip(self.z_int + err * dt, -2.0, 2.0))
            fz = DEPTH_KP * err + DEPTH_KI * self.z_int - DEPTH_KD * self.vz + BUOYANCY_FF

        # may turning be smooth too
        if abs(cmd.angular.z) > 0.05 or self.yaw_sp is None:
            mz = u(cmd.angular.z) * MAX_TURN
            self.yaw_sp = yaw
        else:
            mz = YAW_KP * wrap(self.yaw_sp - yaw) - YAW_KD * self.r

        # thruster mixing I guess
        fx, fy = u(cmd.linear.x) * MAX_FORCE, u(cmd.linear.y) * MAX_FORCE
        t = A_INV @ np.array([fx, fy, mz])
        biggest = np.max(np.abs(t))

        if biggest > self.max_thrust:
            # boomboclat scale it down man
            t *= self.max_thrust / biggest
        for name, f in zip(H, t):
            self.send(name, f)
        for name in V:
            self.send(name, fz / 4.0)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="pool_qual")
    ap.add_argument("--model", default="sunswim")
    ap.add_argument("--max-thrust", type=float, default=40.0, help="N per thruster")
    a = ap.parse_args()
    rclpy.init()

    node = RovDriver(a.world, a.model, a.max_thrust)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for n in node.pubs:
            node.send(n, 0.0)
        time.sleep(0.2)
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()