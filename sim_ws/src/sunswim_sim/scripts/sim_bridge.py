#!/usr/bin/env python3
"""
sim_bridge: makes the simulator look like the real robot.

On the real vehicle, mavros and the ZED camera give us these topics. In the
sim neither exists, so this node fakes them from Gazebo. Everything in
ros2_ws (detector, controller, brain, monitor, recorder) then runs unchanged.

    Gazebo                         this node                    your stack
    ------                         ---------                    ----------
    model pose      ->  /mavros/imu/data                    ->  heading_controller
                        /mavros/global_position/rel_alt     ->  brain, monitor
    /cam/image      ->  /zed/zed_node/rgb/color/rect/image/compressed -> detector
                        /mavros/state, /mavros/battery      ->  brain, monitor
                        /mavros/set_mode, /mavros/cmd/arming (fake services)

    /mavros/rc/override  ->  [ override_to_twist: YOURS ]  ->  /sim/cmd_vel -> rov_driver

Two modes (the launch file picks):
    --drive      autonomous: listen to /mavros/rc/override and drive the sim
    (no flag)    teleop: sensors only, the keyboard drives rov_driver directly

Run by the launch file, or by hand:
    python3 sim_bridge.py --drive
"""

import argparse
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import BatteryState, CompressedImage, Imu
from std_msgs.msg import Float64

from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

# The water surface in pool_qual.sdf is at z = 2.0 (see the buoyancy plugin).
SURFACE_Z = 2.0

# RC channel numbers, counted from 0. Same ones rc_control.cpp writes.
CH_THROTTLE = 2    # up / down
CH_YAW = 3
CH_FORWARD = 4
CH_LATERAL = 5

# Special values in an override message that mean "I am not commanding this
# channel": 0 = release, 65535 = no change.
PWM_IGNORE = (0, OverrideRCIn.CHAN_NOCHANGE)


class SimBridge(Node):

    def __init__(self, world, model, drive, image_topic):
        super().__init__('sim_bridge')
        self.model = model
        self.drive = drive

        self.lock = threading.Lock()
        self.pose = None            # newest (position, orientation) from Gazebo
        self.frame = None           # newest camera frame from Gazebo
        self.frame_is_new = False

        self.armed = False
        self.mode = 'MANUAL'
        self.last_override = None
        self.last_override_time = 0.0

        # ---- what we pretend to be ----
        self.state_pub = self.create_publisher(State, '/mavros/state', 10)
        self.imu_pub = self.create_publisher(Imu, '/mavros/imu/data', qos_profile_sensor_data)
        self.alt_pub = self.create_publisher(
            Float64, '/mavros/global_position/rel_alt', qos_profile_sensor_data)
        self.battery_pub = self.create_publisher(
            BatteryState, '/mavros/battery', qos_profile_sensor_data)
        self.image_pub = self.create_publisher(
            CompressedImage, '/zed/zed_node/rgb/color/rect/image/compressed', 2)

        self.create_service(SetMode, '/mavros/set_mode', self.on_set_mode)
        self.create_service(CommandBool, '/mavros/cmd/arming', self.on_arming)

        # ---- driving the sim (autonomous mode only) ----
        self.sim_cmd_pub = self.create_publisher(Twist, '/sim/cmd_vel', 10)
        if drive:
            self.create_subscription(
                OverrideRCIn, '/mavros/rc/override', self.on_override, 10)

        # ---- listening to Gazebo ----
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, f'/world/{world}/dynamic_pose/info', self.on_gz_pose)
        self.gz.subscribe(GzImage, image_topic, self.on_gz_image)

        self.create_timer(1.0 / 20.0, self.publish_pose)      # imu + depth, 20 Hz
        self.create_timer(1.0 / 10.0, self.publish_image)     # camera, 10 Hz
        self.create_timer(1.0 / 5.0, self.publish_status)     # state + battery, 5 Hz
        if drive:
            self.create_timer(1.0 / 20.0, self.drive_sim)

        self.get_logger().info(
            f'sim_bridge up ({"autonomous: driving from /mavros/rc/override" if drive else "teleop: sensors only"})')

    # ------------------------------------------------------------------
    # yours: RC override -> what the sim vehicle should do
    # ------------------------------------------------------------------

    def pwm_to_effort(self, pwm):
        """
        One RC channel value -> effort from -1.0 to +1.0.

            1500 -> 0.0      (stick centred)
            1900 -> +1.0
            1100 -> -1.0
            0 or 65535 (PWM_IGNORE) -> 0.0   (nobody is commanding this channel)
        """
        if pwm == 0 or pwm == 65535:
            return 0.0
        # Full scale is 1100..1900, like ArduSub. rc_control.cpp sends
        # 1500 + 500 * effort, so anything past 0.8 effort is clipped here,
        # the same as it would be on the real vehicle.
        effort = (pwm - 1500) / 400.0
        return max(-1.0, min(1.0, effort))

    def override_to_twist(self, channels):
        """
        A whole override message -> the Twist that rov_driver understands.

        channels: list of 18 PWM values (use CH_FORWARD, CH_LATERAL, CH_YAW,
                  CH_THROTTLE to index it, and self.pwm_to_effort on each).

        rov_driver.py reads the Twist as efforts from -1 to +1:
            linear.x   + = forward
            linear.y   + = LEFT        (normal ROS)
            linear.z   + = up          (near 0 = it holds depth, like ALT_HOLD)
            angular.z  + = turn LEFT   (normal ROS, counter-clockwise from above)

        ArduSub's sticks, which is what the PWM values mean:
            forward  > 1500 = forward
            lateral  > 1500 = RIGHT
            yaw      > 1500 = turn RIGHT
            throttle > 1500 = up

        So two of the four axes need their sign flipped. If you get one wrong
        the heading PID will run away from its target instead of towards it,
        which is easy to spot on the dashboard (error grows, output saturates).
        """
        twist = Twist()
        twist.linear.x = self.pwm_to_effort(channels[CH_FORWARD])
        twist.linear.y = -self.pwm_to_effort(channels[CH_LATERAL])   # right -> left
        twist.linear.z = self.pwm_to_effort(channels[CH_THROTTLE])
        twist.angular.z = -self.pwm_to_effort(channels[CH_YAW])      # right -> left
        return twist

    # ------------------------------------------------------------------
    # written: driving
    # ------------------------------------------------------------------

    def on_override(self, msg):
        self.last_override = msg
        self.last_override_time = time.monotonic()

    def drive_sim(self):
        """20 times a second: pass the newest command on, or stop."""
        fresh = (self.last_override is not None
                 and time.monotonic() - self.last_override_time < 1.0)
        if self.armed and fresh:
            twist = self.override_to_twist(list(self.last_override.channels))
        else:
            twist = Twist()         # disarmed or nobody talking -> no thrust,
                                    # same as the real vehicle
        self.sim_cmd_pub.publish(twist)

    # ------------------------------------------------------------------
    # written: pretending to be ArduSub
    # ------------------------------------------------------------------

    def on_set_mode(self, request, response):
        self.mode = request.custom_mode
        self.get_logger().info(f'mode -> {self.mode}')
        response.mode_sent = True
        return response

    def on_arming(self, request, response):
        self.armed = bool(request.value)
        self.get_logger().info('ARMED' if self.armed else 'disarmed')
        response.success = True
        return response

    def publish_status(self):
        state = State()
        state.header.stamp = self.get_clock().now().to_msg()
        state.connected = True
        state.armed = self.armed
        state.mode = self.mode
        self.state_pub.publish(state)

        battery = BatteryState()
        battery.header.stamp = state.header.stamp
        battery.voltage = 16.0      # a made-up healthy 4S pack
        self.battery_pub.publish(battery)

    # ------------------------------------------------------------------
    # written: sensors out of Gazebo
    # ------------------------------------------------------------------

    def on_gz_pose(self, msg):
        # Runs in Gazebo's thread, so just stash the data.
        for p in msg.pose:
            if p.name == self.model:
                with self.lock:
                    self.pose = (p.position, p.orientation)

    def on_gz_image(self, msg):
        # Runs in Gazebo's thread. The sim camera sends raw RGB bytes.
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        with self.lock:
            self.frame = frame
            self.frame_is_new = True

    def publish_pose(self):
        with self.lock:
            pose = self.pose
        if pose is None:
            return
        position, orientation = pose
        stamp = self.get_clock().now().to_msg()

        # Gazebo and mavros both use: x east, y north, z up. No conversion.
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'base_link'
        imu.orientation.x = orientation.x
        imu.orientation.y = orientation.y
        imu.orientation.z = orientation.z
        imu.orientation.w = orientation.w
        self.imu_pub.publish(imu)

        # rel_alt is height above the surface, so it is negative underwater,
        # exactly like the real one.
        alt = Float64()
        alt.data = position.z - SURFACE_Z
        self.alt_pub.publish(alt)

    def publish_image(self):
        with self.lock:
            if not self.frame_is_new:
                return
            frame = self.frame
            self.frame_is_new = False

        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        ok, encoded = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        out = CompressedImage()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'camera_link'
        out.format = 'jpeg'
        out.data = encoded.tobytes()
        self.image_pub.publish(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default='pool_qual')
    ap.add_argument('--model', default='sunswim')
    ap.add_argument('--drive', action='store_true',
                    help='drive the sim from /mavros/rc/override (autonomous mode)')
    ap.add_argument('--image-topic', default='/cam/image',
                    help='Gazebo camera topic (/cam/image_water with water_filter.py)')
    a, _ = ap.parse_known_args()

    rclpy.init()
    node = SimBridge(a.world, a.model, a.drive, a.image_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
