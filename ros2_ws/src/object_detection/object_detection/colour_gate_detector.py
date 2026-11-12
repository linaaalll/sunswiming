"""
Colour masking Approach

Colour gate detector: finds the two gate poles by their colour.

In:   the ZED colour image (jpeg compressed)
Out:  /qually/gate                          interfaces/GateDetection
      /qually/gate/debug_image/compressed   the image with what it found drawn on
      /qually/gate/mask/compressed          the black/white mask (if you return one)

The ROS side is written. The actual vision, find_poles(), is yours.

Tune it without the robot by playing a bag back on the laptop:

    ros2 bag play bags/<run>/bag --loop
    ros2 run object_detection colour_gate_detector
    ros2 run rqt_image_view rqt_image_view     # pick the debug_image topic
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage

from interfaces.msg import GateDetection


class ColourGateDetector(Node):

    def __init__(self):
        super().__init__('colour_gate_detector')

        image_topic = self.declare_parameter(
            'image_topic', '/zed/zed_node/rgb/color/rect/image/compressed').value

        # HSV range of the pole colour: [hue 0-179, saturation 0-255, value 0-255].
        # These are placeholders, the real numbers come from pool footage.
        self.hsv_low = np.array(
            self.declare_parameter('hsv_low', [0, 120, 70]).value, dtype=np.uint8)
        self.hsv_high = np.array(
            self.declare_parameter('hsv_high', [10, 255, 255]).value, dtype=np.uint8)

        # A blob smaller than this many pixels is noise, not a pole.
        self.min_area = self.declare_parameter('min_area', 300).value

        # Publish the debug image every Nth frame (1 = every frame).
        self.debug_every = max(1, self.declare_parameter('debug_every', 2).value)

        self.gate_pub = self.create_publisher(GateDetection, '/qually/gate', 10)
        self.debug_pub = self.create_publisher(
            CompressedImage, '/qually/gate/debug_image/compressed', 2)
        self.mask_pub = self.create_publisher(
            CompressedImage, '/qually/gate/mask/compressed', 2)

        self.create_subscription(
            CompressedImage, image_topic, self.on_image, qos_profile_sensor_data)

        self.frame_count = 0
        self.get_logger().info(f'colour gate detector listening on {image_topic}')

    def find_poles(self, bgr):
        """
        Look for the two gate poles in one image.

        bgr: the camera image, a numpy array of shape (height, width, 3),
             in OpenCV's blue-green-red order.

        Return (left_x, right_x, confidence, mask):
            left_x, right_x  pixel column of each pole's centre, or None, None
                             if you can't find two poles
            confidence       0.0 to 1.0, how much you believe it
            mask             the black/white mask you made (or None). It gets
                             published so you can see what the threshold did.
        """

        # from colour -> mask
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        # Apply binary mask
        mask = cv2.inRange(hsv, self.hsv_low, self.hsv_high)

        # clean up the mask
        # The goal is to collect all the thin poles!
        speck = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 5))
        gap = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 15))

        # morphological opening = erosion -> dilation
        # erosion removes white pixels around edges
        # dilation expands the remianing white
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, speck)

        # morphological closing = dilation -> erosion
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, gap)

        # looking for things that looks like pole
        contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        poles = []

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            # too small aiya
            if w*h < self.min_area:
                continue
            # not tall and thin
            if h < 2.5 * w:
                continue
            poles.append((x + w / 2.0, y, h))

        if len(poles) < 2:
            return None, None, 0.0, mask

        # find the two tallest pole and see if they're similar
        poles.sort(key=lambda p : p[2], reverse=True)
        a, b = poles[0], poles[1]

        alike = min(a[2], b[2]) / max(a[2], b[2])
        overlap = min(a[1] + a[2], b[1] + b[2]) - max(a[1], b[1])

        if alike < 0.5 or overlap < 0.5 * min(a[2], b[2]):
            return None, None, 0.0, mask

        return min(a[0], b[0]), max(a[0], b[0]), alike, mask
        
    # ------------------------------------------------------------------
    # written: ROS in, ROS out
    # ------------------------------------------------------------------

    def on_image(self, msg):
        bgr = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            self.get_logger().warn('could not decode image', throttle_duration_sec=2.0)
            return

        height, width = bgr.shape[:2]
        left_x, right_x, confidence, mask = self.find_poles(bgr)
        found = left_x is not None and right_x is not None

        out = GateDetection()
        out.header = msg.header          # same timestamp as the image it came from
        out.source = 'colour'
        out.found = found
        if found:
            left_x, right_x = sorted((float(left_x), float(right_x)))
            centre_x = (left_x + right_x) / 2.0
            # pixel column -> -1 (left edge) .. +1 (right edge)
            out.centre_offset = (centre_x - width / 2.0) / (width / 2.0)
            out.separation = (right_x - left_x) / width
            out.confidence = float(confidence)
        self.gate_pub.publish(out)

        self.frame_count += 1
        if self.frame_count % self.debug_every != 0:
            return

        if self.debug_pub.get_subscription_count() > 0:
            debug = bgr.copy()
            cv2.line(debug, (width // 2, 0), (width // 2, height), (255, 255, 255), 1)
            if found:
                for x in (left_x, right_x):
                    cv2.line(debug, (int(x), 0), (int(x), height), (0, 255, 0), 2)
                cx = int((left_x + right_x) / 2)
                cv2.line(debug, (cx, 0), (cx, height), (0, 255, 255), 2)
                label = (f'offset {out.centre_offset:+.2f}  '
                         f'sep {out.separation:.2f}  conf {out.confidence:.2f}')
            else:
                label = 'no gate'
            cv2.putText(debug, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 0) if found else (0, 0, 255), 2)
            self.publish_jpeg(self.debug_pub, debug, msg.header)

        if mask is not None and self.mask_pub.get_subscription_count() > 0:
            self.publish_jpeg(self.mask_pub, mask, msg.header)

    def publish_jpeg(self, publisher, image, header):
        ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if not ok:
            return
        out = CompressedImage()
        out.header = header
        out.format = 'jpeg'
        out.data = encoded.tobytes()
        publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ColourGateDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
