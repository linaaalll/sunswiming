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

        # The pole finder (see find_poles). Tuned on the pool footage of 13 Nov.
        self.work_width = self.declare_parameter('work_width', 960).value
        self.sat_drop = self.declare_parameter('sat_drop', 22).value          # how much duller than the water beside it
        self.water_sat = self.declare_parameter('water_sat', 215).value       # "open water" = at least this saturated
        self.min_value = self.declare_parameter('min_value', 110).value       # darker than this = the housing, ignore
        self.min_height_frac = self.declare_parameter('min_height_frac', 0.06).value  # pole at least this tall (fraction of image)

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

        Return (left_x, right_x, confidence, mask), x in pixels of the full image,
        or (None, None, 0.0, mask) when there is no believable pair.

        HOW (from the pool footage of 13 Nov): underwater the poles are NOT
        orange to the camera. Their hue is the same as the water's. What does
        differ is saturation: open water is fully saturated cyan (S ~255), a
        pole is duller (S 60-210). The floor is dull too, so a plain HSV range
        cannot separate pole from floor. Instead we look for thin vertical
        things that are duller than the open water right beside them.
        """
        # work on a smaller copy: faster, and the sizes below are in its pixels
        scale = self.work_width / bgr.shape[1]
        small = cv2.resize(bgr, (self.work_width, int(bgr.shape[0] * scale)),
                           interpolation=cv2.INTER_AREA)
        height, width = small.shape[:2]
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        sat = cv2.GaussianBlur(hsv[:, :, 1], (5, 5), 0)

        # 1. saturation of what is beside each pixel (61 px wide: wider than a pole)
        wide = cv2.getStructuringElement(cv2.MORPH_RECT, (61, 1))
        around = cv2.morphologyEx(sat, cv2.MORPH_CLOSE, wide)

        # 2. pole pixel = much duller than its surroundings, and the
        #    surroundings are open water (not floor, not wall)
        stand_out = cv2.subtract(around, sat)
        mask = ((stand_out > self.sat_drop) & (around > self.water_sat)).astype(np.uint8) * 255

        # 3. ignore the dark housing at the edge of the picture, with a margin
        lit = (hsv[:, :, 2] > self.min_value).astype(np.uint8) * 255
        lit = cv2.erode(lit, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (81, 81)))
        mask = cv2.bitwise_and(mask, lit)

        # 4. clean up: remove specks, join a pole that got split
        speck = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 7))
        gap = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 25))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, speck)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, gap)
        full_mask = cv2.resize(mask, (bgr.shape[1], bgr.shape[0]),
                               interpolation=cv2.INTER_NEAREST)

        # 5. blobs that look like a pole: tall and thin
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        pieces = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if h < self.min_height_frac * height:
                continue
            if h < 2.5 * w:
                continue
            pieces.append((x + w / 2.0, y, h))          # centre column, top, height

        # 6. the crossbar cuts each pole in two, and the surface mirrors it:
        #    pieces in the same column are one pole
        pieces.sort(key=lambda p: p[0])
        poles = []
        for x, y, h in pieces:
            if poles and abs(x - poles[-1][0]) < 0.025 * width:
                px, py, ph = poles[-1]
                top, bottom = min(py, y), max(py + ph, y + h)
                poles[-1] = ((px * ph + x * h) / (ph + h), top, bottom - top)
            else:
                poles.append((x, y, h))

        if len(poles) < 2:
            return None, None, 0.0, full_mask

        # 7. the two tallest: do they look like a pair?
        poles.sort(key=lambda p: p[2], reverse=True)
        a, b = poles[0], poles[1]
        alike = min(a[2], b[2]) / max(a[2], b[2])                    # 1.0 = same height
        overlap = min(a[1] + a[2], b[1] + b[2]) - max(a[1], b[1])    # shared rows
        apart = abs(a[0] - b[0])
        if alike < 0.35 or overlap < 0.3 * min(a[2], b[2]) or apart < 0.05 * width:
            return None, None, 0.0, full_mask

        # 8. back to full-image pixels, left one first
        return min(a[0], b[0]) / scale, max(a[0], b[0]) / scale, alike, full_mask

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
