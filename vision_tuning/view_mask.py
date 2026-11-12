#!/usr/bin/env python3
"""
See the colour mask: camera picture on the left, mask on the right.

Use it to pick the HSV numbers BEFORE trusting the pole finder. It reads the
same numbers the robot uses (launch_file/config/qually.yaml, the
colour_gate_detector block), so what you see here is what the detector gets.

Run it on the LAPTOP (it opens a window), with the camera images coming from
the sim, a bag, or the real robot:

    python3 ~/sunswiming/vision_tuning/view_mask.py

Keys (click the window first):
    click on the picture   prints the colour (H, S, V) of that pixel
    r                      re-read qually.yaml (after you edited the numbers)
    s                      save what you see to vision_tuning/snapshots/
    q                      quit

How to tune:
    1. Click on a pole a few times, near and far. Note the H, S, V you get.
    2. In qually.yaml set hsv_low a bit below the smallest of each, and
       hsv_high a bit above the biggest.
    3. Save the file, press r. Poles should be white, the rest black.
    4. Repeat until it holds at every distance.

If the detector node is also running, its answer is drawn on top: green lines
on the poles, yellow on the gate centre.

Options:
    --topic /some/image/compressed    use another camera topic
    --snapshot out.jpg                no window: save one picture and exit
"""

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage

REPO = Path(__file__).resolve().parent.parent
CONFIG = REPO / 'launch_file' / 'config' / 'qually.yaml'
SNAPSHOTS = Path(__file__).resolve().parent / 'snapshots'
VIEW_WIDTH = 1280           # each half of the picture is this wide
WINDOW = 'colour mask   (click = pixel colour, r = reload yaml, s = save, q = quit)'


def load_settings():
    """The colour_gate_detector block of qually.yaml."""
    block = yaml.safe_load(CONFIG.read_text())['colour_gate_detector']['ros__parameters']
    return {
        'topic': block['image_topic'],
        'low': np.array(block['hsv_low'], dtype=np.uint8),
        'high': np.array(block['hsv_high'], dtype=np.uint8),
    }


class Viewer(Node):

    def __init__(self, topic):
        super().__init__('vision_tuning_viewer')
        self.bgr = None
        self.image_time = 0.0
        self.gate = None
        self.gate_time = 0.0
        self.create_subscription(CompressedImage, topic, self.on_image, qos_profile_sensor_data)
        try:
            # Only there once the workspace is built and sourced. Optional.
            from interfaces.msg import GateDetection
            self.create_subscription(GateDetection, '/qually/gate', self.on_gate, 10)
        except ImportError:
            self.get_logger().warn('interfaces not sourced: detector result will not be drawn')

    def on_image(self, msg):
        bgr = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if bgr is not None:
            self.bgr = bgr
            self.image_time = time.monotonic()

    def on_gate(self, msg):
        self.gate = msg
        self.gate_time = time.monotonic()


def compose(viewer, settings):
    """Build the side-by-side picture. Returns (picture, hsv image, scale)."""
    bgr = viewer.bgr
    height, width = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, settings['low'], settings['high'])
    white_percent = 100.0 * cv2.countNonZero(mask) / mask.size

    left = bgr.copy()
    gate = viewer.gate
    if gate is not None and time.monotonic() - viewer.gate_time < 1.0 and gate.found:
        centre = (gate.centre_offset + 1.0) * width / 2.0
        half = gate.separation * width / 2.0
        for x in (centre - half, centre + half):
            cv2.line(left, (int(x), 0), (int(x), height), (0, 255, 0), 3)
        cv2.line(left, (int(centre), 0), (int(centre), height), (0, 255, 255), 3)

    scale = VIEW_WIDTH / width
    size = (VIEW_WIDTH, int(height * scale))
    left = cv2.resize(left, size)
    right = cv2.cvtColor(cv2.resize(mask, size, interpolation=cv2.INTER_NEAREST),
                         cv2.COLOR_GRAY2BGR)

    low, high = settings['low'].tolist(), settings['high'].tolist()
    lines = [f'hsv_low  {low}', f'hsv_high {high}', f'white: {white_percent:.2f}% of the image']
    for i, text in enumerate(lines):
        cv2.putText(right, text, (15, 100 + 42 * i), cv2.FONT_HERSHEY_SIMPLEX, 1.1,
                    (0, 255, 255), 2)
    cv2.putText(left, 'CAMERA', (15, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 0), 3)
    cv2.putText(right, 'MASK: white = pixels inside your colour range', (15, 45),
                cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 255), 2)
    if time.monotonic() - viewer.image_time > 1.0:
        cv2.putText(left, 'NO NEW IMAGES', (15, 95), cv2.FONT_HERSHEY_SIMPLEX, 1.4,
                    (0, 0, 255), 3)
    return np.hstack([left, right]), hsv, scale


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default=None)
    parser.add_argument('--snapshot', default=None)
    args = parser.parse_args()

    settings = load_settings()
    topic = args.topic or settings['topic']

    rclpy.init()
    viewer = Viewer(topic)
    print(f'waiting for images on {topic} ...')

    state = {'hsv': None, 'scale': 1.0}

    def on_click(event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or state['hsv'] is None:
            return
        hsv = state['hsv']
        px = int((x % VIEW_WIDTH) / state['scale'])     # works on either half
        py = int(y / state['scale'])
        if py >= hsv.shape[0] or px >= hsv.shape[1]:
            return
        # A small patch around the click, so one odd pixel doesn't mislead you.
        patch = hsv[max(0, py - 2):py + 3, max(0, px - 2):px + 3].reshape(-1, 3)
        h, s, v = np.median(patch, axis=0).astype(int)
        print(f'clicked ({px}, {py}):  H {h}  S {s}  V {v}')

    if not args.snapshot:
        # Resizable: drag a corner or maximise it, the picture stretches to fit.
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(WINDOW, 1800, 506)
        cv2.setMouseCallback(WINDOW, on_click)

    started = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(viewer, timeout_sec=0.03)
            if viewer.bgr is None:
                if args.snapshot and time.monotonic() - started > 15.0:
                    raise SystemExit(f'no image arrived on {topic} in 15 s')
                if not args.snapshot:
                    # Nothing from the camera yet: say so in the window instead
                    # of leaving it empty.
                    waiting = np.zeros((720, 2 * VIEW_WIDTH, 3), dtype=np.uint8)
                    cv2.putText(waiting, 'NO CAMERA IMAGES YET', (60, 300),
                                cv2.FONT_HERSHEY_SIMPLEX, 3.0, (0, 0, 255), 6)
                    cv2.putText(waiting, f'waiting on {topic}', (60, 400),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.4, (255, 255, 255), 3)
                    cv2.putText(waiting, 'is the sim / bag / robot camera running?', (60, 470),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.4, (255, 255, 255), 3)
                    cv2.imshow(WINDOW, waiting)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                continue

            picture, state['hsv'], state['scale'] = compose(viewer, settings)

            if args.snapshot:
                cv2.imwrite(args.snapshot, picture)
                print(f'saved {args.snapshot}')
                break

            cv2.imshow(WINDOW, picture)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key == ord('r'):
                try:
                    settings = load_settings()
                    print(f"reloaded: hsv_low {settings['low'].tolist()}  "
                          f"hsv_high {settings['high'].tolist()}")
                except Exception as error:           # a typo in the yaml, keep the old numbers
                    print(f'could not read {CONFIG}: {error}')
            if key == ord('s'):
                SNAPSHOTS.mkdir(exist_ok=True)
                path = SNAPSHOTS / time.strftime('mask_%Y-%m-%d_%H-%M-%S.jpg')
                cv2.imwrite(str(path), picture)
                print(f'saved {path}')
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
