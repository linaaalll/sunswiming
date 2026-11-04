#!/usr/bin/env python3
"""
Proudly sponsor by Santa Claude
Live murky-water filter for the ROV camera.

Reads   /cam/image        (clean render from Gazebo)
        /cam/depth_image  (distance of every pixel, metres)
Writes  /cam/image_water  (what the camera would see underwater)

    python3 water_filter.py              # default murk
    python3 water_filter.py --murk 2     # twice as murky
    python3 water_filter.py --murk 0.5   # clearer water

Then in Gazebo: Image Display -> pick /cam/image_water.

Physics per colour channel:  I = J*t + B*(1 - t),  t = exp(-beta * d)
  J = clean colour, d = distance, B = colour of far-away water,
  beta = how fast each colour fades per metre (red fades fastest).
B was sampled from the team's pool photos; beta is a GUESS until fitted.
"""
import argparse
import threading
import time

import numpy as np
from PIL import Image as PILImage, ImageFilter

from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

B_WATER = np.array([70, 158, 190], np.float32)   # far-water colour (R, G, B) from photos
BETA = np.array([0.35, 0.10, 0.07], np.float32)  # fade per metre (R, G, B), GUESS
FAR = 30.0                                        # m, used where the ray hits nothing


def caustic_field(h, w, seed=0):
    """Rippled brightness pattern, computed once and slid around each frame."""
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    f = np.zeros((h, w), np.float32)
    for _ in range(6):
        k = rng.uniform(0.5, 1.5) * 0.04 * np.pi
        th = rng.uniform(0, np.pi)
        f += np.sin(k * (xx * np.cos(th) + yy * np.sin(th)) + rng.uniform(0, 2 * np.pi))
    f = np.abs(f) / 6.0
    return f - f.mean()


class WaterFilter:
    def __init__(self, murk, caustics, noise, blur):
        self.beta = BETA * murk
        self.caustics_on, self.noise, self.blur = caustics, noise, blur
        self.depth = None
        self.field = None
        self.lock = threading.Lock()
        self.node = Node()
        self.pub = self.node.advertise("/cam/image_water", Image)
        self.node.subscribe(Image, "/cam/depth_image", self.on_depth)
        self.node.subscribe(Image, "/cam/image", self.on_rgb)
        self.rng = np.random.default_rng()

    def on_depth(self, msg):
        d = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
        with self.lock:
            self.depth = np.where(np.isfinite(d) & (d > 0), d, FAR)

    def on_rgb(self, msg):
        with self.lock:
            depth = self.depth
        if depth is None or depth.shape != (msg.height, msg.width):
            return
        J = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).astype(np.float32)

        if self.caustics_on:
            if self.field is None:
                self.field = caustic_field(msg.height, msg.width)
            shift = int(time.time() * 20) % msg.width          # slide the ripples over time
            J = J * (1.0 + 0.15 * np.roll(self.field, shift, axis=1))[..., None]

        t = np.exp(-self.beta[None, None, :] * depth[..., None])
        I = J * t + B_WATER * (1.0 - t)
        if self.noise > 0:
            I += self.rng.normal(0, self.noise, I.shape).astype(np.float32)
        out = np.clip(I, 0, 255).astype(np.uint8)
        if self.blur > 0:
            out = np.asarray(PILImage.fromarray(out).filter(ImageFilter.GaussianBlur(self.blur)))

        m = Image()
        m.CopyFrom(msg)                 # same size, format, timestamp, frame
        m.data = out.tobytes()
        self.pub.publish(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--murk", type=float, default=1.0, help="multiply the fade rate (2 = murkier)")
    ap.add_argument("--caustics", action="store_true", help="turn ON the riplling light")
    ap.add_argument("--noise", type=float, default=2.0, help="sensor noise, grey levels")
    ap.add_argument("--blur", type=float, default=0.8, help="blur radius, pixels")
    a = ap.parse_args()
    WaterFilter(a.murk, a.caustics, a.noise, a.blur)
    print("water filter running: /cam/image + /cam/depth_image -> /cam/image_water (Ctrl+C to stop)")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()