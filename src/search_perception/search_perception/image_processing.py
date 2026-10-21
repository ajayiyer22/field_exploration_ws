"""Image decoding and target confirmation, independent of CUDA and ROS nodes."""

from dataclasses import dataclass

import cv2
import numpy as np


def image_to_bgr(message):
    """Decode padded ROS 8-bit images without depending on cv_bridge's NumPy ABI."""
    encoding = message.encoding.lower()
    channels = {'mono8': 1, 'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4}.get(encoding)
    if channels is None:
        raise ValueError(f'Unsupported camera encoding: {message.encoding!r}')
    width, height, step = message.width, message.height, message.step
    if width <= 0 or height <= 0 or step < width * channels:
        raise ValueError('Invalid image dimensions or row stride')
    raw = np.frombuffer(message.data, dtype=np.uint8)
    expected = step * height
    if raw.size < expected:
        raise ValueError(f'Image has {raw.size} bytes, expected {expected}')
    pixels = raw[:expected].reshape(height, step)[:, :width * channels]
    pixels = pixels.reshape(height, width, channels)
    conversion = {'rgb8': cv2.COLOR_RGB2BGR, 'rgba8': cv2.COLOR_RGBA2BGR,
                  'bgra8': cv2.COLOR_BGRA2BGR, 'mono8': cv2.COLOR_GRAY2BGR}
    return cv2.cvtColor(pixels, conversion[encoding]) if encoding in conversion else pixels.copy()


def resolve_target(target, names):
    """Resolve model classes and the red tennis ball alias (COCO sports ball)."""
    target = ' '.join(target.lower().split())
    if not target:
        return None, None  # Detection-only mode; never stop the mission.
    color = 'pink' if target.startswith('pink ') else None
    base = target[5:] if color else target
    if target == 'red tennis ball':
        base, color = 'sports ball', 'red'
    matches = [int(key) for key, value in names.items() if value.lower() == base]
    if not matches:
        raise ValueError(f'Target {target!r} must be an exact YOLO class, optionally prefixed by pink, or red tennis ball')
    return matches[0], color


def pink_fraction(crop, saturation_min=45, value_min=45):
    """Estimate pink/magenta among chromatic pixels; this is a color heuristic."""
    if crop.size == 0:
        return 0.0
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    chromatic = (hsv[..., 1] >= saturation_min) & (hsv[..., 2] >= value_min)
    count = np.count_nonzero(chromatic)
    if not count:
        return 0.0
    # Exclude the low-hue red band so ordinary red objects don't qualify as pink.
    pink = chromatic & (hsv[..., 0] >= 145) & (hsv[..., 0] <= 175)
    return float(np.count_nonzero(pink)) / int(count)


def red_fraction(crop, saturation_min=70, value_min=45):
    """Red hue wraps around both ends of OpenCV's 0..179 HSV hue range."""
    if crop.size == 0:
        return 0.0
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    chromatic = (hsv[..., 1] >= saturation_min) & (hsv[..., 2] >= value_min)
    count = np.count_nonzero(chromatic)
    if not count:
        return 0.0
    red = chromatic & ((hsv[..., 0] <= 10) | (hsv[..., 0] >= 170))
    return float(np.count_nonzero(red)) / int(count)


@dataclass
class HitStreak:
    count: int = 0
    last_stamp: object = None
    last_time: float = 0.0

    def update(self, matched, stamp, now, max_gap):
        if stamp == self.last_stamp:
            return self.count
        if now - self.last_time > max_gap:
            self.count = 0
        self.count = self.count + 1 if matched else 0
        self.last_stamp, self.last_time = stamp, now
        return self.count
