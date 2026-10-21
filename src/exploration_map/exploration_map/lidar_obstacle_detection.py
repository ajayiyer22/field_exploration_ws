"""Lidar obstacle detection, one instance per robot.

Turns a 2D LaserScan into map-frame geometry for the occupancy grid:

  * obstacle points - beam endpoints that hit something (range < range_max)
  * free ray ends   - where each beam stops: the hit point, or max_range for
                      beams that returned nothing

The sensor pose in the map frame is the robot's ground-truth pose composed with
the fixed base_link -> lidar mount transform.
"""

from dataclasses import dataclass
import math
from typing import Dict, Optional, Tuple

import numpy as np


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float

    def compose(self, other: 'Pose2D') -> 'Pose2D':
        """self * other in SE(2)."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(
            self.x + c * other.x - s * other.y,
            self.y + s * other.x + c * other.y,
            math.atan2(math.sin(self.yaw + other.yaw), math.cos(self.yaw + other.yaw)),
        )


@dataclass
class Detection:
    origin: np.ndarray        # (2,) sensor position in map frame
    obstacles: np.ndarray     # (N, 2) hit points
    free_ends: np.ndarray     # (M, 2) end of every usable beam (hit or max range)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class StaticTransformTree:
    """Planar static transforms of one robot, from its /<ns>/tf_static."""

    def __init__(self) -> None:
        self._parent: Dict[str, Tuple[str, Pose2D]] = {}

    def add(self, parent: str, child: str, pose: Pose2D) -> None:
        self._parent[child.strip('/')] = (parent.strip('/'), pose)

    def has(self, frame: str) -> bool:
        return frame.strip('/') in self._parent

    def lookup(self, base: str, frame: str) -> Optional[Pose2D]:
        """base -> frame, walking up from frame; None if not connected."""
        base, frame = base.strip('/'), frame.strip('/')
        chain = []
        current = frame
        for _ in range(64):
            if current == base:
                pose = Pose2D(0.0, 0.0, 0.0)
                for link in reversed(chain):
                    pose = pose.compose(link)
                return pose
            entry = self._parent.get(current)
            if entry is None:
                return None
            current, link = entry
            chain.append(link)
        return None


class LidarObstacleDetector:

    def __init__(self, max_range: float = 8.0, beam_stride: int = 1) -> None:
        self.max_range = max_range
        self.beam_stride = max(1, int(beam_stride))

    def detect(
        self,
        ranges: np.ndarray,
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        sensor_pose: Pose2D,
    ) -> Detection:
        ranges = np.asarray(ranges, dtype=np.float64)
        angles = angle_min + angle_increment * np.arange(ranges.size)
        ranges = ranges[::self.beam_stride]
        angles = angles[::self.beam_stride]

        limit = min(self.max_range, range_max) if range_max > 0 else self.max_range
        valid = ~np.isnan(ranges) & (ranges >= range_min)
        hit = valid & np.isfinite(ranges) & (ranges < range_max) & (ranges <= limit)
        no_return = valid & ~hit  # inf, at/over range_max, or beyond max_range

        lengths = np.where(hit, ranges, limit)
        usable = hit | no_return
        world = sensor_pose.yaw + angles
        xs = sensor_pose.x + lengths * np.cos(world)
        ys = sensor_pose.y + lengths * np.sin(world)
        points = np.stack([xs, ys], axis=1)
        return Detection(
            origin=np.array([sensor_pose.x, sensor_pose.y]),
            obstacles=points[hit],
            free_ends=points[usable],
        )
