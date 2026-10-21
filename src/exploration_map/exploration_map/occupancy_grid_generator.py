"""Occupancy grid generation for the whole map.

Takes the lidar obstacle detections of every robot and marks cells free or
obstacle. Every cell starts unknown and changes only when a beam crosses it
(free) or ends in it (obstacle), so the free/unknown boundary WFD needs exists.

Evidence accumulates in log-odds (values from the field_robotics
hotel_explored_map.py). One beam is enough to report a crossed cell free or a
hit cell occupied; a cell stays unknown only while free and obstacle evidence
cancel out.
"""

from typing import Iterable, Tuple

import numpy as np

UNKNOWN = -1
FREE = 0
OCCUPIED = 100

L_FREE = -0.4
L_OCCUPIED = 0.85
L_LIMIT = 4.0
L_FREE_THRESHOLD = -0.35
L_OCCUPIED_THRESHOLD = 0.5


class OccupancyGridGenerator:

    def __init__(
        self, width: int, height: int, resolution: float,
        origin_x: float, origin_y: float,
    ) -> None:
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.log_odds = np.zeros(self.width * self.height, dtype=np.float32)
        self.seen = np.zeros(self.width * self.height, dtype=bool)
        self.changed = False

    def cell_indices(self, points: np.ndarray) -> np.ndarray:
        """Flat indices of the cells containing the points (outside points dropped)."""
        if points.size == 0:
            return np.empty(0, dtype=np.int64)
        cols = np.floor((points[:, 0] - self.origin_x) / self.resolution).astype(np.int64)
        rows = np.floor((points[:, 1] - self.origin_y) / self.resolution).astype(np.int64)
        inside = (cols >= 0) & (cols < self.width) & (rows >= 0) & (rows < self.height)
        return rows[inside] * self.width + cols[inside]

    def traced_cells(self, origin: np.ndarray, ends: np.ndarray) -> np.ndarray:
        """Cells crossed by rays origin -> end, excluding each ray's last cell."""
        if ends.size == 0:
            return np.empty(0, dtype=np.int64)
        deltas = ends - origin[None, :]
        lengths = np.hypot(deltas[:, 0], deltas[:, 1])
        step = self.resolution * 0.5
        samples = int(np.ceil(lengths.max() / step)) + 1 if lengths.size else 1
        t = np.arange(samples) * step                       # distance along each ray
        reach = (lengths - self.resolution * 0.5)[:, None]  # stop short of the end cell
        mask = t[None, :] < reach
        unit = deltas / np.maximum(lengths, 1e-9)[:, None]
        xs = origin[0] + unit[:, 0:1] * t[None, :]
        ys = origin[1] + unit[:, 1:2] * t[None, :]
        points = np.stack([xs[mask], ys[mask]], axis=1)
        return np.unique(self.cell_indices(points))

    def update(self, origin: np.ndarray, obstacles: np.ndarray, free_ends: np.ndarray) -> None:
        """Integrate one detection: crossed cells get free evidence, hit cells obstacle."""
        hit = np.unique(self.cell_indices(obstacles))
        free = self.traced_cells(origin, free_ends)
        if hit.size:
            free = np.setdiff1d(free, hit, assume_unique=True)
        if free.size:
            self.log_odds[free] = np.maximum(self.log_odds[free] + L_FREE, -L_LIMIT)
            self.seen[free] = True
        if hit.size:
            self.log_odds[hit] = np.minimum(self.log_odds[hit] + L_OCCUPIED, L_LIMIT)
            self.seen[hit] = True
        self.changed = self.changed or bool(free.size or hit.size)

    def clear_rectangles(self, bodies: Iterable[Tuple[float, float, float, float, float]]) -> None:
        """Force cells inside oriented rectangles (x, y, yaw, length, width) to free.

        Used for the robots' own bodies: the lidar cannot see the floor under a
        robot, and a footprint-checking planner treats those unseen cells as
        blocked, but a robot's body cannot contain a wall.
        """
        for cx, cy, yaw, length, width in bodies:
            reach = 0.5 * float(np.hypot(length, width))
            c0 = max(0, int(np.floor((cx - reach - self.origin_x) / self.resolution)))
            c1 = min(self.width - 1, int(np.floor((cx + reach - self.origin_x) / self.resolution)))
            r0 = max(0, int(np.floor((cy - reach - self.origin_y) / self.resolution)))
            r1 = min(self.height - 1, int(np.floor((cy + reach - self.origin_y) / self.resolution)))
            if c1 < c0 or r1 < r0:
                continue
            cols, rows = np.meshgrid(np.arange(c0, c1 + 1), np.arange(r0, r1 + 1))
            dx = self.origin_x + (cols + 0.5) * self.resolution - cx
            dy = self.origin_y + (rows + 0.5) * self.resolution - cy
            c, s = np.cos(yaw), np.sin(yaw)
            along = c * dx + s * dy
            across = -s * dx + c * dy
            inside = (np.abs(along) <= 0.5 * length) & (np.abs(across) <= 0.5 * width)
            index = (rows[inside] * self.width + cols[inside]).ravel()
            if index.size:
                self.log_odds[index] = np.minimum(self.log_odds[index], L_FREE)
                self.seen[index] = True
                self.changed = True

    def values(self) -> np.ndarray:
        """OccupancyGrid data: -1 unknown, 0 free, 100 obstacle (flat, int8)."""
        out = np.full(self.width * self.height, UNKNOWN, dtype=np.int8)
        out[self.seen & (self.log_odds <= L_FREE_THRESHOLD)] = FREE
        out[self.seen & (self.log_odds >= L_OCCUPIED_THRESHOLD)] = OCCUPIED
        return out
