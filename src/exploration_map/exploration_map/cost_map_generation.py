"""Costmap generation, one instance per robot.

From the shared occupancy grid, every obstacle cell costs A = 255 and the cost
decays with distance d_i from the nearest obstacle cell:

    cost = A * exp(-(d_i * decay_rate)),   decay_rate = 1 / S,   S = length * width

so a larger robot sees a slower decay (a wider costly band around obstacles).
d_i is measured in metres. Unknown cells stay unknown.
"""

import numpy as np
from scipy.ndimage import distance_transform_edt

from exploration_map.occupancy_grid_generator import OCCUPIED, UNKNOWN

AMPLITUDE = 255.0


class CostMapGenerator:

    def __init__(self, robot_length: float, robot_width: float, amplitude: float = AMPLITUDE):
        if robot_length <= 0.0 or robot_width <= 0.0:
            raise ValueError('robot length and width must be positive')
        self.area = float(robot_length) * float(robot_width)   # S
        self.decay_rate = 1.0 / self.area
        self.amplitude = float(amplitude)

    def costs(self, occupancy: np.ndarray, resolution: float) -> np.ndarray:
        """Cost 0..A per cell for a 2D (rows, cols) occupancy array; NaN where unknown."""
        obstacle = occupancy == OCCUPIED
        if obstacle.any():
            d = distance_transform_edt(~obstacle) * resolution  # metres to nearest obstacle
            cost = self.amplitude * np.exp(-(d * self.decay_rate))
        else:
            cost = np.zeros(occupancy.shape, dtype=np.float64)
        cost[obstacle] = self.amplitude
        cost[occupancy == UNKNOWN] = np.nan
        return cost

    def occupancy_values(self, occupancy: np.ndarray, resolution: float) -> np.ndarray:
        """Costs as OccupancyGrid data: 0..100 (cost * 100 / A), obstacles 100, unknown -1.

        Non-obstacle cells are capped at 99 so only obstacle cells reach Nav2's
        lethal value.
        """
        cost = self.costs(occupancy, resolution)
        out = np.full(occupancy.shape, UNKNOWN, dtype=np.int8)
        known = ~np.isnan(cost)
        scaled = np.rint(np.nan_to_num(cost) * 100.0 / self.amplitude)
        out[known] = np.minimum(scaled[known], 99).astype(np.int8)
        out[occupancy == OCCUPIED] = 100
        return out
