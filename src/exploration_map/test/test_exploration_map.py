import math

import numpy as np
import pytest

from exploration_map.cost_map_generation import CostMapGenerator
from exploration_map.lidar_obstacle_detection import (
    LidarObstacleDetector, Pose2D, StaticTransformTree)
from exploration_map.occupancy_grid_generator import (
    FREE, OCCUPIED, UNKNOWN, OccupancyGridGenerator)


def single_obstacle(size=41, resolution=0.05):
    grid = np.zeros((size, size), dtype=np.int8)
    grid[size // 2, size // 2] = OCCUPIED
    return grid


def test_cost_is_amplitude_on_obstacle_and_decays_with_distance():
    gen = CostMapGenerator(robot_length=1.0, robot_width=0.5)  # S = 0.5, decay = 2 /m
    cost = gen.costs(single_obstacle(), 0.05)
    c = 20
    assert cost[c, c] == pytest.approx(255.0)
    for cells in (1, 4, 10):
        d = cells * 0.05
        assert cost[c, c + cells] == pytest.approx(255.0 * math.exp(-d * 2.0))


def test_larger_robot_decays_slower():
    occupancy = single_obstacle()
    jackal = CostMapGenerator(0.51, 0.43).costs(occupancy, 0.05)
    warthog = CostMapGenerator(1.52, 1.38).costs(occupancy, 0.05)
    assert warthog[20, 30] > jackal[20, 30]
    assert jackal[20, 30] == pytest.approx(255.0 * math.exp(-0.5 / (0.51 * 0.43)))


def test_unknown_cells_stay_unknown():
    occupancy = single_obstacle()
    occupancy[0:5, 0:5] = UNKNOWN
    gen = CostMapGenerator(1.0, 1.0)
    values = gen.occupancy_values(occupancy, 0.05)
    assert (values[0:5, 0:5] == UNKNOWN).all()
    assert values[20, 20] == 100
    assert values[20, 21] <= 99


def test_ray_marks_crossed_cells_free_and_end_cell_obstacle():
    grid = OccupancyGridGenerator(40, 10, 0.1, 0.0, 0.0)
    origin = np.array([0.05, 0.55])
    hit = np.array([[3.05, 0.55]])
    grid.update(origin, hit, hit)
    values = grid.values().reshape(10, 40)
    assert (values[5, 0:30] == FREE).all()
    assert values[5, 30] == OCCUPIED
    assert (values[5, 31:] == UNKNOWN).all()
    assert (values[0, :] == UNKNOWN).all()


def test_max_range_beam_clears_without_obstacle():
    detector = LidarObstacleDetector(max_range=2.0)
    det = detector.detect(np.array([np.inf, 1.0]), 0.0, math.pi / 2, 0.1, 30.0, Pose2D(0, 0, 0))
    assert det.obstacles.shape == (1, 2)
    assert det.obstacles[0] == pytest.approx([0.0, 1.0])
    assert det.free_ends.shape == (2, 2)
    assert det.free_ends[0] == pytest.approx([2.0, 0.0])


def test_static_transform_chain():
    tree = StaticTransformTree()
    tree.add('base_link', 'mount', Pose2D(0.2, 0.0, math.pi / 2))
    tree.add('mount', 'lidar', Pose2D(0.1, 0.0, 0.0))
    pose = tree.lookup('base_link', 'lidar')
    assert pose.x == pytest.approx(0.2)
    assert pose.y == pytest.approx(0.1)
    assert pose.yaw == pytest.approx(math.pi / 2)
    assert tree.lookup('base_link', 'missing') is None


def test_footprint_clearing_uses_oriented_rectangle():
    grid = OccupancyGridGenerator(40, 40, 0.1, 0.0, 0.0)
    grid.update(np.array([0.05, 0.05]), np.array([[2.05, 2.05]]), np.array([[2.05, 2.05]]))
    assert grid.values().reshape(40, 40)[20, 20] == OCCUPIED
    # 1.0 m x 0.4 m body at (2.05, 2.05), rotated 90 degrees: long axis along y.
    grid.clear_rectangles([(2.05, 2.05, math.pi / 2, 1.0, 0.4)])
    values = grid.values().reshape(40, 40)
    assert values[20, 20] == FREE
    assert values[24, 20] == FREE       # 0.4 m along the long (y) axis: inside
    assert values[20, 24] == UNKNOWN    # 0.4 m along the short (x) axis: outside
