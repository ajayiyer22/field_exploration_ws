import math

import pytest
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid

from search_sim.costmap_visualizer import transform_grid, yaw


def test_rotated_grid_keeps_cell_values_and_size_without_resampling():
    grid = OccupancyGrid()
    grid.header.frame_id = 'odom'
    grid.header.stamp.sec = 42
    grid.info.width, grid.info.height, grid.info.resolution = 3, 2, 0.05
    grid.info.origin.position.x, grid.info.origin.position.y = 2.0, 3.0
    grid.info.origin.orientation.z = math.sin(math.pi / 8)
    grid.info.origin.orientation.w = math.cos(math.pi / 8)
    grid.data = [-1, 0, 1, 98, 99, 100]
    transform = TransformStamped()
    transform.transform.translation.x, transform.transform.translation.y = 10.0, -4.0
    transform.transform.rotation.z = math.sin(math.pi / 4)
    transform.transform.rotation.w = math.cos(math.pi / 4)
    output = transform_grid(grid, transform, 'map')
    # A 90-degree rotation sends (2, 3) to (-3, 2), then adds (10, -4).
    assert output.info.origin.position.x == pytest.approx(7.0)
    assert output.info.origin.position.y == pytest.approx(-2.0)
    assert yaw(output.info.origin.orientation) == pytest.approx(3 * math.pi / 4)
    assert output.header.frame_id == 'map'
    assert output.header.stamp == grid.header.stamp
    assert output.data == grid.data
    assert output.info.width == 3 and output.info.height == 2
    assert output.info.resolution == pytest.approx(0.05)
    assert grid.header.frame_id == 'odom'
    assert grid.info.origin.position.x == 2.0


def test_two_robots_with_same_odom_frame_have_distinct_map_origins():
    grid = OccupancyGrid()
    grid.header.frame_id = 'odom'
    grid.info.origin.orientation.w = 1.0
    grid.data = [100]
    first, second = TransformStamped(), TransformStamped()
    first.transform.rotation.w = second.transform.rotation.w = 1.0
    first.transform.translation.x, second.transform.translation.x = 12.0, 21.0
    a = transform_grid(grid, first, 'map')
    b = transform_grid(grid, second, 'map')
    assert a.info.origin.position.x == 12.0
    assert b.info.origin.position.x == 21.0
    assert a.data == b.data == grid.data
