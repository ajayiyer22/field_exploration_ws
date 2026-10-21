"""Display each robot's local Nav2 grid in the shared map frame.

Robots have separate TF topics containing identically named odom frames. Keep
their buffers separate and transform only grid origins, preserving every cell.
The output is for RViz; Nav2 continues to use its original local costmap.
"""

from copy import deepcopy
import math

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def transform_grid(grid, transform, map_frame):
    """Compose planar map->odom with the grid origin; do not resample cells."""
    output = deepcopy(grid)
    output.header.frame_id = map_frame
    origin = output.info.origin
    t = transform.transform
    angle = yaw(t.rotation)
    c, s = math.cos(angle), math.sin(angle)
    x, y = origin.position.x, origin.position.y
    origin.position.x = t.translation.x + c * x - s * y
    origin.position.y = t.translation.y + s * x + c * y
    origin.position.z += t.translation.z
    angle += yaw(origin.orientation)
    origin.orientation.x = origin.orientation.y = 0.0
    origin.orientation.z = math.sin(angle / 2)
    origin.orientation.w = math.cos(angle / 2)
    return output


class CostmapVisualizer(Node):
    def __init__(self):
        super().__init__('costmap_visualizer')
        self.declare_parameter('robot_namespaces', ['a200_0000'])
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('display_height', 0.03)
        self.map_frame = self.get_parameter('map_frame').value
        self.display_height = float(self.get_parameter('display_height').value)
        self._tf_buffers, self._pending_grids, self._grid_publishers = {}, {}, {}
        self._grid_subscriptions = []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for ns in self.get_parameter('robot_namespaces').value:
            self._tf_buffers[ns] = Buffer(cache_time=Duration(seconds=10))
            self._pending_grids[ns] = None
            self._grid_publishers[ns] = self.create_publisher(
                OccupancyGrid, f'/{ns}/local_costmap/rviz_costmap', latched)
            self._grid_subscriptions.append(self.create_subscription(
                OccupancyGrid, f'/{ns}/local_costmap/costmap',
                lambda msg, ns=ns: self._pending_grids.__setitem__(ns, msg), latched))
            self._grid_subscriptions.append(self.create_subscription(
                TFMessage, f'/{ns}/tf',
                lambda msg, ns=ns: self.on_tf(ns, msg), 100))
        self.create_timer(0.1, self.publish_pending)

    def on_tf(self, ns, message):
        for transform in message.transforms:
            self._tf_buffers[ns].set_transform(transform, ns)

    def publish_pending(self):
        for ns, grid in self._pending_grids.items():
            if grid is None:
                continue
            if grid.header.frame_id == self.map_frame:
                output = deepcopy(grid)
            else:
                try:
                    transform = self._tf_buffers[ns].lookup_transform(
                        self.map_frame, grid.header.frame_id,
                        Time.from_msg(grid.header.stamp))
                except TransformException:
                    # Wait for TF at this grid's timestamp, rather than using
                    # another robot's odom transform or a newer robot pose.
                    continue
                output = transform_grid(grid, transform, self.map_frame)
            # Keep the local overlay above the global map to avoid flicker.
            output.info.origin.position.z += self.display_height
            self._grid_publishers[ns].publish(output)
            self._pending_grids[ns] = None


def main(args=None):
    rclpy.init(args=args)
    node = CostmapVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
