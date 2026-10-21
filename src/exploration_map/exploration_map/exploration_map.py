"""Exploration map node.

    lidar_obstacle_detection (one per robot)
            |  obstacle points + free rays, map frame
            v
    occupancy_grid_generator (whole map)  -> /exploration_map/occupancy
            |
            v
    cost_map_generation (one per robot)   -> /<ns>/exploration_costmap

Robot poses come from /<ns>/ground_truth/pose. The lidar mount comes from the
robot's own /<ns>/tf_static.
"""

import threading
from typing import Dict, List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from exploration_map.cost_map_generation import CostMapGenerator
from exploration_map.lidar_obstacle_detection import (
    LidarObstacleDetector, Pose2D, StaticTransformTree, yaw_from_quaternion)
from exploration_map.occupancy_grid_generator import OccupancyGridGenerator


class Robot:

    def __init__(self, ns: str, length: float, width: float, detector: LidarObstacleDetector):
        self.ns = ns
        self.detector = detector
        self.costmap = CostMapGenerator(length, width)
        self.length = float(length)
        self.width = float(width)
        self.static_tf = StaticTransformTree()
        self.pose: Optional[Pose2D] = None
        self.last_scan_time = -1.0
        self.warned_frame = False
        self.costmap_pub = None


class ExplorationMap(Node):

    def __init__(self) -> None:
        super().__init__('exploration_map')
        p = self.declare_parameter
        p('robot_namespaces', ['a200_0000'])
        p('robot_lengths', [0.99])
        p('robot_widths', [0.67])
        p('scan_topic', 'sensors/lidar2d_0/scan')
        p('base_frame', 'base_link')
        p('lidar_frame_fallback', 'lidar2d_0_laser')
        p('map_frame', 'map')
        p('occupancy_topic', '/exploration_map/occupancy')
        p('costmap_topic', 'exploration_costmap')
        p('grid_resolution', 0.05)
        p('grid_origin_x', 0.0)
        p('grid_origin_y', -47.05)
        p('grid_size_x_m', 38.6)
        p('grid_size_y_m', 47.05)
        p('max_range', 8.0)
        p('beam_stride', 1)
        p('scan_period', 0.2)
        p('publish_period', 0.5)
        p('clear_robot_footprints', True)

        g = self.get_parameter
        namespaces = [str(n).strip('/') for n in g('robot_namespaces').value]
        lengths = [float(v) for v in g('robot_lengths').value]
        widths = [float(v) for v in g('robot_widths').value]
        if not (len(namespaces) == len(lengths) == len(widths)):
            raise ValueError('robot_namespaces, robot_lengths and robot_widths must match')

        self._map_frame = str(g('map_frame').value)
        self._base_frame = str(g('base_frame').value)
        self._lidar_fallback = str(g('lidar_frame_fallback').value)
        self._scan_period = float(g('scan_period').value)
        self._clear_footprints = bool(g('clear_robot_footprints').value)
        resolution = float(g('grid_resolution').value)
        self._grid = OccupancyGridGenerator(
            int(np.ceil(float(g('grid_size_x_m').value) / resolution)),
            int(np.ceil(float(g('grid_size_y_m').value) / resolution)),
            resolution, float(g('grid_origin_x').value), float(g('grid_origin_y').value))
        self._lock = threading.Lock()

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._occupancy_pub = self.create_publisher(
            OccupancyGrid, str(g('occupancy_topic').value), latched)

        # Several latched publishers share /<ns>/tf_static (robot_state_publisher
        # and per-sensor static broadcasters); a depth of 1 would let one
        # overwrite another before the callback runs.
        static_tf_qos = QoSProfile(depth=100)
        static_tf_qos.reliability = ReliabilityPolicy.RELIABLE
        static_tf_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        scan_topic = str(g('scan_topic').value).strip('/')
        costmap_topic = str(g('costmap_topic').value).strip('/')
        self._robots: Dict[str, Robot] = {}
        for ns, length, width in zip(namespaces, lengths, widths):
            robot = Robot(ns, length, width, LidarObstacleDetector(
                float(g('max_range').value), int(g('beam_stride').value)))
            robot.costmap_pub = self.create_publisher(
                OccupancyGrid, f'/{ns}/{costmap_topic}', latched)
            self._robots[ns] = robot
            self.create_subscription(
                PoseStamped, f'/{ns}/ground_truth/pose',
                lambda msg, r=robot: self._on_pose(r, msg), qos_profile_sensor_data)
            self.create_subscription(
                LaserScan, f'/{ns}/{scan_topic}',
                lambda msg, r=robot: self._on_scan(r, msg), qos_profile_sensor_data)
            self.create_subscription(
                TFMessage, f'/{ns}/tf_static',
                lambda msg, r=robot: self._on_static_tf(r, msg), static_tf_qos)
            self.get_logger().info(
                f'{ns}: S = {length:.2f} x {width:.2f} = {robot.costmap.area:.3f} m^2, '
                f'decay_rate = {robot.costmap.decay_rate:.3f} 1/m')

        self.create_timer(float(g('publish_period').value), self._publish)
        self.get_logger().info(
            f'Shared grid {self._grid.width} x {self._grid.height} cells at {resolution} m')

    def _on_pose(self, robot: Robot, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        robot.pose = Pose2D(msg.pose.position.x, msg.pose.position.y,
                            yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def _on_static_tf(self, robot: Robot, msg: TFMessage) -> None:
        for t in msg.transforms:
            q = t.transform.rotation
            robot.static_tf.add(
                t.header.frame_id, t.child_frame_id,
                Pose2D(t.transform.translation.x, t.transform.translation.y,
                       yaw_from_quaternion(q.x, q.y, q.z, q.w)))

    def _lidar_mount(self, robot: Robot, frame: str) -> Optional[Pose2D]:
        candidates = [frame, frame.split('/')[-1], self._lidar_fallback]
        for candidate in candidates:
            mount = robot.static_tf.lookup(self._base_frame, candidate)
            if mount is not None:
                return mount
        if not robot.warned_frame:
            self.get_logger().warn(
                f'{robot.ns}: no static transform {self._base_frame} -> {frame} yet')
            robot.warned_frame = True
        return None

    def _on_scan(self, robot: Robot, msg: LaserScan) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if robot.pose is None or stamp - robot.last_scan_time < self._scan_period:
            return
        mount = self._lidar_mount(robot, msg.header.frame_id)
        if mount is None:
            return
        robot.last_scan_time = stamp
        detection = robot.detector.detect(
            np.asarray(msg.ranges), msg.angle_min, msg.angle_increment,
            msg.range_min, msg.range_max, robot.pose.compose(mount))
        others = [r for r in self._robots.values() if r.pose is not None]
        if self._clear_footprints and detection.obstacles.size:
            # Do not map the robots themselves (their bodies plus a small margin).
            keep = np.ones(len(detection.obstacles), dtype=bool)
            for other in others:
                dx = detection.obstacles[:, 0] - other.pose.x
                dy = detection.obstacles[:, 1] - other.pose.y
                c, s = np.cos(other.pose.yaw), np.sin(other.pose.yaw)
                inside = ((np.abs(c * dx + s * dy) <= 0.5 * other.length + 0.1) &
                          (np.abs(-s * dx + c * dy) <= 0.5 * other.width + 0.1))
                keep &= ~inside
            detection.obstacles = detection.obstacles[keep]
        with self._lock:
            self._grid.update(detection.origin, detection.obstacles, detection.free_ends)
            if self._clear_footprints:
                self._grid.clear_rectangles(
                    (r.pose.x, r.pose.y, r.pose.yaw, r.length, r.width) for r in others)

    def _grid_message(self, data: np.ndarray) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        msg.info.resolution = self._grid.resolution
        msg.info.width = self._grid.width
        msg.info.height = self._grid.height
        msg.info.origin.position.x = self._grid.origin_x
        msg.info.origin.position.y = self._grid.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = data.ravel().tolist()
        return msg

    def _publish(self) -> None:
        with self._lock:
            if not self._grid.changed:
                return
            self._grid.changed = False
            values = self._grid.values()
        self._occupancy_pub.publish(self._grid_message(values))
        occupancy = values.reshape(self._grid.height, self._grid.width)
        for robot in self._robots.values():
            costs = robot.costmap.occupancy_values(occupancy, self._grid.resolution)
            robot.costmap_pub.publish(self._grid_message(costs))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExplorationMap()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
