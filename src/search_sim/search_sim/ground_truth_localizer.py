"""Ground-truth localization for every robot from Gazebo model poses.

Robot localization is treated as solved: the Gazebo pose of each robot model is
the robot pose in the common `map` frame. For robot `<ns>` this node

  * publishes `/<ns>/ground_truth/pose` (PoseStamped, frame `map`), and
  * broadcasts `map -> odom` on `/<ns>/tf`, computed so that
    map->odom * odom->base_link equals the Gazebo pose, using the robot's own
    filtered odometry. Nav2 for that robot then sees a `map` frame.

Poses come straight from Gazebo transport (`/world/<world>/dynamic_pose/info`),
because Harmonic's SceneBroadcaster sets Pose.name but ros_gz_bridge's TF
conversion drops it.
"""

import math
import threading
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage

try:
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node as GazeboNode
    GAZEBO_IMPORT_ERROR = None
except ImportError as error:  # pragma: no cover - only for unit tests without Gazebo
    Pose_V = None
    GazeboNode = None
    # gz.msgs10 also needs the protobuf Python package.
    GAZEBO_IMPORT_ERROR = error


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def map_to_odom_2d(
    map_to_base: Tuple[float, float, float],
    odom_to_base: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """map->odom = map->base * inverse(odom->base), in SE(2)."""
    map_x, map_y, map_yaw = map_to_base
    odom_x, odom_y, odom_yaw = odom_to_base
    yaw = math.atan2(math.sin(map_yaw - odom_yaw), math.cos(map_yaw - odom_yaw))
    c, s = math.cos(yaw), math.sin(yaw)
    return map_x - (c * odom_x - s * odom_y), map_y - (s * odom_x + c * odom_y), yaw


class GroundTruthLocalizer(Node):

    def __init__(self) -> None:
        super().__init__('ground_truth_localizer')
        self.declare_parameter('world', 'sim_world')
        self.declare_parameter('robot_namespaces', ['a200_0000'])
        self.declare_parameter('model_suffix', '/robot')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('odom_topic', 'platform/odom/filtered')

        self._world = str(self.get_parameter('world').value)
        self._namespaces = [str(n).strip('/') for n in self.get_parameter('robot_namespaces').value]
        suffix = str(self.get_parameter('model_suffix').value)
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        odom_topic = str(self.get_parameter('odom_topic').value).strip('/')

        self._models: Dict[str, str] = {ns + suffix: ns for ns in self._namespaces}
        self._odometry: Dict[str, Optional[Odometry]] = {ns: None for ns in self._namespaces}
        self._lock = threading.Lock()
        self._pose_pubs = {}
        self._tf_pubs = {}
        for ns in self._namespaces:
            self._pose_pubs[ns] = self.create_publisher(
                PoseStamped, f'/{ns}/ground_truth/pose', qos_profile_sensor_data)
            self._tf_pubs[ns] = self.create_publisher(TFMessage, f'/{ns}/tf', 100)
            self.create_subscription(
                Odometry, f'/{ns}/{odom_topic}',
                lambda msg, ns=ns: self._on_odometry(ns, msg), qos_profile_sensor_data)

        if GazeboNode is None:
            raise RuntimeError(f'Gazebo Python bindings are not available: {GAZEBO_IMPORT_ERROR}')
        self._transport = GazeboNode()
        self._topic = f'/world/{self._world}/dynamic_pose/info'
        if not self._transport.subscribe(Pose_V, self._topic, self._on_poses):
            raise RuntimeError(f'Cannot subscribe to Gazebo poses on {self._topic}')
        self.get_logger().info(
            f'Ground truth from {self._topic} for models {sorted(self._models)}')

    def _on_odometry(self, ns: str, msg: Odometry) -> None:
        with self._lock:
            self._odometry[ns] = msg

    def _on_poses(self, poses) -> None:
        for pose in poses.pose:
            ns = self._models.get(pose.name.lstrip('/'))
            if ns is None:
                continue
            out = PoseStamped()
            out.header.frame_id = self._map_frame
            out.header.stamp.sec = poses.header.stamp.sec
            out.header.stamp.nanosec = poses.header.stamp.nsec
            out.pose.position.x = pose.position.x
            out.pose.position.y = pose.position.y
            out.pose.position.z = pose.position.z
            out.pose.orientation.x = pose.orientation.x
            out.pose.orientation.y = pose.orientation.y
            out.pose.orientation.z = pose.orientation.z
            out.pose.orientation.w = pose.orientation.w
            self._pose_pubs[ns].publish(out)
            self._publish_correction(ns, out)

    def _publish_correction(self, ns: str, ground_truth: PoseStamped) -> None:
        with self._lock:
            odometry = self._odometry[ns]
        if odometry is None:
            return
        gt = ground_truth.pose
        od = odometry.pose.pose
        x, y, yaw = map_to_odom_2d(
            (gt.position.x, gt.position.y, yaw_from_quaternion(
                gt.orientation.x, gt.orientation.y, gt.orientation.z, gt.orientation.w)),
            (od.position.x, od.position.y, yaw_from_quaternion(
                od.orientation.x, od.orientation.y, od.orientation.z, od.orientation.w)),
        )
        transform = TransformStamped()
        transform.header.stamp = ground_truth.header.stamp
        if transform.header.stamp.sec == 0 and transform.header.stamp.nanosec == 0:
            transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._map_frame
        transform.child_frame_id = self._odom_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        q = yaw_quaternion(yaw)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self._tf_pubs[ns].publish(TFMessage(transforms=[transform]))

    def destroy_node(self):
        try:
            self._transport.unsubscribe(self._topic)
        except Exception:  # noqa: BLE001 - best effort on shutdown
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruthLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
