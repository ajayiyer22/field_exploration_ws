"""Keep RMF world doors in a fixed open/closed state.

RMF demo worlds load a door plugin per door; doors stay closed until a
DoorRequest arrives on /door_requests. The exploration experiment keeps the
listed doors open (and optionally some closed) for the whole run.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rmf_door_msgs.msg import DoorMode, DoorRequest, DoorState


class DoorKeeper(Node):

    def __init__(self):
        super().__init__('door_keeper')
        self.declare_parameter('open_doors', [''])
        self.declare_parameter('closed_doors', [''])
        self.declare_parameter('request_topic', '/door_requests')
        self.declare_parameter('state_topic', '/door_states')
        self.declare_parameter('command_period', 0.2)

        self._open = [d for d in self.get_parameter('open_doors').value if d]
        self._closed = [d for d in self.get_parameter('closed_doors').value if d]
        self._doors = self._open + self._closed
        self._states = {}
        self._next = 0
        self._settled_logged = False

        # Gazebo's door plugins subscribe best-effort; one request per tick
        # avoids dropping part of a burst.
        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self._pub = self.create_publisher(
            DoorRequest, self.get_parameter('request_topic').value, qos)
        self.create_subscription(
            DoorState, self.get_parameter('state_topic').value, self._on_state, qos)
        if self._doors:
            self.create_timer(float(self.get_parameter('command_period').value), self._tick)
        self.get_logger().info(
            f'Doors open: {self._open or "none"}; closed: {self._closed or "none"}')

    def _on_state(self, msg: DoorState):
        if msg.door_name in self._doors:
            self._states[msg.door_name] = int(msg.current_mode.value)

    def _desired(self, door: str) -> int:
        return DoorMode.MODE_CLOSED if door in self._closed else DoorMode.MODE_OPEN

    def _tick(self):
        door = self._doors[self._next]
        self._next = (self._next + 1) % len(self._doors)
        request = DoorRequest()
        request.request_time = self.get_clock().now().to_msg()
        request.requester_id = 'field_exploration_door_keeper'
        request.door_name = door
        request.requested_mode.value = self._desired(door)
        self._pub.publish(request)

        settled = all(self._states.get(d) == self._desired(d) for d in self._doors)
        if settled and not self._settled_logged:
            self.get_logger().info('All doors report their requested state.')
        self._settled_logged = settled


def main(args=None):
    rclpy.init(args=args)
    node = DoorKeeper()
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
