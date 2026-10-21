"""Publish a read-only RViz image panel from the live allocator and dispatcher."""

from PIL import Image as PILImage, ImageDraw, ImageFont
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from search_msgs.msg import MissionState, RobotStatus, TaskPlan
from sensor_msgs.msg import Image


# Match task_allocator_node.cpp's marker colours, in configured team order.
ROBOT_COLOURS = [(217, 84, 26), (0, 115, 189), (120, 171, 48),
                (125, 46, 143), (237, 176, 33)]
MISSION_NAMES = {0: 'Waiting', 1: 'Exploring', 2: 'Exploration complete',
                 3: 'Target found'}


def allocation_rows(namespaces, names, plan, statuses, prefixes=None):
    """Keep robot identity stable even when plan tasks arrive in another order."""
    tasks = {} if plan is None else {t.robot_namespace: t for t in plan.tasks}
    rows = []
    prefixes = prefixes or [f"R{i + 1}" for i in range(len(namespaces))]
    for ns, name, prefix in zip(namespaces, names, prefixes):
        task, status = tasks.get(ns), statuses.get(ns)
        active = None
        state = 'Waiting'
        if status is not None:
            state = {RobotStatus.IDLE: 'Idle', RobotStatus.SENDING: 'Sending',
                     RobotStatus.ACTIVE: 'Active'}.get(status.state, 'Unknown')
            if status.has_active_goal:
                active = int(status.active_frontier_id)
        order = None if task is None else list(task.frontier_ids)
        labels = None if order is None else [f'{prefix}{i + 1}' for i in range(len(order))]
        active_label = None if active is None else (
            labels[order.index(active)] if order is not None and active in order else 'Updating')
        rows.append({'namespace': ns, 'name': name, 'state': state,
                     'active': active,
                     'order': order, 'labels': labels, 'active_label': active_label})
    return rows


def font(size):
    try:
        return ImageFont.truetype('DejaVuSans.ttf', size)
    except OSError:
        return ImageFont.load_default(size=size)


def wrap_order(ids, draw, text_font, width):
    if ids is None:
        return ['Waiting for plan']
    if not ids:
        return ['None']
    lines, current = [], ''
    for frontier in ids:
        candidate = f'{current},{frontier}' if current else str(frontier)
        if current and draw.textlength(candidate, font=text_font) > width:
            lines.append(current + ',')
            current = str(frontier)
        else:
            current = candidate
    return lines + [current]


def render_panel(rows, plan, mission):
    """Render every assigned goal label, growing rows for long queues."""
    width = 720
    body, small, title = font(24), font(18), font(24)
    measuring = ImageDraw.Draw(PILImage.new('RGB', (width, 1)))
    orders = [wrap_order(row['labels'], measuring, body, 365) for row in rows]
    heights = [max(38, len(lines) * 28 + 10) for lines in orders]
    image = PILImage.new('RGB', (width, 62 + sum(heights) + 24), '#f5f7fa')
    draw = ImageDraw.Draw(image)
    draw.text((14, 5), 'Task allocation', font=title, fill='#182230')
    state = 'Waiting for exploration' if mission is None else MISSION_NAMES.get(
        mission.state, 'Unknown mission state')
    summary = state if plan is None else (
        f'{state}   |   Plan {plan.generation}   |   '
        f'Unassigned: {len(plan.unassigned_frontier_ids)}')
    draw.text((230, 10), summary, font=small, fill='#465467')
    draw.rectangle((0, 36, width, 62), fill='#e4e9ef')
    for x, label in [(14, 'Robot'), (145, 'Navigation'), (255, 'Active'),
                     (345, 'Assigned visit order')]:
        draw.text((x, 39), label, font=small, fill='#182230')
    y = 62
    for i, (row, lines, height) in enumerate(zip(rows, orders, heights)):
        colour = ROBOT_COLOURS[i % len(ROBOT_COLOURS)]
        draw.rectangle((0, y, 7, y + height - 1), fill=colour)
        draw.text((15, y + 4), row['name'], font=body, fill=colour)
        draw.text((145, y + 8), row['state'], font=small, fill='#465467')
        draw.text((255, y + 4), 'None' if row['active_label'] is None else row['active_label'],
                  font=body, fill='#182230')
        for j, line in enumerate(lines):
            draw.text((345, y + 4 + 28 * j), line, font=body, fill='#182230')
        y += height
        draw.line((8, y - 1, width, y - 1), fill='#d5dce5')
    draw.text((14, y + 5), 'Goal labels follow the current visit order; H = Husky, J1/J2 = Jackals.',
              font=font(16), fill='#465467')
    return image


class TaskAllocationDisplay(Node):
    def __init__(self):
        super().__init__('task_allocation_display')
        self.declare_parameter('robot_namespaces', ['a200_0000'])
        self.declare_parameter('robot_names', ['Husky'])
        self.namespaces = list(self.get_parameter('robot_namespaces').value)
        self.names = list(self.get_parameter('robot_names').value)
        self.declare_parameter('robot_goal_prefixes',
                               [f'R{i + 1}' for i in range(len(self.namespaces))])
        self.prefixes = list(self.get_parameter('robot_goal_prefixes').value)
        if len(self.prefixes) != len(self.namespaces) or not all(self.prefixes):
            raise ValueError('robot_goal_prefixes needs one nonempty prefix per robot')
        if len(self.namespaces) != len(self.names):
            raise ValueError('robot_names needs one name per robot_namespace')
        self.plan, self.mission, self.statuses = None, None, {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.panel_pub = self.create_publisher(Image, '/search/task_allocation/image', latched)
        self.plan_sub = self.create_subscription(TaskPlan, '/search/task_plan', self.on_plan, latched)
        self.mission_sub = self.create_subscription(
            MissionState, '/search/mission_state', self.on_mission, latched)
        self.status_sub = self.create_subscription(
            RobotStatus, '/search/robot_status', self.on_status, 20)
        self.create_timer(0.5, self.publish_panel)
        self.publish_panel()

    def on_plan(self, msg):
        self.plan = msg

    def on_mission(self, msg):
        self.mission = msg

    def on_status(self, msg):
        self.statuses[msg.robot_namespace] = msg

    def publish_panel(self):
        rows = allocation_rows(self.namespaces, self.names, self.plan, self.statuses, self.prefixes)
        panel = render_panel(rows, self.plan, self.mission)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.width, msg.height = panel.size
        msg.encoding, msg.step = 'rgb8', panel.width * 3
        msg.data = panel.tobytes()
        self.panel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TaskAllocationDisplay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
