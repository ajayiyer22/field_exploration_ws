"""Exercise the real nodes with an isolated ROS graph and mock Nav2 servers."""

import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from search_msgs.msg import (
    Frontier, FrontierArray, MissionState, RobotFrontierConstraintArray,
    RobotStatus, RobotTask, TaskPlan, RobotFrontierConstraint,
)


class Graph:
    def __init__(self, directory):
        # Keep tests separate from any running simulation, including ROS logs.
        self.env = dict(os.environ, ROS_DOMAIN_ID=str(170 + os.getpid() % 30),
                        ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(directory / 'ros_logs'))
        self.context = Context()
        rclpy.init(context=self.context, domain_id=int(self.env['ROS_DOMAIN_ID']))
        self.node = rclpy.create_node(
            'runtime_test', context=self.context,
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.executor = MultiThreadedExecutor(num_threads=4, context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.directory = directory
        self.latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.clock = self.node.create_publisher(Clock, '/clock', 10)
        self.t = 10.0
        self.publishers = {}
        self.subscriptions = []
        self.processes = []
        self.servers = []
        self.handles = []
        self.closing = False
        self.abort_requested = False
        self.abort_robots = set()
        self.refresh = lambda: None

    def stamp(self):
        msg = Clock()
        msg.clock.sec = int(self.t)
        msg.clock.nanosec = int(round((self.t - int(self.t)) * 1e9))
        if msg.clock.nanosec == 1000000000:
            msg.clock.sec += 1
            msg.clock.nanosec = 0
        return msg.clock

    def publisher(self, kind, topic, qos=None):
        if topic not in self.publishers:
            self.publishers[topic] = self.node.create_publisher(
                kind, topic, self.latched if qos is None else qos)
        return self.publishers[topic]

    def watch(self, kind, topic, qos=None):
        messages = []
        self.subscriptions.append(self.node.create_subscription(
            kind, topic, messages.append, self.latched if qos is None else qos))
        return messages

    def launch(self, executable, **parameters):
        binary_dir = Path(os.environ['SEARCH_EXPLORATION_BIN_DIR'])
        argv = [str(binary_dir / executable), '--ros-args', '-p', 'use_sim_time:=true',
                '-p', 'robot_namespaces:=[test_robot]']
        for name, value in parameters.items():
            argv += ['-p', f'{name}:={str(value).lower() if isinstance(value, bool) else value}']
        log = open(self.directory / f'{executable}.log', 'w')
        self.handles.append(log)
        self.processes.append(subprocess.Popen(argv, env=self.env, stdout=log, stderr=log))

    def pump(self, advance=0.1):
        self.t += advance
        self.clock.publish(Clock(clock=self.stamp()))
        self.refresh()
        time.sleep(0.025)
        for process in self.processes:
            assert process.poll() is None, self.logs()

    def until(self, predicate, message, timeout=6.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.pump()
        assert predicate(), message + '\n' + self.logs()

    def advance(self, duration):
        end = self.t + duration
        while self.t < end:
            self.pump()

    def logs(self):
        return '\n'.join(p.read_text() for p in self.directory.glob('*.log'))

    def pose(self, robot="test_robot", x=0.0):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.stamp()
        msg.pose.orientation.w = 1.0
        msg.pose.position.x = x
        self.publisher(PoseStamped, f'/{robot}/ground_truth/pose',
                       qos_profile_sensor_data).publish(msg)

    def costmap(self, cost=0, local_cost=None, local_origin=0.0, robot="test_robot"):
        msg = Costmap()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.stamp()
        msg.metadata.size_x = msg.metadata.size_y = 10
        msg.metadata.resolution = 1.0
        msg.metadata.origin.orientation.w = 1.0
        msg.data = [cost] * 100
        self.publisher(Costmap, f'/{robot}/global_costmap/costmap_raw').publish(msg)
        msg.metadata.origin.position.x = local_origin
        msg.metadata.origin.position.y = local_origin
        msg.data = [cost if local_cost is None else local_cost] * 100
        self.publisher(Costmap, f'/{robot}/local_costmap/costmap_raw').publish(msg)

    def frontiers(self, generation=1, ids=(1,)):
        msg = FrontierArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.stamp()
        msg.generation = generation
        for frontier_id in ids:
            frontier = Frontier(id=frontier_id, size=20)
            frontier.goal.position.x = float(frontier_id)
            frontier.goal.position.y = 1.0
            frontier.goal.orientation.w = 1.0
            frontier.centroid = frontier.goal.position
            frontier.center_point = frontier.goal.position
            frontier.start_point = frontier.goal.position
            msg.frontiers.append(frontier)
        self.publisher(FrontierArray, '/search/frontiers').publish(msg)
        return msg

    def plan(self, generation, frontier_generation, ids=()):
        msg = TaskPlan(generation=generation, frontier_generation=frontier_generation)
        msg.header.frame_id = 'map'
        msg.header.stamp = self.stamp()
        task = RobotTask(robot_namespace='test_robot', frontier_ids=list(ids))
        for frontier_id in ids:
            frontier = Frontier()
            frontier.goal.position.x = float(frontier_id)
            frontier.goal.position.y = 1.0
            frontier.goal.orientation.w = 1.0
            task.goals.append(frontier.goal)
            task.assignment_costs.append(1.0)
        msg.tasks = [task]
        self.publisher(TaskPlan, '/search/task_plan').publish(msg)

    def mission(self, state):
        msg = MissionState(state=state)
        msg.header.stamp = self.stamp()
        self.publisher(MissionState, '/search/mission_state').publish(msg)

    def nav2(self, robot="test_robot"):
        goals = []

        def execute(handle):
            goals.append(handle)
            while (not self.closing and not self.abort_requested and robot not in self.abort_robots
                   and not handle.is_cancel_requested):
                time.sleep(0.01)
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
            return NavigateToPose.Result()

        self.servers.append(ActionServer(
            self.node, NavigateToPose, f'/{robot}/navigate_to_pose', execute,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup()))
        return goals

    def close(self):
        self.closing = True
        for process in self.processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
        for server in self.servers:
            server.destroy()
        self.executor.shutdown(timeout_sec=3)
        self.thread.join(timeout=3)
        self.node.destroy_node()
        self.context.shutdown()
        for handle in self.handles:
            handle.close()


@pytest.fixture
def graph(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    graph = Graph(tmp_path)
    try:
        yield graph
    finally:
        graph.close()


def test_allocator_reacts_to_eligibility_without_frontier_change(graph):
    plans = graph.watch(TaskPlan, '/search/task_plan')
    graph.launch('task_allocator_node', periodic_reallocation_s=60.0,
                 require_mission_exploring=False)
    cost = 254
    graph.refresh = lambda: (graph.pose(), graph.costmap(cost))
    graph.frontiers(ids=(1, 2))
    graph.mission(MissionState.EXPLORING)
    graph.until(lambda: plans, 'initial allocation missing')
    assert list(plans[-1].unassigned_frontier_ids) == [1, 2]
    cost = 0
    graph.until(lambda: plans[-1].tasks[0].frontier_ids, 'eligible goals not allocated')
    assert plans[-1].trigger == 'goal_costmap_eligibility_changed'
    generation = plans[-1].generation
    graph.frontiers(generation=2, ids=(2, 1))
    graph.advance(1.0)
    assert plans[-1].generation == generation, 'reordering caused allocation churn'
    # A paused simulation must not continue executing the allocator timer.
    for _ in range(5):
        graph.pump(advance=0.0)
    generation = plans[-1].generation
    cost = 254
    for _ in range(10):
        graph.pump(advance=0.0)
    assert plans[-1].generation == generation, 'allocator ran while simulation was paused'
    graph.until(lambda: not plans[-1].tasks[0].frontier_ids, 'cost increase was not detected')
    cost = 0
    graph.until(lambda: plans[-1].tasks[0].frontier_ids, 'cost recovery not allocated')
    # Losing freshness also changes eligibility, without a frontier publication.
    graph.refresh = graph.pose
    graph.until(lambda: list(plans[-1].unassigned_frontier_ids) == [2, 1],
                'stale costmap did not invalidate allocation')
    assert plans[-1].trigger == 'goal_costmap_eligibility_changed'
    cost = 0
    graph.refresh = lambda: (graph.pose(), graph.costmap(cost))
    graph.until(lambda: plans[-1].tasks[0].frontier_ids, 'freshness recovery not allocated')
    # A retained terminal event stops solving even if readiness gating is disabled.
    graph.mission(MissionState.TARGET_FOUND)
    graph.advance(0.5)
    generation = plans[-1].generation
    cost = 254
    graph.advance(1.0)
    assert plans[-1].generation == generation


def test_allocator_accepts_29_rejects_30_and_recovers_at_zero(graph):
    plans = graph.watch(TaskPlan, '/search/task_plan')
    graph.launch('task_allocator_node', periodic_reallocation_s=60.0,
                 require_mission_exploring=False)
    cost = 29
    graph.refresh = lambda: (graph.pose(), graph.costmap(cost))
    graph.frontiers(ids=(1, 2))
    graph.until(lambda: plans and len(plans[-1].tasks[0].frontier_ids) == 2,
                'cost 29 must be eligible')
    cost = 30
    graph.until(lambda: set(plans[-1].unassigned_frontier_ids) == {1, 2},
                'cost 30 must be rejected')
    for cost in (253, 254, 255):
        graph.advance(0.5)
        assert not plans[-1].tasks[0].frontier_ids
    cost = 0
    graph.until(lambda: len(plans[-1].tasks[0].frontier_ids) == 2,
                'free endpoints must recover without a new frontier snapshot')


def test_dispatcher_checks_cost_limit_before_sending_and_cancels_on_increase(graph):
    goals = graph.nav2()
    statuses = graph.watch(RobotStatus, '/search/robot_status', 20)
    graph.launch('exploration_dispatcher_node')
    cost = 30
    graph.refresh = lambda: graph.costmap(cost)
    graph.frontiers()
    graph.mission(MissionState.EXPLORING)
    graph.plan(1, 1, (1,))
    graph.until(lambda: statuses, 'dispatcher status missing')
    graph.advance(1.0)
    assert not goals, 'cost 30 endpoint was sent to Nav2'
    cost = 29
    graph.plan(2, 1, (1,))
    graph.until(lambda: len(goals) == 1, 'cost 29 goal was not sent')
    cost = 30
    graph.until(lambda: not goals[0].is_active, 'cost increase did not cancel active goal')


def test_detector_moves_goal_to_low_cost_neighbour_and_keeps_blocked_frontier(graph):
    from visualization_msgs.msg import MarkerArray

    frontiers = graph.watch(FrontierArray, '/search/frontiers')
    markers = graph.watch(MarkerArray, '/search/frontiers/markers')
    graph.launch('frontier_detector_node', period_s=0.05, min_frontier_size_cells=1)
    endpoint_cost = 29

    def refresh():
        graph.pose()
        occupancy = OccupancyGrid()
        occupancy.header.frame_id = 'map'
        occupancy.header.stamp = graph.stamp()
        occupancy.info.width = occupancy.info.height = 10
        occupancy.info.resolution = 1.0
        occupancy.info.origin.orientation.w = 1.0
        occupancy.data = [0] * 50 + [-1] * 50
        graph.publisher(OccupancyGrid, '/exploration_map/occupancy').publish(occupancy)
        costmap = Costmap()
        costmap.header = occupancy.header
        costmap.metadata.size_x = costmap.metadata.size_y = 10
        costmap.metadata.resolution = 1.0
        costmap.metadata.origin.orientation.w = 1.0
        costmap.data = [30] * 100
        costmap.data[41] = endpoint_cost
        graph.publisher(Costmap, '/test_robot/global_costmap/costmap_raw').publish(costmap)
        graph.publisher(Costmap, '/test_robot/local_costmap/costmap_raw').publish(costmap)

    graph.refresh = refresh
    graph.until(lambda: frontiers and len(frontiers[-1].frontiers) == 1 and
                frontiers[-1].frontiers[0].goal.position.x == 1.5,
                'detector did not choose the low-cost free neighbour')
    assert frontiers[-1].frontiers[0].goal.position.y == 4.5
    eligible_generation = frontiers[-1].generation
    endpoint_cost = 30
    graph.until(lambda: frontiers[-1].generation > eligible_generation and
                frontiers[-1].frontiers[0].goal.position.x != 1.5 and markers and
                any(m.ns == 'frontier_goals' and not m.points for m in markers[-1].markers),
                'blocked frontier must not display an admissible goal marker')
    assert len(frontiers[-1].frontiers) == 1, 'blocked frontier disappeared from mission accounting'
    generation = frontiers[-1].generation
    # Costmap updates alone must permit recovery, even when occupancy is unchanged.
    def costs_only():
        graph.pose()
        costmap = Costmap()
        costmap.header.frame_id = 'map'
        costmap.header.stamp = graph.stamp()
        costmap.metadata.size_x = costmap.metadata.size_y = 10
        costmap.metadata.resolution = 1.0
        costmap.metadata.origin.orientation.w = 1.0
        costmap.data = [30] * 100
        costmap.data[41] = 29
        graph.publisher(Costmap, '/test_robot/global_costmap/costmap_raw').publish(costmap)
        graph.publisher(Costmap, '/test_robot/local_costmap/costmap_raw').publish(costmap)
    graph.refresh = costs_only
    graph.until(lambda: frontiers[-1].generation > generation and
                frontiers[-1].frontiers[0].goal.position.x == 1.5,
                'costmap-only recovery was not detected')


def test_mission_requires_empty_frontiers_current_plan_and_fresh_idle_status(graph):
    states = graph.watch(MissionState, '/search/mission_state')
    graph.nav2()
    graph.launch('mission_coordinator_node', start_delay_s=0.0, complete_after_s=2.0,
                 observation_timeout_s=1.0, results_dir=str(graph.directory / 'results'))
    frontier_generation = 1
    frontier_ids = (1,)
    status_enabled = True
    status_busy = False

    def refresh():
        nonlocal frontier_generation
        graph.pose()
        grid = OccupancyGrid()
        grid.header.stamp = graph.stamp()
        grid.header.frame_id = 'map'
        grid.info.resolution = 1.0
        grid.info.width = grid.info.height = 1
        grid.info.origin.orientation.w = 1.0
        grid.data = [0]
        graph.publisher(OccupancyGrid, '/exploration_map/occupancy').publish(grid)
        frontier_generation += 1
        graph.frontiers(frontier_generation, frontier_ids)
        if status_enabled:
            status = RobotStatus(robot_namespace='test_robot', state=RobotStatus.IDLE,
                                 has_active_goal=status_busy)
            status.header.stamp = graph.stamp()
            graph.publisher(RobotStatus, '/search/robot_status', 20).publish(status)

    graph.refresh = refresh
    graph.until(lambda: any(s.state == MissionState.EXPLORING for s in states),
                'mission did not start')
    graph.plan(1, 1)
    graph.advance(35.0)
    assert states[-1].state == MissionState.EXPLORING, 'temporary blocks ended mission'
    first_empty_generation = frontier_generation + 1
    frontier_ids = ()
    graph.advance(3.0)
    assert states[-1].state == MissionState.EXPLORING, 'stale plan ended mission'
    # Even an IDLE status cannot complete while it reports an active goal.
    status_busy = True
    graph.plan(2, first_empty_generation)
    graph.advance(3.0)
    assert states[-1].state == MissionState.EXPLORING
    status_busy = False
    graph.advance(0.5)
    status_enabled = False
    graph.advance(3.0)
    assert states[-1].state == MissionState.EXPLORING, 'missing status ended mission'
    status_enabled = True
    status_busy = False
    graph.until(lambda: states[-1].state == MissionState.EXPLORATION_COMPLETE,
                'verified empty mission did not complete')
    assert len(list((graph.directory / 'results').glob('*.csv'))) == 1


def test_dispatcher_keeps_active_goal_and_cancels_missing_frontier(graph):
    goals = graph.nav2()
    constraints = graph.watch(RobotFrontierConstraintArray, '/search/allocation_constraints')
    graph.launch('exploration_dispatcher_node')
    graph.refresh = lambda: graph.costmap()
    graph.frontiers()
    graph.mission(MissionState.EXPLORING)
    graph.plan(1, 1, (1,))
    graph.until(lambda: len(goals) == 1, 'Nav2 goal not sent')
    graph.until(lambda: constraints and any(c.reason.startswith('active_goal_owned_by:')
                            for c in constraints[-1].constraints), 'ownership not published')
    graph.plan(2, 1, (1,))
    graph.advance(1.0)
    assert len(goals) == 1 and goals[0].is_active, 'new plan replaced valid active goal'
    graph.frontiers(2, ())
    graph.advance(0.5)
    assert goals[0].is_active, 'one missing snapshot canceled goal'
    graph.frontiers(3, ())
    graph.until(lambda: not goals[0].is_active, 'two missing snapshots did not cancel goal')


def test_dispatcher_ignores_zero_feedback_and_refreshes_once(graph):
    from std_msgs.msg import String

    goals = graph.nav2()
    requests = graph.watch(String, '/search/reallocation_request')
    graph.launch('exploration_dispatcher_node')
    graph.refresh = lambda: graph.costmap()
    graph.frontiers()
    graph.mission(MissionState.EXPLORING)
    graph.plan(1, 1, (1,))
    graph.until(lambda: len(goals) == 1, 'Nav2 goal not sent')
    goals[0].publish_feedback(NavigateToPose.Feedback(distance_remaining=0.0))
    graph.advance(0.5)
    assert not any(r.data.startswith('approaching_target:') for r in requests)
    goals[0].publish_feedback(NavigateToPose.Feedback(distance_remaining=1.0))
    graph.until(lambda: any(r.data.startswith('approaching_target:') for r in requests),
                'positive approach feedback did not refresh')
    goals[0].publish_feedback(NavigateToPose.Feedback(distance_remaining=0.5))
    graph.advance(0.5)
    assert sum(r.data.startswith('approaching_target:') for r in requests) == 1
    graph.mission(MissionState.TARGET_FOUND)
    graph.until(lambda: not goals[0].is_active, 'terminal mission did not cancel goal')


def test_dispatcher_excludes_failed_robot_then_retries_after_expiry(graph):
    goals = graph.nav2()
    constraints = graph.watch(RobotFrontierConstraintArray, '/search/allocation_constraints')
    graph.launch('exploration_dispatcher_node', failure_timeout_s=2.0)
    graph.refresh = lambda: graph.costmap()
    graph.frontiers()
    graph.mission(MissionState.EXPLORING)
    graph.plan(1, 1, (1,))
    graph.until(lambda: len(goals) == 1, 'Nav2 goal not sent')
    graph.abort_requested = True
    graph.until(lambda: constraints and any(c.blocked and c.frontier_id == 0 and
                                           c.reason == 'robot_nav2_unavailable:navigate_to_pose_aborted'
                                           for c in constraints[-1].constraints),
                'aborted goal did not exclude the robot')
    graph.abort_requested = False
    graph.plan(2, 1, (1,))
    graph.advance(0.5)
    assert len(goals) == 1, 'failed pair retried before block expired'
    graph.until(lambda: len(goals) == 2, 'failed pair not retried after expiry')
    graph.mission(MissionState.EXPLORATION_COMPLETE)
    graph.until(lambda: not goals[1].is_active, 'completion did not cancel retried goal')


def test_common_global_gate_ignores_local_costs(graph):
    plans = graph.watch(TaskPlan, '/search/task_plan')
    graph.launch('task_allocator_node', robot_namespaces='[r0,r1]',
                 periodic_reallocation_s=60.0, require_mission_exploring=False)
    other_cost = 30

    def refresh():
        for robot in ('r0', 'r1'):
            graph.pose(robot)
            graph.costmap(0 if robot == 'r0' else other_cost, local_cost=254, robot=robot)
    graph.refresh = refresh
    graph.frontiers(ids=(1, 2))
    graph.until(lambda: plans, 'allocation missing')
    assert set(plans[-1].unassigned_frontier_ids) == {1, 2}
    other_cost = 29
    graph.until(lambda: sum(len(t.frontier_ids) for t in plans[-1].tasks) == 2,
                'global cost 29 must pass even when local cost is lethal')
    other_cost = 30
    graph.until(lambda: set(plans[-1].unassigned_frontier_ids) == {1, 2},
                'a goal must pass every global map, including non-owning robots')


def test_nav2_failure_reallocates_entire_queue_to_two_healthy_robots(graph):
    plans = graph.watch(TaskPlan, '/search/task_plan')
    constraints = graph.watch(RobotFrontierConstraintArray, '/search/allocation_constraints')
    team = ('r0', 'r1', 'r2')
    goals = {robot: graph.nav2(robot) for robot in team}
    graph.launch('task_allocator_node', robot_namespaces='[r0,r1,r2]',
                 periodic_reallocation_s=60.0, require_mission_exploring=False)
    graph.launch('exploration_dispatcher_node', robot_namespaces='[r0,r1,r2]',
                 failure_timeout_s=30.0)

    def refresh():
        for index, robot in enumerate(team):
            graph.pose(robot, x=index * 4.0)
            graph.costmap(0, robot=robot)
    graph.refresh = refresh
    graph.frontiers(ids=(1, 2, 3, 4, 5, 6))
    graph.mission(MissionState.EXPLORING)
    graph.until(lambda: all(goals.values()), 'all three robots must receive a Nav2 goal')
    assert [len(t.frontier_ids) for t in plans[-1].tasks] == [2, 2, 2]
    failed_ids = set(plans[-1].tasks[0].frontier_ids)
    healthy_handles = [goals['r1'][0], goals['r2'][0]]
    graph.abort_robots.add('r0')
    graph.until(lambda: constraints and any(c.robot_namespace == 'r0' and
                c.frontier_id == 0 and c.reason.startswith('robot_nav2_unavailable:')
                for c in constraints[-1].constraints), 'robot-wide failure not published')
    graph.until(lambda: not plans[-1].tasks[0].frontier_ids and
                sum(len(t.frontier_ids) for t in plans[-1].tasks) == 6,
                'failed robot queue was not fully reassigned to the two healthy robots')
    assigned = [fid for task in plans[-1].tasks[1:] for fid in task.frontier_ids]
    assert set(assigned) == {1, 2, 3, 4, 5, 6} and len(assigned) == len(set(assigned))
    assert failed_ids <= set(assigned)
    assert [len(t.frontier_ids) for t in plans[-1].tasks] == [0, 3, 3]
    assert all(handle.is_active for handle in healthy_handles)
    graph.abort_robots.clear()
    graph.advance(5.0)
    assert len(goals['r0']) == 1, 'unavailable robot was sent another frontier'
    # A disappearing failed frontier must not make the whole robot available early.
    graph.frontiers(generation=2, ids=(2, 3, 4, 5, 6))
    graph.advance(1.0)
    assert not plans[-1].tasks[0].frontier_ids
    graph.until(lambda: plans[-1].tasks[0].frontier_ids,
                'robot did not rejoin after its timeout', timeout=12.0)


def test_no_available_robots_keeps_all_frontiers_unassigned(graph):
    plans = graph.watch(TaskPlan, '/search/task_plan')
    graph.launch('task_allocator_node', robot_namespaces='[r0,r1,r2]',
                 require_mission_exploring=False)
    graph.refresh = lambda: [(graph.pose(robot), graph.costmap(0, robot=robot))
                             for robot in ('r0', 'r1', 'r2')]
    graph.frontiers(ids=(1, 2, 3))
    graph.until(lambda: plans, 'initial plan missing')
    blocked = RobotFrontierConstraintArray()
    blocked.constraints = [RobotFrontierConstraint(robot_namespace=robot, frontier_id=0,
        blocked=True, reason='robot_nav2_unavailable:navigate_to_pose_aborted')
        for robot in ('r0', 'r1', 'r2')]
    graph.publisher(RobotFrontierConstraintArray, '/search/allocation_constraints').publish(blocked)
    graph.until(lambda: set(plans[-1].unassigned_frontier_ids) == {1, 2, 3},
                'all unavailable robots must leave work unassigned')
    assert all(not task.frontier_ids for task in plans[-1].tasks)
