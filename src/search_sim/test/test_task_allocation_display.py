from PIL import ImageDraw, Image
from search_msgs.msg import RobotStatus, RobotTask, TaskPlan

from search_sim.task_allocation_display import allocation_rows, font, render_panel, wrap_order


def test_rows_use_namespace_and_actual_active_goal_instead_of_first_assigned_goal():
    plan = TaskPlan()
    # Plan task order need not match the display order.
    plan.tasks = [RobotTask(robot_namespace='jackal2', frontier_ids=[9, 4]),
                  RobotTask(robot_namespace='jackal1', frontier_ids=[3, 6])]
    status = RobotStatus(robot_namespace='jackal1', state=RobotStatus.ACTIVE,
                         has_active_goal=True, active_frontier_id=6)
    rows = allocation_rows(['jackal1', 'jackal2'], ['Jackal 1', 'Jackal 2'],
                           plan, {'jackal1': status}, ['J1', 'J2'])
    assert rows[0]['order'] == [3, 6]
    assert rows[0]['active'] == 6
    assert rows[0]['labels'] == ['J11', 'J12']
    assert rows[0]['active_label'] == 'J12'
    assert rows[1]['labels'] == ['J21', 'J22']
    assert rows[1]['order'] == [9, 4]
    assert rows[1]['active'] is None
    assert rows[1]['state'] == 'Waiting'


def test_zero_frontier_is_valid_and_a_cleared_plan_clears_old_assignments():
    status = RobotStatus(state=RobotStatus.ACTIVE, has_active_goal=True, active_frontier_id=0)
    rows = allocation_rows(['jackal'], ['Jackal'], TaskPlan(), {'jackal': status})
    assert rows[0]['active'] == 0
    assert rows[0]['order'] is None
    rows = allocation_rows(['jackal'], ['Jackal'],
                           TaskPlan(tasks=[RobotTask(robot_namespace='jackal')]), {})
    assert rows[0]['order'] == []
    assert rows[0]['active'] is None


def test_long_visit_order_preserves_every_id_and_panel_grows_to_show_it():
    ids = list(range(101, 141))
    draw = ImageDraw.Draw(Image.new('RGB', (720, 1)))
    lines = wrap_order(ids, draw, font(20), 338)
    assert [int(word.strip()) for word in ' '.join(lines).split(',') if word.strip()] == ids
    plan = TaskPlan(tasks=[RobotTask(robot_namespace='jackal', frontier_ids=ids)])
    rows = allocation_rows(['jackal'], ['Jackal'], plan, {})
    image = render_panel(rows, plan, None)
    assert image.width == 720
    short_rows = allocation_rows(['jackal'], ['Jackal'],
                                 TaskPlan(tasks=[RobotTask(robot_namespace='jackal',
                                                          frontier_ids=[101])]), {})
    assert image.height > render_panel(short_rows, plan, None).height
