# Frontiers owned by one robot (Hungarian), in distance order.
string robot_namespace
uint64[] frontier_ids
geometry_msgs/Pose[] goals
# Hungarian cost C_ij = D_ij of each owned frontier, aligned with frontier_ids.
float64[] assignment_costs
