# One robot/frontier constraint published by the exploration dispatcher.
#   blocked = true                  the allocator must not give this frontier to this robot
#                                   (navigation failure, or reason "active_goal_reserved_by:<ns>")
#   blocked = false, reason
#   "active_goal_owned_by:<ns>"     this robot is executing the frontier; keep it there
# frontier_id = 0, blocked = true, reason "robot_nav2_unavailable:<failure>"
# excludes this robot from every frontier until the dispatcher removes it.
string robot_namespace
uint64 frontier_id
bool blocked
string reason
