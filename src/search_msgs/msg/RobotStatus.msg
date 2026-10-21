uint8 IDLE=0
uint8 SENDING=1
uint8 ACTIVE=2

std_msgs/Header header
string robot_namespace
uint8 state
bool has_active_goal
uint64 active_frontier_id
geometry_msgs/Pose active_goal
