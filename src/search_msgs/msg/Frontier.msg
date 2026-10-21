# One WFD frontier region with the geometry used by goal creation and MRTSP.
uint64 id
# Mean world position of the frontier cells.
geometry_msgs/Point centroid
# Frontier cell nearest the centroid (O_c in Liu et al.).
geometry_msgs/Point center_point
# Cell where the inner BFS started (O_a in Liu et al.).
geometry_msgs/Point start_point
# Navigation goal: free neighbour of the frontier nearest the centroid.
geometry_msgs/Pose goal
# Number of frontier cells; the information gain S.
uint32 size
