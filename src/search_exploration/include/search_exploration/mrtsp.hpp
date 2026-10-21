// MRTSP frontier ordering, ported unchanged from the field_robotics workspace
// (frontier_exploration_ros2: mrtsp_ordering.cpp, mrtsp_solver.cpp and
// FrontierAllocationMrtspNode::mrtsp_order).
//
// Based on C. Liu et al., "Enhancing autonomous exploration for robotics via
// real time map optimization and improved frontier costs", Scientific Reports
// 15:12261 (2025), eqs. (5)-(8), with the old workspace's start-row form:
//
//   d(V_i, V_j) = max(d_m + d_u, d_n + d_v) - r_s
//       d_m: V_i center -> V_j center      d_u: V_j center   -> V_j start
//       d_n: V_i center -> V_j centroid    d_v: V_j centroid -> V_j start
//       r_s: effective sensor range
//   G_j     = S_j ^ W_s                    (S_j = frontier size; G <= 0 is infeasible)
//   M(i, j) = W_d d(V_i, V_j) / G_j                              i != 0, j != 0
//   M(0, j) = W_d d(V_0, V_j) / G_j + t_lb(V_0, V_j) / sqrt(G_j)  j != 0
//   M(i, 0) = 0, M(i, i) = inf
//   t_lb    = L / V_max + min(|Q_j|, 2 pi - |Q_j|) / W_max
//
// Node 0 is the robot. From the robot, d_m and L use robot_center_distance when
// it is set (the allocator passes the Hungarian cost C_ij), otherwise the
// Euclidean robot -> V_j center distance.
//
// Solver: the frontiers are ranked by M(0, j) and the best dp_candidate_limit
// are kept; greedy or bounded-horizon DP orders that pool (DP falls back to
// greedy when it finds no route); frontiers outside the order are appended in
// input order.
#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "search_exploration/grid.hpp"

namespace search_exploration
{

// The DP state is a 64-bit visited mask; pools are capped below that.
constexpr int kMaxDpCandidateLimit = 60;

struct MrtspFrontier
{
  Point2 centroid;      // O_b
  Point2 center_point;  // O_c
  Point2 start_point;   // O_a
  double size{1.0};     // S
  // Robot -> frontier distance used for d_m and L; unset = |robot - center_point|.
  std::optional<double> robot_center_distance;
};

struct MrtspRobot
{
  Point2 position;
  double yaw{0.0};
  double v_max{0.5};
  double w_max{1.0};
};

struct MrtspWeights
{
  double w_d{1.0};
  double w_s{1.0};
  double sensor_range{1.5};  // r_s
};

enum class MrtspSolver {GREEDY, DP};

struct MrtspOptions
{
  MrtspWeights weights;
  MrtspSolver solver{MrtspSolver::DP};
  int dp_candidate_limit{12};
  int dp_horizon{8};
};

using MrtspMatrix = std::vector<std::vector<double>>;

// G_j = S_j ^ W_s (S_j when W_s = 1).
double information_gain(const MrtspFrontier & frontier, const MrtspWeights & weights);

// d(V_i, V_j) between two frontiers; `from_center` is V_i's center point.
double path_cost(const Point2 & from_center, const MrtspFrontier & to, double sensor_range);

// d(V_0, V_j) from the robot, using robot_center_distance for d_m when set.
double initial_path_cost(const Point2 & robot, const MrtspFrontier & to, double sensor_range);

// t_lb(V_0, V_j), using robot_center_distance for L when set.
double time_lower_bound(const MrtspRobot & robot, const MrtspFrontier & to);

// M(0, j); infinite when the gain is not positive.
double start_cost(
  const MrtspRobot & robot, const MrtspFrontier & to, const MrtspWeights & weights);

// (n+1) x (n+1) matrix, node 0 = robot, node k = frontiers[k-1].
MrtspMatrix build_cost_matrix(
  const MrtspRobot & robot, const std::vector<MrtspFrontier> & frontiers,
  const MrtspWeights & weights);

// Indices of the best `candidate_limit` frontiers (clamped to 1..kMaxDpCandidateLimit),
// sorted by M(0, j); finite first, ties to larger size, then lower index.
std::vector<int> prune_candidates(
  const MrtspRobot & robot, const std::vector<MrtspFrontier> & frontiers,
  const MrtspWeights & weights, int candidate_limit);

// Greedy traversal of M from node 0 over finite edges; returns frontier indices.
std::vector<int> greedy_order(const MrtspMatrix & matrix);

// Lowest-cost route of min(horizon, n) distinct frontiers, layer by layer over
// visited masks. Empty if no finite route of that length exists.
std::vector<int> bounded_horizon_order(const MrtspMatrix & matrix, int horizon);

// Sum of M along robot -> order[0] -> order[1] ...
double route_cost(const MrtspMatrix & matrix, const std::vector<int> & order);

// Full ordering of `frontiers` (indices into it): prune, solve, append the rest.
std::vector<int> mrtsp_order(
  const MrtspRobot & robot, const std::vector<MrtspFrontier> & frontiers,
  const MrtspOptions & options);

}  // namespace search_exploration
