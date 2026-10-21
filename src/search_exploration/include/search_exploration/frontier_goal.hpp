// Frontier goal creation, ported from the field_robotics workspace
// (frontier_exploration_ros2/src/frontier_search.cpp, build_frontier_candidate).
//
// For one WFD frontier:
//   centroid c    = mean world position of the frontier cells
//   center point  = frontier cell nearest c (ties: lower (y, x))
//   start point   = the cell where the inner BFS started
//   goal          = among free, unblocked neighbours of the frontier cells, the
//                   one nearest c; with min_goal_distance_m > 0, prefer the
//                   nearest one at least that far from every robot
//                   The node supplies a blocked predicate for raw Nav2 cost;
//                   it retains a geometric fallback for unassigned frontiers.
//   size          = number of frontier cells (information gain S)
// A frontier with no admissible goal cell is dropped.
#pragma once

#include <functional>
#include <optional>
#include <vector>

#include "search_exploration/grid.hpp"
#include "search_exploration/wfd.hpp"

namespace search_exploration
{

struct FrontierCandidate
{
  Point2 centroid;
  Point2 center_point;
  Point2 start_point;
  Point2 goal;
  int size{0};
};

struct GoalOptions
{
  double min_goal_distance_m{0.0};
};

// Optional extra rejection test for goal cells (true = blocked).
using CellBlocked = std::function<bool(int cell)>;

std::optional<FrontierCandidate> build_frontier_candidate(
  const Grid & grid, const WfdFrontier & frontier,
  const std::vector<Point2> & robot_positions, const GoalOptions & options = {},
  const CellBlocked & blocked = nullptr);

std::vector<FrontierCandidate> build_frontier_candidates(
  const Grid & grid, const std::vector<WfdFrontier> & frontiers,
  const std::vector<Point2> & robot_positions, const GoalOptions & options = {},
  const CellBlocked & blocked = nullptr);

}  // namespace search_exploration
