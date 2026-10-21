// Robot-frontier cost matrix for the Hungarian step, built the same way as the
// field_robotics allocator (FrontierAllocationMrtspNode::begin_batch):
//
//   if the endpoint filter rejects (i, j):          C = inf
//   else if frontier j has an active-goal owner:    C = 0 for the owner, inf otherwise
//   else if (i, j) is blocked (recent failure):     C = inf
//   else:                                           C = D_ij = |goal_j - robot_i|
#pragma once

#include <cstdint>
#include <vector>

#include "search_exploration/grid.hpp"
#include "search_exploration/hungarian.hpp"

namespace search_exploration
{

struct OwnershipInputs
{
  std::vector<Point2> robots;                // robot i position
  std::vector<Point2> goals;                 // frontier j goal
  std::vector<std::vector<bool>> eligible;   // [i][j] endpoint filter passed
  std::vector<int> pinned_owner;             // [j] robot index or -1
  std::vector<std::vector<bool>> blocked;    // [i][j] recently failed
};

Matrix build_ownership_costs(const OwnershipInputs & inputs);

// Keeps frontier IDs stable across WFD runs: a new frontier reuses the ID of
// the nearest unmatched previous frontier whose goal lies within
// match_distance_m, otherwise it gets a fresh ID.
class FrontierTracker
{
public:
  explicit FrontierTracker(double match_distance_m = 0.75)
  : match_distance_m_(match_distance_m) {}

  std::vector<uint64_t> update(const std::vector<Point2> & goals);

private:
  double match_distance_m_;
  uint64_t next_id_{1};
  std::vector<uint64_t> ids_;
  std::vector<Point2> goals_;
};

}  // namespace search_exploration
