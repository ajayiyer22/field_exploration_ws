#include "search_exploration/allocation.hpp"

#include <algorithm>
#include <limits>
#include <tuple>

namespace search_exploration
{

Matrix build_ownership_costs(const OwnershipInputs & inputs)
{
  const std::size_t robots = inputs.robots.size();
  const std::size_t frontiers = inputs.goals.size();
  Matrix costs(robots, std::vector<double>(frontiers, kInfeasible));
  for (std::size_t i = 0; i < robots; ++i) {
    for (std::size_t j = 0; j < frontiers; ++j) {
      const bool eligible = inputs.eligible.empty() || inputs.eligible[i][j];
      const int owner = inputs.pinned_owner.empty() ? -1 : inputs.pinned_owner[j];
      const bool blocked = !inputs.blocked.empty() && inputs.blocked[i][j];
      if (!eligible) {
        costs[i][j] = kInfeasible;
      } else if (owner >= 0) {
        costs[i][j] = owner == static_cast<int>(i) ? 0.0 : kInfeasible;
      } else if (blocked) {
        costs[i][j] = kInfeasible;
      } else {
        costs[i][j] = distance(inputs.goals[j], inputs.robots[i]);
      }
    }
  }
  return costs;
}

std::vector<uint64_t> FrontierTracker::update(const std::vector<Point2> & goals)
{
  // Greedy closest-pair matching between previous and current goals.
  std::vector<std::tuple<double, std::size_t, std::size_t>> pairs;
  const double limit_sq = match_distance_m_ * match_distance_m_;
  for (std::size_t now = 0; now < goals.size(); ++now) {
    for (std::size_t before = 0; before < goals_.size(); ++before) {
      const double d = squared_distance(goals[now], goals_[before]);
      if (d <= limit_sq) {
        pairs.emplace_back(d, now, before);
      }
    }
  }
  std::sort(pairs.begin(), pairs.end());

  std::vector<uint64_t> ids(goals.size(), 0);
  std::vector<bool> used_before(goals_.size(), false);
  for (const auto & [d, now, before] : pairs) {
    (void)d;
    if (ids[now] == 0 && !used_before[before]) {
      ids[now] = ids_[before];
      used_before[before] = true;
    }
  }
  for (auto & id : ids) {
    if (id == 0) {
      id = next_id_++;
    }
  }
  ids_ = ids;
  goals_ = goals;
  return ids;
}

}  // namespace search_exploration
