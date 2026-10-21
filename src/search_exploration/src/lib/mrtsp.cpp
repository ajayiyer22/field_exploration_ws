#include "search_exploration/mrtsp.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numeric>

namespace search_exploration
{
namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();
// Scores that differ only by floating-point noise fall through to the tie-breakers.
constexpr double kScoreTieEpsilon = 1e-12;

using Mask = std::uint64_t;

double robot_distance(const Point2 & robot, const MrtspFrontier & to)
{
  return to.robot_center_distance.value_or(distance(robot, to.center_point));
}

}  // namespace

double information_gain(const MrtspFrontier & frontier, const MrtspWeights & weights)
{
  return weights.w_s != 1.0 ? std::pow(frontier.size, weights.w_s) : frontier.size;
}

double path_cost(const Point2 & from_center, const MrtspFrontier & to, double sensor_range)
{
  const double d_m = distance(from_center, to.center_point);
  const double d_n = distance(from_center, to.centroid);
  const double d_u = distance(to.center_point, to.start_point);
  const double d_v = distance(to.centroid, to.start_point);
  return std::max(d_m + d_u, d_n + d_v) - sensor_range;
}

double initial_path_cost(const Point2 & robot, const MrtspFrontier & to, double sensor_range)
{
  const double d_m = robot_distance(robot, to);
  const double d_n = distance(robot, to.centroid);
  const double d_u = distance(to.center_point, to.start_point);
  const double d_v = distance(to.centroid, to.start_point);
  return std::max(d_m + d_u, d_n + d_v) - sensor_range;
}

double time_lower_bound(const MrtspRobot & robot, const MrtspFrontier & to)
{
  const double v_max = std::max(robot.v_max, 1e-6);
  const double w_max = std::max(robot.w_max, 1e-6);
  const double heading = std::atan2(
    to.center_point.y - robot.position.y, to.center_point.x - robot.position.x);
  const double delta = heading - robot.yaw;
  // Q_j: yaw between the robot heading and the direction to V_j, in [0, pi].
  const double q = std::abs(std::atan2(std::sin(delta), std::cos(delta)));
  return robot_distance(robot.position, to) / v_max + std::min(q, 2.0 * M_PI - q) / w_max;
}

double start_cost(
  const MrtspRobot & robot, const MrtspFrontier & to, const MrtspWeights & weights)
{
  if (to.size <= 0.0) {
    return kInf;
  }
  const double gain = information_gain(to, weights);
  const double path = initial_path_cost(robot.position, to, weights.sensor_range);
  return weights.w_d * path / gain + time_lower_bound(robot, to) / std::sqrt(gain);
}

MrtspMatrix build_cost_matrix(
  const MrtspRobot & robot, const std::vector<MrtspFrontier> & frontiers,
  const MrtspWeights & weights)
{
  const std::size_t n = frontiers.size();
  MrtspMatrix m(n + 1, std::vector<double>(n + 1, kInf));
  for (std::size_t i = 1; i <= n; ++i) {
    m[i][0] = 0.0;  // returning to the robot node is never chosen
  }
  for (std::size_t j = 1; j <= n; ++j) {
    const auto & to = frontiers[j - 1];
    if (to.size <= 0.0) {
      continue;
    }
    const double gain = information_gain(to, weights);
    m[0][j] = start_cost(robot, to, weights);
    for (std::size_t i = 1; i <= n; ++i) {
      if (i != j) {
        m[i][j] = weights.w_d *
          path_cost(frontiers[i - 1].center_point, to, weights.sensor_range) / gain;
      }
    }
  }
  return m;
}

std::vector<int> prune_candidates(
  const MrtspRobot & robot, const std::vector<MrtspFrontier> & frontiers,
  const MrtspWeights & weights, int candidate_limit)
{
  std::vector<double> score(frontiers.size());
  for (std::size_t k = 0; k < frontiers.size(); ++k) {
    score[k] = start_cost(robot, frontiers[k], weights);
    if (std::isnan(score[k])) {
      score[k] = kInf;
    }
  }
  std::vector<int> ranked(frontiers.size());
  std::iota(ranked.begin(), ranked.end(), 0);
  std::sort(ranked.begin(), ranked.end(), [&](int a, int b) {
      const bool a_finite = std::isfinite(score[a]);
      const bool b_finite = std::isfinite(score[b]);
      if (a_finite != b_finite) {
        return a_finite;
      }
      if (a_finite && std::abs(score[a] - score[b]) > kScoreTieEpsilon) {
        return score[a] < score[b];
      }
      if (!a_finite && score[a] != score[b]) {
        return score[a] < score[b];
      }
      if (frontiers[a].size != frontiers[b].size) {
        return frontiers[a].size > frontiers[b].size;
      }
      return a < b;
    });
  const int limit = std::clamp(candidate_limit, 1, kMaxDpCandidateLimit);
  if (static_cast<int>(ranked.size()) > limit) {
    ranked.resize(limit);
  }
  return ranked;
}

std::vector<int> greedy_order(const MrtspMatrix & matrix)
{
  const int n = static_cast<int>(matrix.size()) - 1;
  std::vector<int> order;
  if (n <= 0) {
    return order;
  }
  std::vector<bool> visited(n + 1, false);
  int current = 0;
  while (true) {
    int best = -1;
    double best_cost = kInf;
    for (int j = 1; j <= n; ++j) {
      if (!visited[j] && matrix[current][j] < best_cost) {
        best_cost = matrix[current][j];
        best = j;
      }
    }
    if (best < 0) {
      break;
    }
    visited[best] = true;
    order.push_back(best - 1);
    current = best;
  }
  return order;
}

std::vector<int> bounded_horizon_order(const MrtspMatrix & matrix, int horizon)
{
  const int n = static_cast<int>(matrix.size()) - 1;
  if (n <= 0 || horizon <= 0 || n > std::numeric_limits<Mask>::digits) {
    return {};
  }
  const int depth = std::min(horizon, n);

  // layer[mask][j]: cheapest route over the frontiers in `mask` ending at j.
  // Only masks of the current route length are kept.
  std::map<Mask, std::vector<double>> layer;
  std::map<Mask, std::vector<int>> parent;
  auto entry = [n](std::map<Mask, std::vector<double>> & costs, Mask mask) -> std::vector<double> & {
      auto & row = costs[mask];
      if (row.empty()) {
        row.assign(n, kInf);
      }
      return row;
    };
  auto parent_entry = [&parent, n](Mask mask) -> std::vector<int> & {
      auto & row = parent[mask];
      if (row.empty()) {
        row.assign(n, -1);
      }
      return row;
    };

  for (int j = 0; j < n; ++j) {
    if (std::isfinite(matrix[0][j + 1])) {
      const Mask mask = Mask{1} << j;
      entry(layer, mask)[j] = matrix[0][j + 1];
      parent_entry(mask);
    }
  }
  if (layer.empty()) {
    return {};
  }

  for (int length = 1; length < depth; ++length) {
    std::map<Mask, std::vector<double>> next_layer;
    for (const auto & [mask, costs] : layer) {
      for (int j = 0; j < n; ++j) {
        if (!(mask & (Mask{1} << j)) || !std::isfinite(costs[j])) {
          continue;
        }
        for (int k = 0; k < n; ++k) {
          const Mask bit = Mask{1} << k;
          const double edge = matrix[j + 1][k + 1];
          if ((mask & bit) || !std::isfinite(edge)) {
            continue;
          }
          const double cost = costs[j] + edge;
          if (!std::isfinite(cost)) {
            continue;
          }
          auto & next = entry(next_layer, mask | bit);
          if (cost < next[k]) {
            next[k] = cost;
            parent_entry(mask | bit)[k] = j;
          }
        }
      }
    }
    if (next_layer.empty()) {
      return {};
    }
    layer = std::move(next_layer);
  }

  double best = kInf;
  Mask best_mask = 0;
  int best_end = -1;
  for (const auto & [mask, costs] : layer) {
    for (int j = 0; j < n; ++j) {
      if (std::isfinite(costs[j]) && costs[j] < best) {
        best = costs[j];
        best_mask = mask;
        best_end = j;
      }
    }
  }
  if (best_end < 0) {
    return {};
  }

  std::vector<int> order;
  Mask mask = best_mask;
  for (int j = best_end; j >= 0; ) {
    order.push_back(j);
    const auto it = parent.find(mask);
    const int previous = it != parent.end() ? it->second[j] : -1;
    mask &= ~(Mask{1} << j);
    j = previous;
  }
  if (static_cast<int>(order.size()) != depth) {
    return {};
  }
  std::reverse(order.begin(), order.end());
  return order;
}

double route_cost(const MrtspMatrix & matrix, const std::vector<int> & order)
{
  double total = 0.0;
  int current = 0;
  for (const int frontier : order) {
    total += matrix[current][frontier + 1];
    current = frontier + 1;
  }
  return total;
}

std::vector<int> mrtsp_order(
  const MrtspRobot & robot, const std::vector<MrtspFrontier> & frontiers,
  const MrtspOptions & options)
{
  const int n = static_cast<int>(frontiers.size());
  std::vector<int> output;
  if (n < 2) {  // nothing to order
    for (int k = 0; k < n; ++k) {
      output.push_back(k);
    }
    return output;
  }

  const auto pool = prune_candidates(robot, frontiers, options.weights, options.dp_candidate_limit);
  std::vector<MrtspFrontier> pool_frontiers;
  for (const int k : pool) {
    pool_frontiers.push_back(frontiers[k]);
  }
  const auto matrix = build_cost_matrix(robot, pool_frontiers, options.weights);
  auto order = options.solver == MrtspSolver::GREEDY ?
    greedy_order(matrix) : bounded_horizon_order(matrix, std::max(1, options.dp_horizon));
  if (order.empty()) {
    order = greedy_order(matrix);
  }

  for (const int p : order) {
    output.push_back(pool[p]);
  }
  // The pool and the DP horizon can be smaller than the group; append the rest
  // in input order.
  for (int k = 0; k < n; ++k) {
    if (std::find(output.begin(), output.end(), k) == output.end()) {
      output.push_back(k);
    }
  }
  return output;
}

}  // namespace search_exploration
