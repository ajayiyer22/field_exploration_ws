#include "search_exploration/frontier_goal.hpp"

#include <cmath>
#include <limits>
#include <unordered_set>

namespace search_exploration
{

std::optional<FrontierCandidate> build_frontier_candidate(
  const Grid & grid, const WfdFrontier & frontier,
  const std::vector<Point2> & robot_positions, const GoalOptions & options,
  const CellBlocked & blocked)
{
  if (frontier.cells.empty()) {
    return std::nullopt;
  }

  FrontierCandidate candidate;
  candidate.size = static_cast<int>(frontier.cells.size());

  double sum_x = 0.0;
  double sum_y = 0.0;
  for (const int cell : frontier.cells) {
    const auto world = grid.world(cell);
    sum_x += world.x;
    sum_y += world.y;
  }
  candidate.centroid = {sum_x / candidate.size, sum_y / candidate.size};

  // Center point: frontier cell nearest the centroid; ties go to the lower (y, x).
  constexpr double kTieEpsilon = 1e-12;
  int center_cell = frontier.cells.front();
  double center_distance_sq = std::numeric_limits<double>::infinity();
  for (const int cell : frontier.cells) {
    const double distance_sq = squared_distance(grid.world(cell), candidate.centroid);
    const bool earlier =
      std::make_pair(grid.cell_y(cell), grid.cell_x(cell)) <
      std::make_pair(grid.cell_y(center_cell), grid.cell_x(center_cell));
    if (distance_sq + kTieEpsilon < center_distance_sq ||
      (std::abs(distance_sq - center_distance_sq) <= kTieEpsilon && earlier))
    {
      center_distance_sq = distance_sq;
      center_cell = cell;
    }
  }
  candidate.center_point = grid.world(center_cell);
  candidate.start_point = grid.world(
    frontier.start_cell >= 0 ? frontier.start_cell : frontier.cells.front());

  // Goal: free, unblocked neighbour of any frontier cell nearest the centroid.
  const bool apply_min_distance = options.min_goal_distance_m > 0.0 && !robot_positions.empty();
  const double min_distance_sq = options.min_goal_distance_m * options.min_goal_distance_m;
  std::unordered_set<int> visited;
  std::optional<Point2> best_any;
  std::optional<Point2> best_far;
  double best_any_sq = std::numeric_limits<double>::infinity();
  double best_far_sq = std::numeric_limits<double>::infinity();

  for (const int cell : frontier.cells) {
    grid.for_each_neighbour(cell, [&](int neighbour) {
        if (!visited.insert(neighbour).second) {
          return;  // the same neighbour can touch several frontier cells
        }
        if (!grid.open_space(neighbour) || (blocked && blocked(neighbour))) {
          return;
        }
        const auto world = grid.world(neighbour);
        const double centroid_sq = squared_distance(world, candidate.centroid);
        if (centroid_sq < best_any_sq) {
          best_any_sq = centroid_sq;
          best_any = world;
        }
        if (apply_min_distance) {
          bool far_enough = true;
          for (const auto & robot : robot_positions) {
            far_enough = far_enough && squared_distance(world, robot) >= min_distance_sq;
          }
          if (far_enough && centroid_sq < best_far_sq) {
            best_far_sq = centroid_sq;
            best_far = world;
          }
        }
      });
  }

  const auto & goal = best_far.has_value() ? best_far : best_any;
  if (!goal.has_value()) {
    return std::nullopt;
  }
  candidate.goal = *goal;
  return candidate;
}

std::vector<FrontierCandidate> build_frontier_candidates(
  const Grid & grid, const std::vector<WfdFrontier> & frontiers,
  const std::vector<Point2> & robot_positions, const GoalOptions & options,
  const CellBlocked & blocked)
{
  std::vector<FrontierCandidate> candidates;
  candidates.reserve(frontiers.size());
  for (const auto & frontier : frontiers) {
    if (auto candidate = build_frontier_candidate(
        grid, frontier, robot_positions, options, blocked))
    {
      candidates.push_back(*candidate);
    }
  }
  return candidates;
}

}  // namespace search_exploration
