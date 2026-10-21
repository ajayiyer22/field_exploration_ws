#include <gtest/gtest.h>

#include <set>
#include <string>

#include "search_exploration/frontier_goal.hpp"
#include "search_exploration/wfd.hpp"

using namespace search_exploration;

namespace
{

// '.' free, '#' occupied, '?' unknown. Row 0 of the picture is the top (max y).
Grid make_grid(const std::vector<std::string> & picture, double resolution = 1.0)
{
  Grid grid;
  grid.height = static_cast<int>(picture.size());
  grid.width = static_cast<int>(picture.front().size());
  grid.resolution = resolution;
  grid.data.assign(grid.size(), -1);
  for (int row = 0; row < grid.height; ++row) {
    const int y = grid.height - 1 - row;
    for (int x = 0; x < grid.width; ++x) {
      const char c = picture[row][x];
      grid.data[grid.index(x, y)] = c == '.' ? 0 : c == '#' ? 100 : -1;
    }
  }
  return grid;
}

Point2 cell_centre(const Grid & grid, int x, int y) {return grid.world(grid.index(x, y));}

}  // namespace

TEST(Wfd, FrontierPointDefinition)
{
  const auto grid = make_grid({"???", "?..", "???"});
  EXPECT_TRUE(is_frontier_point(grid, grid.index(0, 1)));   // unknown next to free
  EXPECT_FALSE(is_frontier_point(grid, grid.index(1, 1)));  // free
  const auto closed = make_grid({"???", "???", "?##"});
  EXPECT_FALSE(is_frontier_point(closed, closed.index(0, 0)));  // only occupied nearby
}

TEST(Wfd, CorridorHasOneFrontierAtEachOpenEnd)
{
  const auto grid = make_grid({
    "##########",
    "?........?",
    "?........?",
    "##########",
  });
  WfdOptions options;
  options.min_frontier_size_cells = 1;
  const auto result = detect_frontiers(grid, {cell_centre(grid, 4, 1)}, options);
  ASSERT_EQ(result.frontiers.size(), 2U);
  for (const auto & frontier : result.frontiers) {
    EXPECT_EQ(frontier.cells.size(), 2U);
  }
}

TEST(Wfd, FullyKnownMapHasNoFrontiers)
{
  const auto grid = make_grid({"#####", "#...#", "#...#", "#####"});
  const auto result = detect_frontiers(grid, {cell_centre(grid, 2, 1)}, {});
  EXPECT_TRUE(result.frontiers.empty());
}

TEST(Wfd, EveryFrontierPointBelongsToExactlyOneFrontier)
{
  const auto grid = make_grid({
    "??????????????",
    "?............?",
    "?.....##.....?",
    "?.....##.....?",
    "?............?",
    "??????????????",
  });
  WfdOptions options;
  options.min_frontier_size_cells = 1;
  const auto result = detect_frontiers(
    grid, {cell_centre(grid, 2, 2), cell_centre(grid, 11, 3)}, options);
  std::set<int> seen;
  std::size_t total = 0;
  for (const auto & frontier : result.frontiers) {
    for (const int cell : frontier.cells) {
      EXPECT_TRUE(is_frontier_point(grid, cell));
      seen.insert(cell);
      ++total;
    }
  }
  EXPECT_EQ(seen.size(), total);  // no duplicates
  int expected = 0;
  for (int cell = 0; cell < grid.size(); ++cell) {
    expected += is_frontier_point(grid, cell) ? 1 : 0;
  }
  EXPECT_EQ(static_cast<int>(total), expected);  // and none missed
}

TEST(Wfd, WallIsNotCrossed)
{
  // The right room is known but the robot cannot reach it through the wall.
  const auto grid = make_grid({
    "#########",
    "#...#...?",
    "#...#...?",
    "#########",
  });
  WfdOptions options;
  options.min_frontier_size_cells = 1;
  const auto result = detect_frontiers(grid, {cell_centre(grid, 1, 1)}, options);
  EXPECT_TRUE(result.frontiers.empty());
}

TEST(Wfd, SmallFrontiersAreRejected)
{
  const auto grid = make_grid({"#####", "#...?", "#...#", "#####"});
  WfdOptions options;
  options.min_frontier_size_cells = 2;
  const auto result = detect_frontiers(grid, {cell_centre(grid, 1, 1)}, options);
  EXPECT_TRUE(result.frontiers.empty());
  EXPECT_EQ(result.rejected_small, 1);
}

TEST(FrontierGoal, CentroidCenterAndNearestFreeGoal)
{
  const auto grid = make_grid({
    "#?????#",
    "#.....#",
    "#.....#",
    "#######",
  });
  WfdOptions options;
  options.min_frontier_size_cells = 1;
  const auto result = detect_frontiers(grid, {cell_centre(grid, 3, 1)}, options);
  ASSERT_EQ(result.frontiers.size(), 1U);
  const auto candidate = build_frontier_candidate(grid, result.frontiers[0], {});
  ASSERT_TRUE(candidate.has_value());
  EXPECT_EQ(candidate->size, 5);
  EXPECT_NEAR(candidate->centroid.x, 3.5, 1e-9);  // cells x = 1..5, centres 1.5..5.5
  EXPECT_NEAR(candidate->centroid.y, 3.5, 1e-9);
  EXPECT_NEAR(candidate->center_point.x, 3.5, 1e-9);
  EXPECT_NEAR(candidate->goal.x, 3.5, 1e-9);  // free cell right below the centre
  EXPECT_NEAR(candidate->goal.y, 2.5, 1e-9);
}

TEST(FrontierGoal, BlockedCellsAreNotGoals)
{
  const auto grid = make_grid({"#???#", "#...#", "#####"});
  WfdOptions options;
  options.min_frontier_size_cells = 1;
  const auto result = detect_frontiers(grid, {cell_centre(grid, 2, 1)}, options);
  ASSERT_EQ(result.frontiers.size(), 1U);
  const auto all_blocked = build_frontier_candidate(
    grid, result.frontiers[0], {}, {}, [](int) {return true;});
  EXPECT_FALSE(all_blocked.has_value());
}

TEST(FrontierGoal, MinimumGoalDistancePrefersFarCells)
{
  const auto grid = make_grid({"#???????#", "#.......#", "#.......#", "#########"});
  WfdOptions options;
  options.min_frontier_size_cells = 1;
  const auto robot = cell_centre(grid, 4, 1);
  const auto result = detect_frontiers(grid, {robot}, options);
  ASSERT_EQ(result.frontiers.size(), 1U);
  GoalOptions goal_options;
  goal_options.min_goal_distance_m = 3.0;
  const auto candidate = build_frontier_candidate(grid, result.frontiers[0], {robot}, goal_options);
  ASSERT_TRUE(candidate.has_value());
  EXPECT_GE(distance(candidate->goal, robot), 3.0);
}
