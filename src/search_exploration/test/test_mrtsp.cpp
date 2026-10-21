#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "search_exploration/allocation.hpp"
#include "search_exploration/mrtsp.hpp"

using namespace search_exploration;

// Tests ported from the field_robotics workspace (test_mrtsp_solver.cpp and the
// MrtspOrderingTests in test_decision_map_and_mrtsp.cpp).

namespace
{

// Collinear frontier on the x axis: centroid, center and start all at (x, 0).
MrtspFrontier at_x(double x, double size = 1.0)
{
  return {{x, 0.0}, {x, 0.0}, {x, 0.0}, size, std::nullopt};
}

const MrtspRobot kRobot{{0.0, 0.0}, 0.0, 1.0, 1.0};
const MrtspWeights kNoRange{1.0, 1.0, 0.0};

MrtspMatrix make_matrix(int frontiers)
{
  MrtspMatrix m(frontiers + 1, std::vector<double>(frontiers + 1, kInfeasible));
  for (int i = 1; i <= frontiers; ++i) {
    m[i][0] = 0.0;
  }
  return m;
}

}  // namespace

TEST(Mrtsp, PathCostEquation5)
{
  // V_j: center (4, 0), centroid (4, 1), start (4, 3); source center at origin.
  MrtspFrontier to{{4, 1}, {4, 0}, {4, 3}, 10, std::nullopt};
  // d_m = 4, d_u = 3, d_n = sqrt(17), d_v = 2 -> max(7, 6.123) - r_s.
  EXPECT_NEAR(path_cost({0, 0}, to, 1.5), 7.0 - 1.5, 1e-9);
}

TEST(Mrtsp, StartCostUsesRobotCenterDistanceForPathAndTranslation)
{
  MrtspFrontier candidate{{0.0, 1.0}, {0.0, 1.0}, {0.0, 1.0}, 4, 5.0};
  MrtspRobot robot{{0.0, 0.0}, 0.0, 2.0, 1.0};
  EXPECT_DOUBLE_EQ(initial_path_cost(robot.position, candidate, 0.0), 5.0);
  // L / V_max + turn / W_max = 5 / 2 + (pi / 2) / 1: a sum, not a minimum.
  EXPECT_NEAR(time_lower_bound(robot, candidate), 2.5 + M_PI / 2.0, 1e-9);
}

TEST(Mrtsp, StartCostFallsBackToEuclideanWithoutRobotCenterDistance)
{
  MrtspFrontier candidate{{0.0, 1.0}, {0.0, 1.0}, {0.0, 1.0}, 4, std::nullopt};
  MrtspRobot robot{{0.0, 0.0}, 0.0, 2.0, 1.0};
  EXPECT_DOUBLE_EQ(initial_path_cost(robot.position, candidate, 0.0), 1.0);
  EXPECT_NEAR(time_lower_bound(robot, candidate), 0.5 + M_PI / 2.0, 1e-9);
}

TEST(Mrtsp, CostMatrixMatchesOldWorkspace)
{
  MrtspRobot robot{{0, 0}, 0.0, 0.5, 1.0};
  std::vector<MrtspFrontier> frontiers = {at_x(4, 4), {{0, 6}, {0, 6}, {0, 6}, 9, std::nullopt}};
  MrtspWeights weights{2.0, 1.0, 1.0};
  const auto m = build_cost_matrix(robot, frontiers, weights);
  ASSERT_EQ(m.size(), 3U);
  // 2 * (4 - 1) / 4 + (4 / 0.5 + 0) / sqrt(4)
  EXPECT_NEAR(m[0][1], 1.5 + 4.0, 1e-9);
  // 2 * (6 - 1) / 9 + (6 / 0.5 + pi / 2) / sqrt(9)
  EXPECT_NEAR(m[0][2], 10.0 / 9.0 + (12.0 + M_PI / 2.0) / 3.0, 1e-9);
  // From V_1 center (4, 0) to V_2: 2 * (sqrt(52) - 1) / 9
  EXPECT_NEAR(m[1][2], 2.0 * (std::sqrt(52.0) - 1.0) / 9.0, 1e-9);
  EXPECT_EQ(m[1][0], 0.0);
  EXPECT_TRUE(std::isinf(m[1][1]));
  // W_s is an exponent on the size: G = 4^2 = 16.
  const auto squared = build_cost_matrix(robot, frontiers, {2.0, 2.0, 1.0});
  EXPECT_NEAR(squared[0][1], 2.0 * 3.0 / 16.0 + 8.0 / 4.0, 1e-9);
}

TEST(Mrtsp, GreedyCanPreferHighGainFrontierOverCloserCandidate)
{
  MrtspRobot robot{{0, 0}, 0.0, 100.0, 1.0};
  const auto m = build_cost_matrix(robot, {at_x(1.0, 1), at_x(2.0, 10)}, kNoRange);
  const auto order = greedy_order(m);
  ASSERT_EQ(order.size(), 2U);
  EXPECT_EQ(order.front(), 1);
  EXPECT_EQ(order, greedy_order(m));
}

TEST(Mrtsp, PruningKeepsTopScoreCandidates)
{
  const auto pool = prune_candidates(
    kRobot, {at_x(5.0), at_x(2.0), at_x(3.0), at_x(1.0)}, kNoRange, 2);
  EXPECT_EQ(pool, (std::vector<int>{3, 1}));
}

TEST(Mrtsp, PruningIsDeterministic)
{
  // Start costs 8.05, 6.83, 10, 6.
  const auto pool = prune_candidates(
    kRobot, {at_x(20.0 / 3.0, 2), at_x(7.5, 3), at_x(5.0, 1), at_x(8.0, 4)}, kNoRange, 4);
  EXPECT_EQ(pool, (std::vector<int>{3, 1, 0, 2}));
}

TEST(Mrtsp, PruningTiesGoToLargerFrontierThenLowerIndex)
{
  // Equal start cost (path / S + t_lb / sqrt(S) = 0 at x = 0), then size, then index.
  const auto pool = prune_candidates(
    kRobot, {at_x(0.0, 2), at_x(0.0, 5), at_x(0.0, 2)}, kNoRange, 3);
  EXPECT_EQ(pool, (std::vector<int>{1, 0, 2}));
}

TEST(Mrtsp, CandidateLimitIsRespectedAndClamped)
{
  std::vector<MrtspFrontier> frontiers;
  for (int k = 0; k < kMaxDpCandidateLimit + 20; ++k) {
    frontiers.push_back(at_x(k + 1.0));
  }
  EXPECT_EQ(prune_candidates(kRobot, frontiers, kNoRange, 15).size(), 15U);
  EXPECT_EQ(
    prune_candidates(kRobot, frontiers, kNoRange, 200).size(),
    static_cast<std::size_t>(kMaxDpCandidateLimit));
  frontiers.resize(10);
  EXPECT_EQ(prune_candidates(kRobot, frontiers, kNoRange, 15).size(), 10U);
}

TEST(Mrtsp, BoundedHorizonCanBeatGreedy)
{
  // A is the cheapest first step; the B -> C -> D -> E corridor is cheaper over four.
  auto m = make_matrix(5);
  m[0][1] = 1.0;
  m[0][2] = 2.0;
  m[0][3] = m[0][4] = m[0][5] = 50.0;
  for (int i = 1; i <= 5; ++i) {
    for (int j = 1; j <= 5; ++j) {
      if (i != j) {
        m[i][j] = 100.0;
      }
    }
  }
  m[2][3] = m[3][4] = m[4][5] = 1.0;
  EXPECT_EQ(greedy_order(m).front(), 0);
  const auto dp = bounded_horizon_order(m, 4);
  ASSERT_EQ(dp.size(), 4U);
  EXPECT_EQ(dp.front(), 1);
}

TEST(Mrtsp, HorizonOneChoosesBestStartEdge)
{
  auto m = make_matrix(3);
  m[0][1] = 5.0;
  m[0][2] = 2.0;
  m[0][3] = 7.0;
  EXPECT_EQ(bounded_horizon_order(m, 1), (std::vector<int>{1}));
}

TEST(Mrtsp, HorizonClampsToCandidateCount)
{
  auto m = make_matrix(3);
  for (int i = 0; i <= 3; ++i) {
    for (int j = 1; j <= 3; ++j) {
      if (i != j) {
        m[i][j] = 1.0;
      }
    }
  }
  EXPECT_EQ(bounded_horizon_order(m, 4).size(), 3U);
}

TEST(Mrtsp, InvalidMatrixReturnsEmptyOrderForFallback)
{
  EXPECT_TRUE(bounded_horizon_order(make_matrix(2), 2).empty());
}

TEST(Mrtsp, DpIsNeverWorseThanGreedy)
{
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> coord(-10.0, 10.0);
  std::uniform_real_distribution<double> size(5.0, 60.0);
  for (int trial = 0; trial < 100; ++trial) {
    std::vector<MrtspFrontier> frontiers;
    const int n = 2 + static_cast<int>(rng() % 7);
    for (int k = 0; k < n; ++k) {
      Point2 c{coord(rng), coord(rng)};
      frontiers.push_back({c, {c.x + 0.2, c.y}, {c.x + 0.6, c.y + 0.3}, size(rng), std::nullopt});
    }
    MrtspRobot robot{{0, 0}, 0.3, 0.5, 1.0};
    const auto m = build_cost_matrix(robot, frontiers, {});
    const auto greedy = greedy_order(m);
    const auto dp = bounded_horizon_order(m, n);
    ASSERT_EQ(dp.size(), static_cast<std::size_t>(n));
    EXPECT_LE(route_cost(m, dp), route_cost(m, greedy) + 1e-9) << "trial " << trial;
  }
}

TEST(Mrtsp, OrderPrunesThenAppendsTheRestInInputOrder)
{
  // Pool of 2 (the frontiers at x = 1 and x = 2), horizon 1: DP orders one of
  // them, then everything else follows in input order.
  const std::vector<MrtspFrontier> frontiers = {at_x(5.0), at_x(2.0), at_x(3.0), at_x(1.0)};
  MrtspOptions options;
  options.weights = kNoRange;
  options.dp_candidate_limit = 2;
  options.dp_horizon = 1;
  EXPECT_EQ(mrtsp_order(kRobot, frontiers, options), (std::vector<int>{3, 0, 1, 2}));
  // Greedy orders the whole pool, then appends the rest.
  options.solver = MrtspSolver::GREEDY;
  EXPECT_EQ(mrtsp_order(kRobot, frontiers, options), (std::vector<int>{3, 1, 0, 2}));
  // Fewer than two frontiers are returned as given.
  EXPECT_EQ(mrtsp_order(kRobot, {at_x(4.0)}, options), (std::vector<int>{0}));
}

TEST(Ownership, PrecedenceMatchesOldAllocator)
{
  OwnershipInputs in;
  in.robots = {{0, 0}, {10, 0}};
  in.goals = {{3, 4}, {10, 1}, {5, 0}};
  in.eligible = {{true, true, false}, {true, true, true}};
  in.pinned_owner = {-1, 0, -1};
  in.blocked = {{false, false, false}, {true, false, false}};
  const auto c = build_ownership_costs(in);
  EXPECT_NEAR(c[0][0], 5.0, 1e-9);       // Euclidean distance
  EXPECT_TRUE(std::isinf(c[1][0]));      // blocked
  EXPECT_EQ(c[0][1], 0.0);               // pinned owner
  EXPECT_TRUE(std::isinf(c[1][1]));      // pinned to someone else
  EXPECT_TRUE(std::isinf(c[0][2]));      // endpoint filter rejects
  EXPECT_NEAR(c[1][2], 5.0, 1e-9);
}

TEST(Tracker, ReusesIdsForNearbyGoals)
{
  FrontierTracker tracker(0.75);
  const auto first = tracker.update({{0, 0}, {5, 5}});
  const auto second = tracker.update({{5.3, 5.0}, {9, 9}, {0.2, 0.1}});
  EXPECT_EQ(second[0], first[1]);
  EXPECT_EQ(second[2], first[0]);
  EXPECT_NE(second[1], first[0]);
  EXPECT_NE(second[1], first[1]);
}
