#include <gtest/gtest.h>

#include <algorithm>
#include <numeric>
#include <random>

#include "search_exploration/hungarian.hpp"

using search_exploration::assign_with_slots;
using search_exploration::kInfeasible;
using search_exploration::kuhn_max_assignment;
using search_exploration::Matrix;
using search_exploration::min_cost_assignment;

namespace
{

// Brute force: best (max feasible count, then min cost) over all injective maps.
std::pair<int, double> brute_force(const Matrix & costs)
{
  const int rows = static_cast<int>(costs.size());
  const int cols = static_cast<int>(costs[0].size());
  const int n = std::max(rows, cols);
  std::vector<int> perm(n);
  std::iota(perm.begin(), perm.end(), 0);
  int best_count = -1;
  double best_cost = 0.0;
  do {
    int count = 0;
    double cost = 0.0;
    for (int i = 0; i < rows; ++i) {
      const int j = perm[i];
      if (j < cols && std::isfinite(costs[i][j])) {
        ++count;
        cost += costs[i][j];
      }
    }
    if (count > best_count || (count == best_count && cost < best_cost - 1e-9)) {
      best_count = count;
      best_cost = cost;
    }
  } while (std::next_permutation(perm.begin(), perm.end()));
  return {best_count, best_cost};
}

}  // namespace

// Kuhn (1955), Section 5: the optimal rating sum is 27, with the cover
// u = (7, 5, 6, 3), v = (1, 0, 2, 3) totalling 27.
TEST(Kuhn, PaperExample)
{
  const Matrix r = {{8, 7, 9, 9}, {5, 2, 7, 8}, {6, 1, 4, 9}, {2, 3, 2, 6}};
  const auto result = kuhn_max_assignment(r);
  EXPECT_NEAR(result.rating_sum, 27.0, 1e-9);
  const double allotment =
    std::accumulate(result.u.begin(), result.u.end(), 0.0) +
    std::accumulate(result.v.begin(), result.v.end(), 0.0);
  EXPECT_NEAR(allotment, 27.0, 1e-9);  // Theorem 6: rating sum = total allotment
  std::vector<bool> used(4, false);
  for (int i = 0; i < 4; ++i) {
    ASSERT_GE(result.row_to_col[i], 0);
    EXPECT_FALSE(used[result.row_to_col[i]]);
    used[result.row_to_col[i]] = true;
    for (int j = 0; j < 4; ++j) {
      EXPECT_GE(result.u[i] + result.v[j], r[i][j] - 1e-9);  // adequate cover
    }
  }
}

TEST(Kuhn, MatchesBruteForceOnRandomMatrices)
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> value(0.0, 50.0);
  std::bernoulli_distribution infeasible(0.2);
  for (int trial = 0; trial < 300; ++trial) {
    const int rows = 1 + static_cast<int>(rng() % 6);
    const int cols = 1 + static_cast<int>(rng() % 6);
    Matrix costs(rows, std::vector<double>(cols));
    for (auto & row : costs) {
      for (auto & c : row) {
        c = infeasible(rng) ? kInfeasible : value(rng);
      }
    }
    const auto expected = brute_force(costs);
    const auto got = min_cost_assignment(costs);
    const int count = static_cast<int>(
      std::count_if(got.row_to_col.begin(), got.row_to_col.end(), [](int j) {return j >= 0;}));
    ASSERT_EQ(count, expected.first) << "trial " << trial;
    ASSERT_NEAR(got.total_cost, expected.second, 1e-6) << "trial " << trial;
  }
}

TEST(Kuhn, AllInfeasibleAssignsNothing)
{
  const Matrix costs(3, std::vector<double>(4, kInfeasible));
  const auto got = min_cost_assignment(costs);
  for (const int j : got.row_to_col) {
    EXPECT_EQ(j, -1);
  }
}

// ALLOCATOR_WORKFLOW.md section 7: robots at x=0 and x=10, goals at 1, 2, 3, 9.
// Capacity 2 per robot gives {1, 2} and {3, 9}, total distance 11.
TEST(Slots, BalancedExampleFromOldWorkspace)
{
  const std::vector<double> goals = {1, 2, 3, 9};
  const std::vector<double> robots = {0, 10};
  Matrix costs(2, std::vector<double>(4));
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 4; ++j) {
      costs[i][j] = std::abs(goals[j] - robots[i]);
    }
  }
  const auto got = assign_with_slots(costs, 0);
  EXPECT_NEAR(got.total_cost, 11.0, 1e-9);
  EXPECT_EQ(got.owned[0], (std::vector<int>{0, 1}));
  EXPECT_EQ(got.owned[1], (std::vector<int>{2, 3}));
  EXPECT_TRUE(got.unassigned.empty());
}

TEST(Slots, OneSlotIsOneFrontierPerRobot)
{
  const Matrix costs = {{1, 5, 9}, {4, 2, 8}};
  const auto got = assign_with_slots(costs, 1);
  EXPECT_EQ(got.owned[0], (std::vector<int>{0}));
  EXPECT_EQ(got.owned[1], (std::vector<int>{1}));
  EXPECT_EQ(got.unassigned, (std::vector<int>{2}));
}
