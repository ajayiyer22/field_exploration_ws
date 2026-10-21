// The Hungarian method for the assignment problem.
//
// H. W. Kuhn, "The Hungarian Method for the Assignment Problem",
// Naval Research Logistics Quarterly 2 (1955), Section 4.
//
// Kuhn maximises the rating sum of a square n x n matrix R. The dual keeps a
// cover {u_i, v_j} with u_i + v_j >= r_ij and minimises sum(u) + sum(v):
//   Routine I  - on Q = {(i,j) : u_i + v_j = r_ij} find a largest set of
//                independent marks (assignments) by transfers, and record the
//                essential rows and columns (Konig: they cover every mark).
//   Routine II - with d = min(u_i + v_j - r_ij) over inessential rows/columns,
//                lower the cover (Case 1 / Case 2) and return to Routine I.
// The loop ends when n marks are independent; then the rating sum equals the
// total allotment (Theorem 6).
//
// Ratings here are real numbers, so equality uses a small tolerance.
#pragma once

#include <limits>
#include <vector>

namespace search_exploration
{

constexpr double kInfeasible = std::numeric_limits<double>::infinity();

using Matrix = std::vector<std::vector<double>>;

struct KuhnResult
{
  std::vector<int> row_to_col;  // assignment: row i -> column row_to_col[i]
  std::vector<double> u;        // final cover, rows
  std::vector<double> v;        // final cover, columns
  double rating_sum{0.0};
  int iterations{0};
};

// Maximum-rating assignment of a square matrix with non-negative ratings.
KuhnResult kuhn_max_assignment(const Matrix & ratings);

struct Assignment
{
  std::vector<int> row_to_col;  // -1 if the row is unassigned
  double total_cost{0.0};
};

// Minimum-cost assignment of a rectangular R x M cost matrix. Entries equal to
// kInfeasible are never assigned. Each row gets at most one column and each
// column at most one row (PDF eqs. 4-6). The number of feasible assignments is
// maximised first, then their total cost is minimised.
Assignment min_cost_assignment(const Matrix & costs);

struct SlotAssignment
{
  std::vector<std::vector<int>> owned;  // robot -> owned column indices
  std::vector<int> unassigned;          // columns no robot owns
  double total_cost{0.0};
};

// Balanced ownership: each robot row is repeated `slots_per_robot` times, then
// one min_cost_assignment over the slots. slots_per_robot <= 0 means
// ceil(M / N). With slots_per_robot = 1 this is the one-frontier-per-robot form.
SlotAssignment assign_with_slots(const Matrix & costs, int slots_per_robot);

}  // namespace search_exploration
