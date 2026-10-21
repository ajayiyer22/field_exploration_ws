#include "search_exploration/hungarian.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <stdexcept>

namespace search_exploration
{
namespace
{

struct Routine1Output
{
  int marks{0};
  std::vector<bool> essential_row;
  std::vector<bool> essential_col;
};

// Routine I. `row_match` / `col_match` hold the current independent marks (1*)
// and are extended by transfers until none frees a qualified column.
Routine1Output routine_one(
  const Matrix & r, const std::vector<double> & u, const std::vector<double> & v,
  double eps, std::vector<int> & row_match, std::vector<int> & col_match)
{
  const int n = static_cast<int>(r.size());
  auto qualified = [&](int i, int j) {return std::abs(u[i] + v[j] - r[i][j]) <= eps;};

  // A transfer starts at an eligible column (no 1*), moves a qualified row into
  // it, frees that row's old column, and so on. Reaching an unassigned
  // qualified row completes the transfer and adds one mark (Alternative Ia).
  bool improved = true;
  while (improved) {
    improved = false;
    for (int j0 = 0; j0 < n; ++j0) {
      if (col_match[j0] >= 0) {
        continue;
      }
      std::vector<int> parent_col(n, -1);  // row -> column it would move into
      std::vector<bool> seen_row(n, false);
      std::deque<int> columns{j0};
      int free_row = -1;
      while (!columns.empty() && free_row < 0) {
        const int j = columns.front();
        columns.pop_front();
        for (int i = 0; i < n && free_row < 0; ++i) {
          if (seen_row[i] || !qualified(i, j)) {
            continue;
          }
          seen_row[i] = true;
          parent_col[i] = j;
          if (row_match[i] < 0) {
            free_row = i;
          } else {
            columns.push_back(row_match[i]);
          }
        }
      }
      if (free_row >= 0) {  // carry out the transfer
        int i = free_row;
        while (i >= 0) {
          const int j = parent_col[i];
          const int previous_row = col_match[j];
          row_match[i] = j;
          col_match[j] = i;
          i = (previous_row >= 0 && previous_row != i) ? previous_row : -1;
        }
        improved = true;
      }
    }
  }

  // Alternative Ib: every transfer leads to a complete assignment. Rows reached
  // by a transfer are essential; columns assigned to inessential rows are
  // essential (Section 2, Lemma 1 and Theorem 1).
  Routine1Output out;
  out.essential_row.assign(n, false);
  out.essential_col.assign(n, false);
  std::deque<int> columns;
  std::vector<bool> seen_col(n, false);
  for (int j = 0; j < n; ++j) {
    if (col_match[j] < 0) {
      columns.push_back(j);
      seen_col[j] = true;
    }
  }
  while (!columns.empty()) {
    const int j = columns.front();
    columns.pop_front();
    for (int i = 0; i < n; ++i) {
      if (!out.essential_row[i] && qualified(i, j) && row_match[i] >= 0) {
        out.essential_row[i] = true;
        const int next = row_match[i];
        if (!seen_col[next]) {
          seen_col[next] = true;
          columns.push_back(next);
        }
      }
    }
  }
  for (int i = 0; i < n; ++i) {
    if (row_match[i] >= 0) {
      ++out.marks;
      if (!out.essential_row[i]) {
        out.essential_col[row_match[i]] = true;
      }
    }
  }
  return out;
}

}  // namespace

KuhnResult kuhn_max_assignment(const Matrix & ratings)
{
  const int n = static_cast<int>(ratings.size());
  KuhnResult result;
  if (n == 0) {
    return result;
  }
  double max_rating = 0.0;
  for (const auto & row : ratings) {
    if (static_cast<int>(row.size()) != n) {
      throw std::invalid_argument("kuhn_max_assignment requires a square matrix");
    }
    for (const double value : row) {
      if (!std::isfinite(value) || value < 0.0) {
        throw std::invalid_argument("ratings must be finite and non-negative");
      }
      max_rating = std::max(max_rating, value);
    }
  }
  const double eps = 1e-9 * std::max(1.0, max_rating);

  // Initial cover (Section 4): a = sum of row maxima, b = sum of column maxima.
  std::vector<double> row_max(n, 0.0);
  std::vector<double> col_max(n, 0.0);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      row_max[i] = std::max(row_max[i], ratings[i][j]);
      col_max[j] = std::max(col_max[j], ratings[i][j]);
    }
  }
  double a = 0.0;
  double b = 0.0;
  for (int k = 0; k < n; ++k) {
    a += row_max[k];
    b += col_max[k];
  }
  std::vector<double> u(n, 0.0);
  std::vector<double> v(n, 0.0);
  if (a <= b) {
    u = row_max;
  } else {
    v = col_max;
  }

  // Initial marks: the first qualified position in each row (a <= b) or column
  // (a > b) whose column (row) has no mark yet.
  std::vector<int> row_match(n, -1);
  std::vector<int> col_match(n, -1);
  auto qualified = [&](int i, int j) {return std::abs(u[i] + v[j] - ratings[i][j]) <= eps;};
  for (int outer = 0; outer < n; ++outer) {
    for (int inner = 0; inner < n; ++inner) {
      const int i = a <= b ? outer : inner;
      const int j = a <= b ? inner : outer;
      if (row_match[i] < 0 && col_match[j] < 0 && qualified(i, j)) {
        row_match[i] = j;
        col_match[j] = i;
        break;
      }
    }
  }

  const int max_iterations = 50 * n * n + 100;
  for (result.iterations = 0; result.iterations < max_iterations; ++result.iterations) {
    const auto routine = routine_one(ratings, u, v, eps, row_match, col_match);
    if (routine.marks == n) {
      break;  // Theorem 6: the marks are an optimal assignment
    }

    // Routine II.
    double d = std::numeric_limits<double>::infinity();
    for (int i = 0; i < n; ++i) {
      if (routine.essential_row[i]) {
        continue;
      }
      for (int j = 0; j < n; ++j) {
        if (!routine.essential_col[j]) {
          d = std::min(d, u[i] + v[j] - ratings[i][j]);
        }
      }
    }
    if (!std::isfinite(d)) {
      break;
    }
    bool some_inessential_row_zero = false;
    double min_u = std::numeric_limits<double>::infinity();
    for (int i = 0; i < n; ++i) {
      if (!routine.essential_row[i]) {
        min_u = std::min(min_u, u[i]);
        some_inessential_row_zero = some_inessential_row_zero || u[i] <= eps;
      }
    }
    if (!some_inessential_row_zero) {
      // Case 1: lower inessential rows, raise essential columns.
      const double m = std::min(d, min_u);
      for (int i = 0; i < n; ++i) {
        if (!routine.essential_row[i]) {u[i] -= m;}
      }
      for (int j = 0; j < n; ++j) {
        if (routine.essential_col[j]) {v[j] += m;}
      }
    } else {
      // Case 2: raise essential rows, lower inessential columns.
      double min_v = std::numeric_limits<double>::infinity();
      for (int j = 0; j < n; ++j) {
        if (!routine.essential_col[j]) {min_v = std::min(min_v, v[j]);}
      }
      const double m = std::min(d, min_v);
      for (int i = 0; i < n; ++i) {
        if (routine.essential_row[i]) {u[i] += m;}
      }
      for (int j = 0; j < n; ++j) {
        if (!routine.essential_col[j]) {v[j] -= m;}
      }
    }
  }

  result.row_to_col = row_match;
  result.u = u;
  result.v = v;
  for (int i = 0; i < n; ++i) {
    if (row_match[i] >= 0) {
      result.rating_sum += ratings[i][row_match[i]];
    }
  }
  return result;
}

Assignment min_cost_assignment(const Matrix & costs)
{
  Assignment out;
  const int rows = static_cast<int>(costs.size());
  if (rows == 0) {
    return out;
  }
  const int cols = static_cast<int>(costs.front().size());
  out.row_to_col.assign(rows, -1);
  if (cols == 0) {
    return out;
  }

  double max_cost = 0.0;
  for (const auto & row : costs) {
    if (static_cast<int>(row.size()) != cols) {
      throw std::invalid_argument("cost matrix rows must have equal length");
    }
    for (const double c : row) {
      if (std::isfinite(c)) {
        if (c < 0.0) {
          throw std::invalid_argument("costs must be non-negative");
        }
        max_cost = std::max(max_cost, c);
      }
    }
  }

  // Square the problem with dummy rows/columns (rating 0). Feasible pairs get
  // rating K - C with K > n * max_cost, so one more feasible assignment always
  // outweighs any cost saving; infeasible pairs get 0 like dummies.
  const int n = std::max(rows, cols);
  const double k = (n + 1) * (max_cost + 1.0);
  Matrix ratings(n, std::vector<double>(n, 0.0));
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (std::isfinite(costs[i][j])) {
        ratings[i][j] = k - costs[i][j];
      }
    }
  }

  const auto kuhn = kuhn_max_assignment(ratings);
  for (int i = 0; i < rows; ++i) {
    const int j = kuhn.row_to_col[i];
    if (j >= 0 && j < cols && std::isfinite(costs[i][j])) {
      out.row_to_col[i] = j;
      out.total_cost += costs[i][j];
    }
  }
  return out;
}

SlotAssignment assign_with_slots(const Matrix & costs, int slots_per_robot)
{
  SlotAssignment out;
  const int robots = static_cast<int>(costs.size());
  out.owned.assign(robots, {});
  if (robots == 0) {
    return out;
  }
  const int frontiers = static_cast<int>(costs.front().size());
  if (frontiers == 0) {
    return out;
  }
  const int slots = slots_per_robot > 0 ?
    slots_per_robot : (frontiers + robots - 1) / robots;

  Matrix expanded;
  std::vector<int> slot_robot;
  expanded.reserve(static_cast<std::size_t>(robots) * slots);
  for (int robot = 0; robot < robots; ++robot) {
    for (int s = 0; s < slots; ++s) {
      expanded.push_back(costs[robot]);
      slot_robot.push_back(robot);
    }
  }

  const auto assignment = min_cost_assignment(expanded);
  std::vector<bool> claimed(frontiers, false);
  for (std::size_t slot = 0; slot < assignment.row_to_col.size(); ++slot) {
    const int j = assignment.row_to_col[slot];
    if (j < 0) {
      continue;
    }
    if (claimed[j]) {
      throw std::logic_error("frontier claimed by two slots");
    }
    claimed[j] = true;
    out.owned[slot_robot[slot]].push_back(j);
  }
  out.total_cost = assignment.total_cost;
  for (int j = 0; j < frontiers; ++j) {
    if (!claimed[j]) {
      out.unassigned.push_back(j);
    }
  }
  for (auto & owned : out.owned) {
    std::sort(owned.begin(), owned.end());
  }
  return out;
}

}  // namespace search_exploration
