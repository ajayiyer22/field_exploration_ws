#include "search_exploration/wfd.hpp"

#include <deque>

namespace search_exploration
{
namespace
{

// A cell can carry several marks at once (e.g. Map-Open and Frontier-Open), so
// each list from Fig. 4 is one bit.
constexpr uint8_t kMapOpen = 1U << 0;
constexpr uint8_t kMapClose = 1U << 1;
constexpr uint8_t kFrontierOpen = 1U << 2;
constexpr uint8_t kFrontierClose = 1U << 3;

bool has_open_space_neighbour(const Grid & grid, int cell)
{
  bool found = false;
  grid.for_each_neighbour(cell, [&](int n) {found = found || grid.open_space(n);});
  return found;
}

}  // namespace

bool is_frontier_point(const Grid & grid, int cell)
{
  return grid.unknown(cell) && has_open_space_neighbour(grid, cell);
}

int nearest_open_cell(const Grid & grid, int cell, int radius_cells)
{
  if (cell < 0) {
    return -1;
  }
  if (grid.open_space(cell)) {
    return cell;
  }
  const int cx = grid.cell_x(cell);
  const int cy = grid.cell_y(cell);
  std::vector<uint8_t> seen(grid.size(), 0);
  std::deque<int> queue{cell};
  seen[cell] = 1;
  while (!queue.empty()) {
    const int current = queue.front();
    queue.pop_front();
    if (grid.open_space(current)) {
      return current;
    }
    grid.for_each_neighbour(current, [&](int n) {
        if (!seen[n] && std::abs(grid.cell_x(n) - cx) <= radius_cells &&
        std::abs(grid.cell_y(n) - cy) <= radius_cells)
        {
          seen[n] = 1;
          queue.push_back(n);
        }
      });
  }
  return -1;
}

WfdResult detect_frontiers(
  const Grid & grid, const std::vector<Point2> & robot_positions, const WfdOptions & options)
{
  WfdResult result;
  if (grid.size() == 0 || static_cast<int>(grid.data.size()) != grid.size()) {
    return result;
  }
  std::vector<uint8_t> marks(grid.size(), 0);

  // Fig. 4 lines 1-3: queue_m <- {} ; ENQUEUE(queue_m, pose) ; mark pose Map-Open.
  // All robots seed one shared wavefront.
  std::deque<int> queue_m;
  for (const auto & position : robot_positions) {
    const int seed = nearest_open_cell(
      grid, grid.cell_at(position), options.seed_search_radius_cells);
    if (seed < 0 || (marks[seed] & kMapOpen)) {
      continue;
    }
    queue_m.push_back(seed);
    marks[seed] |= kMapOpen;
    result.seeds.push_back(seed);
  }

  // Lines 4-30.
  while (!queue_m.empty()) {
    const int p = queue_m.front();
    queue_m.pop_front();

    if (marks[p] & kMapClose) {  // lines 6-7
      continue;
    }

    if (is_frontier_point(grid, p)) {  // lines 8-25: extract the frontier containing p
      std::deque<int> queue_f{p};
      WfdFrontier frontier;
      frontier.start_cell = p;
      marks[p] |= kFrontierOpen;

      while (!queue_f.empty()) {
        const int q = queue_f.front();
        queue_f.pop_front();
        if (marks[q] & (kMapClose | kFrontierClose)) {  // lines 15-16
          continue;
        }
        if (is_frontier_point(grid, q)) {  // lines 17-22
          frontier.cells.push_back(q);
          grid.for_each_neighbour(q, [&](int w) {
              if (!(marks[w] & (kFrontierOpen | kFrontierClose | kMapClose))) {
                queue_f.push_back(w);
                marks[w] |= kFrontierOpen;
              }
            });
        }
        marks[q] |= kFrontierClose;  // line 23
      }

      for (const int cell : frontier.cells) {  // line 25
        marks[cell] |= kMapClose;
      }
      if (static_cast<int>(frontier.cells.size()) >= options.min_frontier_size_cells) {
        result.frontiers.push_back(std::move(frontier));  // line 24
      } else {
        ++result.rejected_small;
      }
    }

    grid.for_each_neighbour(p, [&](int v) {  // lines 26-29
        if (marks[v] & (kMapOpen | kMapClose)) {
          return;
        }
        if (options.block_occupied && grid.occupied(v)) {
          return;
        }
        if (has_open_space_neighbour(grid, v)) {
          queue_m.push_back(v);
          marks[v] |= kMapOpen;
        }
      });
    marks[p] |= kMapClose;  // line 30
  }
  return result;
}

}  // namespace search_exploration
