// Wavefront Frontier Detection (WFD).
//
// Topiwala, Inani, Kathpal, "Frontier Based Exploration for Autonomous Robot",
// arXiv:1806.03581, Section III and the pseudo-code in Fig. 4.
//
//   Frontier point: an unknown cell with at least one open-space neighbour.
//   Frontier:       a connected set of frontier points.
//
// The outer BFS (queue_m) starts at the robot cells and only expands into cells
// that have at least one open-space neighbour (Fig. 4, line 27), so only known
// regions are scanned. When it reaches a frontier point, the inner BFS (queue_f)
// extracts that whole frontier. The four marks below ensure every frontier
// point belongs to exactly one frontier.
#pragma once

#include <cstdint>
#include <vector>

#include "search_exploration/grid.hpp"

namespace search_exploration
{

enum class WfdMark : uint8_t
{
  NONE = 0,
  MAP_OPEN,        // enqueued by the outer BFS
  MAP_CLOSE,       // dequeued by the outer BFS
  FRONTIER_OPEN,   // enqueued by the inner BFS
  FRONTIER_CLOSE,  // dequeued by the inner BFS
};

struct WfdOptions
{
  // Frontiers with fewer cells are rejected.
  int min_frontier_size_cells{20};
  // Fig. 4 line 27 only requires an open-space neighbour. Occupied cells next to
  // open space satisfy that too, which lets the wavefront step through a wall
  // into space a robot cannot reach. When true, occupied cells are never
  // enqueued by the outer BFS.
  bool block_occupied{true};
  // Search radius (cells) for a free seed when a robot cell is not open space.
  int seed_search_radius_cells{40};
};

struct WfdFrontier
{
  std::vector<int> cells;  // frontier point indices
  int start_cell{-1};      // the frontier point where the inner BFS started
};

struct WfdResult
{
  std::vector<WfdFrontier> frontiers;
  std::vector<int> seeds;  // outer-BFS seed cells actually used
  int rejected_small{0};
};

// True if the cell is unknown and has at least one open-space neighbour.
bool is_frontier_point(const Grid & grid, int cell);

// Nearest open-space cell to `cell` within the radius (BFS over the grid), or -1.
int nearest_open_cell(const Grid & grid, int cell, int radius_cells);

// Runs WFD from all robot positions at once (one shared wavefront).
WfdResult detect_frontiers(
  const Grid & grid, const std::vector<Point2> & robot_positions,
  const WfdOptions & options = {});

}  // namespace search_exploration
