// Plain occupancy-grid view shared by the exploration libraries. No ROS types,
// so the algorithms can be unit-tested on synthetic grids.
#pragma once

#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

namespace search_exploration
{

struct Point2
{
  double x{0.0};
  double y{0.0};
};

inline double distance(const Point2 & a, const Point2 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

inline double squared_distance(const Point2 & a, const Point2 & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

// Row-major nav_msgs/OccupancyGrid layout: index = y * width + x.
// Values: -1 unknown, 0..100 occupancy probability.
struct Grid
{
  int width{0};
  int height{0};
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<int8_t> data;
  // Cells at or above this value are occupied; known cells below it are open space.
  int occupied_threshold{65};

  int size() const {return width * height;}
  int index(int x, int y) const {return y * width + x;}
  int cell_x(int index) const {return index % width;}
  int cell_y(int index) const {return index / width;}
  bool inside(int x, int y) const {return x >= 0 && y >= 0 && x < width && y < height;}

  bool unknown(int index) const {return data[index] < 0;}
  bool occupied(int index) const {return data[index] >= occupied_threshold;}
  bool open_space(int index) const
  {
    return data[index] >= 0 && data[index] < occupied_threshold;
  }

  // World coordinates of the cell centre.
  Point2 world(int index) const
  {
    return {
      origin_x + (cell_x(index) + 0.5) * resolution,
      origin_y + (cell_y(index) + 0.5) * resolution};
  }

  // Cell index containing a world point, or -1 if outside the grid.
  int cell_at(const Point2 & point) const
  {
    const int x = static_cast<int>(std::floor((point.x - origin_x) / resolution));
    const int y = static_cast<int>(std::floor((point.y - origin_y) / resolution));
    return inside(x, y) ? index(x, y) : -1;
  }

  // Calls f(neighbour_index) for the 8-connected neighbours inside the grid.
  template<typename F>
  void for_each_neighbour(int cell, F && f) const
  {
    const int cx = cell_x(cell);
    const int cy = cell_y(cell);
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if ((dx != 0 || dy != 0) && inside(cx + dx, cy + dy)) {
          f(index(cx + dx, cy + dy));
        }
      }
    }
  }
};

}  // namespace search_exploration
