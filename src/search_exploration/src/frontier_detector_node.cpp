// Shared frontier detection: WFD on the shared occupancy grid from all robot
// positions, then frontier goal creation. Publishes every frontier with its
// geometry; it never ranks or assigns them.
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "search_exploration/allocation.hpp"
#include "search_exploration/frontier_goal.hpp"
#include "search_exploration/goal_costmap_filter.hpp"
#include "search_exploration/wfd.hpp"
#include "search_msgs/msg/frontier_array.hpp"

namespace search_exploration
{

class FrontierDetectorNode : public rclcpp::Node
{
public:
  FrontierDetectorNode()
  : Node("frontier_detector")
  {
    map_topic_ = declare_parameter<std::string>("map_topic", "/exploration_map/occupancy");
    frontiers_topic_ = declare_parameter<std::string>("frontiers_topic", "/search/frontiers");
    robot_namespaces_ = declare_parameter<std::vector<std::string>>(
      "robot_namespaces", std::vector<std::string>{});
    wfd_.min_frontier_size_cells = declare_parameter<int>("min_frontier_size_cells", 20);
    wfd_.block_occupied = declare_parameter<bool>("block_occupied", true);
    wfd_.seed_search_radius_cells = declare_parameter<int>("seed_search_radius_cells", 40);
    goal_options_.min_goal_distance_m = declare_parameter<double>("min_goal_distance_m", 0.0);
    occupied_threshold_ = declare_parameter<int>("occupied_threshold", 65);
    const double period = declare_parameter<double>("period_s", 1.0);
    tracker_ = FrontierTracker(declare_parameter<double>("id_match_distance_m", 0.75));

    if (robot_namespaces_.empty()) {
      throw std::invalid_argument("robot_namespaces must not be empty");
    }
    goal_filter_ = std::make_unique<GoalCostmapFilter>(*this, robot_namespaces_);

    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, latched, [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        map_ = std::move(msg);
        dirty_ = true;
      });
    poses_.resize(robot_namespaces_.size());
    for (std::size_t i = 0; i < robot_namespaces_.size(); ++i) {
      pose_subs_.push_back(create_subscription<geometry_msgs::msg::PoseStamped>(
          "/" + robot_namespaces_[i] + "/ground_truth/pose", rclcpp::SensorDataQoS(),
          [this, i](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {poses_[i] = *msg;}));
    }
    frontiers_pub_ = create_publisher<search_msgs::msg::FrontierArray>(frontiers_topic_, latched);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      frontiers_topic_ + "/markers", latched);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period), [this]() {process();});
  }

private:
  void process()
  {
    if (!map_ || (!dirty_ && goal_filter_->revision() == last_costmap_revision_)) {
      return;
    }
    std::vector<Point2> robots;
    for (const auto & pose : poses_) {
      if (!pose) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
          "Waiting for every robot's ground-truth pose before running WFD");
        return;
      }
      robots.push_back({pose->pose.position.x, pose->pose.position.y});
    }
    dirty_ = false;
    last_costmap_revision_ = goal_filter_->revision();

    Grid grid;
    grid.width = static_cast<int>(map_->info.width);
    grid.height = static_cast<int>(map_->info.height);
    grid.resolution = map_->info.resolution;
    grid.origin_x = map_->info.origin.position.x;
    grid.origin_y = map_->info.origin.position.y;
    grid.data = map_->data;
    grid.occupied_threshold = occupied_threshold_;

    const auto start = now();
    const auto wfd = detect_frontiers(grid, robots, wfd_);
    // Search the frontier's free neighbours for an endpoint that all robots
    // accept in their global costmaps, instead of first picking a high-cost centroid neighbour.
    // Keep blocked frontiers in the published geometry: filtering them away
    // would incorrectly tell the coordinator that exploration is complete.
    const CellBlocked high_cost = [&](int cell) {
        const auto world = grid.world(cell);
        geometry_msgs::msg::Point point;
        point.x = world.x;
        point.y = world.y;
        return !all_robots_accept(point, map_->header.frame_id);
      };
    std::vector<FrontierCandidate> candidates;
    std::size_t without_low_cost_goal = 0;
    for (const auto & frontier : wfd.frontiers) {
      auto candidate = build_frontier_candidate(grid, frontier, robots, goal_options_, high_cost);
      if (!candidate) {
        ++without_low_cost_goal;
        candidate = build_frontier_candidate(grid, frontier, robots, goal_options_);
      }
      if (candidate) {candidates.push_back(*candidate);}
    }

    std::vector<Point2> goals;
    for (const auto & c : candidates) {
      goals.push_back(c.goal);
    }
    const auto ids = tracker_.update(goals);

    search_msgs::msg::FrontierArray out;
    out.header.stamp = map_->header.stamp;
    out.header.frame_id = map_->header.frame_id;
    out.generation = ++generation_;
    for (std::size_t k = 0; k < candidates.size(); ++k) {
      const auto & c = candidates[k];
      search_msgs::msg::Frontier f;
      f.id = ids[k];
      f.centroid.x = c.centroid.x;
      f.centroid.y = c.centroid.y;
      f.center_point.x = c.center_point.x;
      f.center_point.y = c.center_point.y;
      f.start_point.x = c.start_point.x;
      f.start_point.y = c.start_point.y;
      f.goal.position.x = c.goal.x;
      f.goal.position.y = c.goal.y;
      // Face from the goal towards the unknown side (the frontier centroid).
      const double yaw = std::atan2(c.centroid.y - c.goal.y, c.centroid.x - c.goal.x);
      f.goal.orientation.z = std::sin(yaw / 2.0);
      f.goal.orientation.w = std::cos(yaw / 2.0);
      f.size = static_cast<uint32_t>(c.size);
      out.frontiers.push_back(f);
    }
    frontiers_pub_->publish(out);
    publish_markers(grid, wfd, out);

    if (out.frontiers.size() != last_count_) {
      RCLCPP_INFO(get_logger(),
        "WFD: %zu frontiers (%d rejected as small, %zu without a free goal, "
        "%zu without a low-cost goal) in %.1f ms",
        out.frontiers.size(), wfd.rejected_small, wfd.frontiers.size() - candidates.size(),
        without_low_cost_goal,
        (now() - start).seconds() * 1000.0);
      last_count_ = out.frontiers.size();
    }
  }

  bool all_robots_accept(const geometry_msgs::msg::Point & point, const std::string & frame) const
  {
    return goal_filter_->accepts_all(point, frame);
  }

  void publish_markers(
    const Grid & grid, const WfdResult & wfd, const search_msgs::msg::FrontierArray & out)
  {
    visualization_msgs::msg::MarkerArray markers;
    auto make = [&](const std::string & ns, int type, double scale, float r, float g, float b) {
        visualization_msgs::msg::Marker m;
        m.header = out.header;
        m.ns = ns;
        m.id = 0;
        m.type = type;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = scale;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = 1.0F;
        return m;
      };
    auto cells = make("frontier_cells", visualization_msgs::msg::Marker::POINTS,
        grid.resolution, 1.0F, 0.55F, 0.0F);
    for (const auto & frontier : wfd.frontiers) {
      for (const int cell : frontier.cells) {
        const auto w = grid.world(cell);
        geometry_msgs::msg::Point p;
        p.x = w.x;
        p.y = w.y;
        p.z = 0.02;
        cells.points.push_back(p);
      }
    }
    auto centroids = make("frontier_centroids", visualization_msgs::msg::Marker::SPHERE_LIST,
        0.2, 0.3F, 0.1F, 0.6F);
    auto goals = make("frontier_goals", visualization_msgs::msg::Marker::SPHERE_LIST,
        0.25, 0.0F, 0.75F, 0.75F);
    for (const auto & f : out.frontiers) {
      geometry_msgs::msg::Point c = f.centroid;
      c.z = 0.1;
      centroids.points.push_back(c);
      geometry_msgs::msg::Point g = f.goal.position;
      g.z = 0.1;
      if (all_robots_accept(f.goal.position, out.header.frame_id)) {
        goals.points.push_back(g);
      }
    }
    auto labels_clear = make("frontier_ids", visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
        0.3, 1.0F, 1.0F, 1.0F);
    labels_clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(labels_clear);
    markers.markers.push_back(cells);
    markers.markers.push_back(centroids);
    markers.markers.push_back(goals);
    int label_id = 1;
    for (const auto & f : out.frontiers) {
      // Allocated goal labels come from the allocator (H1/J11/J21 etc.).
      // Keep text here only for frontiers without a common low-cost goal.
      if (all_robots_accept(f.goal.position, out.header.frame_id)) {continue;}
      auto label = make("frontier_ids", visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
          0.35, 1.0F, 1.0F, 1.0F);
      label.id = label_id++;
      const bool eligible = all_robots_accept(f.goal.position, out.header.frame_id);
      label.pose.position = eligible ? f.goal.position : f.centroid;
      label.pose.position.z = 0.5;
      label.text = std::to_string(f.id) + " (" + std::to_string(f.size) + ")";
      if (!eligible) {label.text += " unassigned";}
      markers.markers.push_back(label);
    }
    markers_pub_->publish(markers);
  }

  std::string map_topic_;
  std::string frontiers_topic_;
  std::vector<std::string> robot_namespaces_;
  WfdOptions wfd_;
  GoalOptions goal_options_;
  int occupied_threshold_{65};
  FrontierTracker tracker_;
  std::unique_ptr<GoalCostmapFilter> goal_filter_;
  uint64_t last_costmap_revision_{0};
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  std::vector<std::optional<geometry_msgs::msg::PoseStamped>> poses_;
  bool dirty_{false};
  uint64_t generation_{0};
  std::size_t last_count_{static_cast<std::size_t>(-1)};
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subs_;
  rclcpp::Publisher<search_msgs::msg::FrontierArray>::SharedPtr frontiers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace search_exploration

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<search_exploration::FrontierDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
