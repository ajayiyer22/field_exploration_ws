// Frontier goals use only fresh Nav2 global raw costmaps.
// A common goal must cost less than 30 in every robot's global map.
#pragma once

#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <rclcpp/rclcpp.hpp>

namespace search_exploration
{

inline bool valid_goal_costmap(const nav2_msgs::msg::Costmap & costmap)
{
  const auto & m = costmap.metadata;
  const auto & p = m.origin.position;
  const auto & q = m.origin.orientation;
  const auto expected = static_cast<std::size_t>(m.size_x) * m.size_y;
  return !costmap.header.frame_id.empty() && expected != 0U &&
         costmap.data.size() == expected && std::isfinite(m.resolution) && m.resolution > 0.0 &&
         std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(q.x) &&
         std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
         q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 1e-12;
}

inline std::optional<unsigned int> goal_cell_cost(
  const nav2_msgs::msg::Costmap & costmap, const geometry_msgs::msg::Point & point,
  const std::string & frame)
{
  const auto & metadata = costmap.metadata;
  if (!valid_goal_costmap(costmap) || frame != costmap.header.frame_id ||
    !std::isfinite(point.x) || !std::isfinite(point.y))
  {
    return std::nullopt;
  }
  const auto & rotation = metadata.origin.orientation;
  const double yaw = std::atan2(
    2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
    1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z));
  const double dx = point.x - metadata.origin.position.x;
  const double dy = point.y - metadata.origin.position.y;
  const double cell_x = std::floor((std::cos(yaw) * dx + std::sin(yaw) * dy) / metadata.resolution);
  const double cell_y = std::floor((-std::sin(yaw) * dx + std::cos(yaw) * dy) / metadata.resolution);
  if (!std::isfinite(cell_x) || !std::isfinite(cell_y) || cell_x < 0 || cell_y < 0 ||
    cell_x >= metadata.size_x || cell_y >= metadata.size_y)
  {
    return std::nullopt;
  }
  return costmap.data[static_cast<std::size_t>(cell_y) * metadata.size_x +
         static_cast<std::size_t>(cell_x)];
}

class GoalCostmapFilter
{
public:
  GoalCostmapFilter(rclcpp::Node & node, const std::vector<std::string> & namespaces)
  : clock_(node.get_clock()), costmaps_(namespaces.size())
  {
    max_cost_ = node.declare_parameter<std::vector<double>>(
      "goal_max_cost", std::vector<double>(namespaces.size(), 29.0));
    max_age_ = node.declare_parameter<double>("goal_costmap_timeout_sec", 3.0);
    if (max_cost_.size() != namespaces.size() || !std::isfinite(max_age_) || max_age_ <= 0.0) {
      throw std::invalid_argument("goal_max_cost needs one value per robot; timeout positive");
    }
    for (const double cost : max_cost_) {
      if (!std::isfinite(cost) || cost < 0.0 || cost > 252.0) {
        throw std::invalid_argument("goal_max_cost values must be within 0..252");
      }
    }
    for (std::size_t index = 0; index < namespaces.size(); ++index) {
      const auto topic = "/" + namespaces[index] + "/global_costmap/costmap_raw";
      subscriptions_.push_back(node.create_subscription<nav2_msgs::msg::Costmap>(
          topic, rclcpp::QoS(1).reliable().transient_local(),
          [this, index](nav2_msgs::msg::Costmap::ConstSharedPtr message) {
            costmaps_[index] = std::move(message);
            ++revision_;
          }));
    }
  }

  bool has_costmap(std::size_t robot) const
  {
    return costmaps_.at(robot) != nullptr;
  }
  uint64_t revision() const {return revision_;}

  std::optional<unsigned int> cost(
    std::size_t robot, const geometry_msgs::msg::Point & point, const std::string & frame) const
  {
    const auto & costmap = costmaps_.at(robot);
    if (!fresh(costmap)) {
      return std::nullopt;
    }
    return goal_cell_cost(*costmap, point, frame);
  }

  bool accepts(
    std::size_t robot, const geometry_msgs::msg::Point & point, const std::string & frame) const
  {
    const auto value = cost(robot, point, frame);
    return value && *value <= static_cast<unsigned int>(max_cost_.at(robot));
  }

  bool accepts_all(const geometry_msgs::msg::Point & point, const std::string & frame) const
  {
    for (std::size_t robot = 0; robot < costmaps_.size(); ++robot) {
      if (!accepts(robot, point, frame)) {return false;}
    }
    return true;
  }

private:
  bool fresh(const nav2_msgs::msg::Costmap::ConstSharedPtr & costmap) const
  {
    if (!costmap) {
      return false;
    }
    const auto stamp = rclcpp::Time(costmap->header.stamp, clock_->get_clock_type());
    const double age = (clock_->now() - stamp).seconds();
    return age >= 0.0 && age <= max_age_;
  }

  rclcpp::Clock::SharedPtr clock_;
  std::vector<double> max_cost_;
  double max_age_;
  uint64_t revision_{0};
  std::vector<nav2_msgs::msg::Costmap::ConstSharedPtr> costmaps_;
  std::vector<rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr> subscriptions_;
};

}  // namespace search_exploration
