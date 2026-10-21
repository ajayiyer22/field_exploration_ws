// Mission state for the exploration run.
//
//   WAITING              until every robot pose, the shared map and every Nav2
//                        NavigateToPose server are available
//   EXPLORING            allocation and navigation run
//   EXPLORATION_COMPLETE WFD reports no frontiers, allocation agrees, and fresh
//                        robot status is idle, continuously for complete_after_s
//   TARGET_FOUND         /search/target_found (reserved for target detection)
//
// On completion a CSV summary (time, known area, per-robot distance) is written.
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>

#include "search_msgs/msg/frontier_array.hpp"
#include "search_msgs/msg/mission_state.hpp"
#include "search_msgs/msg/robot_status.hpp"
#include "search_msgs/msg/task_plan.hpp"

namespace search_exploration
{

class MissionCoordinatorNode : public rclcpp::Node
{
  using State = search_msgs::msg::MissionState;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

public:
  MissionCoordinatorNode()
  : Node("mission_coordinator")
  {
    namespaces_ = declare_parameter<std::vector<std::string>>(
      "robot_namespaces", std::vector<std::string>{});
    if (namespaces_.empty()) {
      throw std::invalid_argument("robot_namespaces must not be empty");
    }
    complete_after_s_ = declare_parameter<double>("complete_after_s", 15.0);
    start_delay_s_ = declare_parameter<double>("start_delay_s", 2.0);
    observation_timeout_s_ = declare_parameter<double>("observation_timeout_s", 3.0);
    if (!std::isfinite(complete_after_s_) || complete_after_s_ <= 0.0 ||
      !std::isfinite(start_delay_s_) || start_delay_s_ < 0.0 ||
      !std::isfinite(observation_timeout_s_) || observation_timeout_s_ <= 0.0)
    {
      throw std::invalid_argument("mission delays must be finite; completion and timeout positive");
    }
    results_dir_ = declare_parameter<std::string>("results_dir", default_results_dir());
    const auto action_name = declare_parameter<std::string>("action_name", "navigate_to_pose");

    const std::size_t n = namespaces_.size();
    poses_.resize(n);
    distance_.assign(n, 0.0);
    statuses_.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
      clients_.push_back(rclcpp_action::create_client<NavigateToPose>(
          this, "/" + namespaces_[i] + "/" + action_name));
      pose_subs_.push_back(create_subscription<geometry_msgs::msg::PoseStamped>(
          "/" + namespaces_[i] + "/ground_truth/pose", rclcpp::SensorDataQoS(),
          [this, i](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {on_pose(i, *msg);}));
    }
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/exploration_map/occupancy", latched,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {map_ = msg;});
    frontiers_sub_ = create_subscription<search_msgs::msg::FrontierArray>(
      "/search/frontiers", latched,
      [this](search_msgs::msg::FrontierArray::ConstSharedPtr msg) {
        if (frontiers_ && msg->generation <= frontiers_->generation) {return;}
        if (!msg->frontiers.empty()) {
          empty_since_generation_.reset();
          quiet_since_.reset();
        } else if (!empty_since_generation_) {
          empty_since_generation_ = msg->generation;
        }
        frontiers_ = msg;
      });
    plan_sub_ = create_subscription<search_msgs::msg::TaskPlan>(
      "/search/task_plan", latched,
      [this](search_msgs::msg::TaskPlan::ConstSharedPtr msg) {plan_ = msg;});
    status_sub_ = create_subscription<search_msgs::msg::RobotStatus>(
      "/search/robot_status", rclcpp::QoS(20).reliable(),
      [this](search_msgs::msg::RobotStatus::ConstSharedPtr msg) {
        for (std::size_t i = 0; i < namespaces_.size(); ++i) {
          if (namespaces_[i] == msg->robot_namespace) {
            statuses_[i] = *msg;
            if (msg->state != search_msgs::msg::RobotStatus::IDLE || msg->has_active_goal) {
              quiet_since_.reset();
            }
          }
        }
      });
    target_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/search/target_found", latched, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (msg->data && state_ == State::EXPLORING) {
          finish(State::TARGET_FOUND, "target confirmed");
        }
      });
    state_pub_ = create_publisher<State>("/search/mission_state", latched);
    publish("waiting for robots, map and Nav2");
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.5), [this]() {tick();});
  }

private:
  static std::string default_results_dir()
  {
    const char * ws = std::getenv("FIELD_EXPLORATION_WS");
    return ws ? std::string(ws) + "/results" : std::string("results");
  }

  void on_pose(std::size_t i, const geometry_msgs::msg::PoseStamped & msg)
  {
    if (poses_[i] && state_ == State::EXPLORING) {
      distance_[i] += std::hypot(
        msg.pose.position.x - poses_[i]->pose.position.x,
        msg.pose.position.y - poses_[i]->pose.position.y);
    }
    poses_[i] = msg;
  }

  void publish(const std::string & detail)
  {
    State msg;
    msg.header.stamp = now();
    msg.state = state_;
    msg.detail = detail;
    state_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Mission state %u: %s", state_, detail.c_str());
  }

  void tick()
  {
    if (state_ == State::WAITING) {
      std::string missing;
      for (std::size_t i = 0; i < namespaces_.size(); ++i) {
        if (!poses_[i]) {missing += " pose:" + namespaces_[i];}
        if (!clients_[i]->action_server_is_ready()) {missing += " nav2:" + namespaces_[i];}
      }
      if (!map_) {missing += " map";}
      if (!missing.empty()) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "Waiting for%s", missing.c_str());
        ready_since_.reset();
        return;
      }
      if (!ready_since_) {
        ready_since_ = now();
      }
      if ((now() - *ready_since_).seconds() >= start_delay_s_) {
        state_ = State::EXPLORING;
        start_time_ = now();
        publish("exploring");
      }
      return;
    }
    if (state_ != State::EXPLORING) {
      return;
    }

    // Empty ownership can result from temporary failure blocks or unavailable
    // costmaps. It does not prove that any remaining frontier is unreachable.
    if (completion_ready()) {
      if (!quiet_since_) {
        quiet_since_ = now();
      }
      if ((now() - *quiet_since_).seconds() >= complete_after_s_) {
        finish(State::EXPLORATION_COMPLETE, "exploration complete: no frontiers remain");
      }
    } else {
      quiet_since_.reset();
    }
  }

  bool fresh(const builtin_interfaces::msg::Time & stamp) const
  {
    const double age = (now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds();
    return age >= 0.0 && age <= observation_timeout_s_;
  }

  bool completion_ready() const
  {
    if (!map_ || !frontiers_ || !plan_ || !empty_since_generation_ ||
      !frontiers_->frontiers.empty() || !fresh(map_->header.stamp) ||
      !fresh(frontiers_->header.stamp) ||
      plan_->frontier_generation < *empty_since_generation_ ||
      plan_->frontier_generation > frontiers_->generation ||
      !plan_->unassigned_frontier_ids.empty() || plan_->tasks.size() != namespaces_.size() ||
      !start_time_ || rclcpp::Time(plan_->header.stamp, get_clock()->get_clock_type()) < *start_time_)
    {
      return false;
    }
    // One empty route for every robot; do not accept missing/duplicate routes.
    for (std::size_t i = 0; i < namespaces_.size(); ++i) {
      const auto & status = statuses_[i];
      if (!status || status->state != search_msgs::msg::RobotStatus::IDLE ||
        status->has_active_goal || !fresh(status->header.stamp) ||
        !poses_[i] || !fresh(poses_[i]->header.stamp) ||
        !clients_[i]->action_server_is_ready())
      {
        return false;
      }
      std::size_t matches = 0;
      for (const auto & task : plan_->tasks) {
        if (task.robot_namespace == namespaces_[i]) {
          ++matches;
          if (!task.frontier_ids.empty() || !task.goals.empty() || !task.assignment_costs.empty()) {
            return false;
          }
        }
      }
      if (matches != 1) {return false;}
    }
    return true;
  }

  void finish(uint8_t state, const std::string & detail)
  {
    state_ = state;
    publish(detail);
    write_results(detail);
  }

  void write_results(const std::string & detail)
  {
    const double elapsed = start_time_ ? (now() - *start_time_).seconds() : 0.0;
    double known_area = 0.0;
    if (map_) {
      std::size_t known = 0;
      for (const auto v : map_->data) {
        known += v >= 0 ? 1 : 0;
      }
      known_area = known * map_->info.resolution * map_->info.resolution;
    }
    std::error_code error;
    std::filesystem::create_directories(results_dir_, error);
    const auto stamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::ostringstream name;
    name << results_dir_ << "/exploration_" << std::put_time(std::localtime(&stamp), "%Y%m%d_%H%M%S")
         << ".csv";
    std::ofstream csv(name.str());
    csv << "outcome,elapsed_sim_s,known_area_m2,remaining_frontiers";
    for (const auto & ns : namespaces_) {
      csv << ",distance_" << ns << "_m";
    }
    csv << "\n\"" << detail << "\"," << elapsed << "," << known_area << ","
        << (frontiers_ ? frontiers_->frontiers.size() : 0);
    double total = 0.0;
    for (const double d : distance_) {
      csv << "," << d;
      total += d;
    }
    csv << "\n";
    RCLCPP_INFO(get_logger(),
      "%s after %.1f s: known area %.1f m2, total distance %.1f m -> %s",
      detail.c_str(), elapsed, known_area, total, name.str().c_str());
  }

  std::vector<std::string> namespaces_;
  double complete_after_s_{15.0};
  double start_delay_s_{2.0};
  double observation_timeout_s_{3.0};
  std::string results_dir_;
  uint8_t state_{State::WAITING};
  std::optional<rclcpp::Time> ready_since_;
  std::optional<rclcpp::Time> start_time_;
  std::optional<rclcpp::Time> quiet_since_;
  std::vector<std::optional<geometry_msgs::msg::PoseStamped>> poses_;
  std::vector<double> distance_;
  std::optional<uint64_t> empty_since_generation_;
  std::vector<std::optional<search_msgs::msg::RobotStatus>> statuses_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  search_msgs::msg::FrontierArray::ConstSharedPtr frontiers_;
  search_msgs::msg::TaskPlan::ConstSharedPtr plan_;
  std::vector<rclcpp_action::Client<NavigateToPose>::SharedPtr> clients_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subs_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<search_msgs::msg::FrontierArray>::SharedPtr frontiers_sub_;
  rclcpp::Subscription<search_msgs::msg::TaskPlan>::SharedPtr plan_sub_;
  rclcpp::Subscription<search_msgs::msg::RobotStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_sub_;
  rclcpp::Publisher<State>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace search_exploration

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<search_exploration::MissionCoordinatorNode>());
  rclcpp::shutdown();
  return 0;
}
