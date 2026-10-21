// Hungarian ownership with robot-to-goal Euclidean costs and distance-ordered queues.
// Goals pass all robot global costmaps; Nav2 failures temporarily exclude a robot.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "search_exploration/allocation.hpp"
#include "search_exploration/hungarian.hpp"
#include "search_msgs/msg/frontier_array.hpp"
#include "search_msgs/msg/mission_state.hpp"
#include <std_msgs/msg/string.hpp>

#include "search_exploration/goal_costmap_filter.hpp"
#include "search_msgs/msg/robot_frontier_constraint_array.hpp"
#include "search_msgs/msg/task_plan.hpp"

namespace search_exploration
{

namespace
{
const std::string kReservedPrefix = "active_goal_reserved_by:";
const std::string kOwnedPrefix = "active_goal_owned_by:";

const std::vector<std::array<float, 3>> kRobotColours = {
  {0.85F, 0.33F, 0.10F}, {0.00F, 0.45F, 0.74F}, {0.47F, 0.67F, 0.19F},
  {0.49F, 0.18F, 0.56F}, {0.93F, 0.69F, 0.13F}};
}  // namespace

class TaskAllocatorNode : public rclcpp::Node
{
public:
  TaskAllocatorNode()
  : Node("task_allocator")
  {
    namespaces_ = declare_parameter<std::vector<std::string>>(
      "robot_namespaces", std::vector<std::string>{});
    const auto n = namespaces_.size();
    if (n == 0) {
      throw std::invalid_argument("robot_namespaces must not be empty");
    }
    std::vector<std::string> default_prefixes;
    for (std::size_t i = 0; i < n; ++i) {
      default_prefixes.push_back("R" + std::to_string(i + 1));
    }
    goal_prefixes_ = declare_parameter<std::vector<std::string>>(
      "robot_goal_prefixes", default_prefixes);
    if (goal_prefixes_.size() != n || std::any_of(goal_prefixes_.begin(), goal_prefixes_.end(),
      [](const std::string & prefix) {return prefix.empty();}))
    {
      throw std::invalid_argument("robot_goal_prefixes needs one nonempty prefix per robot");
    }
    goal_filter_ = std::make_unique<GoalCostmapFilter>(*this, namespaces_);
    slots_per_robot_ = declare_parameter<int>("slots_per_robot", 0);
    periodic_s_ = declare_parameter<double>("periodic_reallocation_s", 5.0);
    count_change_ = declare_parameter<int>("frontier_count_change_threshold", 2);
    id_change_ratio_ = declare_parameter<double>("frontier_id_change_ratio", 0.2);
    match_distance_ = declare_parameter<double>("assigned_match_distance_m", 0.75);
    require_mission_ = declare_parameter<bool>("require_mission_exploring", true);

    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    frontiers_sub_ = create_subscription<search_msgs::msg::FrontierArray>(
      "/search/frontiers", latched,
      [this](search_msgs::msg::FrontierArray::ConstSharedPtr msg) {on_frontiers(msg);});
    poses_.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
      const auto ns = "/" + namespaces_[i] + "/";
      pose_subs_.push_back(create_subscription<geometry_msgs::msg::PoseStamped>(
          ns + "ground_truth/pose", rclcpp::SensorDataQoS(),
          [this, i](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {poses_[i] = *msg;}));
    }
    constraints_sub_ = create_subscription<search_msgs::msg::RobotFrontierConstraintArray>(
      "/search/allocation_constraints", latched,
      [this](search_msgs::msg::RobotFrontierConstraintArray::ConstSharedPtr msg) {
        on_constraints(*msg);
      });
    request_sub_ = create_subscription<std_msgs::msg::String>(
      "/search/reallocation_request", latched,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        request(msg->data.empty() ? "external_request" : msg->data);
      });
    mission_sub_ = create_subscription<search_msgs::msg::MissionState>(
      "/search/mission_state", latched,
      [this](search_msgs::msg::MissionState::ConstSharedPtr msg) {
        mission_ = msg->state;
        if (mission_ == search_msgs::msg::MissionState::TARGET_FOUND ||
          mission_ == search_msgs::msg::MissionState::EXPLORATION_COMPLETE)
        {
          mission_stopped_ = true;
          pending_ = false;
          timer_->cancel();
        }
      });

    plan_pub_ = create_publisher<search_msgs::msg::TaskPlan>("/search/task_plan", latched);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/search/task_plan/markers", latched);
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.25), [this]() {tick();});
  }

private:

  int robot_index(const std::string & ns) const
  {
    const auto it = std::find(namespaces_.begin(), namespaces_.end(), ns);
    return it == namespaces_.end() ? -1 : static_cast<int>(it - namespaces_.begin());
  }

  void on_frontiers(const search_msgs::msg::FrontierArray::ConstSharedPtr & msg)
  {
    if (frontiers_ && msg->generation <= frontiers_->generation) {
      return;
    }
    std::set<uint64_t> now_ids;
    for (const auto & f : msg->frontiers) {
      now_ids.insert(f.id);
    }
    if (!has_allocated_) {
      request("first_allocation");
    } else {
      // A goal we assigned no longer has a frontier goal nearby.
      for (const auto & goal : last_assigned_goals_) {
        bool found = false;
        for (const auto & f : msg->frontiers) {
          found = found || std::hypot(f.goal.position.x - goal.x, f.goal.position.y - goal.y) <=
            match_distance_;
        }
        if (!found) {
          request("assigned_frontier_disappeared");
          break;
        }
      }
      const int count_delta = std::abs(
        static_cast<int>(now_ids.size()) - static_cast<int>(last_ids_.size()));
      if (count_delta >= count_change_) {
        request("frontier_count_changed");
      }
      std::vector<uint64_t> changed;
      std::set_symmetric_difference(
        now_ids.begin(), now_ids.end(), last_ids_.begin(), last_ids_.end(),
        std::back_inserter(changed));
      const double base = static_cast<double>(std::max(now_ids.size(), last_ids_.size()));
      if (base > 0.0 && changed.size() / base >= id_change_ratio_) {
        request("frontier_ids_changed");
      }
    }
    frontiers_ = msg;
  }

  // Blocked pairs (failures, capability, reservations) and active-goal owners.
  // Only a change in the non-reservation blocks forces a solve; ownership and
  // reservations just refresh the cached state.
  void on_constraints(const search_msgs::msg::RobotFrontierConstraintArray & msg)
  {
    std::set<std::pair<int, uint64_t>> blocked;
    std::set<std::pair<int, uint64_t>> forcing;
    std::map<uint64_t, int> owners;
    std::set<int> unavailable;
    for (const auto & c : msg.constraints) {
      const int i = robot_index(c.robot_namespace);
      if (i < 0) {
        continue;
      }
      if (c.blocked && c.reason.rfind("robot_nav2_unavailable:", 0) == 0) {
        unavailable.insert(i);
        continue;
      }
      if (c.blocked) {
        blocked.insert({i, c.frontier_id});
        if (c.reason.rfind(kReservedPrefix, 0) != 0) {
          forcing.insert({i, c.frontier_id});
        }
      } else if (c.reason.rfind(kOwnedPrefix, 0) == 0) {
        owners[c.frontier_id] = i;
      }
    }
    if (unavailable != unavailable_robots_) {
      request("robot_availability_changed");
    }
    unavailable_robots_ = std::move(unavailable);
    if (forcing != forcing_constraints_) {
      request("constraints_changed");
    }
    blocked_pairs_ = std::move(blocked);
    forcing_constraints_ = std::move(forcing);
    pinned_owners_ = std::move(owners);
  }

  void request(const std::string & reason)
  {
    if (mission_stopped_) {return;}
    if (!pending_) {
      pending_ = true;
      trigger_ = reason;
    }
  }

  void tick()
  {
    if (mission_stopped_) {return;}
    if (require_mission_ && mission_ != search_msgs::msg::MissionState::EXPLORING) {
      return;
    }
    if (!frontiers_) {
      return;
    }
    for (std::size_t i = 0; i < poses_.size(); ++i) {
      if (unavailable_robots_.count(static_cast<int>(i)) != 0U) {continue;}
      if (!poses_[i] || !goal_filter_->has_costmap(i)) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
          "Waiting for pose and costmap of %s", namespaces_[i].c_str());
        return;
      }
    }
    // Compare by identity, not vector position: WFD may reorder an unchanged
    // candidate set. This also notices freshness expiry/recovery without a
    // frontier change or an active-goal cancellation.
    Eligibility eligibility;
    for (std::size_t i = 0; i < poses_.size(); ++i) {
      for (const auto & frontier : frontiers_->frontiers) {
        eligibility[{i, frontier.id}] = unavailable_robots_.count(static_cast<int>(i)) == 0U &&
          goal_filter_->accepts_all(frontier.goal.position, frontiers_->header.frame_id);
      }
    }
    if (has_allocated_ && eligibility != last_eligibility_) {
      request("goal_costmap_eligibility_changed");
    }
    if (has_allocated_ && (now() - last_allocation_).seconds() >= periodic_s_)
    {
      request("periodic");
    }
    if (pending_) {
      solve(eligibility);
    }
  }

  using Eligibility = std::map<std::pair<std::size_t, uint64_t>, bool>;

  void solve(const Eligibility & eligibility)
  {
    pending_ = false;
    const auto & frontiers = frontiers_->frontiers;
    const std::size_t n = namespaces_.size();
    const std::size_t m = frontiers.size();

    OwnershipInputs in;
    in.eligible.assign(n, std::vector<bool>(m, false));
    in.blocked.assign(n, std::vector<bool>(m, false));
    in.pinned_owner.assign(m, -1);
    for (std::size_t i = 0; i < n; ++i) {
      in.robots.push_back(poses_[i] ?
        Point2{poses_[i]->pose.position.x, poses_[i]->pose.position.y} : Point2{0.0, 0.0});
    }
    for (std::size_t j = 0; j < m; ++j) {
      const auto & goal = frontiers[j].goal.position;
      in.goals.push_back({goal.x, goal.y});
      const auto owner = pinned_owners_.find(frontiers[j].id);
      if (owner != pinned_owners_.end()) {
        in.pinned_owner[j] = owner->second;
      }
      for (std::size_t i = 0; i < n; ++i) {
        in.eligible[i][j] = eligibility.at({i, frontiers[j].id});
        in.blocked[i][j] = blocked_pairs_.count({static_cast<int>(i), frontiers[j].id}) > 0;
      }
    }

    const auto costs = build_ownership_costs(in);
    const auto available = n - unavailable_robots_.size();
    const int slots = slots_per_robot_ > 0 ? slots_per_robot_ :
      (available > 0U ? static_cast<int>((m + available - 1U) / available) : 1);
    const auto ownership = assign_with_slots(costs, std::max(1, slots));

    search_msgs::msg::TaskPlan plan;
    plan.header.stamp = now();
    plan.header.frame_id = frontiers_->header.frame_id;
    plan.generation = ++generation_;
    plan.frontier_generation = frontiers_->generation;
    plan.total_assignment_cost = ownership.total_cost;
    plan.trigger = trigger_;
    last_assigned_goals_.clear();

    for (std::size_t i = 0; i < n; ++i) {
      search_msgs::msg::RobotTask task;
      task.robot_namespace = namespaces_[i];
      auto owned = ownership.owned[i];
      std::stable_sort(owned.begin(), owned.end(),
        [&](int a, int b) {return costs[i][a] < costs[i][b];});
      for (const int j : owned) {
        task.frontier_ids.push_back(frontiers[j].id);
        task.goals.push_back(frontiers[j].goal);
        task.assignment_costs.push_back(costs[i][j]);
        last_assigned_goals_.push_back(in.goals[j]);
      }
      plan.tasks.push_back(task);
    }
    for (const int j : ownership.unassigned) {
      plan.unassigned_frontier_ids.push_back(frontiers[j].id);
    }
    plan_pub_->publish(plan);
    publish_markers(plan, in);

    last_ids_.clear();
    for (const auto & f : frontiers) {
      last_ids_.insert(f.id);
    }
    has_allocated_ = true;
    last_eligibility_ = eligibility;
    last_allocation_ = now();

    std::ostringstream summary;
    for (const auto & task : plan.tasks) {
      summary << " " << task.robot_namespace << "=" << task.frontier_ids.size();
    }
    RCLCPP_INFO(get_logger(), "Plan %lu (%s): %zu frontiers, owned:%s, unassigned=%zu, cost=%.2f",
      static_cast<unsigned long>(plan.generation), trigger_.c_str(), m, summary.str().c_str(),
      plan.unassigned_frontier_ids.size(), plan.total_assignment_cost);
    if (get_logger().get_effective_level() <= rclcpp::Logger::Level::Debug) {
      std::ostringstream matrix;
      for (std::size_t i = 0; i < n; ++i) {
        matrix << "\n  " << namespaces_[i] << ":";
        for (std::size_t j = 0; j < m; ++j) {
          matrix << " " << (std::isfinite(costs[i][j]) ? std::to_string(costs[i][j]) : "inf");
        }
      }
      RCLCPP_DEBUG(get_logger(), "C = D_ij%s", matrix.str().c_str());
    }
  }

  void publish_markers(const search_msgs::msg::TaskPlan & plan, const OwnershipInputs & in)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header = plan.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    int id = 0;
    for (std::size_t i = 0; i < plan.tasks.size(); ++i) {
      const auto & colour = kRobotColours[i % kRobotColours.size()];
      visualization_msgs::msg::Marker route;
      route.header = plan.header;
      route.ns = plan.tasks[i].robot_namespace + "/visit_order";
      route.id = id++;
      route.type = visualization_msgs::msg::Marker::LINE_STRIP;
      route.action = visualization_msgs::msg::Marker::ADD;
      route.pose.orientation.w = 1.0;
      route.scale.x = 0.08;
      route.color.r = colour[0];
      route.color.g = colour[1];
      route.color.b = colour[2];
      route.color.a = 0.9F;
      geometry_msgs::msg::Point start;
      start.x = in.robots[i].x;
      start.y = in.robots[i].y;
      start.z = 0.15;
      route.points.push_back(start);
      visualization_msgs::msg::Marker owned = route;
      owned.ns = plan.tasks[i].robot_namespace + "/owned_goals";
      owned.id = id++;
      owned.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      owned.scale.x = owned.scale.y = owned.scale.z = 0.35;
      owned.points.clear();
      for (std::size_t k = 0; k < plan.tasks[i].goals.size(); ++k) {
        auto p = plan.tasks[i].goals[k].position;
        p.z = 0.15;
        route.points.push_back(p);
        owned.points.push_back(p);
        visualization_msgs::msg::Marker label = route;
        label.ns = plan.tasks[i].robot_namespace + "/order";
        label.id = id++;
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.points.clear();
        label.scale.z = 0.4;
        label.pose.position = p;
        label.pose.position.z = 0.7;
        label.text = goal_prefixes_[i] + std::to_string(k + 1);
        markers.markers.push_back(label);
      }
      if (route.points.size() > 1) {
        markers.markers.push_back(route);
      }
      if (!owned.points.empty()) {
        markers.markers.push_back(owned);
      }
    }
    markers_pub_->publish(markers);
  }

  std::vector<std::string> namespaces_, goal_prefixes_;
  std::unique_ptr<GoalCostmapFilter> goal_filter_;
  int slots_per_robot_{0};
  double periodic_s_{5.0};
  int count_change_{2};
  double id_change_ratio_{0.2};
  double match_distance_{0.75};
  bool require_mission_{true};
  bool mission_stopped_{false};
  uint8_t mission_{search_msgs::msg::MissionState::WAITING};

  search_msgs::msg::FrontierArray::ConstSharedPtr frontiers_;
  std::vector<std::optional<geometry_msgs::msg::PoseStamped>> poses_;
  std::set<int> unavailable_robots_;
  std::set<std::pair<int, uint64_t>> blocked_pairs_;
  std::set<std::pair<int, uint64_t>> forcing_constraints_;
  std::map<uint64_t, int> pinned_owners_;
  std::set<uint64_t> last_ids_;
  Eligibility last_eligibility_;
  std::vector<Point2> last_assigned_goals_;
  bool pending_{false};
  bool has_allocated_{false};
  std::string trigger_;
  rclcpp::Time last_allocation_{0, 0, RCL_ROS_TIME};
  uint64_t generation_{0};

  rclcpp::Subscription<search_msgs::msg::FrontierArray>::SharedPtr frontiers_sub_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subs_;
  rclcpp::Subscription<search_msgs::msg::RobotFrontierConstraintArray>::SharedPtr constraints_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr request_sub_;
  rclcpp::Subscription<search_msgs::msg::MissionState>::SharedPtr mission_sub_;
  rclcpp::Publisher<search_msgs::msg::TaskPlan>::SharedPtr plan_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace search_exploration

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<search_exploration::TaskAllocatorNode>());
  rclcpp::shutdown();
  return 0;
}
