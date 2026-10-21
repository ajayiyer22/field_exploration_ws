// One active Nav2 goal per robot. Goals use global costmaps only.
// Failed Nav2 navigation makes the whole robot unavailable during its timeout;
// constraints trigger Hungarian reallocation among the remaining robots.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

#include "search_exploration/goal_costmap_filter.hpp"
#include "search_msgs/msg/frontier_array.hpp"
#include "search_msgs/msg/mission_state.hpp"
#include "search_msgs/msg/robot_frontier_constraint_array.hpp"
#include "search_msgs/msg/robot_status.hpp"
#include "search_msgs/msg/task_plan.hpp"

namespace search_exploration
{

class ExplorationDispatcher : public rclcpp::Node
{
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using Constraint = search_msgs::msg::RobotFrontierConstraint;
  using ConstraintArray = search_msgs::msg::RobotFrontierConstraintArray;
  using Frontier = search_msgs::msg::Frontier;
  using FrontierArray = search_msgs::msg::FrontierArray;
  using Mission = search_msgs::msg::MissionState;
  using Status = search_msgs::msg::RobotStatus;

  enum class ExecutionState {IDLE, WAITING_SERVER, SENDING, ACTIVE, CANCELING};

  struct RouteTarget
  {
    uint64_t frontier_id{0};
    geometry_msgs::msg::Pose goal;
  };

  struct RobotExecution
  {
    std::string robot_namespace;
    std::string action_name;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client;
    ExecutionState state{ExecutionState::IDLE};
    std::vector<RouteTarget> latest_route;
    uint64_t active_frontier_id{0};
    geometry_msgs::msg::Pose active_goal;
    uint64_t dispatch_token{0};
    uint64_t plan_generation{0};
    int missing_snapshots{0};
    bool cancel_on_accept{false};
    bool approach_requested{false};
    GoalHandle::SharedPtr goal_handle;
  };

  struct FailureBlock
  {
    Constraint constraint;
    rclcpp::Time expires;
  };

public:
  ExplorationDispatcher()
  : Node("exploration_dispatcher")
  {
    declare_parameter<std::vector<std::string>>("robot_namespaces", std::vector<std::string>{});
    declare_parameter<std::string>("action_name", "navigate_to_pose");
    declare_parameter<std::string>("task_plan_topic", "/search/task_plan");
    declare_parameter<std::string>("frontiers_topic", "/search/frontiers");
    declare_parameter<std::string>("mission_state_topic", "/search/mission_state");
    declare_parameter<std::string>(
      "capability_constraints_topic", "/search/capability_constraints");
    declare_parameter<std::string>(
      "allocation_constraints_topic", "/search/allocation_constraints");
    declare_parameter<std::string>("reallocation_request_topic", "/search/reallocation_request");
    declare_parameter<std::string>("execution_state_topic", "/search/execution_state");
    declare_parameter<std::string>("mission_result_topic", "/search/mission_result");
    declare_parameter<std::string>("robot_status_topic", "/search/robot_status");
    declare_parameter<double>("frontier_match_distance_m", 0.75);
    declare_parameter<double>("approach_reallocation_distance_m", 1.5);
    declare_parameter<int>("invalid_frontier_snapshots", 2);
    declare_parameter<double>("failure_timeout_s", 30.0);
    declare_parameter<double>("dispatch_rate_hz", 5.0);
    declare_parameter<bool>("dispatch_enabled", true);
    declare_parameter<bool>("require_mission_exploring", true);

    const auto namespaces = get_parameter("robot_namespaces").as_string_array();
    if (namespaces.empty()) {
      throw std::runtime_error("robot_namespaces must not be empty");
    }
    const auto action_name = get_parameter("action_name").as_string();
    match_distance_m_ = std::max(0.05, get_parameter("frontier_match_distance_m").as_double());
    approach_distance_m_ = std::max(
      0.0, get_parameter("approach_reallocation_distance_m").as_double());
    invalid_snapshot_limit_ = std::max<int64_t>(
      1, get_parameter("invalid_frontier_snapshots").as_int());
    failure_timeout_s_ = std::max(0.0, get_parameter("failure_timeout_s").as_double());
    dispatch_enabled_ = get_parameter("dispatch_enabled").as_bool();
    require_mission_ = get_parameter("require_mission_exploring").as_bool();
    goal_filter_ = std::make_unique<GoalCostmapFilter>(*this, namespaces);

    robots_.reserve(namespaces.size());
    for (const auto & ns : namespaces) {
      RobotExecution robot;
      robot.robot_namespace = ns;
      robot.action_name = "/" + ns + "/" + action_name;
      robot.client = rclcpp_action::create_client<NavigateToPose>(this, robot.action_name);
      robots_.push_back(std::move(robot));
    }

    auto latched = rclcpp::QoS(1).reliable().transient_local();
    plan_sub_ = create_subscription<search_msgs::msg::TaskPlan>(
      get_parameter("task_plan_topic").as_string(), latched,
      [this](search_msgs::msg::TaskPlan::ConstSharedPtr plan) {on_plan(*plan);});
    frontier_sub_ = create_subscription<FrontierArray>(
      get_parameter("frontiers_topic").as_string(), latched,
      [this](FrontierArray::ConstSharedPtr frontiers) {on_frontiers(*frontiers);});
    mission_sub_ = create_subscription<Mission>(
      get_parameter("mission_state_topic").as_string(), latched,
      [this](Mission::ConstSharedPtr mission) {on_mission(*mission);});
    capability_sub_ = create_subscription<ConstraintArray>(
      get_parameter("capability_constraints_topic").as_string(), latched,
      [this](ConstraintArray::ConstSharedPtr constraints) {
        capability_constraints_ = constraints->constraints;
        publish_constraints();
        for (std::size_t index = 0; index < robots_.size(); ++index) {
          const auto & robot = robots_[index];
          if (robot.active_frontier_id != 0U &&
            pair_blocked(robot.robot_namespace, robot.active_frontier_id))
          {
            cancel_goal(index, "capability_mismatch");
          }
        }
        request_reallocation("capability_constraints_changed");
      });

    constraint_pub_ = create_publisher<ConstraintArray>(
      get_parameter("allocation_constraints_topic").as_string(), latched);
    request_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("reallocation_request_topic").as_string(), latched);
    state_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("execution_state_topic").as_string(), latched);
    mission_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("mission_result_topic").as_string(), latched);
    status_pub_ = create_publisher<Status>(
      get_parameter("robot_status_topic").as_string(), rclcpp::QoS(20).reliable());

    const double rate = std::max(0.5, get_parameter("dispatch_rate_hz").as_double());
    // Simulation-time timer so dispatch_rate_hz holds regardless of the real-time factor.
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate),
      [this]() {
        expire_failures();
        reconcile_all();
        publish_robot_status();
      });
    publish_constraints();
    publish_state("WAITING_FOR_PLAN");
    RCLCPP_INFO(get_logger(),
      "Exploration dispatcher ready for %zu robots; one active Nav2 goal per robot",
      robots_.size());
  }

private:
  static double goal_distance(
    const geometry_msgs::msg::Pose & first, const geometry_msgs::msg::Pose & second)
  {
    return std::hypot(first.position.x - second.position.x, first.position.y - second.position.y);
  }

  static std::string state_name(ExecutionState state)
  {
    switch (state) {
      case ExecutionState::IDLE: return "IDLE";
      case ExecutionState::WAITING_SERVER: return "WAITING_SERVER";
      case ExecutionState::SENDING: return "SENDING";
      case ExecutionState::ACTIVE: return "ACTIVE";
      case ExecutionState::CANCELING: return "CANCELING";
    }
    return "UNKNOWN";
  }

  static bool reserves_active_frontier(const RobotExecution & robot)
  {
    return robot.active_frontier_id != 0U && robot.state != ExecutionState::IDLE &&
           robot.state != ExecutionState::WAITING_SERVER;
  }

  bool dispatch_allowed() const
  {
    return dispatch_enabled_ && !mission_stopped_ &&
           (!require_mission_ || mission_state_ == Mission::EXPLORING);
  }

  std::optional<Frontier> matching_frontier(const geometry_msgs::msg::Pose & goal) const
  {
    if (!frontiers_) {
      return std::nullopt;
    }
    const Frontier * best = nullptr;
    double best_distance = match_distance_m_;
    for (const auto & frontier : frontiers_->frontiers) {
      const double distance = goal_distance(goal, frontier.goal);
      if (distance <= best_distance) {
        best = &frontier;
        best_distance = distance;
      }
    }
    return best ? std::optional<Frontier>(*best) : std::nullopt;
  }

  bool pair_blocked(const std::string & robot, uint64_t frontier_id) const
  {
    const auto matches = [&](const Constraint & c) {
        return c.blocked && c.robot_namespace == robot &&
               (c.frontier_id == frontier_id || c.reason.rfind("robot_nav2_unavailable:", 0) == 0);
      };
    return std::any_of(failures_.begin(), failures_.end(),
             [&](const FailureBlock & f) {return matches(f.constraint);}) ||
           std::any_of(capability_constraints_.begin(), capability_constraints_.end(), matches);
  }

  bool reserved_by_other(std::size_t robot_index, const geometry_msgs::msg::Pose & goal) const
  {
    for (std::size_t index = 0; index < robots_.size(); ++index) {
      if (index == robot_index) {
        continue;
      }
      const auto state = robots_[index].state;
      if (state != ExecutionState::IDLE && state != ExecutionState::WAITING_SERVER &&
        goal_distance(goal, robots_[index].active_goal) <= match_distance_m_)
      {
        return true;
      }
    }
    return false;
  }

  bool recently_completed(const geometry_msgs::msg::Pose & goal) const
  {
    return std::any_of(completed_goals_.begin(), completed_goals_.end(),
             [&](const geometry_msgs::msg::Pose & completed) {
               return goal_distance(goal, completed) <= match_distance_m_;
             });
  }

  std::optional<RouteTarget> select_next(std::size_t robot_index) const
  {
    const auto & robot = robots_[robot_index];
    for (const auto & planned : robot.latest_route) {
      const auto current = matching_frontier(planned.goal);
      if (!current) {continue;}
      if (pair_blocked(robot.robot_namespace, current->id)) {continue;}
      if (recently_completed(current->goal)) {continue;}
      if (reserved_by_other(robot_index, current->goal)) {continue;}
      if (!goal_filter_->accepts_all(current->goal.position,
        frontiers_->header.frame_id)) {continue;}
      return RouteTarget{current->id, current->goal};
    }
    return std::nullopt;
  }

  void on_mission(const Mission & mission)
  {
    mission_state_ = mission.state;
    if (mission.state == Mission::TARGET_FOUND || mission.state == Mission::EXPLORATION_COMPLETE) {
      stop_mission(mission);
    } else {
      reconcile_all();
    }
  }

  void on_plan(const search_msgs::msg::TaskPlan & plan)
  {
    if (mission_stopped_ || plan.generation < latest_plan_generation_) {
      return;
    }
    latest_plan_generation_ = plan.generation;
    for (auto & robot : robots_) {
      robot.latest_route.clear();
    }
    for (const auto & task : plan.tasks) {
      auto robot = std::find_if(robots_.begin(), robots_.end(),
          [&](const RobotExecution & candidate) {
            return candidate.robot_namespace == task.robot_namespace;
          });
      if (robot == robots_.end()) {
        RCLCPP_WARN(get_logger(), "Ignoring route for unknown robot '%s'",
          task.robot_namespace.c_str());
        continue;
      }
      const auto count = std::min(task.frontier_ids.size(), task.goals.size());
      for (std::size_t index = 0; index < count; ++index) {
        robot->latest_route.push_back(RouteTarget{task.frontier_ids[index], task.goals[index]});
      }
      robot->plan_generation = plan.generation;
    }
    publish_state("PLAN_RECEIVED");
    reconcile_all();
  }

  void on_frontiers(const FrontierArray & incoming)
  {
    if (frontiers_ && incoming.generation <= frontiers_->generation) {
      return;
    }
    frontiers_ = incoming;

    completed_goals_.erase(std::remove_if(completed_goals_.begin(), completed_goals_.end(),
      [&](const geometry_msgs::msg::Pose & completed) {
        return !matching_frontier(completed).has_value();
      }), completed_goals_.end());

    bool reservation_changed = false;
    for (std::size_t index = 0; index < robots_.size(); ++index) {
      auto & robot = robots_[index];
      if (robot.state == ExecutionState::IDLE || robot.state == ExecutionState::WAITING_SERVER) {
        continue;
      }
      const auto current = matching_frontier(robot.active_goal);
      if (current) {
        reservation_changed = reservation_changed || robot.active_frontier_id != current->id;
        robot.active_frontier_id = current->id;
        robot.missing_snapshots = 0;
      } else {
        ++robot.missing_snapshots;
        if (robot.missing_snapshots >= invalid_snapshot_limit_) {
          cancel_goal(index, "active_frontier_disappeared");
          request_reallocation("active_frontier_disappeared");
        }
      }
    }
    if (reservation_changed) {
      publish_constraints();
    }
    reconcile_all();
  }

  void expire_failures()
  {
    const auto before = failures_.size();
    const auto now_time = now();
    failures_.erase(std::remove_if(failures_.begin(), failures_.end(),
      [&](const FailureBlock & failure) {return now_time >= failure.expires;}), failures_.end());
    if (failures_.size() != before) {
      publish_constraints();
      request_reallocation("failure_constraints_timed_out");
    }
  }

  void reconcile_all()
  {
    if (!dispatch_allowed() || !frontiers_) {
      return;
    }
    for (std::size_t index = 0; index < robots_.size(); ++index) {
      const auto & robot = robots_[index];
      if ((robot.state == ExecutionState::ACTIVE || robot.state == ExecutionState::SENDING) &&
        !goal_filter_->accepts_all(robot.active_goal.position, frontiers_->header.frame_id))
      {
        cancel_goal(index, "goal_costmap_ineligible");
        request_reallocation("goal_costmap_ineligible");
      }
      reconcile(index);
    }
  }

  void reconcile(std::size_t robot_index)
  {
    auto & robot = robots_[robot_index];
    if (!dispatch_allowed() || robot.state == ExecutionState::ACTIVE ||
      robot.state == ExecutionState::SENDING || robot.state == ExecutionState::CANCELING)
    {
      return;
    }
    const auto target = select_next(robot_index);
    if (!target) {
      robot.state = ExecutionState::IDLE;
      return;
    }
    if (!robot.client->action_server_is_ready()) {
      robot.state = ExecutionState::WAITING_SERVER;
      return;
    }
    send_goal(robot_index, *target);
  }

  void send_goal(std::size_t robot_index, const RouteTarget & target)
  {
    auto & robot = robots_[robot_index];
    robot.state = ExecutionState::SENDING;
    robot.active_frontier_id = target.frontier_id;
    robot.active_goal = target.goal;
    robot.missing_snapshots = 0;
    robot.cancel_on_accept = false;
    robot.approach_requested = false;
    const uint64_t token = ++robot.dispatch_token;
    // The active frontier stays owned by this robot while Nav2 executes it, and
    // is reserved against every other robot.
    publish_constraints();

    NavigateToPose::Goal request;
    request.pose.header = frontiers_->header;
    request.pose.header.stamp = now();
    request.pose.pose = target.goal;
    if (request.pose.pose.orientation.x == 0.0 && request.pose.pose.orientation.y == 0.0 &&
      request.pose.pose.orientation.z == 0.0 && request.pose.pose.orientation.w == 0.0)
    {
      request.pose.pose.orientation.w = 1.0;
    }

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    // Refresh the allocation once the robot is close to its frontier: the
    // approach corridor is mapped by then, and the next goal is ready on arrival.
    options.feedback_callback = [this, robot_index, token](
      GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        if (!feedback || mission_stopped_) {return;}
        auto & current = robots_[robot_index];
        if (token != current.dispatch_token || current.approach_requested) {return;}
        const double remaining = static_cast<double>(feedback->distance_remaining);
        // Nav2 reports 0 before its first path exists.
        if (!std::isfinite(remaining) || remaining <= 0.0 || remaining > approach_distance_m_) {
          return;
        }
        current.approach_requested = true;
        RCLCPP_INFO(get_logger(), "%s within %.2f m of its frontier; refreshing allocation",
          current.robot_namespace.c_str(), remaining);
        request_reallocation("approaching_target:" + current.robot_namespace);
      };
    options.goal_response_callback = [this, robot_index, token](GoalHandle::SharedPtr handle) {
        auto & current = robots_[robot_index];
        if (token != current.dispatch_token) {
          if (handle) {current.client->async_cancel_goal(handle);}
          return;
        }
        if (!handle) {
          handle_failure(robot_index, "goal_rejected");
          return;
        }
        current.goal_handle = handle;
        current.state = ExecutionState::ACTIVE;
        RCLCPP_INFO(get_logger(), "%s executing frontier %lu at (%.2f, %.2f)",
          current.robot_namespace.c_str(), static_cast<unsigned long>(current.active_frontier_id),
          current.active_goal.position.x, current.active_goal.position.y);
        if (!goal_filter_->accepts_all(current.active_goal.position,
          frontiers_->header.frame_id))
        {
          cancel_goal(robot_index, "goal_costmap_ineligible");
          request_reallocation("goal_costmap_ineligible");
        } else if (mission_stopped_ || current.cancel_on_accept) {
          cancel_goal(robot_index, mission_stopped_ ? "mission_stopped" : "frontier_invalid");
        }
        publish_state("GOAL_ACCEPTED");
      };
    options.result_callback = [this, robot_index, token](const GoalHandle::WrappedResult & result) {
        auto & current = robots_[robot_index];
        if (token != current.dispatch_token) {return;}
        const auto completed_goal = current.active_goal;
        const auto frontier_id = current.active_frontier_id;
        current.goal_handle.reset();
        current.state = ExecutionState::IDLE;
        current.active_frontier_id = 0;
        current.missing_snapshots = 0;
        publish_constraints();
        if (mission_stopped_) {
          publish_state("MISSION_STOPPED");
          return;
        }
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          completed_goals_.push_back(completed_goal);
          RCLCPP_INFO(get_logger(), "%s reached frontier %lu; selecting from latest plan",
            current.robot_namespace.c_str(), static_cast<unsigned long>(frontier_id));
          publish_state("GOAL_SUCCEEDED");
          reconcile(robot_index);
        } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
          publish_state("GOAL_CANCELED");
          reconcile(robot_index);
        } else {
          current.active_frontier_id = frontier_id;
          current.active_goal = completed_goal;
          handle_failure(robot_index, "navigate_to_pose_aborted");
        }
      };

    try {
      robot.client->async_send_goal(request, options);
      publish_state("GOAL_SENT");
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Failed to send goal for %s: %s",
        robot.robot_namespace.c_str(), error.what());
      handle_failure(robot_index, "send_goal_exception");
    }
  }

  void cancel_goal(std::size_t robot_index, const std::string & reason)
  {
    auto & robot = robots_[robot_index];
    if (robot.state == ExecutionState::SENDING) {
      robot.cancel_on_accept = true;
      return;
    }
    if (!robot.goal_handle || robot.state == ExecutionState::CANCELING) {
      return;
    }
    robot.state = ExecutionState::CANCELING;
    RCLCPP_INFO(get_logger(), "Canceling %s frontier %lu: %s", robot.robot_namespace.c_str(),
      static_cast<unsigned long>(robot.active_frontier_id), reason.c_str());
    robot.client->async_cancel_goal(robot.goal_handle);
    publish_state("CANCEL_REQUESTED_" + reason);
  }

  void handle_failure(std::size_t robot_index, const std::string & reason)
  {
    auto & robot = robots_[robot_index];
    Constraint constraint;
    constraint.robot_namespace = robot.robot_namespace;
    const auto failed_frontier = robot.active_frontier_id;
    constraint.frontier_id = 0U;  // Robot-wide availability constraint, not a frontier pair.
    constraint.blocked = true;
    constraint.reason = "robot_nav2_unavailable:" + reason;
    const bool exists = std::any_of(failures_.begin(), failures_.end(),
        [&](const FailureBlock & existing) {
          return existing.constraint.robot_namespace == constraint.robot_namespace;
        });
    if (!exists) {
      failures_.push_back({constraint, now() + rclcpp::Duration::from_seconds(failure_timeout_s_)});
    }
    RCLCPP_INFO(get_logger(), "%s failed frontier %lu (%s); robot unavailable for %.0f s",
      robot.robot_namespace.c_str(), static_cast<unsigned long>(failed_frontier),
      reason.c_str(), failure_timeout_s_);
    robot.goal_handle.reset();
    robot.state = ExecutionState::IDLE;
    robot.active_frontier_id = 0;
    robot.latest_route.clear();
    publish_constraints();
    request_reallocation(reason);
    publish_state("GOAL_FAILED_" + reason);
  }

  void publish_constraints()
  {
    ConstraintArray output;
    if (frontiers_) {
      output.header = frontiers_->header;
    }
    output.constraints = capability_constraints_;
    for (const auto & failure : failures_) {
      output.constraints.push_back(failure.constraint);
    }
    for (const auto & owner : robots_) {
      if (!reserves_active_frontier(owner)) {
        continue;
      }
      for (const auto & other : robots_) {
        if (other.robot_namespace == owner.robot_namespace) {
          continue;
        }
        Constraint reservation;
        reservation.robot_namespace = other.robot_namespace;
        reservation.frontier_id = owner.active_frontier_id;
        reservation.blocked = true;
        reservation.reason = "active_goal_reserved_by:" + owner.robot_namespace;
        output.constraints.push_back(std::move(reservation));
      }
      // Positive counterpart: names the owner so the allocator can pin it.
      Constraint ownership;
      ownership.robot_namespace = owner.robot_namespace;
      ownership.frontier_id = owner.active_frontier_id;
      ownership.blocked = false;
      ownership.reason = "active_goal_owned_by:" + owner.robot_namespace;
      output.constraints.push_back(std::move(ownership));
    }
    constraint_pub_->publish(output);
  }

  void request_reallocation(const std::string & reason)
  {
    if (mission_stopped_) {
      return;
    }
    std_msgs::msg::String request;
    request.data = reason;
    request_pub_->publish(request);
  }

  void stop_mission(const Mission & mission)
  {
    if (mission_stopped_) {
      return;
    }
    mission_stopped_ = true;
    for (auto & robot : robots_) {
      robot.latest_route.clear();
    }
    for (std::size_t index = 0; index < robots_.size(); ++index) {
      cancel_goal(index, "mission_stopped");
    }
    std_msgs::msg::String result;
    result.data = "state=" + std::string(
      mission.state == Mission::TARGET_FOUND ? "TARGET_FOUND" : "EXPLORATION_COMPLETE") +
      "; active_goals_cancel_requested=true; detail=" + mission.detail;
    mission_pub_->publish(result);
    publish_state("MISSION_STOPPED");
    RCLCPP_WARN(get_logger(), "Mission stopped (%s); all Nav2 goals canceled",
      mission.detail.c_str());
  }

  void publish_robot_status()
  {
    for (const auto & robot : robots_) {
      Status msg;
      msg.header.stamp = now();
      msg.robot_namespace = robot.robot_namespace;
      msg.state = robot.state == ExecutionState::ACTIVE ? Status::ACTIVE :
        (robot.state == ExecutionState::SENDING || robot.state == ExecutionState::CANCELING) ?
        Status::SENDING : Status::IDLE;
      msg.has_active_goal = reserves_active_frontier(robot);
      msg.active_frontier_id = robot.active_frontier_id;
      msg.active_goal = robot.active_goal;
      status_pub_->publish(msg);
    }
  }

  void publish_state(const std::string & event)
  {
    std::ostringstream stream;
    stream << "{\"mission_state\":\"" << (mission_stopped_ ? "STOPPED" : "EXPLORING") <<
      "\",\"event\":\"" << event << "\",\"plan_generation\":" << latest_plan_generation_ <<
      ",\"frontier_generation\":" << (frontiers_ ? frontiers_->generation : 0U) <<
      ",\"robots\":[";
    for (std::size_t index = 0; index < robots_.size(); ++index) {
      if (index) {stream << ',';}
      const auto & robot = robots_[index];
      stream << "{\"namespace\":\"" << robot.robot_namespace << "\",\"state\":\"" <<
        state_name(robot.state) << "\",\"active_frontier_id\":" << robot.active_frontier_id <<
        ",\"queued_targets\":" << robot.latest_route.size() << '}';
    }
    stream << "]}";
    std_msgs::msg::String message;
    message.data = stream.str();
    state_pub_->publish(message);
  }

  std::vector<RobotExecution> robots_;
  std::optional<FrontierArray> frontiers_;
  std::vector<geometry_msgs::msg::Pose> completed_goals_;
  std::vector<FailureBlock> failures_;
  std::vector<Constraint> capability_constraints_;
  double match_distance_m_{0.75};
  double approach_distance_m_{1.5};
  int64_t invalid_snapshot_limit_{2};
  double failure_timeout_s_{30.0};
  bool dispatch_enabled_{true};
  bool require_mission_{true};
  bool mission_stopped_{false};
  uint8_t mission_state_{Mission::WAITING};
  uint64_t latest_plan_generation_{0};

  rclcpp::Subscription<search_msgs::msg::TaskPlan>::SharedPtr plan_sub_;
  rclcpp::Subscription<FrontierArray>::SharedPtr frontier_sub_;
  rclcpp::Subscription<Mission>::SharedPtr mission_sub_;
  rclcpp::Subscription<ConstraintArray>::SharedPtr capability_sub_;
  rclcpp::Publisher<ConstraintArray>::SharedPtr constraint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr request_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_pub_;
  rclcpp::Publisher<Status>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<GoalCostmapFilter> goal_filter_;
};

}  // namespace search_exploration

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<search_exploration::ExplorationDispatcher>());
  rclcpp::shutdown();
  return 0;
}
