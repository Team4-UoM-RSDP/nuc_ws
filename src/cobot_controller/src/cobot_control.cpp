/**
 * @file cobot_node.cpp
 * @brief Pick place block task using MoveIt 2 with STOMP and OMPL fallback.
 *
 * @author Team 4 - University of Manchester RSDP
 * @date April 2026
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <std_msgs/msg/int8.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <memory>
#include <mutex>
#include <vector>
#include <chrono>
#include <cmath>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <controller_interfaces/srv/controller_set.hpp>

/**
 * @class CobotNode
 * @brief Controls the cobot using MoveIt 2 with STOMP and OMPL.
 */
class CobotNode
{
public:
  /**
   * @brief Constructor: initializes subscriptions, publishers, and MoveIt2 planner
   * @param node ROS2 node for logging and publishers
   */
  explicit CobotNode(rclcpp::Node::SharedPtr node)
      : node_(node),
        arm_group_(node, "arm")
  {
    initializeControllerSetService();
    initializePublishers();
    initializePlanner();
  }

private:
  // ============================================================================
  // ROS 2 COMPONENTS
  // ============================================================================
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface arm_group_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr gripper_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Service<controller_interfaces::srv::ControllerSet>::SharedPtr controller_set_server_;
  
  // ============================================================================
  // STATE MANAGEMENT
  // ============================================================================
  sensor_msgs::msg::JointState current_joint_state_;
  mutable std::mutex joint_state_mutex_;

  // ============================================================================
  // INITIALIZATION HELPERS
  // ============================================================================

  /**
   * @brief Set up the ControllerSet service and joint state feedback subscription
   */
  void initializeControllerSetService()
  {
    controller_set_server_ = node_->create_service<controller_interfaces::srv::ControllerSet>(
        "/controller_set",
        [this](const std::shared_ptr<controller_interfaces::srv::ControllerSet::Request> request,
               std::shared_ptr<controller_interfaces::srv::ControllerSet::Response> response)
        {
          response->success = this->handleControllerSetRequest(request->config);
        });

    joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg)
        {
          this->jointStateCallback(msg);
        });

    RCLCPP_INFO(node_->get_logger(), "ControllerSet service and joint state subscription initialized");
  }

  /**
   * @brief Set up publishers for gripper commands and trajectory output
   */
  void initializePublishers()
  {
    gripper_pub_ = node_->create_publisher<std_msgs::msg::Int8>("/gripper_cmd", 10);
    RCLCPP_INFO(node_->get_logger(), "Publishers initialized");
  }

  /**
   * @brief Configure MoveIt2 planner: check robot description and set planning parameters
   */
  void initializePlanner()
  {
    RCLCPP_INFO(node_->get_logger(), "Initializing MoveGroupInterface for 'arm' group");

    if (!node_->has_parameter("robot_description"))
    {
      RCLCPP_WARN(node_->get_logger(),
                  "robot_description parameter not available yet. "
                  "MoveGroupInterface will wait for it to be published.");
    }

    // Set default to STOMP planner with motion parameters
    setPlanningPipeline("stomp", "STOMP");
    arm_group_.setPlanningTime(5.0);
    arm_group_.setMaxVelocityScalingFactor(0.3);
    arm_group_.setMaxAccelerationScalingFactor(0.3);

    RCLCPP_INFO(node_->get_logger(), "MoveGroupInterface initialized successfully");
    RCLCPP_INFO(node_->get_logger(), "Planning pipeline: %s, Planner: %s",
                arm_group_.getPlanningPipelineId().c_str(),
                arm_group_.getPlannerId().c_str());
  }

  // ============================================================================
  // PLANNING & EXECUTION
  // ============================================================================

  /**
   * @brief Set the planning pipeline and planner ID
   * @param pipeline Pipeline ID (e.g., "stomp", "ompl")
   * @param planner Planner ID (e.g., "STOMP", "RRTConnectkConfigDefault")
   */
  void setPlanningPipeline(const std::string &pipeline, const std::string &planner)
  {
    arm_group_.setPlanningPipelineId(pipeline);
    arm_group_.setPlannerId(planner);
  }

  /**
   * @brief Attempt to plan and execute motion to a named target
   * @param pipeline Pipeline ID (e.g., "stomp", "ompl")
   * @param planner Planner ID (e.g., "STOMP", "RRTConnectkConfigDefault")
   * @param target Named target position
   * @return true if planning and execution succeeded, false otherwise
   */
  bool planAndExecute(const std::string &pipeline, const std::string &planner,
                      const std::string &target)
  {
    setPlanningPipeline(pipeline, planner);
    arm_group_.setNamedTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!static_cast<bool>(arm_group_.plan(plan)))
    {
      RCLCPP_INFO(node_->get_logger(), "%s planning failed for target '%s'",
                  planner.c_str(), target.c_str());
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "%s planning succeeded for target '%s'",
                planner.c_str(), target.c_str());

    auto result = arm_group_.execute(plan);
    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(node_->get_logger(), "%s execution succeeded for target '%s'",
                  planner.c_str(), target.c_str());
      return true;
    }

    RCLCPP_WARN(node_->get_logger(), "%s execution failed (code %d) for target '%s'",
                planner.c_str(), result.val, target.c_str());
    return false;
  }

  /**
   * @brief Move to a target position: try STOMP first, fall back to OMPL
   * @param target Named target position
   * @return true if movement successful, false otherwise
   */
  bool moveToPosition(const std::string &target)
  {
    RCLCPP_INFO(node_->get_logger(), "Moving to position: %s", target.c_str());

    // Try STOMP first
    if (planAndExecute("stomp", "STOMP", target))
    {
      return true;
    }

    // Fall back to OMPL
    RCLCPP_WARN(node_->get_logger(), "STOMP failed for '%s', falling back to OMPL", target.c_str());
    return planAndExecute("ompl", "RRTConnectkConfigDefault", target);
  }

  /**
   * @brief Execute a scan sequence: move to scan_range_0 then scan_range_1
   * @param range Either "far" or "near"
   * @return true if both moves succeed, false otherwise
   */
  bool scan(const std::string &range)
  {
    std::string start = "scan_" + range + "_0";
    std::string end = "scan_" + range + "_1";

    RCLCPP_INFO(node_->get_logger(), "Starting %s scan sequence", range.c_str());

    if (!moveToPosition(start))
    {
      RCLCPP_ERROR(node_->get_logger(), "Scan %s: failed to reach start position", range.c_str());
      return false;
    }

    if (!moveToPosition(end))
    {
      RCLCPP_ERROR(node_->get_logger(), "Scan %s: failed to reach end position", range.c_str());
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Scan %s: sequence completed successfully", range.c_str());
    return true;
  }

  bool scanBlock()
  {
    RCLCPP_INFO(node_->get_logger(), "Scanning final block position");
    if (!moveToPosition("scan_block"))
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to reach scan block position");
      return false;
    }
    rclcpp::sleep_for(std::chrono::seconds(10)); // Hold position for 10 seconds
    return true;
  }

  /**
   * @brief Handle a ControllerSet request by mapping config values to motions
   * @param config Requested controller configuration
   * @return true if the requested action completed successfully, false otherwise
   */
  bool handleControllerSetRequest(int config)
  {
    RCLCPP_INFO(node_->get_logger(), "Received ControllerSet request: %d", config);

    switch (config)
    {
    case 0:
      return moveToPosition("home"); // Home position
    case 1:
      return scan("far"); // Rotate around to scan the distance
    case 2:
      return scan("near"); // Rotate around to scan close to the rover
    case 3:
      return scanBlock(); // Hold position for to scan the block
    default:
      RCLCPP_WARN(node_->get_logger(), "Unknown ControllerSet config: %d", config);
      return false;
    }
  }

  // ============================================================================
  // GRIPPER CONTROL
  // ============================================================================

  /**
   * @brief Publish a gripper command open = 100, close = 0
   * @param command "open" or "close"
   */
  void publishGripperCommand(const std::string &command)
  {
    if (command == "open") {
      auto msg = std_msgs::msg::Int8();
      msg.data = 100;
      gripper_pub_->publish(msg);
      RCLCPP_INFO(node_->get_logger(), "Gripper command: %s", command.c_str());
    } else if (command == "close") {
      auto msg = std_msgs::msg::Int8();
      msg.data = 0;
      gripper_pub_->publish(msg);
      RCLCPP_INFO(node_->get_logger(), "Gripper command: %s", command.c_str());
    } else {
      RCLCPP_WARN(node_->get_logger(), "Unknown gripper command '%s'; not publishing", command.c_str());
      return;
    }
  }

  // ============================================================================
  // SUBSCRIPTIONS & CALLBACKS
  // ============================================================================

  /**
   * @brief Callback for joint state feedback from hardware
   */
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    current_joint_state_ = *msg;
  }

  // ============================================================================
  // STATE QUERY UTILITIES
  // ============================================================================

  /**
   * @brief Get current joint state from hardware (thread-safe)
   * @return Current joint state message
   */
  sensor_msgs::msg::JointState getCurrentJointState() const
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    return current_joint_state_;
  }
};

/**
 * @brief Main function
 * @param argc Command line argument count
 * @param argv Command line arguments
 * @return Exit code
 */
int main(int argc, char **argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create a node with automatic parameter declaration
  auto const node = std::make_shared<rclcpp::Node>(
      "pick_block_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto const logger = rclcpp::get_logger("cobot_node");
  RCLCPP_INFO(logger, "Cobot node started");

  try
  {
    // Create the cobot controller node
    CobotNode cobot_controller(node);

    // Spin briefly to allow connections and receive initial joint states
    RCLCPP_INFO(logger, "Waiting for joint state feedback from hardware...");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(3))
    {
      executor.spin_some(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(logger, "Initialization complete. Ready to receive requests on /controller_set");

    // Continuously wait for service requests and dispatch them
    RCLCPP_INFO(logger, "Listening for /controller_set requests...");
    executor.spin();

    // Shutdown (only reached if spin() returns, e.g., on interrupt)
    RCLCPP_INFO(logger, "Shutting down");
    rclcpp::shutdown();
    return 0;
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(logger, "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
}
