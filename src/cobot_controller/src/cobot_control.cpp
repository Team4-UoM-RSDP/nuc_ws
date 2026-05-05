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
#include <controller_interfaces/srv/controller_position_set.hpp>

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
  rclcpp::Service<controller_interfaces::srv::ControllerPositionSet>::SharedPtr controller_position_set_server_;
  // ============================================================================
  // STATE MANAGEMENT
  // ============================================================================
  sensor_msgs::msg::JointState current_joint_state_;
  mutable std::mutex joint_state_mutex_;
  // Latest block pose received via /controller_position_set (vision pick)
  geometry_msgs::msg::Pose last_block_pose_;
  bool has_block_pose_ = false;
  mutable std::mutex block_pose_mutex_;

  // Joint names for the arm
  const std::vector<std::string> joint_names_ = {
      "link1_to_link2",
      "link2_to_link3",
      "link3_to_link4",
      "link4_to_link5",
      "link5_to_link6",
      "link6_to_link6_flange"};

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

    controller_position_set_server_ = node_->create_service<controller_interfaces::srv::ControllerPositionSet>(
        "/controller_position_set",
        [this](const std::shared_ptr<controller_interfaces::srv::ControllerPositionSet::Request> request,
               std::shared_ptr<controller_interfaces::srv::ControllerPositionSet::Response> response)
        {
          response->success = this->handleControllerPositionSetRequest(request->x, request->y, request->z);
        });

    joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg)
        {
          this->jointStateCallback(msg);
        });

    // Subscribe to target joint positions for custom movement
    node_->create_subscription<sensor_msgs::msg::JointState>(
        "/target_joint_positions", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg)
        {
          this->targetJointPositionsCallback(msg);
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
   * @brief Plan and execute with the current target (already set on arm_group_)
   * @param pipeline Pipeline ID (e.g., "stomp", "ompl")
   * @param planner Planner ID (e.g., "STOMP", "RRTConnectkConfigDefault")
   * @return true if planning and execution succeeded, false otherwise
   */
  bool planAndExecute(const std::string &pipeline, const std::string &planner)
  {
    setPlanningPipeline(pipeline, planner);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!static_cast<bool>(arm_group_.plan(plan)))
    {
      RCLCPP_INFO(node_->get_logger(), "%s planning failed", planner.c_str());
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "%s planning succeeded", planner.c_str());

    auto result = arm_group_.execute(plan);
    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(node_->get_logger(), "%s execution succeeded", planner.c_str());
      return true;
    }

    RCLCPP_WARN(node_->get_logger(), "%s execution failed (code %d)", planner.c_str(), result.val);
    return false;
  }

  /**
   * @brief Attempt motion with STOMP first, fallback to OMPL
   * @return true if either planner succeeded, false otherwise
   */
  bool moveWithFallback()
  {
    // Try STOMP first
    if (planAndExecute("stomp", "STOMP"))
    {
      return true;
    }

    // Fall back to OMPL
    RCLCPP_WARN(node_->get_logger(), "STOMP failed, falling back to OMPL");
    return planAndExecute("ompl", "RRTConnectkConfigDefault");
  }

  /**
   * @brief Move to a named target position: try STOMP first, fall back to OMPL
   * @param target Named target position
   * @return true if movement successful, false otherwise
   */
  bool moveToSetPosition(const std::string &target)
  {
    RCLCPP_INFO(node_->get_logger(), "Moving to position: %s", target.c_str());
    arm_group_.setNamedTarget(target);
    return moveWithFallback();
  }

  /**
   * @brief Move to arbitrary joint positions: try STOMP first, fall back to OMPL
   * @param joint_angles Vector of 6 joint angles in radians
   * @return true if movement successful, false otherwise
   */
  bool moveToCustomPosition(const std::vector<double> &joint_angles)
  {
    if (joint_angles.size() != 6)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Invalid joint angles: expected 6 values, got %zu",
                   joint_angles.size());
      return false;
    }

    RCLCPP_INFO(node_->get_logger(),
                "Moving to joint positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                joint_angles[0], joint_angles[1], joint_angles[2],
                joint_angles[3], joint_angles[4], joint_angles[5]);

    arm_group_.setJointValueTarget(joint_names_, joint_angles);
    return moveWithFallback();
  }

  /**
   * @brief Move to a Cartesian pose using the current planner pipeline
   * @param pose Target end-effector pose
   * @return true if movement succeeded, false otherwise
   */
  bool moveToPose(const geometry_msgs::msg::Pose &pose)
  {
    RCLCPP_INFO(node_->get_logger(), "Moving to pose: x=%.3f y=%.3f z=%.3f",
                pose.position.x, pose.position.y, pose.position.z);
    arm_group_.setPoseTarget(pose);
    return moveWithFallback();
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

    if (!moveToSetPosition(start))
    {
      RCLCPP_ERROR(node_->get_logger(), "Scan %s: failed to reach start position", range.c_str());
      return false;
    }

    if (!moveToSetPosition(end))
    {
      RCLCPP_ERROR(node_->get_logger(), "Scan %s: failed to reach end position", range.c_str());
      return false;
    }

    if (range == "promo")
    {
      if (!moveToSetPosition("scan_block_promo"))
      {
        RCLCPP_ERROR(node_->get_logger(), "Scan %s: failed to reach block promo position", range.c_str());
        return false;
      }
      rclcpp::sleep_for(std::chrono::seconds(5)); // Hold position for 5 seconds
    }

    RCLCPP_INFO(node_->get_logger(), "Scan %s: sequence completed successfully", range.c_str());
    return true;
  }

  bool scanBlock()
  {
    RCLCPP_INFO(node_->get_logger(), "Scanning final block position");
    if (!moveToSetPosition("scan_block"))
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to reach scan block position");
      return false;
    }
    rclcpp::sleep_for(std::chrono::seconds(10)); // Hold position for 10 seconds
    return true;
  }

  /**
   * @brief Pick up a block: open gripper, move to pre-pick, pick, close gripper
   * @param vision If true, use vision-based picking (custom position), if false use predefined positions
   * @return true if pick succeeded, false otherwise
   */
  bool pickBlock(bool vision)
  {
    RCLCPP_INFO(node_->get_logger(), "Starting pick block sequence (vision=%s)", vision ? "true" : "false");

    // Open gripper
    publishGripperCommand("open");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    // Move to pre-pick joint configuration first (common for both modes)
    if (!moveToSetPosition("pre_pick"))
    {
      RCLCPP_ERROR(node_->get_logger(), "Pick: failed to reach pre-pick position");
      return false;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    if (vision)
    {
      // Vision-based: plan Cartesian approach above block then descend
      geometry_msgs::msg::Pose target;
      {
        std::lock_guard<std::mutex> lock(block_pose_mutex_);
        if (!has_block_pose_)
        {
          RCLCPP_WARN(node_->get_logger(), "Vision pick requested but no block pose available; call /controller_position_set first");
          return false;
        }
        target = last_block_pose_;
      }

      geometry_msgs::msg::Pose above = target;
      above.position.z += 0.10; // 10 cm above

      // Move to above the block
      if (!moveToPose(above))
      {
        RCLCPP_ERROR(node_->get_logger(), "Vision Pick: failed to reach above-block pose");
        return false;
      }

      rclcpp::sleep_for(std::chrono::milliseconds(500));

      // Move down to the block
      if (!moveToPose(target))
      {
        RCLCPP_ERROR(node_->get_logger(), "Vision Pick: failed to reach block pose");
        return false;
      }

      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
    else
    {
      // No vision, move to predefined pick poition
      if (!moveToSetPosition("pick"))
      {
        RCLCPP_ERROR(node_->get_logger(), "Pick: failed to reach pick position");
        return false;
      }

      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    // Close gripper
    publishGripperCommand("close");
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // Retreat back to pre-pick joint configuration
    if (!moveToSetPosition("pre_pick"))
    {
      RCLCPP_WARN(node_->get_logger(), "Pick: failed to retreat to pre-pick position");
    }

    // Return to home
    if (!moveToSetPosition("home"))
    {
      RCLCPP_WARN(node_->get_logger(), "Pick: failed to reach home position");
    }

    rclcpp::sleep_for(std::chrono::milliseconds(500));
    RCLCPP_INFO(node_->get_logger(), "Pick block sequence completed successfully");
    return true;
  }

  /**
   * @brief Drop the block: move to full extension, open gripper, return to home
   * @return true if drop succeeded, false otherwise
   */
  bool dropBlock()
  {
    RCLCPP_INFO(node_->get_logger(), "Starting drop block sequence");

    // Move to full extension position
    if (!moveToSetPosition("full_extension"))
    {
      RCLCPP_ERROR(node_->get_logger(), "Drop: failed to reach full extension position");
      return false;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // Open gripper to release block
    publishGripperCommand("open");
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // Return to home position
    if (!moveToSetPosition("home"))
    {
      RCLCPP_ERROR(node_->get_logger(), "Drop: failed to return to home position");
      return false;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    RCLCPP_INFO(node_->get_logger(), "Drop block sequence completed successfully");
    return true;
  }

  bool handleControllerSetRequest(int config)
  {
    RCLCPP_INFO(node_->get_logger(), "Received ControllerSet request: %d", config);

    switch (config)
    {
    case 0:
      return moveToSetPosition("home"); // Home position
    case 1:
      return scan("far"); // Rotate around to scan the distance
    case 2:
      return scan("near"); // Rotate around to scan close to the rover
    case 3:
      return scan("promo"); // Adjusted scan positions for promo video
    case 4:
      return scanBlock(); // Hold position to scan the block
    case 5:
      return pickBlock(false); // Pick block with predefined positions
    case 6:
      return pickBlock(true); // Pick block with vision-based positioning
    case 7:
      return dropBlock(); // Drop block in drop zone
    default:
      RCLCPP_WARN(node_->get_logger(), "Unknown ControllerSet config: %d", config);
      return false;
    }
  }

  bool handleControllerPositionSetRequest(double x, double y, double z)
  {
    RCLCPP_INFO(node_->get_logger(), "Received ControllerPositionSet request: x=%.3f, y=%.3f, z=%.3f", x, y, z);
    {
      std::lock_guard<std::mutex> lock(block_pose_mutex_);
      last_block_pose_.position.x = x;
      last_block_pose_.position.y = y;
      last_block_pose_.position.z = z;
      // Default orientation: no rotation (w=1)
      last_block_pose_.orientation.x = 0.0;
      last_block_pose_.orientation.y = 0.0;
      last_block_pose_.orientation.z = 0.0;
      last_block_pose_.orientation.w = 1.0;
      has_block_pose_ = true;
    }

    RCLCPP_INFO(node_->get_logger(), "Stored block pose for vision pick");
    return true;
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
    if (command == "open")
    {
      auto msg = std_msgs::msg::Int8();
      msg.data = 100;
      gripper_pub_->publish(msg);
      RCLCPP_INFO(node_->get_logger(), "Gripper command: %s", command.c_str());
    }
    else if (command == "close")
    {
      auto msg = std_msgs::msg::Int8();
      msg.data = 0;
      gripper_pub_->publish(msg);
      RCLCPP_INFO(node_->get_logger(), "Gripper command: %s", command.c_str());
    }
    else
    {
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

  /**
   * @brief Callback for target joint positions - receives custom position and executes movement
   * @param msg JointState message with 6 joint position values
   */
  void targetJointPositionsCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!msg || msg->position.size() < 6)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Invalid target joint positions: expected 6 values, got %zu",
                  msg ? msg->position.size() : 0);
      return;
    }

    // Extract the first 6 values as target joint angles
    std::vector<double> target_angles(msg->position.begin(), msg->position.begin() + 6);
    moveToCustomPosition(target_angles);
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
      "cobot_node",
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
