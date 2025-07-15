/**
 * @file action_node.cpp
 * @brief A ROS2 node for controlling the Franka robot to perform a pick-and-place task.
 *
 * This node waits for an object to be detected by the sorting_node, then commands the robot
 * to pick up the object and place it at a predefined location. It uses MoveIt2 for motion
 * planning and a ROS2 action client for gripper control.
 * @copyright Copyright (c) 2025
 */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose.hpp>
#include <franka_msgs/action/grasp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <map>

/**
 * @class ActionNode
 * @brief Controls the Franka robot to perform a pick-and-place task.
 *
 * This class implements a state machine that waits for an object to be detected,
 * then moves the robot arm to pick it up and place it at a target location.
 * It uses MoveIt2 for motion planning and communicates with the Franka gripper
 * via a ROS2 action server.
 */
class ActionNode : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Action Node object.
     *
     * Initializes the TF2 buffer and listener, subscribes to the object width topic,
     * and sets up a timer to drive the main state machine.
     */
    ActionNode() : Node("action_node")
    {
        // Initialize TF2 listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Declare and load placement poses from parameters
        this->declare_parameter("placements.object0.x", 0.0);
        this->declare_parameter("placements.object0.y", -0.2);
        this->declare_parameter("placements.object0.z", 0.2);
        this->declare_parameter("placements.object1.x", 0.2);
        this->declare_parameter("placements.object1.y", -0.2);
        this->declare_parameter("placements.object1.z", 0.2);
        this->declare_parameter("placements.object2.x", -0.2);
        this->declare_parameter("placements.object2.y", -0.2);
        this->declare_parameter("placements.object2.z", 0.2);

        placement_poses_["object0"] = get_pose_from_params("placements.object0");
        placement_poses_["object1"] = get_pose_from_params("placements.object1");
        placement_poses_["object2"] = get_pose_from_params("placements.object2");


        // Subscriber for object width
        width_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/object_width", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                this->object_width_ = msg->data;
                this->has_received_width_ = true;
            });

        // Subscriber for object class
        class_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/object_class", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                this->object_class_ = msg->data;
            });

        // Subscriber for gripper joint states to check grasp success
        gripper_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/franka_gripper/joint_states", 10,
            std::bind(&ActionNode::gripper_joint_callback, this, std::placeholders::_1));

        // Action client for the gripper
        grasp_client_ = rclcpp_action::create_client<franka_msgs::action::Grasp>(this, "/franka_gripper/grasp");

        // Subscribers for GUI control
        start_movement_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/start_movement", 10,
            std::bind(&ActionNode::start_movement_callback, this, std::placeholders::_1));

        cancel_operation_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/cancel_operation", 10,
            std::bind(&ActionNode::cancel_operation_callback, this, std::placeholders::_1));

        // Main timer to run the state machine
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ActionNode::run_state_machine, this));
    }

private:
    /**
     * @brief Defines the states for the pick-and-place operation.
     */
    enum class State
    {
        IDLE,               ///< Waiting for a valid object to be detected.
        MOVE_TO_PRE_GRASP,  ///< Move to a position above the object.
        MOVE_TO_GRASP,      ///< Move to the final grasp position.
        GRASP_OBJECT,       ///< Close the gripper to grasp the object.
        CHECK_GRASP,        ///< Verify if the object was successfully grasped.
        MOVE_TO_PLACE,      ///< Move the object to the target placement location.
        RELEASE_OBJECT,     ///< Open the gripper to release the object.
        RETURN_HOME         ///< Move the robot back to a ready/home position.
    };

    /**
     * @brief The main state machine controlling the robot's actions.
     *
     * This function is called periodically by a timer and transitions the robot
     * through the pick-and-place sequence based on the current state.
     */
    void run_state_machine()
    {
        // Do not run the state machine if movement is not enabled
        if (!movement_enabled_) {
            return;
        }

        // This function is periodically called by the timer
        switch (state_)
        {
        case State::IDLE:
            RCLCPP_INFO_ONCE(this->get_logger(), "State: IDLE. Waiting for object...");
            if (is_object_ready())
            {
                RCLCPP_INFO(this->get_logger(), "Object detected. Starting pick and place.");
                open_gripper();
                state_ = State::MOVE_TO_PRE_GRASP;
            }
            break;

        case State::MOVE_TO_PRE_GRASP:
            RCLCPP_INFO(this->get_logger(), "State: MOVE_TO_PRE_GRASP");
            if (move_to_pose(get_pre_grasp_pose()))
            {
                state_ = State::MOVE_TO_GRASP;
            }
            break;

        case State::MOVE_TO_GRASP:
            RCLCPP_INFO(this->get_logger(), "State: MOVE_TO_GRASP");
            if (move_to_pose(get_grasp_pose()))
            {
                state_ = State::GRASP_OBJECT;
            }
            break;

        case State::GRASP_OBJECT:
            RCLCPP_INFO(this->get_logger(), "State: GRASP_OBJECT");
            close_gripper();
            // Give time for the action to complete
            rclcpp::sleep_for(std::chrono::seconds(2));
            state_ = State::CHECK_GRASP;
            break;

        case State::CHECK_GRASP:
            RCLCPP_INFO(this->get_logger(), "State: CHECK_GRASP");
            if (abs(current_gripper_width_ - object_width_) < 0.01) { // 1cm tolerance
                RCLCPP_INFO(this->get_logger(), "Grasp successful. Gripper width: %.3f m", current_gripper_width_);
                state_ = State::MOVE_TO_PLACE;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Grasp failed. Expected width: %.3f, actual: %.3f. Releasing and returning to IDLE.", object_width_, current_gripper_width_);
                open_gripper(); // Release whatever was (or wasn't) grabbed
                state_ = State::IDLE;
                has_received_width_ = false;
            }
            break;

        case State::MOVE_TO_PLACE:
            RCLCPP_INFO(this->get_logger(), "State: MOVE_TO_PLACE");
            if (move_to_pose(get_place_pose()))
            {
                state_ = State::RELEASE_OBJECT;
            }
            break;

        case State::RELEASE_OBJECT:
            RCLCPP_INFO(this->get_logger(), "State: RELEASE_OBJECT");
            open_gripper();
            rclcpp::sleep_for(std::chrono::seconds(2));
            state_ = State::RETURN_HOME;
            break;

        case State::RETURN_HOME:
            RCLCPP_INFO(this->get_logger(), "State: RETURN_HOME");
            move_to_named_pose("ready");
            RCLCPP_INFO(this->get_logger(), "Pick and place complete. Returning to IDLE state.");
            state_ = State::IDLE;
            has_received_width_ = false; // Reset for next object
            object_class_ = "object0"; // Reset to default
            movement_enabled_ = false; // Wait for next GUI command
            break;
        }
    }

    /**
     * @brief Checks if a valid object is ready to be picked.
     * @return true if the object's TF frame is available and its width has been received.
     * @return false otherwise.
     */
    bool is_object_ready()
    {
        if (!has_received_width_ || object_width_ <= 0)
        {
            return false;
        }
        return tf_buffer_->canTransform("fr3_link0", "object_link", tf2::TimePointZero);
    }

    /**
     * @brief Retrieves the grasp pose from the TF tree.
     * @return The grasp pose of the object in the `fr3_link0` frame.
     */
    geometry_msgs::msg::Pose get_grasp_pose()
    {
        geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform("fr3_link0", "object_link", tf2::TimePointZero);
        geometry_msgs::msg::Pose pose;
        pose.position.x = t.transform.translation.x;
        pose.position.y = t.transform.translation.y;
        pose.position.z = t.transform.translation.z;
        pose.orientation = t.transform.rotation;
        return pose;
    }

    /**
     * @brief Calculates the pre-grasp pose, which is slightly above the object.
     * @return The pre-grasp pose.
     */
    geometry_msgs::msg::Pose get_pre_grasp_pose()
    {
        geometry_msgs::msg::Pose grasp_pose = get_grasp_pose();
        grasp_pose.position.z += 0.10; // 10cm above the object
        return grasp_pose;
    }

    /**
     * @brief Retrieves the placement pose based on the detected object's class.
     * @return The target placement pose.
     */
    geometry_msgs::msg::Pose get_place_pose()
    {
        if (placement_poses_.count(object_class_)) {
            RCLCPP_INFO(this->get_logger(), "Using placement pose for class: '%s'", object_class_.c_str());
            return placement_poses_[object_class_];
        }
        RCLCPP_WARN(this->get_logger(), "No placement pose found for class: '%s'. Using default.", object_class_.c_str());
        return placement_poses_["object0"];
    }

    /**
     * @brief Commands the robot to move to a specified Cartesian pose.
     * @param pose The target pose for the end-effector.
     * @return true if the motion plan is successfully executed.
     * @return false if planning or execution fails.
     */
    bool move_to_pose(const geometry_msgs::msg::Pose& pose)
    {
        moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), "franka_arm");
        move_group.setPoseTarget(pose);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
            move_group.execute(my_plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan motion to pose.");
        }
        return success;
    }

    /**
     * @brief Commands the robot to move to a predefined, named pose.
     * @param pose_name The name of the target pose (e.g., "ready").
     */
    void move_to_named_pose(const std::string& pose_name)
    {
        moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), "franka_arm");
        move_group.setNamedTarget(pose_name);
        move_group.move();
    }

    /**
     * @brief Callback for the start movement subscriber.
     * @param msg The incoming Bool message.
     */
    void start_movement_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            if (state_ == State::IDLE) {
                RCLCPP_INFO(this->get_logger(), "Movement enabled by GUI.");
                movement_enabled_ = true;
            } else {
                RCLCPP_WARN(this->get_logger(), "Cannot start movement, operation already in progress.");
            }
        }
    }

    /**
     * @brief Callback for the cancel operation subscriber.
     * @param msg The incoming Bool message.
     */
    void cancel_operation_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Operation cancelled by GUI. Returning to IDLE state.");
            // Potentially stop the robot here if it's moving
            // moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), "franka_arm");
            // move_group.stop();
            state_ = State::IDLE;
            movement_enabled_ = false;
            has_received_width_ = false;
            object_class_ = "object0";
        }
    }

    /**
     * @brief Callback for the gripper joint state subscriber.
     * @param msg The incoming JointState message.
     *
     * This function reads the position of the two gripper fingers and calculates the
     * current opening width.
     */
    void gripper_joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() >= 2) {
            // The gripper width is the sum of the positions of the two fingers
            current_gripper_width_ = msg->position[0] + msg->position[1];
        }
    }

    /**
     * @brief Sends a goal to the gripper action server to open the gripper.
     */
    void open_gripper()
    {
        if (!grasp_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Grasp action server not available.");
            return;
        }
        auto goal_msg = franka_msgs::action::Grasp::Goal();
        goal_msg.width = 0.08; // Max width
        goal_msg.speed = 0.1;
        goal_msg.force = 5.0;
        grasp_client_->async_send_goal(goal_msg);
        RCLCPP_INFO(this->get_logger(), "Sent open gripper goal.");
    }

    /**
     * @brief Sends a goal to the gripper action server to close the gripper.
     *
     * The gripper closes with a specified force until it makes contact.
     */
    void close_gripper()
    {
        if (!grasp_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Grasp action server not available.");
            return;
        }
        auto goal_msg = franka_msgs::action::Grasp::Goal();
        goal_msg.width = 0.0; // Close fully
        goal_msg.speed = 0.1;
        goal_msg.force = 35.0; // Apply sufficient force to hold
        grasp_client_->async_send_goal(goal_msg);
        RCLCPP_INFO(this->get_logger(), "Sent close gripper goal (force-based).");
    }

    /**
     * @brief Helper function to construct a Pose message from parameters.
     * @param param_base_name The base name for the x, y, z parameters (e.g., "placements.default").
     * @return A geometry_msgs::msg::Pose object.
     */
    geometry_msgs::msg::Pose get_pose_from_params(const std::string& param_base_name)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = this->get_parameter(param_base_name + ".x").as_double();
        pose.position.y = this->get_parameter(param_base_name + ".y").as_double();
        pose.position.z = this->get_parameter(param_base_name + ".z").as_double();
        pose.orientation.w = 1.0; // Neutral orientation
        return pose;
    }

    // Member variables
    /// @brief Action client for the franka gripper.
    rclcpp_action::Client<franka_msgs::action::Grasp>::SharedPtr grasp_client_;
    /// @brief TF2 buffer for storing transforms.
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    /// @brief TF2 listener for receiving transforms.
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    /// @brief Subscriber for the detected object's width.
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr width_sub_;
    /// @brief Subscriber for the detected object's class.
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr class_sub_;
    /// @brief Subscribers for GUI control signals.
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_movement_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_operation_sub_;
    /// @brief Subscriber for the gripper joint states.
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gripper_joint_sub_;
    /// @brief Timer to drive the state machine.
    rclcpp::TimerBase::SharedPtr timer_;
    /// @brief The current state of the pick-and-place machine.
    State state_ = State::IDLE;
    /// @brief The last received width of the object.
    double object_width_ = 0.0;
    /// @brief The class name of the detected object.
    std::string object_class_ = "object0";
    /// @brief The current measured width of the gripper opening.
    double current_gripper_width_ = 0.0;
    /// @brief Flag to indicate if a valid width has been received.
    bool has_received_width_ = false;
    /// @brief Flag to control whether the state machine is active.
    bool movement_enabled_ = false;
    /// @brief Map of placement poses loaded from parameters.
    std::map<std::string, geometry_msgs::msg::Pose> placement_poses_;
};

/**
 * @brief The main function for the action_node.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit code.
 */
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionNode>();
    // Use a MultiThreadedExecutor to allow MoveIt and actions to run in parallel with the timer
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
