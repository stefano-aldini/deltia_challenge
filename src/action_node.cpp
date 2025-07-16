/**
 * @file action_node.cpp
 * @brief A ROS2 node that controls the robot's pick and place actions.
 * @copyright Copyright (c) 2025
 */
#include "deltia_challenge/action_node.hpp"

ActionNode::ActionNode(const rclcpp::NodeOptions & options) : Node("action_node", options)
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

    // Publisher to control the sorting_node
    detection_control_pub_ = this->create_publisher<std_msgs::msg::Bool>("/start_detection", 10);

    // Main timer to run the state machine
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ActionNode::run_state_machine, this));
}

void ActionNode::run_state_machine()
{
    if (!movement_enabled_) {
        return;
    }

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
        rclcpp::sleep_for(std::chrono::seconds(2));
        state_ = State::CHECK_GRASP;
        break;

    case State::CHECK_GRASP:
        RCLCPP_INFO(this->get_logger(), "State: CHECK_GRASP");
        if (abs(current_gripper_width_ - object_width_) < 0.01) {
            RCLCPP_INFO(this->get_logger(), "Grasp successful. Gripper width: %.3f m", current_gripper_width_);
            RCLCPP_INFO(this->get_logger(), "Pausing object detection.");
            publish_detection_control(false); // Pause detection
            state_ = State::MOVE_TO_PLACE;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Grasp failed. Expected width: %.3f, actual: %.3f. Releasing and returning to IDLE.", object_width_, current_gripper_width_);
            open_gripper();
            state_ = State::IDLE;
            has_received_width_ = false;
        }
        break;

    case State::MOVE_TO_PLACE:
            RCLCPP_INFO(this->get_logger(), "State: MOVE_TO_PLACE");
            // Continuously check if the object has been dropped
            if (current_gripper_force_ < 5.0) { // Threshold for grasp loss in Newtons
                RCLCPP_ERROR(this->get_logger(), "Object lost during transport! Force dropped to %.2f N.", current_gripper_force_);
                // Stop current motion (though MoveIt might need more to halt immediately)
                // Re-enable detection and go home to restart the process
                publish_detection_control(true);
                move_to_named_pose("ready");
                state_ = State::IDLE;
                has_received_width_ = false;
                movement_enabled_ = false; // Require GUI to start again
                break;
            }

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
        RCLCPP_INFO(this->get_logger(), "Resuming object detection.");
        publish_detection_control(true); // Resume detection
        state_ = State::IDLE;
        has_received_width_ = false;
        object_class_ = "object0";
        movement_enabled_ = false;
        break;
    }
}

bool ActionNode::is_object_ready()
{
    if (!has_received_width_ || object_width_ <= 0)
    {
        return false;
    }
    return tf_buffer_->canTransform("fr3_link0", "object_link", tf2::TimePointZero);
}

geometry_msgs::msg::Pose ActionNode::get_grasp_pose()
{
    geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform("fr3_link0", "object_link", tf2::TimePointZero);
    geometry_msgs::msg::Pose pose;
    pose.position.x = t.transform.translation.x;
    pose.position.y = t.transform.translation.y;
    pose.position.z = t.transform.translation.z;
    pose.orientation = t.transform.rotation;
    return pose;
}

geometry_msgs::msg::Pose ActionNode::get_pre_grasp_pose()
{
    geometry_msgs::msg::Pose grasp_pose = get_grasp_pose();
    grasp_pose.position.z += 0.10;
    return grasp_pose;
}

geometry_msgs::msg::Pose ActionNode::get_place_pose()
{
    if (placement_poses_.count(object_class_)) {
        RCLCPP_INFO(this->get_logger(), "Using placement pose for class: '%s'", object_class_.c_str());
        return placement_poses_[object_class_];
    }
    RCLCPP_WARN(this->get_logger(), "No placement pose found for class: '%s'. Using default.", object_class_.c_str());
    return placement_poses_["object0"];
}

bool ActionNode::move_to_pose(const geometry_msgs::msg::Pose& pose)
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

void ActionNode::move_to_named_pose(const std::string& pose_name)
{
    moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), "franka_arm");
    move_group.setNamedTarget(pose_name);
    move_group.move();
}

void ActionNode::start_movement_callback(const std_msgs::msg::Bool::SharedPtr msg)
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

void ActionNode::cancel_operation_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        RCLCPP_INFO(this->get_logger(), "Operation cancelled by GUI. Returning to IDLE state.");
        publish_detection_control(true); // Also re-enable detection on cancel
        state_ = State::IDLE;
        movement_enabled_ = false;
        has_received_width_ = false;
        object_class_ = "object0";
    }
}

void ActionNode::gripper_joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->position.size() >= 2) {
        current_gripper_width_ = msg->position[0] + msg->position[1];
    }
    if (msg->effort.size() >= 2) {
        // Sum the absolute effort of both gripper fingers
        current_gripper_force_ = std::abs(msg->effort[0]) + std::abs(msg->effort[1]);
    }
}

void ActionNode::open_gripper()
{
    if (!grasp_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Grasp action server not available.");
        return;
    }
    auto goal_msg = franka_msgs::action::Grasp::Goal();
    goal_msg.width = 0.08;
    goal_msg.speed = 0.1;
    goal_msg.force = 5.0;
    grasp_client_->async_send_goal(goal_msg);
    RCLCPP_INFO(this->get_logger(), "Sent open gripper goal.");
}

void ActionNode::close_gripper()
{
    if (!grasp_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Grasp action server not available.");
        return;
    }
    auto goal_msg = franka_msgs::action::Grasp::Goal();
    goal_msg.width = 0.0;
    goal_msg.speed = 0.1;
    goal_msg.force = 35.0;
    grasp_client_->async_send_goal(goal_msg);
    RCLCPP_INFO(this->get_logger(), "Sent close gripper goal (force-based).");
}

geometry_msgs::msg::Pose ActionNode::get_pose_from_params(const std::string& param_base_name)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = this->get_parameter(param_base_name + ".x").as_double();
    pose.position.y = this->get_parameter(param_base_name + ".y").as_double();
    pose.position.z = this->get_parameter(param_base_name + ".z").as_double();
    pose.orientation.w = 1.0;
    return pose;
}

void ActionNode::publish_detection_control(bool enable)
{
    auto msg = std_msgs::msg::Bool();
    msg.data = enable;
    detection_control_pub_->publish(msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<ActionNode>(node_options));
    rclcpp::shutdown();
    return 0;
}
