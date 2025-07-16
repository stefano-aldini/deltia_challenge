#ifndef ACTION_NODE_HPP_
#define ACTION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "franka_msgs/action/grasp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <map>
#include <string>
#include <memory>

/**
 * @class ActionNode
 * @brief Controls the Franka robot to perform a pick-and-place task.
 */
class ActionNode : public rclcpp::Node
{
public:
    explicit ActionNode(const rclcpp::NodeOptions & options);

private:
    enum class State
    {
        IDLE,
        MOVE_TO_PRE_GRASP,
        MOVE_TO_GRASP,
        GRASP_OBJECT,
        CHECK_GRASP,
        MOVE_TO_PLACE,
        RELEASE_OBJECT,
        RETURN_HOME
    };

    void run_state_machine();
    bool is_object_ready();
    geometry_msgs::msg::Pose get_pre_grasp_pose();
    geometry_msgs::msg::Pose get_grasp_pose();
    geometry_msgs::msg::Pose get_place_pose();
    bool move_to_pose(const geometry_msgs::msg::Pose& target_pose);
    void move_to_named_pose(const std::string& pose_name);
    void open_gripper();
    void close_gripper();
    void gripper_joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void start_movement_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void cancel_operation_callback(const std_msgs::msg::Bool::SharedPtr msg);
    geometry_msgs::msg::Pose get_pose_from_params(const std::string& param_name);
    void publish_detection_control(bool enable);

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detection_control_pub_;
    rclcpp_action::Client<franka_msgs::action::Grasp>::SharedPtr grasp_client_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr width_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr class_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_movement_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_operation_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gripper_joint_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    State state_ = State::IDLE;
    double object_width_ = 0.0;
    std::string object_class_ = "object0";
    double current_gripper_width_ = 0.0;
    double current_gripper_force_ = 0.0;
    bool has_received_width_ = false;
    bool movement_enabled_ = false;
    std::map<std::string, geometry_msgs::msg::Pose> placement_poses_;
};

#endif // ACTION_NODE_HPP_