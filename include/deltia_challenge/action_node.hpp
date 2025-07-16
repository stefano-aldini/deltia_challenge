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
 * @brief Controls the Franka robot to perform a continuous, autonomous pick-and-place task.
 *
 * This class implements a state machine to manage the robot's actions, from detecting an object
 * to picking it up and placing it in a designated location. It features real-time trajectory
 * correction, grasp stability checking, and target locking based on object classification.
 */
class ActionNode : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Action Node object.
     * @param options Node options for rclcpp.
     */
    explicit ActionNode(const rclcpp::NodeOptions & options);

private:
    /// @brief Defines the states of the robot's pick-and-place operation.
    enum class State
    {
        IDLE,               ///< Waiting for an object and for the cycle to be enabled.
        MOVE_TO_PRE_GRASP,  ///< Moving to a safe position above the target object.
        MOVE_TO_GRASP,      ///< Moving down to the object to grasp it.
        GRASP_OBJECT,       ///< Closing the gripper.
        CHECK_GRASP,        ///< Verifying that the grasp was successful.
        MOVE_TO_PLACE,      ///< Moving the object to its designated drop-off location.
        RELEASE_OBJECT,     ///< Opening the gripper to release the object.
        RETURN_HOME         ///< Moving back to a neutral 'ready' position.
    };

    /**
     * @brief The main state machine loop, called by a periodic timer.
     */
    void run_state_machine();

    /**
     * @brief Checks if a valid object is ready to be picked up.
     * @return true if an object is detected and meets criteria, false otherwise.
     */
    bool is_object_ready();

    /**
     * @brief Calculates the pre-grasp pose, typically 10cm above the object.
     * @return The calculated pre-grasp pose.
     */
    geometry_msgs::msg::Pose get_pre_grasp_pose();

    /**
     * @brief Gets the current, real-time pose of the target object from TF2.
     * @return The current pose of the object. Returns an invalid pose if not found or if class lock is lost.
     */
    geometry_msgs::msg::Pose get_current_object_pose();

    /**
     * @brief Gets the designated placement pose for the currently held object class.
     * @return The placement pose.
     */
    geometry_msgs::msg::Pose get_place_pose();

    /**
     * @brief Plans and executes a motion to a target pose using MoveIt.
     * @param target_pose The destination pose for the end-effector.
     * @return true if the motion was successful, false otherwise.
     */
    bool move_to_pose(const geometry_msgs::msg::Pose& target_pose);

    /**
     * @brief Moves the robot to a pre-defined, named pose from the SRDF.
     * @param pose_name The name of the pose (e.g., "ready").
     */
    void move_to_named_pose(const std::string& pose_name);

    /**
     * @brief Opens the gripper to a default width.
     */
    void open_gripper();

    /**
     * @brief Closes the gripper using a target force to grasp an object.
     */
    void close_gripper();

    /**
     * @brief Callback for gripper joint states, used to read width and force.
     * @param msg The incoming joint state message.
     */
    void gripper_joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    /**
     * @brief Callback for the GUI "Start Movement" signal.
     * @param msg The incoming boolean message.
     */
    void start_movement_callback(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief Callback for the GUI "Cancel Operation" signal.
     * @param msg The incoming boolean message.
     */
    void cancel_operation_callback(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief Helper function to load a pose from the parameter server.
     * @param param_name The base name of the pose parameters.
     * @return The loaded pose.
     */
    geometry_msgs::msg::Pose get_pose_from_params(const std::string& param_name);

    /**
     * @brief Publishes a message to enable or disable the perception node.
     * @param enable True to enable detection, false to disable.
     */
    void publish_detection_control(bool enable);

    /**
     * @brief Calculates the Euclidean distance between two poses.
     * @param pose1 The first pose.
     * @param pose2 The second pose.
     * @return The distance in meters.
     */
    double calculate_distance(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2);

    /**
     * @brief Converts the current state to a string for debugging or publishing.
     * @return A string representing the current state.
     */
    std_msgs::msg::String get_state_as_string();

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detection_control_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
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
    std::string locked_target_class_;
};

#endif // ACTION_NODE_HPP_