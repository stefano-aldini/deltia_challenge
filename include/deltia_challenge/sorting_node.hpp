#ifndef SORTING_NODE_HPP_
#define SORTING_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

class SortingNode : public rclcpp::Node
{
public:
    SortingNode();
    double object_width_m = 0.0;

private:
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
    void detection_callback(const depthai_ros_msgs::msg::SpatialDetectionArray::ConstSharedPtr& msg);

    rclcpp::Subscription<depthai_ros_msgs::msg::SpatialDetectionArray>::SharedPtr detection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr width_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr class_publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_detection_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_operation_sub_;
    bool camera_intrinsics_received_ = false;
    bool detection_enabled_ = false;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_point_cloud_;
    sensor_msgs::msg::CameraInfo camera_intrinsics_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif // SORTING_NODE_HPP_