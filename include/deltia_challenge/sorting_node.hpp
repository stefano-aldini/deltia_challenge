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

/**
 * @class SortingNode
 * @brief A ROS2 node for object detection, classification, and pose estimation.
 *
 * This node subscribes to spatial detection data and point cloud information from a camera
 * (e.g., OAK-D). It processes this data to identify the closest object, determine its class,
 * calculate its physical width, and broadcast its pose as a TF2 transform (`object_link`).
 * It can be enabled or disabled via a topic subscription.
 */
class SortingNode : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Sorting Node object.
     */
    SortingNode();

private:
    /**
     * @brief Callback for incoming point cloud data. Caches the latest point cloud.
     * @param msg The PointCloud2 message.
     */
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

    /**
     * @brief Callback for camera intrinsic parameters. Caches the intrinsics.
     * @param msg The CameraInfo message.
     */
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);

    /**
     * @brief Main callback for processing spatial detections from the camera.
     *
     * This function finds the closest detected object, extracts its bounding box from the
     * point cloud to calculate its width, and publishes the object's class, width, and
     * TF2 transform.
     * @param msg The SpatialDetectionArray message from the depthai_ros driver.
     */
    void detection_callback(const depthai_ros_msgs::msg::SpatialDetectionArray::ConstSharedPtr& msg);

    /**
     * @brief Callback to enable or disable the detection process.
     * @param msg A boolean message where true enables detection and false disables it.
     */
    void detection_control_callback(const std_msgs::msg::Bool::SharedPtr& msg);

    // ROS Interfaces
    rclcpp::Subscription<depthai_ros_msgs::msg::SpatialDetectionArray>::SharedPtr detection_sub_; ///< Subscriber for spatial detection data.
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;             ///< Subscriber for the point cloud.
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;               ///< Subscriber for camera intrinsic parameters.
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr width_publisher_;                        ///< Publisher for the calculated object width.
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr class_publisher_;                         ///< Publisher for the detected object class name.
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_detection_sub_;                    ///< Subscriber for the signal to start/stop detection.
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;                               ///< Broadcaster for publishing the object's pose as a TF transform.
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                                                  ///< TF2 buffer for transform operations.
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;                                     ///< TF2 listener for transform operations.

    // Node state and data
    bool camera_intrinsics_received_ = false;                                                     ///< Flag to ensure camera intrinsics are received before processing.
    bool detection_enabled_ = false;                                                              ///< Flag to enable/disable the detection callback logic.
    sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_point_cloud_;                            ///< Cached pointer to the most recent point cloud message.
    sensor_msgs::msg::CameraInfo camera_intrinsics_;                                              ///< Cached camera intrinsic parameters.
};

#endif // SORTING_NODE_HPP_