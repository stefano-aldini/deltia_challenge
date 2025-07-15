/**
 * @file sorting_node.cpp
 * @brief A ROS2 node that subscribes to spatial detection messages, identifies an object within a defined workspace,
 * and publishes its pose as a TF2 frame.
 * @copyright Copyright (c) 2025
 */
#include "deltia_challenge/sorting_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/transforms.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <limits>
#include <memory>


/**
 * @brief Construct a new Sorting Node object.
 *
 * Initializes the node, creates subscriptions to spatial detection and camera info topics,
 * and sets up the TF2 broadcaster and listener.
 */
SortingNode::SortingNode() : Node("sorting_node")
{
    // Initialize the transform buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscription to the spatial detection topic from the depthai_ros_driver
    detection_sub_ = this->create_subscription<depthai_ros_msgs::msg::SpatialDetectionArray>(
        "/camera/color/spatial_detections", 10,
        std::bind(&SortingNode::detection_callback, this, std::placeholders::_1));

    // Subscription to the point cloud topic
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points", 10,
        std::bind(&SortingNode::point_cloud_callback, this, std::placeholders::_1));

    // Subscription to the camera info topic to get intrinsic parameters
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/color/camera_info", 10,
        std::bind(&SortingNode::camera_info_callback, this, std::placeholders::_1));

    // TF2 broadcaster to publish the object's frame
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Publisher for the object width
    width_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/object_width", 10);

    // Publisher for the object class
    class_publisher_ = this->create_publisher<std_msgs::msg::String>("/object_class", 10);

    // Subscribers for GUI control
    start_detection_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/start_detection", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (msg->data) {
                RCLCPP_INFO(this->get_logger(), "Detection enabled by GUI.");
                this->detection_enabled_ = true;
            }
        });

    cancel_operation_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/cancel_operation", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (msg->data) {
                RCLCPP_INFO(this->get_logger(), "Detection cancelled by GUI.");
                this->detection_enabled_ = false;
            }
        });
}

/**
 * @brief Callback function for storing the latest point cloud message.
 * @param msg The incoming PointCloud2 message.
 */
void SortingNode::point_cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    latest_point_cloud_ = msg;
}

/**
 * @brief Callback function for receiving camera intrinsic parameters.
 * @param msg The incoming CameraInfo message.
 */
void SortingNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
    if (!camera_intrinsics_received_) {
        camera_intrinsics_ = *msg;
        camera_intrinsics_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera intrinsics received.");
        // We can unsubscribe after receiving the info once, as it's static.
        camera_info_sub_.reset();
    }
}

/**
 * @brief Callback function for processing spatial detection messages.
 * @param msg The incoming SpatialDetectionArray message.
 *
 * This function iterates through detected objects, transforms their positions into the
 * `fr3_link0` frame, and checks if they fall within a predefined workspace.
 * The first object found within the workspace is selected, and its pose is
 * published as the `object_link` frame relative to the `camera_link` frame.
 */
void SortingNode::detection_callback(const depthai_ros_msgs::msg::SpatialDetectionArray::ConstSharedPtr& msg)
{
    // Only run detection logic if it's enabled
    if (!detection_enabled_) {
        // Publish zero width when detection is disabled to clear any previous state
        std_msgs::msg::Float64 width_msg;
        width_msg.data = 0.0;
        width_publisher_->publish(width_msg);
        return;
    }

    // Ensure camera intrinsics and a point cloud have been received
    if (!camera_intrinsics_received_ || !latest_point_cloud_) {
        RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for camera intrinsics and point cloud...");
        return;
    }

    if (msg->detections.empty()) {
        object_width_m = 0.0; // Reset width if no objects are detected
        return;
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        // Look up the transform from the camera's frame to the robot's base frame
        transform_stamped = tf_buffer_->lookupTransform("fr3_link0", "camera_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform camera_link to fr3_link0: %s", ex.what());
        return;
    }

    const depthai_ros_msgs::msg::SpatialDetection* selected_detection = nullptr;

    // Find the first detection within the defined workspace
    for (const auto& detection : msg->detections) {
        geometry_msgs::msg::PointStamped point_in_camera_frame;
        point_in_camera_frame.header.frame_id = "camera_link";
        point_in_camera_frame.header.stamp = msg->header.stamp;
        point_in_camera_frame.point = detection.position;

        geometry_msgs::msg::PointStamped point_in_robot_frame;
        tf2::doTransform(point_in_camera_frame, point_in_robot_frame, transform_stamped);

        // Check if the object is within the workspace [0, 0.4]x, [-0.2, 0.2]y, [0, 0.4]z
        if (point_in_robot_frame.point.x >= 0.0 && point_in_robot_frame.point.x <= 0.4 &&
            point_in_robot_frame.point.y >= -0.2 && point_in_robot_frame.point.y <= 0.2 &&
            point_in_robot_frame.point.z >= 0.0 && point_in_robot_frame.point.z <= 0.4)
        {
            selected_detection = &detection;
            break; // Select the first object found in the workspace
        }
    }

    if (selected_detection) {
        // Convert ROS PointCloud2 message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*latest_point_cloud_, *cloud);

        // --- Extract Object Point Cloud ---
        // Create a smaller point cloud containing only the points within the 2D bbox of the detection
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            // Project 3D point to 2D image plane
            double u = (cloud->points[i].x * camera_intrinsics_.k[0]) / cloud->points[i].z + camera_intrinsics_.k[2];
            double v = (cloud->points[i].y * camera_intrinsics_.k[4]) / cloud->points[i].z + camera_intrinsics_.k[5];

            if (u >= selected_detection->bbox.center.position.x - selected_detection->bbox.size_x / 2 &&
                u <= selected_detection->bbox.center.position.x + selected_detection->bbox.size_x / 2 &&
                v >= selected_detection->bbox.center.position.y - selected_detection->bbox.size_y / 2 &&
                v <= selected_detection->bbox.center.position.y + selected_detection->bbox.size_y / 2)
            {
                inliers->indices.push_back(i);
            }
        }

        if (inliers->indices.empty()) {
            RCLCPP_WARN(this->get_logger(), "No points found in the bounding box.");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*object_cloud);

        // --- Get Oriented Bounding Box (OBB) in fr3_link0 frame ---
        // Transform the object cloud to the robot frame
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Matrix4f transform_matrix = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
        pcl::transformPointCloud(*object_cloud, *transformed_cloud, transform_matrix);

        // Use PCL to get the OBB
        pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
        feature_extractor.setInputCloud(transformed_cloud);
        feature_extractor.compute();

        pcl::PointXYZ position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;
        pcl::PointXYZ min_point_OBB, max_point_OBB;
        feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

        // --- POSITION AND ORIENTATION LOGIC ---
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "fr3_link0";
        t.child_frame_id = "object_link";

        // Correctly calculate the top-center of the object.
        // X and Y are from the center of the OBB.
        t.transform.translation.x = position_OBB.x;
        t.transform.translation.y = position_OBB.y;

        // To find the true top Z, we find the max Z of the OBB's 8 corners.
        Eigen::Vector3f p1(min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
        Eigen::Vector3f p2(min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
        Eigen::Vector3f p3(min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
        Eigen::Vector3f p4(min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
        Eigen::Vector3f p5(max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
        Eigen::Vector3f p6(max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
        Eigen::Vector3f p7(max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
        Eigen::Vector3f p8(max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
        
        float max_z = std::max({p1.z(), p2.z(), p3.z(), p4.z(), p5.z(), p6.z(), p7.z(), p8.z()});
        t.transform.translation.z = max_z;

        // Orientation: Z-axis points down, Y-axis aligns with the shorter side
        Eigen::Matrix3f rotation = rotational_matrix_OBB;
        // Ensure Z is pointing mostly up before flipping
        if (rotation.col(2).z() < 0) {
            rotation.col(0) = -rotation.col(0);
            rotation.col(2) = -rotation.col(2);
        }

        // Flip Z to point down (180 deg rotation around new X-axis)
        rotation.col(1) = -rotation.col(1);
        rotation.col(2) = -rotation.col(2);

        // Align Y-axis with the shorter side
        float size_x = max_point_OBB.x - min_point_OBB.x;
        float size_y = max_point_OBB.y - min_point_OBB.y;
        if (size_y > size_x) { // If Y is the longest side, swap axes
            std::swap(size_x, size_y);
            Eigen::Vector3f temp = rotation.col(0);
            rotation.col(0) = rotation.col(1);
            rotation.col(1) = -temp; // new Y is old -X
        }

        Eigen::Quaternionf q(rotation);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // --- OBJECT WIDTH CALCULATION ---
        object_width_m = std::min(size_x, size_y);

        // Broadcast the transform
        tf_broadcaster_->sendTransform(t);

        // Publish the object width
        std_msgs::msg::Float64 width_msg;
        width_msg.data = object_width_m;
        width_publisher_->publish(width_msg);

        // Publish the object class
        std_msgs::msg::String class_msg;
        if (!selected_detection->results.empty()) {
            class_msg.data = selected_detection->results[0].class_id;
        } else {
            class_msg.data = "object0"; // Fallback class
        }
        class_publisher_->publish(class_msg);


        RCLCPP_INFO(this->get_logger(), "Object in workspace found at robot frame: (%.2f, %.2f, %.2f m), width: %.3f m, class: %s",
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z, object_width_m, class_msg.data.c_str());
    } else {
        object_width_m = 0.0; // Reset width if no object is found in the workspace
        // Publish zero width when no valid object is detected
        std_msgs::msg::Float64 width_msg;
        width_msg.data = object_width_m;
        width_publisher_->publish(width_msg);
    }
}

/**
 * @brief The main function for the sorting_node.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit code.
 */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SortingNode>());
    rclcpp::shutdown();
    return 0;
}
