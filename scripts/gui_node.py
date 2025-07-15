#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import CameraInfo
from franka_msgs.msg import FrankaRobotState
import tkinter as tk
from rclpy.duration import Duration

class ControlGuiNode(Node):
    """
    A ROS2 node that provides a simple Tkinter GUI for controlling the pick-and-place operation.
    """
    def __init__(self):
        super().__init__('control_gui_node')
        # Publishers for control signals
        self.start_detection_pub = self.create_publisher(Bool, '/start_detection', 10)
        self.start_movement_pub = self.create_publisher(Bool, '/start_movement', 10)
        self.cancel_operation_pub = self.create_publisher(Bool, '/cancel_operation', 10)

        # Subscribers for status checking
        self.robot_status_sub = self.create_subscription(
            FrankaRobotState, '/franka_robot_state_broadcaster/robot_state', self.robot_status_callback, 10)
        self.camera_status_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_status_callback, 10)

        # Status variables
        self.robot_last_seen = None
        self.camera_last_seen = None
        self.robot_connected = False
        self.camera_connected = False

        # Timer to check for connection timeouts
        self.status_timer = self.create_timer(1.0, self.check_status)

    def robot_status_callback(self, msg):
        """Callback for robot status topic. Updates the last seen timestamp."""
        self.robot_last_seen = self.get_clock().now()

    def camera_status_callback(self, msg):
        """Callback for camera status topic. Updates the last seen timestamp."""
        self.camera_last_seen = self.get_clock().now()

    def check_status(self):
        """Periodically checks if the robot and camera are still connected."""
        now = self.get_clock().now()
        timeout = Duration(seconds=2.0) # Correctly use a Duration object

        if self.robot_last_seen and (now - self.robot_last_seen) < timeout:
            self.robot_connected = True
        else:
            self.robot_connected = False

        if self.camera_last_seen and (now - self.camera_last_seen) < timeout:
            self.camera_connected = True
        else:
            self.camera_connected = False

    def send_start_detection(self):
        """Publishes a message to start the object detection."""
        msg = Bool()
        msg.data = True
        self.start_detection_pub.publish(msg)
        self.get_logger().info('Sent "Start Detection" signal.')

    def send_start_movement(self):
        """Publishes a message to start the robot's pick-and-place movement."""
        msg = Bool()
        msg.data = True
        self.start_movement_pub.publish(msg)
        self.get_logger().info('Sent "Start Movement" signal.')

    def send_cancel_operation(self):
        """Publishes a message to cancel all ongoing operations."""
        msg = Bool()
        msg.data = True
        self.cancel_operation_pub.publish(msg)
        self.get_logger().info('Sent "Cancel Operation" signal.')

def main(args=None):
    rclpy.init(args=args)
    node = ControlGuiNode()

    # --- GUI Setup ---
    root = tk.Tk()
    root.title("Deltia Challenge Control")
    root.geometry("300x280")

    # --- Button Callbacks ---
    def on_start_detection():
        node.send_start_detection()

    def on_start_movement():
        node.send_start_movement()

    def on_cancel():
        node.send_cancel_operation()

    # --- GUI Widgets ---
    label = tk.Label(root, text="Robot Control Panel", font=("Arial", 14))
    label.pack(pady=10)

    # Status Labels
    status_frame = tk.Frame(root)
    status_frame.pack(pady=5)
    robot_status_label = tk.Label(status_frame, text="Robot: Disconnected", fg="red", font=("Arial", 10))
    robot_status_label.pack(side=tk.LEFT, padx=10)
    camera_status_label = tk.Label(status_frame, text="Camera: Disconnected", fg="red", font=("Arial", 10))
    camera_status_label.pack(side=tk.LEFT, padx=10)

    start_detection_btn = tk.Button(root, text="Start Detection", command=on_start_detection, height=2, width=20)
    start_detection_btn.pack(pady=5)

    start_movement_btn = tk.Button(root, text="Start Movement", command=on_start_movement, height=2, width=20)
    start_movement_btn.pack(pady=5)

    cancel_btn = tk.Button(root, text="Cancel Operation", command=on_cancel, bg="red", fg="white", height=2, width=20)
    cancel_btn.pack(pady=5)

    # Function to update GUI status labels and spin ROS
    def spin_and_update():
        # Process a single ROS event
        rclpy.spin_once(node, timeout_sec=0.01)

        # Update GUI labels
        if node.robot_connected:
            robot_status_label.config(text="Robot: Connected", fg="green")
        else:
            robot_status_label.config(text="Robot: Disconnected", fg="red")

        if node.camera_connected:
            camera_status_label.config(text="Camera: Connected", fg="green")
        else:
            camera_status_label.config(text="Camera: Disconnected", fg="red")

        # Schedule the next update
        root.after(100, spin_and_update)

    # Start the combined spinner and GUI updater
    spin_and_update()

    # Start the Tkinter main loop (this is blocking)
    root.mainloop()

    # Cleanup after the GUI is closed
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
