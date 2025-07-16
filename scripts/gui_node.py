#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64
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
        self.object_width_sub = self.create_subscription(
            Float64, '/object_width', self.object_status_callback, 10)
        self.fsm_state_sub = self.create_subscription(
            String, '/robot_fsm_state', self.fsm_state_callback, 10)

        # Status variables
        self.robot_last_seen = None
        self.camera_last_seen = None
        self.object_last_seen = None
        self.robot_connected = False
        self.camera_connected = False
        self.object_detected = False
        self.detection_active = False
        self.robot_fsm_state = "UNKNOWN"

        # Timer to check for connection timeouts
        self.status_timer = self.create_timer(1.0, self.check_status)

    def robot_status_callback(self, msg):
        self.robot_last_seen = self.get_clock().now()

    def camera_status_callback(self, msg):
        self.camera_last_seen = self.get_clock().now()

    def object_status_callback(self, msg):
        if msg.data > 0:
            self.object_last_seen = self.get_clock().now()

    def fsm_state_callback(self, msg):
        self.robot_fsm_state = msg.data

    def check_status(self):
        now = self.get_clock().now()
        timeout = Duration(seconds=2.0)

        self.robot_connected = self.robot_last_seen and (now - self.robot_last_seen) < timeout
        self.camera_connected = self.camera_last_seen and (now - self.camera_last_seen) < timeout
        # Object is detected if detection is active and we've seen an object recently
        self.object_detected = self.detection_active and self.object_last_seen and (now - self.object_last_seen) < timeout

    def send_start_detection(self):
        msg = Bool()
        msg.data = True
        self.start_detection_pub.publish(msg)
        self.detection_active = True
        self.get_logger().info('Sent "Start Detection" signal.')

    def send_start_movement(self):
        msg = Bool()
        msg.data = True
        self.start_movement_pub.publish(msg)
        self.get_logger().info('Sent "Start Movement" signal.')

    def send_cancel_operation(self):
        msg = Bool()
        msg.data = True
        self.cancel_operation_pub.publish(msg)
        # Also send a stop detection signal
        stop_msg = Bool()
        stop_msg.data = False
        self.start_detection_pub.publish(stop_msg)
        self.detection_active = False
        self.get_logger().info('Sent "Cancel Operation" signal.')

def main(args=None):
    rclpy.init(args=args)
    node = ControlGuiNode()

    root = tk.Tk()
    root.title("Deltia Challenge Control")
    root.geometry("350x350")

    def on_start_detection():
        node.send_start_detection()
        start_detection_btn.config(state=tk.DISABLED)
        start_movement_btn.config(state=tk.NORMAL)
        cancel_btn.config(state=tk.NORMAL)

    def on_start_movement():
        node.send_start_movement()
        start_movement_btn.config(state=tk.DISABLED)

    def on_cancel():
        node.send_cancel_operation()
        start_detection_btn.config(state=tk.NORMAL)
        start_movement_btn.config(state=tk.DISABLED)
        cancel_btn.config(state=tk.DISABLED)

    tk.Label(root, text="Robot Control Panel", font=("Arial", 16, "bold")).pack(pady=10)

    # --- Status Frame ---
    status_frame = tk.LabelFrame(root, text="System Status", padx=10, pady=10)
    status_frame.pack(pady=10, padx=10, fill="x")

    robot_status_label = tk.Label(status_frame, text="Robot: Disconnected", fg="red")
    robot_status_label.pack(anchor="w")
    camera_status_label = tk.Label(status_frame, text="Camera: Disconnected", fg="red")
    camera_status_label.pack(anchor="w")
    detection_status_label = tk.Label(status_frame, text="Detection: Inactive", fg="orange")
    detection_status_label.pack(anchor="w")
    robot_fsm_label = tk.Label(status_frame, text="Robot State: UNKNOWN")
    robot_fsm_label.pack(anchor="w")

    # --- Control Frame ---
    control_frame = tk.LabelFrame(root, text="Controls", padx=10, pady=10)
    control_frame.pack(pady=10, padx=10, fill="x")

    start_detection_btn = tk.Button(control_frame, text="1. Start Detection", command=on_start_detection)
    start_detection_btn.pack(fill="x", pady=5)
    start_movement_btn = tk.Button(control_frame, text="2. Start Autonomous Cycle", command=on_start_movement, state=tk.DISABLED)
    start_movement_btn.pack(fill="x", pady=5)
    cancel_btn = tk.Button(control_frame, text="Cancel Operation", command=on_cancel, bg="#FF5555", fg="white", state=tk.DISABLED)
    cancel_btn.pack(fill="x", pady=5)

    def spin_and_update():
        rclpy.spin_once(node, timeout_sec=0.01)

        robot_status_label.config(text=f"Robot: {'Connected' if node.robot_connected else 'Disconnected'}",
                                  fg="green" if node.robot_connected else "red")
        camera_status_label.config(text=f"Camera: {'Connected' if node.camera_connected else 'Disconnected'}",
                                   fg="green" if node.camera_connected else "red")

        if not node.detection_active:
            detection_status_label.config(text="Detection: Inactive", fg="orange")
        elif node.object_detected:
            detection_status_label.config(text="Detection: Object Found!", fg="green")
        else:
            detection_status_label.config(text="Detection: Searching...", fg="blue")

        robot_fsm_label.config(text=f"Robot State: {node.robot_fsm_state}")

        root.after(100, spin_and_update)

    spin_and_update()
    root.mainloop()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
