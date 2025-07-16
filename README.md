# Deltia Challenge - Autonomous Pick and Place

This ROS 2 package implements an autonomous pick-and-place system for the Franka Emika robot arm. The system is designed to detect, classify, and sort objects in a workspace using a camera for perception and a state-machine-driven controller for robust robot actions.

## Features

- **Autonomous Loop**: Once started, the system continuously detects, picks, and places objects until the workspace is clear or the operation is cancelled.
- **Real-time Trajectory Correction**: The robot tracks the target object's position in real-time during its approach, allowing it to grasp objects that move slightly.
- **Target Locking by Class**: To avoid getting distracted by multiple objects, the system "locks on" to the class of a target object and will ignore other objects until the current cycle is complete.
- **Grasp Stability Check**: After grasping, the robot monitors the gripper force. If the object is dropped, the robot aborts the current task and returns home to restart the cycle.
- **Robust State Machine**: The core logic is built around a state machine that handles the sequence of operations and gracefully recovers from failures (e.g., failed motion plan, lost object).
- **Simple User Interface**: A Python-based GUI allows for easy starting, stopping, and monitoring of the system's status.

## System Components

The system consists of three main nodes:

- **`action_node` (C++)**: The brain of the operation. This node runs the core state machine, interfaces with MoveIt2 for motion planning, controls the gripper, and makes decisions based on perception data.
- **`sorting_node` (C++)**: The perception system. This node processes camera data to detect objects, determine their class and pose, and publishes this information for the `action_node`.
- **`gui_node.py` (Python)**: A Tkinter-based GUI that provides simple controls (Start/Cancel) and status feedback (Robot/Camera connection, Detection status, Robot state).

## Dependencies

This package depends on the following ROS 2 packages:
- `rclcpp`, `rclcpp_action`, `rclpy`
- `moveit_ros_planning_interface`
- `franka_msgs`
- `tf2_ros`, `tf2_geometry_msgs`
- `geometry_msgs`, `sensor_msgs`, `std_msgs`
- `depthai_ros_msgs` (for the perception node)
- `pcl_conversions`

## How to Use

1.  **Build the Workspace**
    ```bash
    cd /home/stef/dev
    colcon build --packages-select deltia_challenge
    ```

2.  **Source the Workspace**
    ```bash
    source /home/stef/dev/install/setup.bash
    ```

3.  **Launch the System**
    Run the main launch file which starts all the required nodes.
    ```bash
    ros2 launch deltia_challenge challenge.launch.py
    ```

4.  **Operate the GUI**
    - **1. Start Detection**: Click this to activate the perception system. The status indicators will show when the camera is active and if an object is found.
    - **2. Start Autonomous Cycle**: Click this to enable robot movement. The robot will begin its continuous pick-and-place loop.
    - **Cancel Operation**: Click this at any time to stop the robot, return it to its home position, and deactivate the autonomous cycle.

## Configuration

The drop-off locations for different object classes are configured in the `config/placements.yaml` file. You can edit this file to change the target poses for each object type.
