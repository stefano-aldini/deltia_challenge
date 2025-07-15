# Deltia Challenge: Vision-Based Pick, Sort, and Place

This ROS 2 project implements a complete system for a robot to perform a vision-based pick, sort, and place task. The system is designed to identify objects of different classes using a 3D camera, pick them up with a robotic arm, and place them in different, pre-configured locations based on their class.

## Target Hardware

*   **Robot:** Franka Emulation Robot. The control is managed through `ros2_control` and MoveIt2.
*   **Camera:** OAK-D 3D Camera. The project uses the `depthai_ros_driver` to get spatial detection data and point clouds.

The system can be run in a simulated environment (`use_fake_hardware:=true`) without needing the physical hardware.

## System Architecture

The application consists of three main ROS 2 nodes that work together:

### 1. `gui_node.py` - Control Interface

A simple Python-based GUI built with Tkinter that provides high-level control over the system. It features:
*   **Start Detection:** Enables the `sorting_node` to start looking for objects.
*   **Start Movement:** Enables the `action_node` to begin the pick-and-place sequence once an object is found.
*   **Cancel Operation:** Stops the current operation and returns both nodes to an idle state.
*   **Status Indicators:** Shows the connection status ("Connected"/"Disconnected") for the robot and camera.

### 2. `sorting_node` (C++) - Perception

This node is responsible for all vision processing tasks.
*   **Input:** Subscribes to spatial detection messages and point cloud data from the OAK-D camera.
*   **Processing:** It identifies objects within a defined workspace, calculates their precise 3D pose and orientation using the Point Cloud Library (PCL), and determines the object's class from the detection message.
*   **Output:** Publishes the object's pose as a TF2 transform (`object_link`), its width, and its class name to be used by the `action_node`.

### 3. `action_node` (C++) - Robot Control

This node controls the robot's actions based on the information from the perception node.
*   **State Machine:** Implements a robust state machine to manage the entire pick-and-place sequence (e.g., `MOVE_TO_PRE_GRASP`, `GRASP_OBJECT`, `MOVE_TO_PLACE`).
*   **Motion Planning:** Uses the MoveIt2 `MoveGroupInterface` for all motion planning, ensuring collision-free and singularity-free trajectories.
*   **Grasping:**
    *   It uses a force-based grasp for robustness.
    *   It verifies a successful grasp by checking the final gripper width against the expected object width.
*   **Sorting:** Subscribes to the object's class name and uses it to look up the correct placement coordinates from a configuration file.

## Configuration

The placement locations for different object classes are not hardcoded. They can be easily modified by editing the `config/placements.yaml` file. This allows for quick reconfiguration of the sorting bins without recompiling the code.

Example `placements.yaml`:
```yaml
action_node:
  ros__parameters:
    placements:
      object0:
        x: 0.0
        y: -0.2
        z: 0.2
      object1:
        x: 0.2
        y: -0.2
        z: 0.2
      object2:
        x: -0.2
        y: -0.2
        z: 0.2
```

## How to Run

1.  Build the workspace with `colcon build`.
2.  Source the workspace: `source install/setup.bash`.
3.  Launch the entire system using the main launch file:
    ```bash
    ros2 launch deltia_challenge deltia.launch.py
    ```
    You can use launch arguments to configure the startup, for example:
    ```bash
    # Run in simulation mode
    ros2 launch deltia_challenge deltia.launch.py use_fake_hardware:=true

    # Run with a real robot
    ros2 launch deltia_challenge deltia.launch.py use_fake_hardware:=false robot_ip:=<your_robot_ip>
    ```
