---
sidebar_position: 3
---

# Chapter 2: ROS 2 for Humanoid Robot Control

## Introduction to ROS 2

Robot Operating System 2 (ROS 2) is the next-generation framework for developing robot applications. Unlike its predecessor, ROS 2 is designed for production environments with improved security, real-time capabilities, and better cross-platform support. For humanoid robotics, ROS 2 provides the essential communication infrastructure that connects perception, planning, control, and learning systems.

### Why ROS 2 for Humanoid Robotics?

Humanoid robots require complex coordination between multiple subsystems:

- Joint controllers managing dozens of actuators
- Perception systems processing visual, auditory, and tactile data
- Planning systems determining safe and efficient movements
- Control systems maintaining balance and executing tasks

ROS 2's distributed architecture enables these subsystems to communicate efficiently while maintaining modularity and fault tolerance.

## ROS 2 Architecture

### Nodes and Communication

In ROS 2, functionality is organized into **nodes** - processes that perform computation. Nodes communicate through:

- **Topics**: Publish/subscribe messaging for continuous data streams
- **Services**: Request/response communication for specific tasks
- **Actions**: Goal-oriented communication for long-running tasks

For humanoid robots, this means:
- Joint state publishers continuously broadcast actuator positions
- Perception nodes publish sensor data to multiple subscribers
- Planning nodes request specific tasks through services
- Walking controllers use actions to execute complex movement sequences

### Quality of Service (QoS)

ROS 2 introduces Quality of Service profiles that allow fine-tuning of communication behavior:

- **Reliability**: Ensure messages are delivered (reliable) or allow some loss (best effort)
- **Durability**: Guarantee delivery to late-joining subscribers
- **History**: Control how many messages to keep in history

For safety-critical humanoid robot control, appropriate QoS settings ensure critical messages like emergency stops are reliably delivered.

## Setting Up ROS 2 for Humanoid Robotics

### Installation

ROS 2 can be installed on various platforms. For humanoid robotics, Ubuntu Linux is commonly used:

```bash
# Add ROS 2 GPG key and repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /etc/null

sudo apt update
sudo apt install ros-humble-desktop
```

### Workspace Setup

Create a workspace for your humanoid robot project:

```bash
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws
colcon build
source install/setup.bash
```

## Core ROS 2 Concepts for Humanoid Robots

### Message Types

ROS 2 defines standard message types that are particularly relevant to humanoid robotics:

- `sensor_msgs/JointState`: Joint positions, velocities, and efforts
- `geometry_msgs/Twist`: Linear and angular velocities for base movement
- `geometry_msgs/Pose`: Position and orientation in 3D space
- `sensor_msgs/Image`: Camera image data
- `sensor_msgs/Imu`: Inertial measurement unit data

### TF (Transforms)

The Transform (TF) system is crucial for humanoid robots, tracking the position and orientation of all robot parts relative to each other. For a humanoid, TF tracks:
- Head position relative to torso
- Hand position relative to arm
- Foot position relative to world
- Camera position relative to head

## ROS 2 Packages for Humanoid Robotics

### robot_state_publisher

Publishes the state of the robot's joints to the TF system using URDF (Unified Robot Description Format) models.

### joint_state_publisher

Provides joint state information for visualization and control.

### controller_manager

Manages robot controllers, allowing switching between different control modes.

### moveit2

Advanced motion planning for complex humanoid manipulation tasks.

## Practical Example: Creating a Simple ROS 2 Node

Let's create a simple ROS 2 node that controls a humanoid robot's joint:

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class JointController : public rclcpp::Node
{
public:
    JointController() : Node("joint_controller")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/joint_group_position_controller/commands", 10);

        timer_ = this->create_wall_timer(
            50ms, std::bind(&JointController::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::Float64MultiArray();
        // Example: Send target positions for humanoid joints
        message.data = {0.0, 0.5, -0.5, 0.0, 0.5, -0.5}; // Joint positions
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointController>());
    rclcpp::shutdown();
    return 0;
}
```

## ROS 2 Launch Files

Launch files allow you to start multiple nodes with a single command:

```xml
<launch>
  <!-- Start robot state publisher -->
  <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
    <param name="robot_description" value="$(var robot_description)"/>
  </node>

  <!-- Start joint state publisher -->
  <node pkg="joint_state_publisher" exec="joint_state_publisher" name="joint_state_publisher"/>

  <!-- Start controller manager -->
  <node pkg="controller_manager" exec="ros2_control_node" name="ros2_control_node">
    <param name="robot_description" value="$(var robot_description)"/>
  </node>
</launch>
```

## ROS 2 for Real-time Control

For humanoid robots requiring real-time performance:

### Real-time Kernel

Consider using a real-time Linux kernel to ensure deterministic behavior for critical control loops.

### Real-time Scheduling

Use appropriate scheduling policies (SCHED_FIFO) for time-critical control nodes.

### Memory Management

Pre-allocate memory to avoid dynamic allocation during critical control periods.

## Troubleshooting Common ROS 2 Issues

### Network Configuration

For distributed humanoid robot systems, ensure proper ROS 2 domain IDs and network configuration.

### Timing Issues

Use ROS 2's time abstraction to handle timing-sensitive operations correctly.

### Resource Management

Monitor CPU and memory usage, especially for embedded systems on humanoid robots.

## Summary

ROS 2 provides the essential communication infrastructure for humanoid robotics. Understanding its architecture, message passing mechanisms, and ecosystem of packages is crucial for developing sophisticated humanoid robots. The next chapter will explore how to simulate these robots using Gazebo and other simulation environments.