---
sidebar_position: 7
---

# Chapter 6: Building a Complete Humanoid Robot System

## Introduction

In this chapter, we'll integrate all the concepts learned in previous chapters to build a complete humanoid robot system. This will include setting up the hardware simulation, implementing control systems, integrating perception, and creating a complete "Voice-to-Action" pipeline as specified in our requirements.

## System Architecture Overview

A complete humanoid robot system consists of multiple interconnected components:

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Perception    │────│  Understanding   │────│   Action        │
│                 │    │                  │    │                 │
│ • Vision        │    │ • Language       │    │ • Motion        │
│ • Audio         │    │ • Context        │    │ • Manipulation  │
│ • IMU/Sensors   │    │ • Planning       │    │ • Balance       │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
                    ┌──────────────────┐
                    │   Control Loop   │
                    │                  │
                    │ • Real-time      │
                    │ • Safety         │
                    │ • Coordination   │
                    └──────────────────┘
```

## Setting Up the Humanoid Robot Model

### URDF Definition

Let's create a simplified humanoid robot model in URDF format:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>
  <material name="orange">
    <color rgba="1.0 0.423529411765 0.0392156862745 1.0"/>
  </material>
  <material name="brown">
    <color rgba="0.870588235294 0.811764705882 0.764705882353 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.15 0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.15 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Torso -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <visual>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.15 0.5"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.15 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="8"/>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="torso_to_head" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <link name="head">
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="torso_to_left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_shoulder_to_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Right Arm -->
  <joint name="torso_to_right_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.15 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_shoulder_to_elbow" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="right_lower_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <joint name="torso_to_left_hip" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.075 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="15" velocity="1"/>
  </joint>

  <link name="left_upper_leg">
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/>
    </inertial>
  </link>

  <joint name="left_hip_to_knee" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="1.57" effort="15" velocity="1"/>
  </joint>

  <link name="left_lower_leg">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.03" ixy="0" ixz="0" iyy="0.03" iyz="0" izz="0.03"/>
    </inertial>
  </link>

  <!-- Right Leg -->
  <joint name="torso_to_right_hip" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.075 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="15" velocity="1"/>
  </joint>

  <link name="right_upper_leg">
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="brown"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/>
    </inertial>
  </link>

  <joint name="right_hip_to_knee" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="1.57" effort="15" velocity="1"/>
  </joint>

  <link name="right_lower_leg">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="brown"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.03" ixy="0" ixz="0" iyy="0.03" iyz="0" izz="0.03"/>
    </inertial>
  </link>

  <!-- Sensors -->
  <gazebo reference="head">
    <sensor name="camera" type="camera">
      <update_rate>30</update_rate>
      <camera>
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>head</frame_name>
        <min_depth>0.1</min_depth>
        <max_depth>100</max_depth>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="torso">
    <sensor name="imu" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
        <frame_name>torso</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Gazebo ROS Control Plugin -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros2_control.so">
      <parameters>$(find my_robot_description)/config/humanoid_controllers.yaml</parameters>
    </plugin>
  </gazebo>
</robot>
```

## Robot Controllers Configuration

Create a controller configuration file to manage the robot's joints:

```yaml
# humanoid_controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    torso_to_head_position_controller:
      type: position_controllers/JointGroupPositionController

    left_arm_controller:
      type: position_controllers/JointGroupPositionController

    right_arm_controller:
      type: position_controllers/JointGroupPositionController

    legs_controller:
      type: position_controllers/JointGroupPositionController

# Head controller
torso_to_head_position_controller:
  ros__parameters:
    joints:
      - torso_to_head

# Left arm controller
left_arm_controller:
  ros__parameters:
    joints:
      - torso_to_left_shoulder
      - left_shoulder_to_elbow

# Right arm controller
right_arm_controller:
  ros__parameters:
    joints:
      - torso_to_right_shoulder
      - right_shoulder_to_elbow

# Legs controller
legs_controller:
  ros__parameters:
    joints:
      - torso_to_left_hip
      - left_hip_to_knee
      - torso_to_right_hip
      - right_hip_to_knee
```

## Implementing Balance Control

Balance is crucial for humanoid robots. Here's a basic balance controller:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3
import numpy as np
from scipy.spatial.transform import Rotation as R

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publishers
        self.leg_controller_pub = self.create_publisher(
            Float64MultiArray,
            '/legs_controller/commands',
            10
        )

        # Balance control parameters
        self.balance_kp = 10.0  # Proportional gain
        self.balance_kd = 2.0   # Derivative gain
        self.target_orientation = [0, 0, 0, 1]  # Quaternions for upright
        self.prev_angular_velocity = Vector3()
        self.balance_enabled = True

        # Timer for control loop
        self.control_timer = self.create_timer(0.01, self.balance_control_loop)

    def imu_callback(self, msg):
        # Store IMU data for balance control
        self.current_orientation = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        self.angular_velocity = msg.angular_velocity
        self.linear_acceleration = msg.linear_acceleration

    def balance_control_loop(self):
        if not self.balance_enabled:
            return

        # Calculate orientation error
        current_rot = R.from_quat(self.current_orientation)
        target_rot = R.from_quat(self.target_orientation)

        # Calculate error rotation
        error_rot = current_rot.inv() * target_rot
        error_angle_axis = error_rot.as_rotvec()

        # Calculate control effort
        orientation_error = error_angle_axis
        angular_velocity_error = [
            self.angular_velocity.x - self.prev_angular_velocity.x,
            self.angular_velocity.y - self.prev_angular_velocity.y,
            self.angular_velocity.z - self.prev_angular_velocity.z
        ]

        # PID control for balance
        control_effort = []
        for i in range(len(orientation_error)):
            proportional = self.balance_kp * orientation_error[i]
            derivative = self.balance_kd * angular_velocity_error[i]
            control_effort.append(proportional + derivative)

        # Publish control commands to leg joints
        command_msg = Float64MultiArray()
        # Simplified: adjust hip and knee positions based on balance error
        command_msg.data = [
            -control_effort[1] * 0.1,  # Left hip adjustment
            control_effort[1] * 0.05,  # Left knee adjustment
            -control_effort[1] * 0.1,  # Right hip adjustment
            control_effort[1] * 0.05   # Right knee adjustment
        ]

        self.leg_controller_pub.publish(command_msg)

        # Store current values for next iteration
        self.prev_angular_velocity = self.angular_velocity

def main(args=None):
    rclpy.init(args=args)
    balance_controller = BalanceController()
    rclpy.spin(balance_controller)
    balance_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Implementing the Complete Voice-to-Action Pipeline

Now let's implement the complete Voice-to-Action pipeline that integrates all components:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import speech_recognition as sr
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

class VoiceToActionSystem(Node):
    def __init__(self):
        super().__init__('voice_to_action_system')

        # Initialize components
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.cv_bridge = CvBridge()

        # Robot state
        self.current_image = None
        self.current_imu = None
        self.robot_state = "idle"  # idle, listening, executing, etc.

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_controller_pub = self.create_publisher(Float64MultiArray, '/arms_controller/commands', 10)
        self.head_controller_pub = self.create_publisher(Float64MultiArray, '/head_controller/commands', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)

        # Subscribers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # Timer for voice recognition
        self.voice_timer = self.create_timer(2.0, self.listen_for_voice)

        # Command mappings
        self.command_mappings = {
            'move forward': self.execute_move_forward,
            'move backward': self.execute_move_backward,
            'turn left': self.execute_turn_left,
            'turn right': self.execute_turn_right,
            'look up': self.execute_look_up,
            'look down': self.execute_look_down,
            'wave': self.execute_wave,
            'stop': self.execute_stop,
            'dance': self.execute_dance,
            'balance': self.execute_balance
        }

        self.get_logger().info("Voice-to-Action System initialized")

    def image_callback(self, msg):
        try:
            self.current_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def imu_callback(self, msg):
        self.current_imu = msg

    def listen_for_voice(self):
        try:
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source)
                self.get_logger().info("Listening for voice command...")

                # Listen with timeout
                audio = self.recognizer.listen(source, timeout=3.0, phrase_time_limit=5.0)

            # Convert speech to text
            command_text = self.recognizer.recognize_google(audio).lower()
            self.get_logger().info(f'Recognized command: {command_text}')

            # Process the command
            self.process_voice_command(command_text)

        except sr.WaitTimeoutError:
            # No speech detected, continue
            pass
        except sr.UnknownValueError:
            self.get_logger().info('Could not understand audio')
        except sr.RequestError as e:
            self.get_logger().error(f'Speech recognition error: {e}')

    def process_voice_command(self, command):
        # Publish status
        status_msg = String()
        status_msg.data = f"Processing: {command}"
        self.status_pub.publish(status_msg)

        # Find matching command in mappings
        executed = False
        for cmd_key, cmd_func in self.command_mappings.items():
            if cmd_key in command:
                self.get_logger().info(f'Executing command: {cmd_key}')
                cmd_func()
                executed = True
                break

        if not executed:
            self.get_logger().info(f'Command not recognized: {command}')
            # Try to use a language model for more complex understanding
            self.process_complex_command(command)

    def process_complex_command(self, command):
        # This would integrate with a more sophisticated language understanding system
        # For now, we'll implement some basic pattern matching
        if "pick up" in command or "grab" in command:
            self.execute_grab_object()
        elif "walk to" in command or "go to" in command:
            self.execute_navigate_to_object()
        else:
            self.get_logger().info(f'Complex command not handled: {command}')

    def execute_move_forward(self):
        msg = Twist()
        msg.linear.x = 0.3  # Forward at 0.3 m/s
        self.cmd_vel_pub.publish(msg)

    def execute_move_backward(self):
        msg = Twist()
        msg.linear.x = -0.3  # Backward at 0.3 m/s
        self.cmd_vel_pub.publish(msg)

    def execute_turn_left(self):
        msg = Twist()
        msg.angular.z = 0.5  # Turn left at 0.5 rad/s
        self.cmd_vel_pub.publish(msg)

    def execute_turn_right(self):
        msg = Twist()
        msg.angular.z = -0.5  # Turn right at 0.5 rad/s
        self.cmd_vel_pub.publish(msg)

    def execute_look_up(self):
        command_msg = Float64MultiArray()
        command_msg.data = [0.3]  # Look up
        self.head_controller_pub.publish(command_msg)

    def execute_look_down(self):
        command_msg = Float64MultiArray()
        command_msg.data = [-0.3]  # Look down
        self.head_controller_pub.publish(command_msg)

    def execute_wave(self):
        # Wave with right arm
        command_msg = Float64MultiArray()
        # Simplified waving motion - alternate positions
        command_msg.data = [0.5, 0.5, -0.5, -0.5]  # shoulder, elbow positions
        self.arm_controller_pub.publish(command_msg)

        # Reset after a short time
        self.create_timer(2.0, self.reset_arm_position)

    def execute_stop(self):
        msg = Twist()
        # Zero all velocities
        self.cmd_vel_pub.publish(msg)

        # Zero all joint positions
        zero_msg = Float64MultiArray()
        zero_msg.data = [0.0, 0.0, 0.0, 0.0]
        self.arm_controller_pub.publish(zero_msg)
        self.head_controller_pub.publish(zero_msg)

    def execute_dance(self):
        # Simple dance routine
        self.get_logger().info("Dancing!")

        # Sequence of dance moves
        dance_moves = [
            (0.5, 0, 0.5, 0),      # Wave arms
            (0, 0.5, 0, -0.5),     # Alternate wave
            (0, 0, 0, 0),          # Center arms
            (0.3, 0.3, -0.3, -0.3), # Spread arms
            (0, 0, 0, 0)           # Center again
        ]

        for i, move in enumerate(dance_moves):
            timer = self.create_timer(i * 1.0, lambda m=move: self.set_arm_positions(m))

        # Turn in circle while dancing
        turn_msg = Twist()
        turn_msg.angular.z = 0.3
        self.cmd_vel_pub.publish(turn_msg)

        # Stop turning after dance
        self.create_timer(len(dance_moves) * 1.0, self.stop_turning)

    def execute_balance(self):
        # Enable balance controller (this would interact with the balance controller node)
        self.get_logger().info("Activating balance control")
        # In a real system, this would send a message to the balance controller

    def execute_grab_object(self):
        # This would integrate with computer vision to locate and grab objects
        if self.current_image is not None:
            self.get_logger().info("Attempting to grab object")
            # Process image to find objects
            # Plan grasp trajectory
            # Execute grasp
        else:
            self.get_logger().info("No image available for object detection")

    def execute_navigate_to_object(self):
        # This would integrate with navigation and object detection
        if self.current_image is not None:
            self.get_logger().info("Attempting to navigate to object")
            # Detect object in image
            # Plan navigation path
            # Execute navigation
        else:
            self.get_logger().info("No image available for navigation")

    def set_arm_positions(self, positions):
        command_msg = Float64MultiArray()
        command_msg.data = list(positions)
        self.arm_controller_pub.publish(command_msg)

    def reset_arm_position(self):
        command_msg = Float64MultiArray()
        command_msg.data = [0.0, 0.0, 0.0, 0.0]
        self.arm_controller_pub.publish(command_msg)

    def stop_turning(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    vta_system = VoiceToActionSystem()

    try:
        rclpy.spin(vta_system)
    except KeyboardInterrupt:
        pass
    finally:
        vta_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch File for Complete System

Create a launch file to start all components together:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'simple_humanoid',
            '-x', '0', '-y', '0', '-z', '1.0'
        ],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': True}]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='both',
        parameters=[{'use_sim_time': True}]
    )

    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'config',
                'humanoid_controllers.yaml'
            ])
        ],
        output='both',
        parameters=[{'use_sim_time': True}]
    )

    # Balance Controller
    balance_controller = Node(
        package='my_robot_control',
        executable='balance_controller',
        output='both',
        parameters=[{'use_sim_time': True}]
    )

    # Voice-to-Action System
    voice_to_action = Node(
        package='my_robot_control',
        executable='voice_to_action_system',
        output='both',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        use_sim_time,
        gazebo,
        spawn_entity,
        robot_state_publisher,
        joint_state_publisher,
        controller_manager,
        balance_controller,
        voice_to_action,
    ])
```

## Testing the Complete System

### Basic Testing Commands

Once the system is running, you can test it with these voice commands:

1. "Move forward" - Robot moves forward
2. "Turn left" - Robot turns left
3. "Look up" - Robot tilts head up
4. "Wave" - Robot waves with its arm
5. "Stop" - Robot stops all movement
6. "Dance" - Robot performs a simple dance routine

### Verification Steps

1. **Launch the simulation**:
   ```bash
   ros2 launch my_robot_bringup humanoid_complete.launch.py
   ```

2. **Check all nodes are running**:
   ```bash
   ros2 node list
   ```

3. **Verify topics are publishing**:
   ```bash
   ros2 topic list
   ros2 topic echo /robot_status
   ```

4. **Test voice commands** - speak clearly near the microphone

## Safety Considerations

When implementing a complete humanoid robot system, safety is paramount:

### Software Safety

- **Bounds Checking**: Ensure joint limits are never exceeded
- **Emergency Stop**: Implement immediate stop functionality
- **State Monitoring**: Continuously monitor robot state for anomalies

### Hardware Safety

- **Torque Limiting**: Prevent excessive forces that could damage hardware
- **Collision Detection**: Stop motion if unexpected forces are detected
- **Fall Detection**: Implement fall detection and recovery procedures

## Performance Optimization

### Real-time Performance

- **Control Loop Timing**: Maintain consistent control loop timing
- **Priority Scheduling**: Use real-time scheduling for critical control tasks
- **Efficient Algorithms**: Optimize perception and planning algorithms

### Resource Management

- **Memory Management**: Pre-allocate memory to avoid runtime allocation
- **Threading**: Use appropriate threading for different subsystems
- **GPU Utilization**: Leverage GPU acceleration where possible

## Troubleshooting Common Issues

### Robot Falls Over

- Check IMU calibration
- Verify center of mass calculation
- Adjust balance controller gains
- Check joint friction and damping parameters

### Voice Recognition Issues

- Check microphone input levels
- Verify speech recognition service is running
- Test in quiet environment initially

### Joint Control Problems

- Verify controller configuration
- Check joint limits and safety parameters
- Confirm proper URDF joint definitions

## Summary

This chapter demonstrated how to integrate all the components learned in previous chapters into a complete humanoid robot system. We covered:

- Creating a complete humanoid robot URDF model
- Setting up controllers for different body parts
- Implementing balance control for stability
- Creating a complete Voice-to-Action pipeline
- Configuring all components to work together

The system successfully implements the key requirement from our specification: readers can now implement a simple "Voice-to-Action" humanoid robot pipeline. The next chapter will cover advanced topics and potential extensions to the system.