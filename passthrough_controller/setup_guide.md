# Passthrough Controller Setup Guide

This guide will help you set up a passthrough controller that mirrors your real robot's joint states in Gazebo simulation.

## Overview

The passthrough controller subscribes to your real robot's joint states topic and applies those joint positions directly to the simulated robot in Gazebo. This creates a real-time mirror of your physical robot's movements.

## Directory Structure

Create your ROS2 package with this structure:

```
passthrough_controller/
├── CMakeLists.txt
├── package.xml
├── passthrough_controller_plugin.xml
├── include/
│   └── passthrough_controller/
│       └── passthrough_controller.hpp
├── src/
│   └── passthrough_controller.cpp
├── config/
│   └── passthrough_controller.yaml
└── launch/
    └── passthrough_controller_launch.py
```

## Setup Steps

### 1. Create the Package

```bash
cd ~/your_ws/src
ros2 pkg create --build-type ament_cmake passthrough_controller
cd passthrough_controller
```

### 2. Copy the Files

Copy all the provided files into their respective directories:

- Header file → `include/passthrough_controller/passthrough_controller.hpp`
- Implementation → `src/passthrough_controller.cpp`
- Plugin XML → `passthrough_controller_plugin.xml`
- Package files → `package.xml` and `CMakeLists.txt`
- Configuration → `config/passthrough_controller.yaml`
- Launch file → `launch/passthrough_controller_launch.py`

### 3. Configure Your Robot

Edit `config/passthrough_controller.yaml`:

```yaml
passthrough_controller:
  ros__parameters:
    joints:
      - your_joint_1_name
      - your_joint_2_name
      - your_joint_3_name
      # Add all your robot's joint names as they appear in joint_states
    
    real_robot_joint_states_topic: "/your_real_robot/joint_states"
```

### 4. Update Your Robot's URDF/XACRO

Ensure your simulated robot's URDF includes ros2_control tags. Add this to your robot's URDF:

```xml
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  
  <!-- For each joint in your robot -->
  <joint name="joint_name">
    <command_interface name="position">
      <param name="min">-3.14159</param>
      <param name="max">3.14159</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>

<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(find your_package)/config/passthrough_controller.yaml</parameters>
  </plugin>
</gazebo>
```

### 5. Build the Package

```bash
cd ~/your_ws
colcon build --packages-select passthrough_controller
source install/setup.bash
```

## Usage

### Method 1: Using the Launch File

```bash
ros2 launch passthrough_controller passthrough_controller_launch.py \
    robot_description_file:=/path/to/your/robot.urdf \
    config_file:=/path/to/your/config.yaml
```

### Method 2: Manual Steps

1. **Start Gazebo:**
   ```bash
   gazebo
   ```

2. **Launch your robot in Gazebo:**
   ```bash
   ros2 launch your_robot_gazebo your_robot_gazebo.launch.py
   ```

3. **Start the controller manager:**
   ```bash
   ros2 run controller_manager ros2_control_node \
       --ros-args --params-file config/passthrough_controller.yaml
   ```

4. **Load and activate the controller:**
   ```bash
   ros2 run controller_manager spawner passthrough_controller
   ```

## Verification

### Check Topics

Verify the topics are working:

```bash
# Check if real robot is publishing joint states
ros2 topic echo /your_real_robot/joint_states

# Check controller status
ros2 control list_controllers

# Check if controller is receiving data
ros2 topic echo /dynamic_joint_states
```

### Monitor Performance

```bash
# Check controller update rate
ros2 topic hz /your_real_robot/joint_states

# Monitor any controller warnings
ros2 log show controller_manager
```

## Troubleshooting

**Controller fails to load:**
- Verify joint names in config match exactly with your URDF
- Check that ros2_control plugin is loaded in Gazebo
- Ensure all dependencies are installed

**Joint states not being applied:**
- Verify the real robot's joint_states topic name
- Check that joint names match between real robot and simulation
- Ensure the controller is active: `ros2 control list_controllers`

**Performance issues:**
- Reduce update rate in config if needed
- Check network latency if real robot is on different machine
- Monitor CPU usage of controller_manager process

## Customization

### Adding Velocity Control

To also mirror joint velocities, modify the controller to request velocity command interfaces:

```cpp
// In command_interface_configuration()
config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
```

### Filtering Joint States

Add joint filtering logic in the `update()` method if you need to:
- Smooth joint trajectories
- Filter out specific joints
- Apply joint limits
- Add safety checks

### Multiple Robot Support

For multiple robots, create separate controller instances with different configurations:

```yaml
passthrough_controller_robot1:
  ros__parameters:
    joints: [robot1_joint1, robot1_joint2, ...]
    real_robot_joint_states_topic: "/robot1/joint_states"

passthrough_controller_robot2:
  ros__parameters:
    joints: [robot2_joint1, robot2_joint2, ...]
    real_robot_joint_states_topic: "/robot2/joint_states"
```