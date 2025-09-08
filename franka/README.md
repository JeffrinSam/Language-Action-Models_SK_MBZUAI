# Franka Control - Single Panda Arm with Joystick

This workspace provides a complete setup for controlling a single Franka Panda arm using a joystick/gamepad controller with MoveIt Servo.

## Features

- **Real-time servo control** with joystick input
- **Collision avoidance** and singularity handling
- **Simulated robot** using ros2_control with mock components
- **RViz visualization** for monitoring robot state
- **Xbox controller support** (easily configurable for other controllers)

## Prerequisites

Make sure you have the following packages installed:

```bash
sudo apt install ros-humble-joy
sudo apt install ros-humble-moveit-servo
sudo apt install ros-humble-moveit-resources-panda-description
sudo apt install ros-humble-moveit-resources-panda-moveit-config
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-controller-manager
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-rviz2
sudo apt install ros-humble-tf2-ros
sudo apt install ros-humble-xacro
```

## Setup

1. **Build the workspace:**
   ```bash
   cd /home/kngn/franka
   colcon build
   source install/setup.bash
   ```

2. **Check joystick detection:**
   ```bash
   ls /dev/input/js*
   ```
   You should see `/dev/input/js0` (or similar) if your joystick is connected.

3. **Test joystick input:**
   ```bash
   ros2 run joy joy_node
   # In another terminal:
   ros2 topic echo /joy
   ```

## Usage

### Launch the Franka Control System

```bash
# Source the workspace
source /home/kngn/franka/install/setup.bash

# Launch the servo system with joystick control
ros2 launch franka_control franka_servo.launch.py
```

### Start Servo Control

In a new terminal:
```bash
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
```

### Joystick Controls

The system is configured for Xbox controller input:

- **Left stick (X/Y)**: Linear movement in X and Y directions
- **Right stick (Z)**: Angular rotation around Z axis
- **Triggers**: Linear movement in Z direction
- **Buttons**: Frame switching and control mode changes

### Alternative: Keyboard Control

If you don't have a joystick, you can use keyboard control:

```bash
# With the servo system running, in another terminal:
ros2 run moveit2_tutorials servo_keyboard_input
```

**Keyboard Controls:**
- **Arrow keys**: Cartesian movement
- **`.` and `;`**: Linear Z movement
- **W/E**: Switch between world and end-effector frames
- **1-7**: Joint jogging commands
- **R**: Reverse joint direction

## Configuration

### Joystick Device Path

If your joystick is not detected on `/dev/input/js0`, modify the launch file:
```python
# In franka_servo.launch.py, find the joy_node ComposableNode
# and add parameters:
parameters=[{"dev": "/dev/input/js1"}]  # Change to your device path
```

### Servo Parameters

Modify `config/panda_simulated_config.yaml` to adjust:
- **Scale factors**: Linear/rotational/joint velocity limits
- **Collision thresholds**: Proximity detection distances
- **Publishing rate**: Command update frequency

## Troubleshooting

1. **Joystick not detected:**
   - Check device permissions: `sudo chmod 666 /dev/input/js*`
   - Verify joystick is recognized: `lsusb` or `ls /dev/input/`

2. **Servo not responding:**
   - Ensure servo service is started: `ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}`
   - Check servo status: `ros2 topic echo /servo_node/status`

3. **Robot not moving:**
   - Verify controllers are loaded: `ros2 control list_controllers`
   - Check joint states: `ros2 topic echo /joint_states`

## File Structure

```
franka/
├── src/
│   └── franka_control/
│       ├── launch/
│       │   └── franka_servo.launch.py
│       ├── config/
│       │   ├── panda_simulated_config.yaml
│       │   ├── ros2_controllers.yaml
│       │   ├── initial_positions.yaml
│       │   ├── panda.srdf
│       │   ├── panda_arm.xacro
│       │   └── demo_rviz_config.rviz
│       ├── urdf/
│       │   ├── panda.urdf
│       │   ├── panda.urdf.xacro
│       │   ├── panda.ros2_control.xacro
│       │   └── panda_hand.ros2_control.xacro
│       ├── meshes/
│       │   ├── visual/
│       │   └── collision/
│       ├── CMakeLists.txt
│       └── package.xml
```

## Safety Notes

- This is a **simulation environment** - no real robot is controlled
- Servo includes built-in collision avoidance and singularity handling
- Always test thoroughly before using with real hardware
- Monitor robot behavior in RViz during operation

## Support

For issues or questions:
1. Check ROS2 logs: `ros2 log`
2. Verify topic connections: `ros2 topic list`
3. Monitor system status: `ros2 node list`
