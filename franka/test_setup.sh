#!/bin/bash

echo "=== Franka Control Setup Test ==="
echo ""

# Check if workspace is sourced
if [ -z "$AMENT_PREFIX_PATH" ]; then
    echo "❌ Workspace not sourced. Please run: source install/setup.bash"
    exit 1
fi

echo "✅ Workspace sourced successfully"
echo ""

# Check if franka_control package is available
if ros2 pkg list | grep -q "franka_control"; then
    echo "✅ franka_control package found"
else
    echo "❌ franka_control package not found"
    exit 1
fi

# Check if launch file exists
if ros2 launch franka_control franka_servo.launch.py --show-args > /dev/null 2>&1; then
    echo "✅ Launch file syntax is correct"
else
    echo "❌ Launch file has syntax errors"
    exit 1
fi

# Check if joystick is available
if ls /dev/input/js* > /dev/null 2>&1; then
    echo "✅ Joystick device detected: $(ls /dev/input/js*)"
else
    echo "⚠️  No joystick detected. You can still use keyboard control."
fi

echo ""
echo "=== Setup Complete ==="
echo ""
echo "To launch the system:"
echo "  ros2 launch franka_control franka_servo.launch.py"
echo ""
echo "To start servo control:"
echo "  ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}"
echo ""
echo "For keyboard control (if no joystick):"
echo "  ros2 run moveit2_tutorials servo_keyboard_input"
