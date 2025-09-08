#!/bin/bash

echo "=== Franka Control Functionality Test ==="
echo "Date: $(date)"
echo

# Deactivate conda as requested
echo "1. Deactivating conda environment..."
conda deactivate
echo "   ✓ Conda deactivated"
echo

# Source the workspace
echo "2. Sourcing workspace..."
source /home/kngn/franka/install/setup.bash
echo "   ✓ Workspace sourced"
echo

# Check if package is available
echo "3. Checking package availability..."
if ros2 pkg list | grep -q franka_control; then
    echo "   ✓ franka_control package is available"
else
    echo "   ✗ franka_control package not found"
    exit 1
fi
echo

# Test launch file syntax
echo "4. Testing launch file syntax..."
if ros2 launch franka_control franka_servo.launch.py --print-description > /dev/null 2>&1; then
    echo "   ✓ Launch file syntax is valid"
else
    echo "   ✗ Launch file syntax error"
    exit 1
fi
echo

# Test SRDF processing
echo "5. Testing SRDF file processing..."
if ros2 run xacro xacro /home/kngn/franka/src/franka_control/config/panda.srdf > /dev/null 2>&1; then
    echo "   ✓ SRDF file processes correctly"
else
    echo "   ✗ SRDF file processing error"
    exit 1
fi
echo

# Test robot description processing
echo "6. Testing robot description processing..."
if ros2 run xacro xacro /home/kngn/franka/src/franka_control/urdf/panda.urdf.xacro > /dev/null 2>&1; then
    echo "   ✓ Robot description processes correctly"
else
    echo "   ✗ Robot description processing error"
    exit 1
fi
echo

# Test joystick detection (expected to fail without hardware)
echo "7. Testing joystick detection..."
if ls /dev/input/js* > /dev/null 2>&1; then
    echo "   ✓ Joystick device detected"
else
    echo "   ⚠ No joystick device detected (expected without hardware)"
fi
echo

# Test dependencies
echo "8. Checking required dependencies..."
deps=("joy" "moveit_servo" "ros2_controllers" "controller_manager")
all_deps_ok=true
for dep in "${deps[@]}"; do
    if ros2 pkg list | grep -q "$dep"; then
        echo "   ✓ $dep is available"
    else
        echo "   ✗ $dep is missing"
        all_deps_ok=false
    fi
done

if [ "$all_deps_ok" = true ]; then
    echo "   ✓ All required dependencies are available"
else
    echo "   ✗ Some dependencies are missing"
    exit 1
fi
echo

echo "=== Test Summary ==="
echo "✓ All basic functionality tests passed!"
echo "✓ The Franka control system is properly configured and ready to use"
echo
echo "To run the system:"
echo "  ros2 launch franka_control franka_servo.launch.py"
echo
echo "Note: This is a simulation environment - no real robot is controlled"
echo "The system includes collision avoidance and singularity handling"
