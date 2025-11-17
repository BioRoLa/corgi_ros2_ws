#!/bin/bash
# Test script to verify corgi_utils ROS2 conversion and functionality

set -e  # Exit on any error

echo "======================================"
echo "Testing corgi_utils ROS2 Package"
echo "======================================"
echo ""

# Navigate to workspace
cd ~/corgi_ws/corgi_ros_ws

echo "1. Building corgi_utils..."
colcon build --packages-select corgi_utils --cmake-args -DLOCAL_PACKAGE_PATH=${HOME}/corgi_ws/install --allow-overriding corgi_utils
echo "✅ Build successful"
echo ""

echo "2. Sourcing workspace..."
source install/setup.bash
echo "✅ Workspace sourced"
echo ""

echo "3. Checking if package is registered..."
if ros2 pkg list | grep -q "corgi_utils"; then
    echo "✅ corgi_utils is registered in ROS2"
else
    echo "❌ corgi_utils is NOT registered in ROS2"
    exit 1
fi
echo ""

echo "4. Checking installed files..."
if [ -f "install/corgi_utils/lib/libcorgi_utils.a" ]; then
    echo "✅ Library file exists"
else
    echo "❌ Library file NOT found"
    exit 1
fi

if [ -f "install/corgi_utils/include/leg_model.hpp" ]; then
    echo "✅ Header files exist"
else
    echo "❌ Header files NOT found"
    exit 1
fi

if [ -f "install/corgi_utils/lib/corgi_utils/leg_model" ]; then
    echo "✅ Executable exists"
else
    echo "❌ Executable NOT found"
    exit 1
fi
echo ""

echo "5. Running leg_model executable..."
if timeout 5 ros2 run corgi_utils leg_model > /tmp/corgi_utils_test.log 2>&1; then
    if grep -q "Forward kinematics example" /tmp/corgi_utils_test.log && \
       grep -q "Inverse kinematics example" /tmp/corgi_utils_test.log && \
       grep -q "Move example" /tmp/corgi_utils_test.log; then
        echo "✅ Executable runs successfully with expected output"
    else
        echo "❌ Executable output is incomplete"
        cat /tmp/corgi_utils_test.log
        exit 1
    fi
else
    echo "❌ Executable failed to run"
    cat /tmp/corgi_utils_test.log
    exit 1
fi
echo ""

echo "6. Testing integration with dependent package..."
if colcon build --packages-select corgi_walk --cmake-args -DLOCAL_PACKAGE_PATH=${HOME}/corgi_ws/install --allow-overriding corgi_walk > /tmp/corgi_walk_build.log 2>&1; then
    echo "✅ Dependent package (corgi_walk) builds successfully"
else
    echo "❌ Dependent package failed to build"
    tail -20 /tmp/corgi_walk_build.log
    exit 1
fi
echo ""

echo "======================================"
echo "✅ All tests passed!"
echo "======================================"
echo ""
echo "corgi_utils is fully converted to ROS2 and operational."
echo ""
echo "Summary:"
echo "  - Package builds successfully"
echo "  - Library and headers installed correctly"
echo "  - Executable runs without errors"
echo "  - Integration with dependent packages works"
echo "  - Ready for production use"
