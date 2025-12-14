#!/bin/bash

echo "================================================"
echo "Yahboomcar Gazebo Simulation Startup Script"
echo "================================================"
echo ""

# Clean up old processes
echo "1. Cleaning up old Gazebo processes..."
killall -9 gzserver gzclient 2>/dev/null
sleep 2

# Source workspace
echo "2. Sourcing workspace..."
source install/setup.bash

# Set critical environment variables
echo "3. Setting Gazebo environment variables..."
export GAZEBO_MODEL_PATH="${PWD}/src/robot_simulation/models:${PWD}/install/yahboomcar_description/share:${PWD}/src/robot_description/models:${GAZEBO_MODEL_PATH}"
export GAZEBO_RESOURCE_PATH="${PWD}/install/yahboomcar_description/share:${GAZEBO_RESOURCE_PATH}"

# Display configuration
echo ""
echo "Environment variable configuration:"
echo "  GAZEBO_MODEL_PATH includes:"
echo "$GAZEBO_MODEL_PATH" | tr ':' '\n' | grep -v '^$' | sed 's/^/    - /'
echo ""

# Launch simulation
echo "4. Launching Gazebo simulation..."
echo "   Waiting for Gazebo to load..."
echo ""

ros2 launch robot_simulation simulation.launch.py

echo ""
echo "================================================"
echo "Simulation exited"
echo "================================================"
