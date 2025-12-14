#!/bin/bash

echo "================================================"
echo "Yahboomcar Gazebo 仿真启动脚本"
echo "================================================"
echo ""

# 清理旧进程
echo "1. 清理旧的 Gazebo 进程..."
killall -9 gzserver gzclient 2>/dev/null
sleep 2

# Source workspace
echo "2. Source 工作空间..."
source install/setup.bash

# 设置关键环境变量
echo "3. 设置 Gazebo 环境变量..."
export GAZEBO_MODEL_PATH="${PWD}/src/robot_simulation/models:${PWD}/install/yahboomcar_description/share:${PWD}/src/robot_description/models:${GAZEBO_MODEL_PATH}"
export GAZEBO_RESOURCE_PATH="${PWD}/install/yahboomcar_description/share:${GAZEBO_RESOURCE_PATH}"

# 显示配置
echo ""
echo "环境变量配置:"
echo "  GAZEBO_MODEL_PATH 包含:"
echo "$GAZEBO_MODEL_PATH" | tr ':' '\n' | grep -v '^$' | sed 's/^/    - /'
echo ""

# 启动仿真
echo "4. 启动 Gazebo 仿真..."
echo "   等待 Gazebo 加载..."
echo ""

ros2 launch robot_simulation simulation.launch.py

echo ""
echo "================================================"
echo "仿真已退出"
echo "================================================"
