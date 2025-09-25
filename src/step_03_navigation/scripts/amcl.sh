#!/bin/bash
set -e  # dừng ngay nếu có lỗi

# Build workspace
colcon build
source install/setup.bash

/home/yi/agv_ws/src/step_03_navigation/launch/covariance.py &

# Chạy spawn robot + mapping song song
ros2 launch step_03_navigation spawn_agv.launch.py &
PID1=$!

ros2 launch step_03_navigation localization.launch.py &
PID2=$!

# Khi ấn Ctrl+C thì kill cả 2
trap "echo 'Stopping...'; kill -INT $PID1 $PID2" INT

# Giữ script chạy cho tới khi process chết
wait
