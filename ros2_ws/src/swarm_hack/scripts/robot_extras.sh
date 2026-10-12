#!/bin/bash
# RUN ON THE ROBOT:  bash ~/robot_extras.sh
# Adds two windows to tmux session "robot": camera + bridge (for the WSL laptop).
# Does NOT touch bringup / SLAM (teammates' Fast DDS setup stays as it is).
BR=$(find /opt/ros/humble -type f -perm -u+x -name 'zenoh*bridge*dds*' 2>/dev/null | head -1)
[ -z "$BR" ] && { echo "bridge not installed: sudo apt install -y ros-humble-zenoh-bridge-dds"; exit 1; }
ros2 pkg prefix camera_ros >/dev/null 2>&1 || { echo "camera driver missing: sudo apt install -y ros-humble-camera-ros ros-humble-image-transport-plugins"; exit 1; }
tmux has-session -t robot 2>/dev/null || tmux new-session -d -s robot -n shell
tmux kill-window -t robot:camera 2>/dev/null; tmux kill-window -t robot:bridge 2>/dev/null
pkill -f v4l2_camera_node; pkill -f camera_ros; pkill -f "$BR"; sleep 1
tmux new-window -t robot -n camera
# Pi camera on Ubuntu 22.04 = libcamera (unicam) -> camera_ros, not v4l2_camera
# 15 fps + JPEG 60: ~1/6 of the Wi-Fi load of 30 fps/JPEG 95 (iPhone hotspot was saturated; YOLO uses 5 fps anyway)
tmux send-keys -t robot:camera "ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480 -p format:=RGB888 -p jpeg_quality:=60 -p 'FrameDurationLimits:=[66666,66666]'" C-m
tmux new-window -t robot -n bridge
# listen for the laptop; forward only what the laptop needs (bridge 0.5 has no rate limiter)
tmux send-keys -t robot:bridge "$BR -d \$ROS_DOMAIN_ID -l tcp/0.0.0.0:7447 -a 'scan|tf|map|camera_info|compressed|cmd_vel|odom|joint_states|robot_description|goal_pose|plan|initialpose|objects'" C-m
echo "camera + bridge started in tmux 'robot'.  Watch: tmux attach -t robot"
