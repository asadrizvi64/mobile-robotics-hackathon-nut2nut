#!/bin/bash
# Run ON THE ROBOT:  bash ~/start_robot.sh
# Starts zenoh router + bringup + SLAM inside tmux, so they survive Wi-Fi / SSH drops.
tmux kill-session -t robot 2>/dev/null
pkill -f rmw_zenohd; pkill -f turtlebot3_bringup; pkill -f turtlebot3_ros; pkill -f slam_toolbox
sleep 2
tmux new-session -d -s robot -n router
tmux send-keys -t robot:router "ros2 run rmw_zenoh_cpp rmw_zenohd" C-m
tmux new-window -t robot -n bringup
tmux send-keys -t robot:bringup "sleep 3; ros2 launch turtlebot3_bringup robot.launch.py" C-m
tmux new-window -t robot -n slam
tmux send-keys -t robot:slam "sleep 12; ros2 launch slam_toolbox online_async_launch.py slam_params_file:=\$HOME/slam_tb3.yaml" C-m
echo "Robot started (router, bringup, slam). Check in ~15 s:  ros2 node list"
echo "Watch: tmux attach -t robot   (Ctrl+b n = next window, Ctrl+b d = leave)"
