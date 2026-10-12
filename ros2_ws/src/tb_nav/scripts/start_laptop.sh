#!/bin/bash
# Run ON THE LAPTOP (WSL):  bash ~/ws/src/tb_nav/scripts/start_laptop.sh [ROBOT_IP]
# Starts the laptop zenoh router + Nav2 inside tmux. RViz and teleop stay in their own windows.
ROBOT_IP=${1:-172.20.10.3}
command -v tmux >/dev/null || { echo "install tmux first: sudo apt install -y tmux"; exit 1; }
tmux kill-session -t laptop 2>/dev/null
pkill -f rmw_zenohd; pkill -f tb_nav.launch; pkill -f nav2_; pkill -f nav_manager
sleep 2
tmux new-session -d -s laptop -n router
tmux send-keys -t laptop:router "export ZENOH_CONFIG_OVERRIDE='connect/endpoints=[\"tcp/$ROBOT_IP:7447\"]'; ros2 run rmw_zenoh_cpp rmw_zenohd" C-m
tmux new-window -t laptop -n nav2
tmux send-keys -t laptop:nav2 "sleep 4; ros2 launch tb_nav tb_nav.launch.py mode:=slam run_slam:=false" C-m
echo "Laptop started (router -> $ROBOT_IP, Nav2)."
echo "Watch: tmux attach -t laptop   (Ctrl+b n = next window, Ctrl+b d = leave)"
