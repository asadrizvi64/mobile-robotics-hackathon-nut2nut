#!/bin/bash
# Full robot restart (bringup + SLAM + camera + bridge). No fixed waits: everything starts at once,
# the script returns as soon as /scan, /odom and the camera are up.
# PUT THE ROBOT ON MARK A FIRST - SLAM's (0,0) is where it starts.
# From the LAPTOP:   ssh turtle@172.20.10.3 "bash ~/robot_start.sh"
source /opt/ros/humble/setup.bash
[ -f "$HOME/turtlebot3_ws/install/setup.bash" ] && source "$HOME/turtlebot3_ws/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
T0=$(date +%s)
echo "== uptime $(uptime -p); power record $(vcgencmd get_throttled 2>/dev/null | cut -d= -f2) (0x0 = fine)"
tmux kill-server 2>/dev/null
pkill -f hlds_laser; pkill -f turtlebot3_ros; pkill -f slam_toolbox; pkill -f camera_node; pkill -f zenoh_bridge
pkill -f target_mission; pkill -f straight_avoid
tmux new-session -d -s robot -n bringup "bash -ic 'ros2 launch turtlebot3_bringup robot.launch.py'"
tmux new-window -t robot -n slam "bash -ic 'ros2 launch slam_toolbox online_async_launch.py slam_params_file:=\$HOME/slam_tb3.yaml'"
bash ~/robot_extras.sh >/dev/null
for i in $(seq 1 60); do                    # ready as soon as the topics exist (no fixed sleep)
  L=$(ros2 topic list 2>/dev/null)
  echo "$L" | grep -qx /scan && echo "$L" | grep -qx /odom && echo "$L" | grep -q image_raw/compressed && break
done
echo "== robot ready after $(( $(date +%s) - T0 )) s: $(tmux list-windows -t robot -F '#W' | tr '\n' ' ')"
