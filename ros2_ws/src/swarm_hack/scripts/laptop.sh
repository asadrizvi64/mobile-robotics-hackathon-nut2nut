#!/bin/bash
# RUN ON THE LAPTOP (WSL):  bash ~/ws/src/swarm_hack/scripts/laptop.sh [ROBOT_IP]
# tmux session "lap":  bridge | mapper (YOLO) | swarm (fake drones)
ROBOT_IP=${1:-172.20.10.3}
BR=$(find /opt/ros/humble -type f -perm -u+x -name 'zenoh*bridge*dds*' 2>/dev/null | head -1)
[ -z "$BR" ] && { echo "bridge not installed: sudo apt install -y ros-humble-zenoh-bridge-dds"; exit 1; }
CFG=~/ws/install/swarm_hack/share/swarm_hack/config/swarm.yaml
tmux kill-session -t lap 2>/dev/null
pkill -f "$BR"; pkill -f object_mapper; pkill -f swarm_coordinator; pkill -f crazyflie_bridge; sleep 1
tmux new-session -d -s lap -n bridge
tmux send-keys -t lap:bridge "$BR -d \$ROS_DOMAIN_ID -e tcp/$ROBOT_IP:7447 --no-multicast-scouting" C-m
tmux new-window -t lap -n mapper
tmux send-keys -t lap:mapper "sleep 5; ros2 run swarm_hack object_mapper --ros-args --params-file $CFG" C-m
if [ "${SWARM_SIM:-0}" = "1" ]; then   # old fake-drone swarm; OFF by default (clashes with cf_fleet TF)
  tmux new-window -t lap -n swarm
  tmux send-keys -t lap:swarm "sleep 5; ros2 launch swarm_hack swarm.launch.py sim:=true mapper:=false" C-m
fi
tmux new-window -t lap -n cmd
echo "started tmux 'lap' (bridge -> $ROBOT_IP, mapper)."
echo "Open it: tmux attach -t lap     Switch: Ctrl+b then 0/1/2/3     Leave: Ctrl+b then d"
