#!/bin/bash
# ONE COMMAND for a mission run, on the LAPTOP (WSL). No fixed waits - every step skips what is already done.
#     bash /mnt/c/Users/HP/Downloads/go.sh                  # visit every bottle within 1.5 m of mark A
#     bash /mnt/c/Users/HP/Downloads/go.sh -p max_targets:=2
#     bash /mnt/c/Users/HP/Downloads/go.sh fix              # mission says WAITING? restart both bridge ends first
# Ctrl+C at any time stops the robot.
ROBOT=turtle@172.20.10.3
DL=/mnt/c/Users/HP/Downloads
SSHO="-o ControlMaster=auto -o ControlPath=/tmp/go-ssh-%r@%h -o ControlPersist=600 -o ConnectTimeout=6"
CFG=$HOME/ws/install/swarm_hack/share/swarm_hack/config/swarm.yaml
BR=$(find /opt/ros/humble -type f -perm -u+x -name 'zenoh*bridge*dds*' 2>/dev/null | head -1)
ok()   { echo -e "   \e[32mOK\e[0m  $*"; }
fail() { echo -e "\n\e[31mSTOP: $*\e[0m\n"; exit 1; }
source /opt/ros/humble/setup.bash
[ -f "$HOME/ws/install/setup.bash" ] && source "$HOME/ws/install/setup.bash"

laptop_bridge() {     # (re)start the laptop end of the Wi-Fi bridge
  pkill -f "$BR"; tmux kill-window -t lap:bridge 2>/dev/null
  tmux new-window -d -t lap -n bridge
  tmux send-keys -t lap:bridge "$BR -d \$ROS_DOMAIN_ID -e tcp/172.20.10.3:7447 --no-multicast-scouting" C-m
}

# --- robot
ssh $SSHO -O exit $ROBOT >/dev/null 2>&1; rm -f /tmp/go-ssh-* 2>/dev/null
ping -c 1 -W 2 172.20.10.3 >/dev/null || fail "robot not reachable (hotspot dropped?)"
ssh $SSHO -fN $ROBOT || fail "ssh login failed"
if ! ssh $SSHO $ROBOT "tmux has-session -t robot 2>/dev/null"; then
  echo -e "   \e[33mrobot stack is DOWN (rebooted).\e[0m"
  read -p "   Robot on MARK A, then Enter... " _
  scp $SSHO -q "$DL/robot_start.sh" "$DL/robot_extras.sh" $ROBOT:~/ || fail "scp failed"
  ssh $SSHO $ROBOT "bash ~/robot_start.sh" || fail "robot_start.sh failed"
  laptop_bridge                                   # fresh robot bridge -> fresh laptop end too
fi
ok "robot stack running"

# --- laptop code: rebuild only if the zip changed
tmux has-session -t lap 2>/dev/null || { bash ~/ws/src/swarm_hack/scripts/laptop.sh 172.20.10.3 >/dev/null; tmux kill-window -t lap:swarm 2>/dev/null; }
SUM=$(md5sum "$DL/swarm_hack.zip" | cut -d' ' -f1)
if [ "$SUM" != "$(cat ~/.go_zip_md5 2>/dev/null)" ]; then
  MAP_FILES="$HOME/ws/src/swarm_hack/swarm_hack/object_mapper.py $HOME/ws/src/swarm_hack/config/swarm.yaml"
  OLD=$(cat $MAP_FILES 2>/dev/null | md5sum)
  (cd ~/ws/src && unzip -o -q "$DL/swarm_hack.zip") || fail "unzip failed"
  (cd ~/ws && colcon build --symlink-install --packages-select swarm_hack >/tmp/go_build.log 2>&1) || { tail -20 /tmp/go_build.log; fail "build failed"; }
  echo "$SUM" > ~/.go_zip_md5
  if [ "$OLD" != "$(cat $MAP_FILES | md5sum)" ]; then
    pkill -f "lib/swarm_hack/object_mapper"        # mapper code changed -> must restart (YOLO reload ~30 s)
    ok "laptop code updated (mapper changed -> restarting it)"
  else
    ok "laptop code updated (mapper unchanged, keeps running)"
  fi
fi

# --- bridges / mapper: only (re)start what is not running
[ "$1" = fix ] && { shift; laptop_bridge; ssh $SSHO $ROBOT "bash -ic 'bash ~/robot_extras.sh'" >/dev/null 2>&1; ok "both bridge ends restarted"; }
if ! pgrep -f "lib/swarm_hack/object_mapper" >/dev/null; then
  tmux kill-window -t lap:mapper 2>/dev/null
  tmux new-window -d -t lap -n mapper
  tmux send-keys -t lap:mapper "ros2 run swarm_hack object_mapper --ros-args --params-file $CFG" C-m
  ok "mapper started (YOLO loads in the background; the robot holds still until it is ready)"
else
  ok "mapper running"
fi

# --- mission
scp $SSHO -q "$DL/target_mission.py" $ROBOT:~/ || fail "scp failed"
echo -e "\e[1m== RUN $(ssh $SSHO $ROBOT "grep -o 'target_mission v[0-9]*' ~/target_mission.py | head -1") - Ctrl+C stops the robot.\e[0m"
echo "   (if it keeps saying WAITING: Ctrl+C, then  bash $DL/go.sh fix )"
RA=""; [ $# -gt 0 ] && RA="--ros-args $*"
ssh $SSHO -t $ROBOT "bash -ic 'python3 ~/target_mission.py $RA'"
