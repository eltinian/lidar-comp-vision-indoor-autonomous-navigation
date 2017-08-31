#!/bin/bash

cd ~/shared/masters-bot

if tmux ls | grep ros
then
	tmux attach
else
	tmux new-session -ds ros -n ros-control "cd ~/shared/masters-bot; source devel/setup.bash; roscore"
	tmux split-window -h -s ros -n ros-control "cd ~/shared/masters-bot; source devel/setup.bash; rosrun autobot motor_control" 
	tmux split-window -v -s ros -n ros-control "cd ~/shared/masters-bot; source devel/setup.bash; rosrun teensyESC rosserial_python serial_node.py /dev/ttyACM0"

	tmux -2 attach-session -t ros
fi
