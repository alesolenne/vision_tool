#!/bin/bash

# Set up the user and path to the catkin workspace
user=$(whoami)
catkin_directory="/home/${user}/catkin_ws"
cd ${catkin_directory}
source devel/setup.bash

tmux new-session -s tool -n alessandro -d

tmux split-window -h -t 0
tmux select-layout even-horizontal
tmux split-window -v -t 1
tmux split-window -v -t 2

#Launch the camera node driver
tmux send-keys -t 0 "roslaunch vision_tool vision_camera.launch" C-m
sleep 1.0

#Launch the image resizer node
tmux send-keys -t 1 "roslaunch vision_tool vision_tag.launch" C-m
sleep 1.0

#Launch the MegaPose server
tmux send-keys -t 2 "roslaunch vision_tool vision_tf.launch" C-m
sleep 1.0

tmux attach-session -t tool