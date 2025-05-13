#!/bin/bash

SESSION_1="camera"
SESSION_2="object_detection"
SESSION_3="tool_detection"

# Set up the user and path to the catkin workspace
user=$(whoami)
catkin_directory="$HOME/${user}/catkin_ws"

# Percorso al setup ROS e workspace
ROS_SETUP="/opt/ros/noetic/setup.bash"  # Cambia con la tua distro se usi un'altra
WORKSPACE_SETUP="$HOME/catkin_ws/devel/setup.bash"

# --- Sessione bringup DARKO con 3 finestre ---
if ! tmux has-session -t $SESSION_1 2>/dev/null; then
  # Crea la sessione per camera
  tmux new-session -d -s $SESSION_1 -n "camera" \
    "bash -c 'source $ROS_SETUP && source $WORKSPACE_SETUP && roslaunch vision_tool vision_camera.launch'"

  sleep 1.0

  # Crea la finestra per i nodi di ridimensionamento con due pannelli verticali
  tmux new-window -t $SESSION_1:1 -n "resize" \
    "bash -c 'source $ROS_SETUP && source $WORKSPACE_SETUP && roslaunch ros_imresize imresize_color.launch'"

  # Splitta verticalmente e lancia il secondo comando
  tmux split-window -h -t $SESSION_1:1 \
    "bash -c 'source $ROS_SETUP && source $WORKSPACE_SETUP && roslaunch ros_imresize imresize_depth.launch'"

  # Attivare il focus sul primo pannello
  tmux select-pane -t $SESSION_1:1.0

  sleep 2.0

  # Seleziona la finestra 'camera'
  tmux select-window -t $SESSION_1:0
fi

# --- Sessione control DARKO con 2 finestre ---
if ! tmux has-session -t $SESSION_2 2>/dev/null; then
  # Crea la sessione megapose
  tmux new-session -d -s $SESSION_2 -n "server"

  # Sessione per il server
  tmux send-keys -t $SESSION_2:0 "user=${user} && . /home/${user}/catkin_ws/src/visp_megapose/bringup/megapose_env.sh" C-m
  sleep 1.0

  tmux send-keys -t $SESSION_2:0 "roslaunch visp_megapose megapose_server.launch" C-m

  sleep 2.0

  # Sessione per il client
  tmux new-window -t $SESSION_2:1 -n "client" \
    "bash -c 'source $ROS_SETUP && source $WORKSPACE_SETUP && roslaunch visp_megapose megapose_client_command.launch'"
  
  sleep 1.0

  tmux split-window -h -t $SESSION_2:1 \
    "bash -c 'source $ROS_SETUP && source $WORKSPACE_SETUP && roslaunch visp_megapose command.launch'"

  # Attivare il focus sul primo pannello
  tmux select-pane -t $SESSION_2:1.0

  # Seleziona la finestra 'server'
  tmux select-window -t $SESSION_2:0
fi  

if ! tmux has-session -t $SESSION_3 2>/dev/null; then
  # Crea la sessione megapose
  tmux new-session -d -s $SESSION_3 -n "tool"

  # Sessione per la visione del tool
  tmux send-keys -t $SESSION_3:0 "source $ROS_SETUP && source $WORKSPACE_SETUP && roslaunch vision_tool vision_tag.launch" C-m

  tmux split-window -h -t $SESSION_3:0 \
    "bash -c 'source $ROS_SETUP && source $WORKSPACE_SETUP && roslaunch vision_tool vision_tf.launch'"

  # Attivare il focus sul primo pannello
  tmux select-pane -t $SESSION_3:0.0

  # Sessione per arduino
  tmux new-window -t $SESSION_3:1 -n "arduino" \
    "bash -c 'source $ROS_SETUP && source $WORKSPACE_SETUP && roslaunch vision_tool run_arduino.launch'"

  sleep 3.0

  # Sessione per il servoing con il tool
  tmux new-window -t $SESSION_3:2 -n "servoing" \
    "bash -c 'source $ROS_SETUP && source $WORKSPACE_SETUP && roslaunch servoing_pkg servoing.launch'"

  # Seleziona la finestra 'server'
  tmux select-window -t $SESSION_3:0
fi 

# --- Attacchiamo alla sessione di default ---
# In base alla tua preferenza, puoi scegliere la sessione da attaccare
tmux attach -t $SESSION_1
# tmux attach -t $SESSION_2  # Sostituisci con SESSION_2 se desideri attaccarti a control invece di bringup
