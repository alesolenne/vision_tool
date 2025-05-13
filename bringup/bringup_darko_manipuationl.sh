#!/bin/bash

SESSION_1="bringup"
SESSION_2="control"

# Percorso al setup ROS e workspace
ROS_SETUP="/opt/ros/noetic/setup.bash"  # Cambia con la tua distro se usi un'altra
WORKSPACE_SETUP="$HOME/darko_ws/devel/setup.bash"

# --- Sessione bringup DARKO con 3 finestre ---
if ! tmux has-session -t $SESSION_1 2>/dev/null; then
  # Crea la sessione darko
  tmux new-session -d -s $SESSION_1 -n "bring1" \
    "bash -c 'source $ROS_SETUP && source $WORKSPACE_SETUP && roslaunch darko_launch_system manipulation_complete.launch'"

  sleep 5.0

  # Crea la finestra manip2
  tmux new-window -t $SESSION_1:1 -n "bring2" \
    "bash -c 'source $ROS_SETUP && source $WORKSPACE_SETUP && roslaunch darko_robot_arm_control Moveitinterface.launch'"

  sleep 10.0

  # Crea la finestra manip3
  tmux new-window -t $SESSION_1:2 -n "bring3" \
    "bash -c 'source $ROS_SETUP && source $WORKSPACE_SETUP && roslaunch darko_launch_system rviz_stefano.launch'"

  sleep 2.0

  # Seleziona la finestra 'manip1'
  tmux select-window -t $SESSION_1:0
fi

# --- Sessione control DARKO con 2 finestre ---
if ! tmux has-session -t $SESSION_2 2>/dev/null; then
  # Crea la sessione panda
  tmux new-session -d -s $SESSION_2 -n "ctrl1" \
    "bash -c 'source $ROS_SETUP && source $WORKSPACE_SETUP && roslaunch panda_wrist_softhand_control launchControlServer.launch'"

  sleep 4.0

  # Crea la finestra ctrl2
  tmux new-window -t $SESSION_2:1 -n "ctrl2" \
    "bash -c 'source $ROS_SETUP && source $WORKSPACE_SETUP && roslaunch panda_wrist_softhand_control launchTaskServer.launch'"

  # Seleziona la finestra 'ctrl1'
  tmux select-window -t $SESSION_2:0
fi  

# --- Attacchiamo alla sessione di default ---
# In base alla tua preferenza, puoi scegliere la sessione da attaccare
tmux attach -t $SESSION_1
# tmux attach -t $SESSION_2  # Sostituisci con SESSION_2 se desideri attaccarti a control invece di bringup
