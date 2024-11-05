#!/bin/bash

# Check if the policy file was provided as an argument
if [ -z "$1" ]; then
    echo "Usage: $0 <policy-file>"
    exit 1
fi

# Define the variables
POLICY_FILE=$1
REMOTE_USER="pi"
REMOTE_HOST="pupper.local"
REMOTE_PATH="/home/pi/ros2_ws/src/neural_controller/launch/policy.json"
LAUNCH_COMMAND="ros2 launch /home/pi/ros2_ws/src/neural_controller/launch/launch.py"
TMUX_SESSION="ros2_launch_session"

# Step 1: Copy the policy file to the remote location using scp
echo "Copying policy to remote..."
scp "$POLICY_FILE" "$REMOTE_USER@$REMOTE_HOST:$REMOTE_PATH"

if [ $? -ne 0 ]; then
    echo "Error copying policy file"
    exit 1
fi

# Step 2: Run the command on the remote machine via ssh in a tmux session
echo "Running ROS2 launch on remote in tmux session..."
ssh "$REMOTE_USER@$REMOTE_HOST" "
    if ! tmux has-session -t $TMUX_SESSION 2>/dev/null; then
        tmux new-session -d -s $TMUX_SESSION
    fi
    tmux send-keys -t $TMUX_SESSION '$LAUNCH_COMMAND' C-m
"

if [ $? -ne 0 ]; then
    echo "Error running ROS2 launch command in tmux"
    exit 1
fi

echo "Done!"

