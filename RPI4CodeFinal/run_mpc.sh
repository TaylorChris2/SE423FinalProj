#!/usr/bin/env bash

# Kill old session if it exists
tmux kill-session -t "$SESSION" 2>/dev/null
tmux set-option -g destroy-unattached on

tmux new-session -d -s demo

tmux send-keys -t demo "cd serial_COMandOpti && make && sudo ./shmdelete.sh || true && sudo ./serial_COMandOpti" C-m

tmux split-window -h -t demo
tmux send-keys -t demo "cd mpc && make && sudo ./ladar_server" C-m

tmux split-window -v -t demo
tmux send-keys -t demo "cd mpc && source ~/mpc-env/bin/activate && sudo ~/mpc-env/bin/python3 mpc.py" C-m

tmux attach -t demo
