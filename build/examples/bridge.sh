#!/bin/sh
[ "$SSH_CONNECTION" ] || exit 1
set -- $SSH_CONNECTION
export ROS_MASTER_URI=http://$1:11311/
export ROS_HOSTNAME=$3
export ROS_NAMESPACE=/beam
export ROS_LOG_DIR=/var/log_permanent/ros
nohup /usr/bin/rosbeam-bridge >/dev/null &
