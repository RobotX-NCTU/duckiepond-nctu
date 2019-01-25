#!/usr/bin/env bash
echo "Setting ROS_IP..."
if [ $# -gt 0 ]; then
	export ROS_IP=$1
    echo "ROS_IP set to $ROS_IP"
else
	echo "Please provide your ip."
fi
