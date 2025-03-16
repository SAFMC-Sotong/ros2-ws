#!/usr/bin/bash

echo -e "\033[33mStarting UDP MAVROS bridge...\033[0m"
ros2 run udp_mavros_bridge udp_mavros_bridge_2 --ros-args -p udp_port:=14551
