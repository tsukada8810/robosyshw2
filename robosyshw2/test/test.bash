#!/bin/bash
# SPDX-FileCopyrightText: 2025 Hayato Tsukada
# SPDX-License-Identifier: BSD-3-Clause

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
colcon build
source install/setup.bash

ros2 run robosyshw2 distance > /tmp/mypkg.log 2>&1 &
PID=$!
sleep 5

send_data(){
    ros2 topic pub --once /mouse_pos geometry_msgs/msg/Point "{x: $1, y:$2, z: 0.0}" > /dev/null 2>&1
}

send_data 10.0 20.0
check_log "Position: X=10, Y=20"

send_data 0.0 0.0
check_log "Position: X=0, Y=0"

send_data -50.0 -30.0
check_log "Position: X=-50, Y=-30"

send_data 123.6 67.2
check_log "Position: X=124, Y=67"

kill $NODE_PID
exit 0
