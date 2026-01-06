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

ros2 topic pub --once /mouse_pos geometry_msgs/msg/Point "{x: 10.0, y: 20.0, z: 0.0}"
sleep 10

cat /tmp/robosyshw2.log | grep 'Position: X=10'
