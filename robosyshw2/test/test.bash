#!/bin/bash
# SPDX-FileCopyrightText: 2025 Hayato Tsukada
# SPDX-License-Identifier: BSD-3-Clause

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
colcon build
source install/setup.bash

timeout 10 ros2 run robosyshw2 distance > /tmp/robosyshw2.log &
sleep 2

ros2 topic pub --once /mouse_pos geometry_msgs/msg/Point "{x: 10.0, y: 20.0, z: 0.0}"
sleep 10

cat /tmp/robosyshw2.log | grep 'Position: X=10'
