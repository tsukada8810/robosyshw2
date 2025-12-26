#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Hayato Tsukada
# SPDX-License-Identifier: BSD-3-Clause
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Point

class DistanceSubscriber(Node):
    def __init__(self):
        super().__init__('distance_subscriber')
        self.sub = self.create_subscription(Point, 'mouse_pos', self.cb, 10)
        self.x = None
        self.y = None
        self.sum = 0.0

    def cb(self, msg):
        if self.x is None:
            self.x, self.y = msg.x, msg.y
            return

        # 距離計算 (math.hypotで短縮)
        self.sum += math.hypot(msg.x - self.x, msg.y - self.y)
        self.x, self.y = msg.x, msg.y

        # シンプルに出力
        self.get_logger().info(f'Distance: {self.sum * 0.0264:.2f} cm')

def main():
    rclpy.init()
    try:
        rclpy.spin(DistanceSubscriber())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
