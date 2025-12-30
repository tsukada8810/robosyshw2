#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Hayato Tsukada
# SPDX-License-Identifier: BSD-3-Clause
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class DistanceSubscriber(Node):
    def __init__(self):
        super().__init__('distance_subscriber')
        self.sub = self.create_subscription(Point, 'mouse_pos', self.cb, 10)

    def cb(self, msg):
        # 距離計算をやめて、座標(X, Y)をそのまま表示する
        self.get_logger().info(f'Position: X={msg.x:.0f}, Y={msg.y:.0f}')

def main():
    rclpy.init()
    try:
        rclpy.spin(DistanceSubscriber())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
