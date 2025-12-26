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
        self.subscription = self.create_subscription(
            Point,
            'mouse_pos',
            self.listener_callback,
            10)
        self.prev_x = None
        self.prev_y = None
        self.total_distance = 0.0

    def listener_callback(self, msg):
        current_x = msg.x
        current_y = msg.y

        if self.prev_x is None:
            self.prev_x = current_x
            self.prev_y = current_y
            self.get_logger().info('Measurement started! Move your mouse.')
            return

        dx = current_x - self.prev_x
        dy = current_y - self.prev_y
        dist = math.sqrt(dx**2 + dy**2)
        self.total_distance += dist

        # cm換算（目安）
        dist_cm = self.total_distance * 0.0264
        self.get_logger().info(f'Total Distance: {dist_cm:.2f} cm ({int(self.total_distance)} px)')

        self.prev_x = current_x
        self.prev_y = current_y

def main(args=None):
    rclpy.init(args=args)
    node = DistanceSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
