#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Hayato Tsukada
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from pynput import mouse

class MousePub(Node):
    def __init__(self):
        super().__init__('mouse_publisher')
        self.pub = self.create_publisher(Point, 'mouse_pos', 10)
        self.listener = mouse.Listener(on_move=self.on_move)
        self.listener.start()

    def on_move(self, x, y):
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = MousePub()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.listener.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
