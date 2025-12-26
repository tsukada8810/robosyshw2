#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Hayato Tsukada
# SPDX-License-Identifier: BSD-3-Clause
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import tkinter as tk

class MousePub(Node):
    def __init__(self):
        super().__init__('mouse_publisher')
        self.pub = self.create_publisher(Point, 'mouse_pos', 10)
        self.root = tk.Tk()
        self.root.geometry("300x300")
        self.root.bind('<Motion>', self.cb)
        self.create_timer(0.01, self.update_gui)

    def cb(self, event):
        msg = Point()
        msg.x, msg.y = float(event.x), float(event.y)
        self.pub.publish(msg)

    def update_gui(self):
        self.root.update_idletasks()
        self.root.update()

def main():
    rclpy.init()
    try:
        rclpy.spin(MousePub())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
