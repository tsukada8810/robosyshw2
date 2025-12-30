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

def main():
    rclpy.init()
    node = MousePub()

    root = tk.Tk()
    root.title("Mouse Tracker")
    root.geometry("500x500")

    def on_motion(event):
        msg = Point()
        msg.x = float(event.x)
        msg.y = float(event.y)
        node.pub.publish(msg)

    def on_close():
        root.destroy()
        node.destroy_node()
        rclpy.shutdown()

    root.bind('<Motion>', on_motion)
    root.protocol("WM_DELETE_WINDOW", on_close)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        on_close()

if __name__ == '__main__':
    main()
