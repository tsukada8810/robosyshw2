#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Hayato Tsukada
# SPDX-License-Identifier: BSD-3-Clause
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import tkinter as tk

class MousePublisher(Node):
    def __init__(self):
        super().__init__('mouse_publisher')
        self.publisher_ = self.create_publisher(Point, 'mouse_pos', 10)
        
        # GUIウィンドウの設定
        self.root = tk.Tk()
        self.root.title("Mouse Tracker")
        self.root.geometry("300x300")
        
        # マウスが動いた時のイベントを登録
        self.root.bind('<Motion>', self.on_motion)
        
        # ROSの通信を維持しながらGUIを更新するためのタイマー
        self.timer = self.create_timer(0.01, self.gui_loop)

    def on_motion(self, event):
        # ウィンドウ内でのマウス座標を取得して送信
        msg = Point()
        msg.x = float(event.x)
        msg.y = float(event.y)
        msg.z = 0.0
        self.publisher_.publish(msg)

    def gui_loop(self):
        # tkinterの描画更新
        self.root.update_idletasks()
        self.root.update()

def main(args=None):
    rclpy.init(args=args)
    node = MousePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
