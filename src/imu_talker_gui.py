#!/usr/bin/env python3
import tkinter as tk
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped
import threading

class SliderPublisher(Node):
    def __init__(self):
        super().__init__('talker_quaternion')
        self.pub = self.create_publisher(QuaternionStamped, 'orientation', 10)

    def publish_rpy(self, roll_deg, pitch_deg, yaw_deg):
        r = math.radians(roll_deg)
        p = math.radians(pitch_deg)
        y = math.radians(yaw_deg)

        cr, sr = math.cos(r/2), math.sin(r/2)
        cp, sp = math.cos(p/2), math.sin(p/2)
        cy, sy = math.cos(y/2), math.sin(y/2)

        msg = QuaternionStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.quaternion.w = cr*cp*cy + sr*sp*sy
        msg.quaternion.x = sr*cp*cy - cr*sp*sy
        msg.quaternion.y = cr*sp*cy + sr*cp*sy
        msg.quaternion.z = cr*cp*sy - sr*sp*cy

        self.pub.publish(msg)
        self.get_logger().info(
            f'RPY: [{roll_deg:.1f}, {pitch_deg:.1f}, {yaw_deg:.1f}] → '
            f'q: [w={msg.quaternion.w:.4f}, x={msg.quaternion.x:.4f}, '
            f'y={msg.quaternion.y:.4f}, z={msg.quaternion.z:.4f}]'
        )

def main():
    rclpy.init()
    node = SliderPublisher()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    root = tk.Tk()
    root.title("Quaternion Publisher")
    root.resizable(False, False)

    sliders = {}
    for i, axis in enumerate(['Roll', 'Pitch', 'Yaw']):
        tk.Label(root, text=axis, width=6, anchor='w').grid(row=i, column=0, padx=10, pady=8)
        var = tk.DoubleVar(value=0.0)
        tk.Scale(root, from_=-180, to=180, resolution=1,
                 orient=tk.HORIZONTAL, length=300, variable=var).grid(row=i, column=1, padx=10)
        val_label = tk.Label(root, text="0°", width=5)
        val_label.grid(row=i, column=2, padx=5)
        sliders[axis] = (var, val_label)

    def on_slider_change():
        r = sliders['Roll'][0].get()
        p = sliders['Pitch'][0].get()
        y = sliders['Yaw'][0].get()
        for axis, val in zip(['Roll', 'Pitch', 'Yaw'], [r, p, y]):
            sliders[axis][1].config(text=f"{val:.0f}°")
        node.publish_rpy(r, p, y)

    for axis, (var, _) in sliders.items():
        var.trace_add('write', lambda *_: on_slider_change())

    tk.Button(root, text="Publish", command=on_slider_change,
              bg='#4CAF50', fg='white', padx=20).grid(row=3, column=1, pady=12)

    root.protocol("WM_DELETE_WINDOW", lambda: (rclpy.shutdown(), root.destroy()))
    root.mainloop()

if __name__ == '__main__':
    main()