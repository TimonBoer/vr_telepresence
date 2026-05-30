#!/usr/bin/env python3
"""
Movella DOT → ROS 2 QuaternionStamped publisher (using bleak)
Install: pip install bleak --break-system-packages
"""

import asyncio
import struct
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped

# ── Replace this with your DOT's Bluetooth MAC address ────────────────────────
DOT_ADDRESS = "XX:XX:XX:XX:XX:XX"

# ── BLE UUIDs from Movella DOT BLE Service Specification ─────────────────────
CONTROL_CHARACTERISTIC   = "15172001-4947-11e9-8646-d663bd873d93"
SHORT_PAYLOAD_NOTIFY     = "15172004-4947-11e9-8646-d663bd873d93"

# Payload mode 5 = Orientation (Quaternion) — writes to control characteristic
# Bytes: [0x01, 0x01, 0x05]  →  start measurement, mode 5
START_ORIENTATION_QUAT = bytes([0x01, 0x01, 0x05])
STOP_MEASUREMENT       = bytes([0x01, 0x00, 0x00])


class MovellaDotTalker(Node):

    def __init__(self):
        super().__init__('movella_dot_talker')

        self.declare_parameter('frame_id', 'imu_link')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.pub = self.create_publisher(QuaternionStamped, 'imu/orientation', 10)

        # Run the bleak BLE loop in a background thread
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_ble, daemon=True)
        self._thread.start()

        self.get_logger().info(f'Connecting to DOT at {DOT_ADDRESS} ...')

    # ── BLE thread ────────────────────────────────────────────────────────────
    def _run_ble(self):
        self._loop.run_until_complete(self._ble_loop())

    async def _ble_loop(self):
        from bleak import BleakClient

        async with BleakClient(DOT_ADDRESS) as client:
            self.get_logger().info('Connected to Movella DOT')

            # Subscribe to quaternion notifications
            await client.start_notify(SHORT_PAYLOAD_NOTIFY, self._on_packet)

            # Start measurement in orientation quaternion mode
            await client.write_gatt_char(CONTROL_CHARACTERISTIC, START_ORIENTATION_QUAT)

            # Keep running until ROS shuts down
            while rclpy.ok():
                await asyncio.sleep(0.1)

            # Stop measurement cleanly
            await client.write_gatt_char(CONTROL_CHARACTERISTIC, STOP_MEASUREMENT)

    # ── Packet parser ─────────────────────────────────────────────────────────
    def _on_packet(self, sender, data: bytearray):
        # Short payload (mode 5 — Orientation Quaternion) is 20 bytes:
        # [0..3]  timestamp (uint32, ms)
        # [4..7]  w (float32)
        # [8..11] x (float32)
        # [12..15] y (float32)
        # [16..19] z (float32)
        if len(data) < 20:
            return

        _, w, x, y, z = struct.unpack_from('<Iffff', data, 0)

        msg = QuaternionStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.quaternion.w = w
        msg.quaternion.x = x
        msg.quaternion.y = y
        msg.quaternion.z = z

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MovellaDotTalker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()