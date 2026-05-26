#!/usr/bin/env python3
"""
forklift_driver.py  —  ROS2 node that bridges SPI ↔ FPGA motor controller

Subscribed topics:
  /forklift/target_pos  (std_msgs/Int32)            — move fork to encoder count
  /forklift/gains       (std_msgs/Int32MultiArray)  — [Kp, Ki, Kd] Q8 values 0-255
  /forklift/limits      (std_msgs/Int32MultiArray)  — [lo_count, hi_count]
  /forklift/reset_pos   (std_msgs/Empty)            — zero the encoder now

Published topics:
  /forklift/position    (std_msgs/Int32)            — current position @ 10 Hz

SPI frame (24-bit, Mode 0, 500 kHz):
  [23:16] CMD byte
  [15:0]  VALUE signed 16-bit

Run on Jetson Nano:
  ros2 run forklift_ros2 forklift_driver
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray, Empty

import spidev
import time


# ── SPI command constants (must match spi_slave.v) ────────────────────────────
CMD_SET_TARGET   = 0x01
CMD_SET_KP       = 0x02
CMD_SET_KI       = 0x03
CMD_SET_KD       = 0x04
CMD_SET_LIMIT_HI = 0x05
CMD_SET_LIMIT_LO = 0x06
CMD_RESET_POS    = 0x07
CMD_CALIBRATE    = 0x08


class ForkliftSPI:
    """Low-level SPI driver — talks to the FPGA over /dev/spidev0.0."""

    def __init__(self, bus: int = 0, device: int = 0, speed_hz: int = 500_000):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = speed_hz
        self.spi.mode = 0           # CPOL=0, CPHA=0  — matches spi_slave.v
        self.spi.bits_per_word = 8
        self.spi.no_cs = False
        self._target = 0            # cached so read_position() is a true no-op

    def _send(self, cmd: int, value: int) -> int:
        """
        Send one 24-bit SPI frame and return the 16-bit position MISO readback.
        The FPGA shifts out pos_readback[15:0] simultaneously with receiving cmd+value.
        """
        value = max(-32768, min(32767, int(value)))
        val_u16 = value & 0xFFFF
        frame = [cmd, (val_u16 >> 8) & 0xFF, val_u16 & 0xFF]
        resp = self.spi.xfer2(frame)
        raw = (resp[1] << 8) | resp[2]
        if raw >= 0x8000:           # sign-extend from 16-bit
            raw -= 0x10000
        return raw

    def set_target(self, counts: int) -> int:
        self._target = counts
        return self._send(CMD_SET_TARGET, counts)

    def set_kp(self, kp: int):
        self._send(CMD_SET_KP, kp & 0xFF)

    def set_ki(self, ki: int):
        self._send(CMD_SET_KI, ki & 0xFF)

    def set_kd(self, kd: int):
        self._send(CMD_SET_KD, kd & 0xFF)

    def set_limit_hi(self, hi: int):
        self._send(CMD_SET_LIMIT_HI, hi)

    def set_limit_lo(self, lo: int):
        self._send(CMD_SET_LIMIT_LO, lo)

    def reset_position(self):
        self._send(CMD_RESET_POS, 0)

    def read_position(self) -> int:
        # Re-send current target — FPGA echoes position on MISO, no state changes
        return self._send(CMD_SET_TARGET, self._target)

    def close(self):
        self.spi.close()


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class ForkliftDriverNode(Node):

    def __init__(self):
        super().__init__('forklift_driver')

        # Declare parameters so they can be set from the launch file or CLI
        self.declare_parameter('spi_bus',    0)
        self.declare_parameter('spi_device', 0)
        self.declare_parameter('spi_speed',  500_000)
        self.declare_parameter('poll_hz',    10.0)

        bus    = self.get_parameter('spi_bus').value
        device = self.get_parameter('spi_device').value
        speed  = self.get_parameter('spi_speed').value
        hz     = self.get_parameter('poll_hz').value

        self.spi = ForkliftSPI(bus, device, speed)
        self.get_logger().info(f'SPI open: /dev/spidev{bus}.{device} @ {speed} Hz')

        # ── Subscribers ───────────────────────────────────────────────────────

        self.create_subscription(
            Int32,
            '/forklift/target_pos',
            self._cb_target,
            10
        )

        self.create_subscription(
            Int32MultiArray,
            '/forklift/gains',
            self._cb_gains,
            10
        )

        self.create_subscription(
            Int32MultiArray,
            '/forklift/limits',
            self._cb_limits,
            10
        )

        self.create_subscription(
            Empty,
            '/forklift/reset_pos',
            self._cb_reset,
            10
        )

        # ── Publisher ─────────────────────────────────────────────────────────

        self.pub_pos = self.create_publisher(Int32, '/forklift/position', 10)

        # ── Timer: poll position and publish ─────────────────────────────────

        self.create_timer(1.0 / hz, self._poll_position)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_target(self, msg: Int32):
        """Move fork to target encoder count."""
        self.spi.set_target(msg.data)
        self.get_logger().info(f'Target → {msg.data} counts')

    def _cb_gains(self, msg: Int32MultiArray):
        """
        Set PID gains. Expects exactly 3 values: [Kp, Ki, Kd].
        Values are Q8 fixed-point (0–255). Real gain = value / 256.
        Publish with:
          ros2 topic pub /forklift/gains std_msgs/Int32MultiArray \
            "{data: [50, 5, 10]}"
        """
        if len(msg.data) != 3:
            self.get_logger().warn('gains needs exactly 3 values: [Kp, Ki, Kd]')
            return
        kp, ki, kd = msg.data
        self.spi.set_kp(kp)
        time.sleep(0.001)
        self.spi.set_ki(ki)
        time.sleep(0.001)
        self.spi.set_kd(kd)
        self.get_logger().info(f'Gains → Kp={kp} Ki={ki} Kd={kd}')

    def _cb_limits(self, msg: Int32MultiArray):
        """
        Set soft travel limits. Expects exactly 2 values: [lo, hi] in counts.
        Publish with:
          ros2 topic pub /forklift/limits std_msgs/Int32MultiArray \
            "{data: [0, 600]}"
        """
        if len(msg.data) != 2:
            self.get_logger().warn('limits needs exactly 2 values: [lo, hi]')
            return
        lo, hi = msg.data
        self.spi.set_limit_lo(lo)
        time.sleep(0.001)
        self.spi.set_limit_hi(hi)
        self.get_logger().info(f'Limits → lo={lo} hi={hi}')

    def _cb_reset(self, _msg: Empty):
        """Zero the encoder counter at the current physical position."""
        self.spi.reset_position()
        self.get_logger().info('Encoder position zeroed')

    def _poll_position(self):
        """Read position over SPI and publish it."""
        pos = self.spi.read_position()
        out = Int32()
        out.data = pos
        self.pub_pos.publish(out)

    def destroy_node(self):
        self.spi.close()
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ForkliftDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
