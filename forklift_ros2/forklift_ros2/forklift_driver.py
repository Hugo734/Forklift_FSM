#!/usr/bin/env python3
"""
forklift_driver.py  —  ROS2 node: Jetson → FPGA forklift controller via SPI

High-level interface (what the rest of the robot sees):
  /lifter_cmd   (std_msgs/UInt8)   — 0=LOW  1=MID  2=HIGH
  /lifter_status (std_msgs/UInt8)  — current level (same encoding), 10 Hz

Low-level interface (direct tuning / calibration):
  /forklift/gains   (std_msgs/Int32MultiArray) — [Kp, Ki, Kd]  Q8 (0-255)
  /forklift/limits  (std_msgs/Int32MultiArray) — [lo_counts, hi_counts]
  /forklift/reset_pos (std_msgs/Empty)         — zero encoder now

SPI frame to FPGA (24-bit, Mode 0):
  [23:16] CMD byte
  [15:0]  VALUE signed 16-bit

FPGA returns current encoder position (16-bit signed) on every MISO frame.
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Int32MultiArray, Empty

import spidev


# ── SPI command bytes (must match spi_slave.v) ────────────────────────────────
CMD_SET_TARGET   = 0x01
CMD_SET_KP       = 0x02
CMD_SET_KI       = 0x03
CMD_SET_KD       = 0x04
CMD_SET_LIMIT_HI = 0x05
CMD_SET_LIMIT_LO = 0x06
CMD_RESET_POS    = 0x07
CMD_CALIBRATE    = 0x08

# ── Named lift levels ─────────────────────────────────────────────────────────
LEVEL_LOW  = 0
LEVEL_MID  = 1
LEVEL_HIGH = 2


class FpgaSpi:
    """Thin wrapper around spidev that speaks the 24-bit forklift protocol."""

    def __init__(self, bus: int, device: int, speed_hz: int) -> None:
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = speed_hz
        self.spi.mode = 0
        self.spi.bits_per_word = 8
        self.spi.no_cs = False
        self._last_target: int = 0

    def _send(self, cmd: int, value: int) -> int:
        """Send 24-bit frame, return 16-bit signed position from MISO."""
        value = max(-32768, min(32767, int(value)))
        v = value & 0xFFFF
        rx = self.spi.xfer2([cmd, (v >> 8) & 0xFF, v & 0xFF])
        raw = (rx[1] << 8) | rx[2]
        return raw - 0x10000 if raw >= 0x8000 else raw

    def set_target(self, counts: int) -> int:
        self._last_target = counts
        return self._send(CMD_SET_TARGET, counts)

    def read_position(self) -> int:
        return self._send(CMD_SET_TARGET, self._last_target)

    def set_kp(self, v: int) -> None: self._send(CMD_SET_KP, v & 0xFF)
    def set_ki(self, v: int) -> None: self._send(CMD_SET_KI, v & 0xFF)
    def set_kd(self, v: int) -> None: self._send(CMD_SET_KD, v & 0xFF)
    def set_limit_hi(self, v: int) -> None: self._send(CMD_SET_LIMIT_HI, v)
    def set_limit_lo(self, v: int) -> None: self._send(CMD_SET_LIMIT_LO, v)
    def reset_position(self) -> None: self._send(CMD_RESET_POS, 0)

    def close(self) -> None:
        self.spi.close()


class ForkliftDriverNode(Node):

    def __init__(self) -> None:
        super().__init__('forklift_driver')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('spi_bus',    0)
        self.declare_parameter('spi_device', 0)
        self.declare_parameter('spi_speed',  500_000)
        self.declare_parameter('poll_hz',    10.0)

        # Named position counts — tune these after running calibration
        self.declare_parameter('counts_low',  0)
        self.declare_parameter('counts_mid',  300)
        self.declare_parameter('counts_high', 600)

        # PID gains sent to FPGA on startup (Q8 format: real = value/256)
        self.declare_parameter('kp', 50)
        self.declare_parameter('ki',  5)
        self.declare_parameter('kd', 10)

        bus    = self.get_parameter('spi_bus').value
        device = self.get_parameter('spi_device').value
        speed  = self.get_parameter('spi_speed').value
        hz     = self.get_parameter('poll_hz').value

        self._counts = {
            LEVEL_LOW:  self.get_parameter('counts_low').value,
            LEVEL_MID:  self.get_parameter('counts_mid').value,
            LEVEL_HIGH: self.get_parameter('counts_high').value,
        }

        self._current_level: int = LEVEL_LOW

        # ── SPI ───────────────────────────────────────────────────────────────
        self._spi = FpgaSpi(bus, device, speed)
        self.get_logger().info(f'SPI open: /dev/spidev{bus}.{device} @ {speed} Hz')

        # Push initial gains and limits to the FPGA
        self._configure_fpga()

        # ── Subscribers ───────────────────────────────────────────────────────

        # Main command: 0=LOW, 1=MID, 2=HIGH
        self.create_subscription(UInt8, '/lifter_cmd', self._cb_cmd, 10)

        # Low-level tuning
        self.create_subscription(
            Int32MultiArray, '/forklift/gains',  self._cb_gains,  10)
        self.create_subscription(
            Int32MultiArray, '/forklift/limits', self._cb_limits, 10)
        self.create_subscription(
            Empty, '/forklift/reset_pos', self._cb_reset, 10)

        # ── Publisher ─────────────────────────────────────────────────────────
        self._pub_status = self.create_publisher(UInt8, '/lifter_status', 10)

        # ── Timer ─────────────────────────────────────────────────────────────
        self.create_timer(1.0 / hz, self._poll)

        self.get_logger().info(
            f'ForkliftDriver ready — LOW={self._counts[0]} '
            f'MID={self._counts[1]} HIGH={self._counts[2]} counts'
        )

    # ── Startup configuration ─────────────────────────────────────────────────

    def _configure_fpga(self) -> None:
        """Push gains and limits to the FPGA once at startup."""
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        lo = self._counts[LEVEL_LOW]
        hi = self._counts[LEVEL_HIGH]

        self._spi.set_kp(kp)
        time.sleep(0.002)
        self._spi.set_ki(ki)
        time.sleep(0.002)
        self._spi.set_kd(kd)
        time.sleep(0.002)
        self._spi.set_limit_lo(lo)
        time.sleep(0.002)
        self._spi.set_limit_hi(hi)
        time.sleep(0.002)

        self.get_logger().info(
            f'FPGA configured: Kp={kp} Ki={ki} Kd={kd} '
            f'limits=[{lo}, {hi}]'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_cmd(self, msg: UInt8) -> None:
        """Receive HIGH-LEVEL position command: 0=LOW 1=MID 2=HIGH."""
        level = int(msg.data)
        if level not in self._counts:
            self.get_logger().warn(f'Unknown level {level}, use 0/1/2')
            return

        counts = self._counts[level]
        self._spi.set_target(counts)
        self._current_level = level

        names = {0: 'LOW', 1: 'MID', 2: 'HIGH'}
        self.get_logger().info(f'Moving to {names[level]} ({counts} counts)')

    def _cb_gains(self, msg: Int32MultiArray) -> None:
        """Re-tune PID gains at runtime: [Kp, Ki, Kd]."""
        if len(msg.data) != 3:
            self.get_logger().warn('gains needs [Kp, Ki, Kd]')
            return
        kp, ki, kd = msg.data
        self._spi.set_kp(kp)
        time.sleep(0.002)
        self._spi.set_ki(ki)
        time.sleep(0.002)
        self._spi.set_kd(kd)
        self.get_logger().info(f'Gains updated: Kp={kp} Ki={ki} Kd={kd}')

    def _cb_limits(self, msg: Int32MultiArray) -> None:
        """Update soft travel limits: [lo_counts, hi_counts]."""
        if len(msg.data) != 2:
            self.get_logger().warn('limits needs [lo, hi]')
            return
        lo, hi = msg.data
        self._spi.set_limit_lo(lo)
        time.sleep(0.002)
        self._spi.set_limit_hi(hi)
        self.get_logger().info(f'Limits updated: lo={lo} hi={hi}')

    def _cb_reset(self, _: Empty) -> None:
        """Zero the encoder at the current physical position."""
        self._spi.reset_position()
        self.get_logger().info('Encoder zeroed')

    # ── Position polling ──────────────────────────────────────────────────────

    def _poll(self) -> None:
        """Read encoder position from FPGA and publish current level."""
        pos = self._spi.read_position()

        # Derive which named level we are closest to
        level = min(self._counts, key=lambda k: abs(self._counts[k] - pos))

        msg = UInt8()
        msg.data = level
        self._pub_status.publish(msg)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        self._spi.close()
        super().destroy_node()


def main(args=None) -> None:
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
