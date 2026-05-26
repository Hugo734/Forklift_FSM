#!/usr/bin/env python3
"""
forklift_spi.py  —  Jetson Nano SPI driver for forklift FPGA controller

Hardware: Jetson Nano 2GB → Tang Nano 20K
SPI bus: /dev/spidev0.0  (SPI1 on Jetson 40-pin header)
  Pin 19 → MOSI
  Pin 21 → MISO
  Pin 23 → SCLK
  Pin 24 → CS0

Install: pip3 install spidev

Usage:
    ctrl = ForkliftSPI()
    ctrl.set_target(300)       # move fork to count 300
    ctrl.set_gains(50, 5, 10)  # Kp, Ki, Kd
    pos = ctrl.read_position() # read current position
    ctrl.calibrate()           # interactive calibration
"""

import spidev
import time
import struct

# ── SPI Commands ──────────────────────────────────────────────
CMD_SET_TARGET   = 0x01
CMD_SET_KP       = 0x02
CMD_SET_KI       = 0x03
CMD_SET_KD       = 0x04
CMD_SET_LIMIT_HI = 0x05
CMD_SET_LIMIT_LO = 0x06
CMD_RESET_POS    = 0x07
CMD_CALIBRATE    = 0x08


class ForkliftSPI:
    def __init__(self, bus=0, device=0, speed_hz=500_000):
        """
        Initialize SPI connection to FPGA.
        speed_hz: keep <= 1MHz for reliable operation with Jetson GPIO
        """
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = speed_hz
        self.spi.mode = 0           # CPOL=0, CPHA=0
        self.spi.bits_per_word = 8
        self.spi.no_cs = False

        # Track state locally
        self._target   = 0
        self._limit_hi = 1200   # default: 1 rev @ 100:1 gear ratio
        self._limit_lo = 0

    def _send(self, cmd: int, value: int) -> int:
        """
        Send a 24-bit frame: [CMD 8-bit | VALUE 16-bit signed]
        Returns 16-bit position readback from MISO.
        Value is clamped to signed 16-bit.
        """
        value = max(-32768, min(32767, int(value)))
        # Pack as unsigned for SPI transfer
        val_u16 = value & 0xFFFF
        frame = [cmd, (val_u16 >> 8) & 0xFF, val_u16 & 0xFF]
        resp  = self.spi.xfer2(frame)
        # MISO returns 16-bit position (upper 8 bits in byte 1, lower in byte 2)
        raw = (resp[1] << 8) | resp[2]
        # Sign-extend from 16-bit
        if raw >= 0x8000:
            raw -= 0x10000
        return raw

    # ── High-level commands ───────────────────────────────────

    def set_target(self, counts: int) -> int:
        """Move fork to target position in encoder counts."""
        self._target = counts
        return self._send(CMD_SET_TARGET, counts)

    def set_gains(self, kp: int, ki: int, kd: int):
        """
        Set PID gains. Values are Q8 fixed-point (0..255).
        Real gain = value / 256
        Typical starting point: Kp=50, Ki=5, Kd=10
        """
        self._send(CMD_SET_KP, kp & 0xFF)
        time.sleep(0.001)
        self._send(CMD_SET_KI, ki & 0xFF)
        time.sleep(0.001)
        self._send(CMD_SET_KD, kd & 0xFF)

    def set_limits(self, lo: int, hi: int):
        """Set soft travel limits in encoder counts."""
        self._limit_lo = lo
        self._limit_hi = hi
        self._send(CMD_SET_LIMIT_LO, lo)
        time.sleep(0.001)
        self._send(CMD_SET_LIMIT_HI, hi)

    def reset_position(self):
        """Zero the encoder counter at current position."""
        return self._send(CMD_RESET_POS, 0)

    def read_position(self) -> int:
        """
        Read current position without changing anything.
        Sends a no-op SET_TARGET with current target.
        """
        return self._send(CMD_SET_TARGET, self._target)

    def enter_calibrate(self):
        """Enter calibration mode — MISO returns live position."""
        self._send(CMD_CALIBRATE, 1)

    def exit_calibrate(self):
        """Exit calibration mode."""
        self._send(CMD_CALIBRATE, 0)

    # ── Calibration routine ───────────────────────────────────

    def calibrate(self):
        """
        Interactive calibration routine.
        Run this once to find your gear ratio and set limits.

        Steps:
        1. Enter calibrate mode
        2. Zero position at bottom of travel
        3. Move fork to top manually (power motor briefly)
        4. Read count at top → set as upper limit
        5. Exit calibrate mode
        """
        print("=== Forklift Calibration ===")
        print("Step 1: Enter calibration mode")
        self.enter_calibrate()
        time.sleep(0.1)

        print("Step 2: Move fork to BOTTOM of travel manually")
        input("Press Enter when fork is at the BOTTOM...")
        self.reset_position()
        pos = self.read_position()
        print(f"  Position zeroed. Current: {pos}")

        print("Step 3: Move fork to TOP of travel")
        print("  (Use set_target() or move manually with power off)")
        input("Press Enter when fork is at the TOP...")

        pos = self.read_position()
        print(f"  Top position: {pos} counts")
        print(f"  This is your COUNTS_PER_REV equivalent for full travel")

        use_it = input(f"Set upper limit to {pos}? [Y/n]: ").strip().lower()
        if use_it != 'n':
            self.set_limits(0, pos)
            print(f"  Limits set: 0 → {pos}")

        self.exit_calibrate()
        print("Calibration complete!")
        print(f"\nAdd to your FPGA parameter:")
        print(f"  parameter GEAR_RATIO = <measured_ratio>;")
        return pos

    # ── Monitoring ────────────────────────────────────────────

    def monitor(self, interval_s=0.1, duration_s=10.0):
        """Print position readback for duration seconds."""
        print(f"{'Time':>8}  {'Position':>10}  {'Target':>10}")
        print("-" * 35)
        t0 = time.time()
        while time.time() - t0 < duration_s:
            pos = self.read_position()
            t   = time.time() - t0
            print(f"{t:8.2f}  {pos:10d}  {self._target:10d}")
            time.sleep(interval_s)

    def close(self):
        self.spi.close()


# ── Example usage ─────────────────────────────────────────────
if __name__ == "__main__":
    ctrl = ForkliftSPI(bus=0, device=0, speed_hz=500_000)

    try:
        # 1. Set PID gains (tune these after calibration)
        ctrl.set_gains(kp=50, ki=5, kd=10)

        # 2. Set travel limits (tune after calibration)
        ctrl.set_limits(lo=0, hi=600)

        # 3. Move fork to 50% height
        mid = 300
        print(f"Moving to position {mid}...")
        ctrl.set_target(mid)

        # 4. Monitor for 5 seconds
        ctrl.monitor(interval_s=0.05, duration_s=5.0)

        # 5. Move to top
        print("Moving to top...")
        ctrl.set_target(600)
        ctrl.monitor(interval_s=0.05, duration_s=5.0)

        # 6. Return to bottom
        print("Returning to bottom...")
        ctrl.set_target(0)
        ctrl.monitor(interval_s=0.05, duration_s=5.0)

    finally:
        ctrl.close()
