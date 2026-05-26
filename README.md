# Forklift FPGA Motor Controller

Closed-loop position controller for a DC geared motor (scale forklift lift mechanism).  
The FPGA handles all real-time motor control. A Jetson Nano sends high-level position commands over SPI.

```
Jetson Nano  ──SPI──►  Tang Nano 20K FPGA  ──PWM──►  L298N  ──►  DC Motor
                               ▲                                       │
                               └──────── Encoder feedback ─────────────┘
```

---

## Table of Contents

1. [Hardware](#hardware)
2. [System Architecture](#system-architecture)
3. [FPGA Modules](#fpga-modules)
4. [SPI Protocol](#spi-protocol)
5. [ROS2 Node](#ros2-node)
6. [Wiring](#wiring)
7. [Setup](#setup)
8. [Calibration](#calibration)
9. [Usage](#usage)
10. [Tuning the PID](#tuning-the-pid)
11. [File Structure](#file-structure)

---

## Hardware

| Component | Part | Notes |
|-----------|------|-------|
| FPGA | Tang Nano 20K (Gowin GW2AR-18) | 27 MHz system clock |
| Motor | GBMQ-GM12BY20 DC geared motor | ~100:1 gear ratio |
| Encoder | 3 pulses per revolution (motor shaft) | 4x quadrature = 1200 counts/rev at output |
| Motor driver | L298N H-bridge | IN1/IN2 = direction, ENA = PWM |
| Host computer | Jetson Nano 2GB | Runs ROS2, sends SPI commands |

**Encoder resolution:**  
3 PPR × 4 (quadrature) × 100 (gear ratio) = **1200 counts per output shaft revolution**

---

## System Architecture

The system is split into two layers:

### Low level — FPGA (Tang Nano 20K)

Everything that needs to happen in real time runs on the FPGA:

- Reads the motor encoder at 100 kHz (debounced quadrature decoder)
- Runs a PID control loop every 1 ms
- Drives the L298N motor driver with a 6.6 kHz PWM signal
- Enforces soft travel limits (no physical end-stops needed)
- Accepts commands from the Jetson over SPI
- Streams current position back to the Jetson on every SPI transaction

### High level — Jetson Nano (ROS2)

The Jetson only needs to:

- Receive a position command (`LOW`, `MID`, or `HIGH`) from the rest of the robot
- Convert that to an encoder count target
- Send `SET_TARGET` over SPI
- Read the position back and publish status

```
┌──────────────────────────────────────┐     SPI (500 kHz)
│           Jetson Nano                │◄──────────────────────────────────────────┐
│                                      │                                           │
│  /lifter_cmd (UInt8: 0/1/2)          │  CMD_SET_TARGET(counts)                   │
│       │                              │ ───────────────────────────────────────►  │
│       ▼                              │                                           │
│  ForkliftDriverNode                  │  pos_readback (16-bit MISO)               │
│    counts_low  = 0                   │ ◄───────────────────────────────────────  │
│    counts_mid  = 300                 │                                           │
│    counts_high = 600                 │  ┌──────────────────────────────────────┐ │
│       │                              │  │         Tang Nano 20K FPGA           │ │
│  /lifter_status (UInt8: 0/1/2)       │  │                                      │ │
│       │                              │  │  spi_slave.v  → target_pos register  │ │
└──────────────────────────────────────┘  │       │                              │ │
                                          │       ▼                              │ │
                                          │  pid_controller.v  (every 1 ms)     │ │
                                          │       │ pid_out (signed 13-bit)      │ │
                                          │       ▼                              │ │
                                          │  soft_limits.v                       │ │
                                          │       │ limit_hit                    │ │
                                          │       ▼                              │ │
                                          │  pwm_gen.v                           │ │
                                          │       │ ENA (PWM) / IN1 / IN2        │ │
                                          │       ▼                              │ │
                                          │     L298N ──► DC Motor               │ │
                                          │       ▲                              │ │
                                          │  quad_decoder.v ◄── Encoder A/B      │ │
                                          │  current_pos (32-bit) ───────────────┘ │
                                          └──────────────────────────────────────┘
```

---

## FPGA Modules

### `forklift_top.v` — Top level

Instantiates all other modules and connects their signals. Contains only two pieces of logic:

**Power-on reset:** Holds `rst_n` low for ~4.8 ms (2¹⁷ cycles at 27 MHz) at startup so all modules initialise cleanly before the motor is allowed to run.

**Signal wiring:** Connects encoder → decoder → PID → soft_limits → pwm_gen, and connects the SPI slave outputs to the PID and limits modules.

---

### `quad_decoder.v` — Quadrature encoder reader

Reads the two encoder channels (A and B) and maintains a signed 32-bit position counter.

| Parameter | Value | Meaning |
|-----------|-------|---------|
| Sample rate | 100 kHz | Encoder sampled every 270 clock cycles |
| Filter depth | 4 stages | All 4 samples must agree before a transition is accepted |
| Resolution | 4x | Both edges of both channels are counted |

**How direction is determined:**  
The decoder looks at the previous and current state of A and B together. The Gray-code pattern tells it whether the motor moved forward or backward:

```
{a_prev, b_prev, a_now, b_now}
  0001, 0111, 1110, 1000  →  position + 1  (fork UP)
  0010, 1011, 1101, 0100  →  position - 1  (fork DOWN)
  anything else           →  ignored (noise or missed pulse)
```

**Output:**  
- `position` — signed 32-bit absolute count. Zero at reset. Positive = UP, negative = DOWN.
- `valid` — pulses high for one clock when position changes.

---

### `pid_controller.v` — PID control loop

Runs a fixed-point PID algorithm every 1 ms and outputs a signed 13-bit motor command.

**Algorithm:**
```
error     = target_pos - current_pos
P term    = Kp × error
I term    = Ki × Σ(error)          [clamped at ±500,000 for anti-windup]
D term    = Kd × (error − prev_error)
pid_out   = (P + I + D) >> 8       [Q8 scale-down]
```

**Fixed-point gains (Q8 format):**  
Gains are integers 0–255. The real gain = value ÷ 256.

| Parameter | Default | Real value |
|-----------|---------|------------|
| Kp | 50 | ≈ 0.195 |
| Ki | 5  | ≈ 0.020 |
| Kd | 10 | ≈ 0.039 |

**Output range:** −4095 to +4095.  
Positive = drive UP, negative = drive DOWN.

---

### `pwm_gen.v` — L298N motor driver

Converts the signed PID output into the three signals the L298N needs.

**L298N truth table:**

| IN1 | IN2 | ENA | Motor action |
|-----|-----|-----|--------------|
| 1   | 0   | PWM | Fork UP |
| 0   | 1   | PWM | Fork DOWN |
| 0   | 0   | 0   | Brake (motor short-circuited) |

**Key parameters:**

| Parameter | Default | Effect |
|-----------|---------|--------|
| PWM frequency | ~6.6 kHz | 27 MHz ÷ 4096 |
| `DEADBAND` | 20 counts | PID output below this → motor brakes. Prevents hunting at setpoint. |
| `RAMP_STEP` | 4 counts/cycle | Duty cycle ramps up gradually on start/direction change. Prevents current spikes and mechanical shock. |

Ramp time to full duty (2000): 2000 ÷ 4 × 151 µs ≈ **75 ms**.

---

### `soft_limits.v` — Travel limits

Prevents the fork from being driven beyond its physical range. No end-stop switches required.

```
limit_hit = (position ≥ limit_hi  AND  PID trying to go UP)
          OR (position ≤ limit_lo  AND  PID trying to go DOWN)
```

`limit_hit` feeds directly into `pwm_gen` and cuts the motor immediately. The motor can still move *away* from a limit freely.

**Defaults:** `limit_lo = 0`, `limit_hi = 1200` (one full output shaft revolution).

---

### `spi_slave.v` — SPI command receiver

Receives 24-bit frames from the Jetson Nano and decodes them into register writes. Simultaneously sends the current encoder position back over MISO.

See [SPI Protocol](#spi-protocol) below for the full frame format.

**Clock domain crossing:**  
SPI signals arrive asynchronous to the FPGA's 27 MHz clock. Each signal passes through a 3-stage synchroniser before edge detection:
```verilog
wire sclk_rise = (sclk_r[2:1] == 2'b01);  // rising edge
wire sclk_fall = (sclk_r[2:1] == 2'b10);  // falling edge
```
MOSI uses a 2-stage synchroniser (lower latency needed for data sampling).

---

### `uart_tx.v` — UART debug output

Sends the encoder position over serial every 100 ms. Useful for debugging without a Jetson.

| Parameter | Value |
|-----------|-------|
| Baud rate | 115200 |
| Format | 8N1 |
| Output pin | Tang Nano pin 17 |
| Message | `POS:+00342\r\n` |

Connect a USB-serial adapter to pin 17 and open any terminal at 115200 baud.

---

## SPI Protocol

**Configuration:** Mode 0 (CPOL=0, CPHA=0), 500 kHz, MSB first, 8 bits per word.

**Frame format — 3 bytes (24 bits) per transaction:**

```
Byte 0       Byte 1         Byte 2
[CMD 8-bit]  [VALUE[15:8]]  [VALUE[7:0]]
```

- `CMD` — command byte (see table below)
- `VALUE` — signed 16-bit integer

**While the Jetson sends those 3 bytes, the FPGA simultaneously shifts out the current 16-bit encoder position on MISO** (bytes 1 and 2 of the response). Every SPI transaction is both a write and a position read.

**Command table:**

| CMD | Hex | Value meaning | Effect |
|-----|-----|---------------|--------|
| SET_TARGET   | `0x01` | Target position (counts) | PID moves fork to this count |
| SET_KP       | `0x02` | Kp gain (0–255, Q8) | Update proportional gain |
| SET_KI       | `0x03` | Ki gain (0–255, Q8) | Update integral gain |
| SET_KD       | `0x04` | Kd gain (0–255, Q8) | Update derivative gain |
| SET_LIMIT_HI | `0x05` | Upper limit (counts) | Soft upper travel limit |
| SET_LIMIT_LO | `0x06` | Lower limit (counts) | Soft lower travel limit |
| RESET_POS    | `0x07` | (ignored) | Zero encoder counter now |
| CALIBRATE    | `0x08` | 1=enter / 0=exit | Calibration mode |

**Python example (raw):**
```python
import spidev
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 500_000
spi.mode = 0

# Send SET_TARGET = 300 counts
resp = spi.xfer2([0x01, 0x01, 0x2C])  # 0x012C = 300
pos = (resp[1] << 8) | resp[2]        # current position in MISO
```

---

## ROS2 Node

**Package:** `forklift_ros2`  
**Executable:** `forklift_driver`  
**Node name:** `forklift_driver`

### Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/lifter_cmd` | `std_msgs/UInt8` | Subscribed | Position command: `0`=LOW, `1`=MID, `2`=HIGH |
| `/lifter_status` | `std_msgs/UInt8` | Published (10 Hz) | Current level: `0`=LOW, `1`=MID, `2`=HIGH |
| `/forklift/gains` | `std_msgs/Int32MultiArray` | Subscribed | Live PID retune: `[Kp, Ki, Kd]` |
| `/forklift/limits` | `std_msgs/Int32MultiArray` | Subscribed | Live limit update: `[lo_counts, hi_counts]` |
| `/forklift/reset_pos` | `std_msgs/Empty` | Subscribed | Zero the encoder at current position |

### Parameters

Loaded from `config/forklift_params.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `spi_bus` | `0` | SPI bus (`/dev/spidev0.x`) |
| `spi_device` | `0` | SPI device (`/dev/spidevx.0`) |
| `spi_speed` | `500000` | SPI clock in Hz |
| `poll_hz` | `10.0` | Position poll and publish rate |
| `counts_low` | `0` | Encoder counts for LOW position |
| `counts_mid` | `300` | Encoder counts for MID position |
| `counts_high` | `600` | Encoder counts for HIGH position |
| `kp` | `50` | Proportional gain sent to FPGA at startup |
| `ki` | `5` | Integral gain sent to FPGA at startup |
| `kd` | `10` | Derivative gain sent to FPGA at startup |

### Startup behaviour

On node start, the driver automatically sends `SET_KP`, `SET_KI`, `SET_KD`, `SET_LIMIT_LO`, and `SET_LIMIT_HI` to the FPGA using the values from the YAML. No manual initialisation is needed.

### Status mapping

`/lifter_status` publishes whichever named level the current encoder count is **closest to**:

```python
level = min({LOW, MID, HIGH}, key=lambda k: abs(counts[k] - current_pos))
```

---

## Wiring

### Jetson Nano 40-pin header → Tang Nano 20K

| Jetson pin | Signal | Tang Nano pin | FPGA port |
|-----------|--------|---------------|-----------|
| 19 | MOSI | 34 | `spi_mosi` |
| 21 | MISO | 35 | `spi_miso` |
| 23 | SCLK | 32 | `spi_clk` |
| 24 | CS0 | 33 | `spi_cs_n` |
| GND | GND | GND | — |

### Tang Nano 20K → L298N

| Tang Nano pin | FPGA port | L298N pin | Signal |
|---------------|-----------|-----------|--------|
| 72 | `ena` | ENA | PWM enable |
| 28 | `in1` | IN1 | Direction bit 0 |
| 29 | `in2` | IN2 | Direction bit 1 |

### Tang Nano 20K → Encoder

| Tang Nano pin | FPGA port | Encoder |
|---------------|-----------|---------|
| 25 | `enc_a` | Channel A |
| 26 | `enc_b` | Channel B |

### Tang Nano 20K — debug serial

| Tang Nano pin | Signal | Connect to |
|---------------|--------|------------|
| 17 | `uart_tx_pin` | USB-serial RX (115200 baud) |

---

## Setup

### 1. Enable SPI on the Jetson Nano

```bash
# Add spidev to modules loaded at boot
sudo nano /etc/modules-load.d/modules.conf
# Add this line:
spidev

# Enable SPI1 on the 40-pin header using Nvidia's tool
sudo /opt/nvidia/jetson-io/jetson-io.pyv
# Select: Configure Jetson 40pin Header → SPI1 → confirm (*)  → Save & Reboot
```

After reboot, verify:
```bash
ls /dev/spidev*
# Should show: /dev/spidev0.0  /dev/spidev0.1  /dev/spidev1.0  /dev/spidev1.1
```

### 2. Install Python dependencies

```bash
pip3 install spidev
```

### 3. Flash the FPGA

Open the project in **Gowin EDA IDE**:
- Project file: `forklift/forklift.gprj`
- Run synthesis → place & route → generate bitstream
- Program the Tang Nano 20K via USB

### 4. Build the ROS2 package

```bash
mkdir -p ~/ros2_ws/src
cp -r /path/to/forklift_ros2 ~/ros2_ws/src/
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Calibration

The `counts_low`, `counts_mid`, and `counts_high` parameters in `forklift_params.yaml` must match the real physical positions of your fork. Run the calibration routine to measure them:

```bash
# From the Jetson Nano, with FPGA powered and motor wired
python3 forklift/forklift_spi.py
```

The interactive routine will:
1. Enter calibration mode
2. Ask you to move the fork to the **bottom** of travel → zeros the encoder
3. Ask you to move the fork to the **top** of travel → reads the count at the top
4. Set soft limits automatically

**Steps after calibration:**

1. Note the count at each desired position (LOW, MID, HIGH)
2. Update `forklift_ros2/config/forklift_params.yaml`:

```yaml
counts_low:  0      # fork at floor level (encoder zero)
counts_mid:  300    # transport / carry height  ← replace with measured value
counts_high: 600    # top of travel             ← replace with measured value
```

3. Rebuild the package: `cd ~/ros2_ws && colcon build`

---

## Usage

### Launch the driver

```bash
ros2 launch forklift_ros2 forklift.launch.py
```

### Send position commands

```bash
# Move to LOW (floor level)
ros2 topic pub --once /lifter_cmd std_msgs/UInt8 "{data: 0}"

# Move to MID (transport height)
ros2 topic pub --once /lifter_cmd std_msgs/UInt8 "{data: 1}"

# Move to HIGH (max height)
ros2 topic pub --once /lifter_cmd std_msgs/UInt8 "{data: 2}"
```

### Monitor status

```bash
ros2 topic echo /lifter_status
```

### Zero the encoder at current position

Do this when the fork is physically at the LOW position:
```bash
ros2 topic pub --once /forklift/reset_pos std_msgs/Empty "{}"
```

### Retune PID gains at runtime

```bash
ros2 topic pub --once /forklift/gains std_msgs/Int32MultiArray "{data: [60, 5, 15]}"
# [Kp, Ki, Kd] — Q8 format, real gain = value / 256
```

### Update travel limits at runtime

```bash
ros2 topic pub --once /forklift/limits std_msgs/Int32MultiArray "{data: [0, 600]}"
# [lo_counts, hi_counts]
```

---

## Tuning the PID

All three gains are Q8 fixed-point: **real gain = value ÷ 256**.

| Symptom | Action |
|---------|--------|
| Fork moves too slowly to target | Increase Kp |
| Fork overshoots and oscillates | Decrease Kp, or increase Kd |
| Fork settles near target but not exactly on it | Increase Ki |
| Motor buzzes / hunts at the setpoint | Increase `DEADBAND` in `pwm_gen.v` (requires FPGA reflash) |
| Fork lurches on start or direction change | Decrease `RAMP_STEP` in `pwm_gen.v` (requires FPGA reflash) |

**Suggested tuning sequence:**
1. Set `Ki=0`, `Kd=0`, increase `Kp` until the fork responds quickly but does not overshoot
2. Add `Kd` (start at 5–10) to damp any oscillation
3. Add `Ki` (start at 2–5) only if there is a consistent steady-state error

```bash
# Example: increase Kp only
ros2 topic pub --once /forklift/gains std_msgs/Int32MultiArray "{data: [80, 0, 0]}"
```

---

## File Structure

```
forklift/
├── forklift_top.v          Top-level module — wires all submodules together
├── spi_slave.v             SPI Mode 0 slave — receives commands, sends position
├── pid_controller.v        Fixed-point PID (Q8 gains, 1 ms update rate)
├── pwm_gen.v               L298N driver (PWM, direction, dead-band, soft-start)
├── quad_decoder.v          4x quadrature encoder decoder (debounced, 100 kHz)
├── soft_limits.v           Software travel limits — no end-stop switches needed
├── uart_tx.v               UART debug TX — streams position at 115200 baud
├── forklift_top.cst        Pin constraints for Tang Nano 20K
├── forklift_spi.py         Standalone Python SPI driver (calibration tool)
└── PROJECT_GUIDE.md        Detailed per-module reference

forklift_ros2/
├── forklift_ros2/
│   └── forklift_driver.py  ROS2 node — bridges /lifter_cmd to FPGA over SPI
├── config/
│   └── forklift_params.yaml  All tunable parameters (positions, gains, SPI config)
├── launch/
│   └── forklift.launch.py  ROS2 launch file
├── package.xml
└── setup.py
```
