# Forklift FPGA Motor Controller — Project Guide

## What this project does

This project controls a DC geared motor (the lift mechanism of a scale forklift) using an FPGA. The FPGA reads the motor's position from an encoder, runs a PID control loop to hold or reach a target position, and drives an L298N motor driver chip with PWM signals.

A Jetson Nano (connected via SPI) acts as the "brain" that sends commands — move to position X, change speed gains, set travel limits, etc. The FPGA handles all the real-time motor control.

```
Jetson Nano  ──SPI──►  Tang Nano 20K FPGA  ──PWM──►  L298N  ──►  DC Motor
                              ▲                                        │
                              └──────── Encoder feedback ──────────────┘
```

---

## Hardware

| Component | Details |
|-----------|---------|
| FPGA | Tang Nano 20K — Gowin GW2AR-18, 27 MHz clock |
| Motor | GBMQ-GM12BY20 DC geared motor |
| Encoder | 3 pulses per revolution (PPR), on the motor shaft |
| Gear ratio | ~100:1 (motor turns 100× per output shaft turn) |
| Motor driver | L298N H-bridge |
| Host | Jetson Nano 2GB |

**Counts per output shaft revolution** = 3 PPR × 4 (quadrature) × 100 (gear ratio) = **1200 counts/rev**

---

## File structure

```
forklift_top.v       ← Top-level module: wires everything together
pid_controller.v     ← PID control algorithm
pwm_gen.v            ← Converts PID output to PWM signals for L298N
quad_decoder.v       ← Reads the quadrature encoder
soft_limits.v        ← Software travel limits (replaces physical end-stops)
spi_slave.v          ← Receives commands from the Jetson Nano over SPI
uart_tx.v            ← Sends encoder position over serial for debugging
forklift_top.cst     ← Pin assignments (which FPGA pin connects to what)
forklift_spi.py      ← Python driver to run on the Jetson Nano
```

---

## Module by module

---

### `forklift_top.v` — The top level

This is the "glue" file. It does not contain any logic of its own — it just instantiates all the other modules and connects their signals together.

**Ports (physical pins):**

| Port | Direction | Pin | What it connects to |
|------|-----------|-----|---------------------|
| `clk` | in | 4 | 27 MHz crystal on the board |
| `enc_a`, `enc_b` | in | 25, 26 | Encoder channel A and B |
| `spi_clk/cs_n/mosi/miso` | in/out | 32–35 | SPI bus to Jetson Nano |
| `ena` | out | 72 | L298N enable (PWM signal) |
| `in1`, `in2` | out | 28, 29 | L298N direction pins |
| `uart_tx_pin` | out | 17 | Serial debug output |

**Power-on reset:**
```verilog
reg [16:0] por_cnt;
reg        rst_n;
always @(posedge clk) begin
    if (por_cnt[16]) rst_n <= 1'b1;
    else begin
        rst_n   <= 1'b0;
        por_cnt <= por_cnt + 1'b1;
    end
end
```
When the FPGA powers up, `rst_n` is held low (reset active) for 2^17 / 27MHz ≈ **4.8 milliseconds**. This gives all modules time to initialise cleanly before the motor is allowed to run. After that, `rst_n` goes high and the system starts.

**Auto-jog test (current mode):**
```verilog
localparam JOG_CYCLES = 26'd54_000_000; // 2s @ 27MHz
reg [25:0] jog_cnt;
reg        jog_dir;

// counts to 54 million, then flips direction
wire signed [12:0] motor_cmd = jog_dir ? -13'sd2000 : 13'sd2000;
```
Right now the system ignores the PID and runs the motor at ~49% duty in one direction for 2 seconds, then the other direction for 2 seconds, repeating forever. This is just for testing. When you want to use the PID properly, replace `motor_cmd` with `pid_out` in the `soft_limits` and `pwm_gen` instantiations.

---

### `quad_decoder.v` — Reading the encoder

The motor has an encoder with two signals: **A** and **B**. They pulse as the motor turns. By looking at which one pulses first, you can tell the direction. By reading both edges of both signals (4x decoding), you get 4× the resolution.

**Sampling:**
The encoder signals are sampled at **100 kHz** (every 270 clock cycles at 27MHz). This is fast enough for the motor but slow enough to avoid noise.

**Debounce filter:**
```verilog
reg [FILTER_DEPTH-1:0] filt_a, filt_b; // FILTER_DEPTH = 4
```
Each encoder signal is passed through a 4-stage shift register. The output only changes if all 4 samples agree. This removes electrical noise spikes from the encoder wires.

**Quadrature decode table:**
```verilog
case ({a_prev, b_prev, a_clean, b_clean})
    4'b0001: position <= position + 1;  // forward
    4'b0010: position <= position - 1;  // reverse
    // ... 4 forward patterns, 4 reverse patterns
```
The state machine looks at the previous and current values of A and B together. Specific patterns mean "moved forward one step", others mean "moved backward one step". Illegal transitions (noise or missed pulses) are ignored.

**Output:**
- `position` — signed 32-bit counter. Zero at startup (or after a reset command). Goes up when the fork moves UP, down when it moves DOWN.
- `valid` — pulses high for one clock cycle each time the position updates.

**To change something:** If you want to zero the position at a specific point, send `CMD_RESET_POS` via SPI (the SPI slave handles this and briefly resets the decoder).

---

### `pid_controller.v` — The control algorithm

PID stands for **Proportional, Integral, Derivative**. It is the algorithm that decides how hard to drive the motor to reach and hold a target position.

**How it works:**
```
error = target_position - current_position

P term = Kp × error                    (responds to current error)
I term = Ki × sum_of_all_past_errors   (corrects long-term offset)
D term = Kd × (error - previous_error) (dampens oscillation)

output = P + I + D
```

**Update rate:** Every **1 millisecond** (27,000 clock cycles at 27MHz). This is the PID tick.

**Fixed-point math (Q8 format):**
The gains Kp, Ki, Kd are integers from 0–255, but they represent real numbers divided by 256:
- Kp = 50 → real gain = 50/256 ≈ 0.195
- Ki = 5  → real gain = 5/256 ≈ 0.020
- Kd = 10 → real gain = 10/256 ≈ 0.039

This avoids floating-point math in the FPGA, which would be expensive.

**Anti-windup:**
```verilog
if (integral >  $signed(IMAX)) integral =  $signed(IMAX);
if (integral < -$signed(IMAX)) integral = -$signed(IMAX);
```
The integral term is clamped at ±500,000. Without this, if the motor is stuck against a limit for a long time, the integrator would grow huge and cause a violent lurch when the motor is freed.

**Output:** `pid_out` — a signed 13-bit value from **-4095 to +4095**. Positive means drive UP, negative means drive DOWN. The magnitude is the duty cycle intensity.

**To tune the PID:** Send new Kp, Ki, Kd values via SPI commands `0x02`, `0x03`, `0x04`. Start with Kp, set Ki and Kd to zero, then add them gradually. If the fork oscillates, reduce Kp or increase Kd.

---

### `pwm_gen.v` — Driving the L298N

This module converts the signed PID output into the three signals the L298N motor driver needs.

**L298N truth table:**

| IN1 | IN2 | ENA | Motor action |
|-----|-----|-----|--------------|
| 1 | 0 | PWM | Fork goes UP |
| 0 | 1 | PWM | Fork goes DOWN |
| 0 | 0 | 0 | Motor brakes (short circuit across motor) |

**PWM frequency:** 27MHz / 4096 = **~6.6 kHz**. The duty cycle of ENA determines how much power reaches the motor.

**Dead-band:**
```verilog
parameter DEADBAND = 13'd20
```
If the PID output is less than 20 (in either direction), the motor brakes instead of running. This prevents the motor from constantly hunting (buzzing back and forth) when it is close to the target.

**Soft-start ramp:**
```verilog
parameter RAMP_STEP = 12'd4
```
When the motor starts (or changes direction), the duty cycle does not jump immediately to the target — it ramps up by 4 counts per PWM cycle. This prevents:
- Current spikes that could trip a fuse or damage the L298N
- Mechanical shock on the gearbox

Ramp time to reach duty=2000: 2000 ÷ 4 steps × 151µs/cycle ≈ **75 ms**. This is what makes direction changes feel smooth.

**To change something:** Increase `RAMP_STEP` for faster acceleration, decrease it for smoother starts. Increase `DEADBAND` if the motor buzzes at the setpoint.

---

### `soft_limits.v` — Travel limits without physical switches

This module prevents the fork from being driven past its physical travel range, without needing real end-stop switches. The limits are set in encoder counts via SPI.

```verilog
limit_hit <= (position >= pos_hi && going_up) ||
             (position <= pos_lo && going_down);
```

It only cuts the motor if the fork is **at** a limit **and** the PID is trying to push it further in the wrong direction. The motor can still move away from a limit freely.

**Default limits:**
- `limit_lo = 0` (bottom of travel)
- `limit_hi = 1200` (top of travel — one full output shaft revolution at 100:1)

**Important:** In the current test mode, `limit_hit` is forced to `1'b0` (disabled) so the motor can move in both directions from the starting position of zero. When you switch to PID mode, re-enable it by passing `limit_hit` back to `pwm_gen`.

**To set limits:** Use the calibration routine in `forklift_spi.py`, or send SPI commands `0x05` (limit high) and `0x06` (limit low) manually.

---

### `spi_slave.v` — Receiving commands from the Jetson Nano

SPI (Serial Peripheral Interface) is a simple 4-wire protocol. The Jetson Nano is the master; the FPGA is the slave.

**SPI Mode 0:** Data is sent MSB first. MOSI is sampled on the rising edge of SCLK. MISO shifts out on the falling edge.

**Frame format (24 bits per transaction):**
```
[23:16]  Command byte  (8 bits)
[15:0]   Value         (16 bits, signed)
```

**Commands:**

| Hex | Name | What it does |
|-----|------|--------------|
| `0x01` | SET_TARGET | Move fork to this encoder count |
| `0x02` | SET_KP | Set proportional gain |
| `0x03` | SET_KI | Set integral gain |
| `0x04` | SET_KD | Set derivative gain |
| `0x05` | SET_LIMIT_HI | Set upper travel limit |
| `0x06` | SET_LIMIT_LO | Set lower travel limit |
| `0x07` | RESET_POS | Zero the encoder counter now |
| `0x08` | CALIBRATE | Enter/exit calibration mode |

**MISO readback:** Every frame the FPGA sends the current 16-bit position back over MISO while it is receiving the command. So every SPI transaction is both a command and a position read at the same time.

**Clock domain crossing:** The SPI clock comes from the Jetson Nano, which is asynchronous to the FPGA's 27MHz clock. The module uses a 3-stage shift register to safely synchronise each SPI signal into the FPGA clock domain and detect rising/falling edges:
```verilog
wire sclk_rise = (sclk_r[2:1] == 2'b01);
wire sclk_fall = (sclk_r[2:1] == 2'b10);
```

---

### `uart_tx.v` — Serial debug output

This module sends the current encoder position over UART (serial) every **100 milliseconds**, so you can monitor the motor in real time without needing the Jetson Nano. Connect a USB-serial adapter to pin 17 and open a terminal at **115200 baud**.

**Output format:**
```
POS:+00342
POS:+00341
POS:-00005
```

**How it works:**
1. Every 100ms a `send_tick` fires
2. The position integer is converted to ASCII digits
3. A byte-by-byte UART transmitter sends the string at 115200 baud (234 clock cycles per bit at 27MHz)
4. Each byte is sent with a start bit, 8 data bits, and a stop bit (standard 8N1 format)

**To change baud rate:** Change the `BAUD` parameter. The `CLKS_PER_BIT` value is calculated automatically.

---

### `forklift_spi.py` — Python driver (runs on Jetson Nano)

This is the software side. It wraps the SPI protocol into easy Python calls.

**Key methods:**

```python
ctrl = ForkliftSPI()          # opens /dev/spidev0.0 at 500kHz

ctrl.set_gains(50, 5, 10)     # Kp=50, Ki=5, Kd=10
ctrl.set_limits(0, 600)       # travel from count 0 to 600
ctrl.set_target(300)          # move to count 300 (mid-travel)
pos = ctrl.read_position()    # read current position
ctrl.reset_position()         # zero the counter here
ctrl.calibrate()              # interactive calibration routine
ctrl.monitor(interval_s=0.1, duration_s=10.0)  # print position for 10s
```

**SPI wiring (Jetson 40-pin header → Tang Nano 20K):**

| Jetson Pin | Signal | Tang Nano Pin |
|-----------|--------|---------------|
| 19 | MOSI | 34 |
| 21 | MISO | 35 |
| 23 | SCLK | 32 |
| 24 | CS0 | 33 |
| GND | GND | GND |

**Internal `_send()` method:**
```python
frame = [cmd, (val_u16 >> 8) & 0xFF, val_u16 & 0xFF]
resp  = self.spi.xfer2(frame)
```
It packs the 24-bit frame as 3 bytes, sends them, and reads the 3 response bytes back. The position is in bytes 1 and 2 of the response.

---

## Signal flow summary

```
Encoder A/B
    │
    ▼
quad_decoder  ──► position (32-bit signed count)
                      │
                      ├──► spi_slave  ──► MISO (sent back to Jetson)
                      │
                      ├──► pid_controller ◄── target_pos, Kp, Ki, Kd (from SPI)
                      │         │
                      │         ▼ pid_out (signed 13-bit)
                      │
                      └──► soft_limits ◄── limit_hi, limit_lo (from SPI)
                                │
                                ▼ limit_hit
                                │
                           pwm_gen
                                │
                    ┌───────────┼───────────┐
                    ▼           ▼           ▼
                   ENA         IN1         IN2
                    │           │           │
                    └───────────┴───────────┘
                                │
                            L298N chip
                                │
                             DC Motor
```

---

## How to switch from test mode to real PID mode

Right now `forklift_top.v` overrides the PID with a fixed jog command. To use the PID:

**In `forklift_top.v`, remove the jog block:**
```verilog
// DELETE these lines:
localparam JOG_CYCLES = 26'd54_000_000;
reg [25:0] jog_cnt;
reg        jog_dir;
always @(posedge clk or negedge rst_n) begin ... end
wire signed [12:0] motor_cmd = jog_dir ? -13'sd2000 : 13'sd2000;
```

**Then change the two module connections back to use `pid_out`:**
```verilog
// In soft_limits instantiation:
.pid_out   (pid_out),      // was motor_cmd

// In pwm_gen instantiation:
.pid_out   (pid_out),      // was motor_cmd
.limit_hit (limit_hit),    // was 1'b0
```

Then run `forklift_spi.py` from the Jetson to send target positions and gains.

---

## Key numbers to remember

| Parameter | Value | Meaning |
|-----------|-------|---------|
| Clock | 27 MHz | FPGA system clock |
| PID update rate | 1 ms | How often the control loop runs |
| PWM frequency | ~6.6 kHz | Motor drive frequency |
| Counts per rev | 1200 | At 100:1 gear ratio, 3PPR, 4x quadrature |
| Default Kp | 50 (= 0.195 real) | Proportional gain |
| Default Ki | 5 (= 0.020 real) | Integral gain |
| Default Kd | 10 (= 0.039 real) | Derivative gain |
| SPI speed | 500 kHz | Jetson ↔ FPGA communication |
| UART baud | 115200 | Debug serial output |
| Jog duty (test) | 2000/4095 = 49% | Current test motor power |
