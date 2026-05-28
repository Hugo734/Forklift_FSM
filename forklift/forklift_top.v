// ============================================================
//  forklift_top.v  —  MG90S smooth button control
//  Tang Nano 20K (GW2AR-18, 27 MHz)
//  btn_cw  held → ramps toward 180° (positive)
//  btn_ccw held → ramps toward   0° (negative)
//  released      → holds position
// ============================================================
module forklift_top (
    input  wire clk,        // pin 4  — 27 MHz
    input  wire btn_cw,     // pin 88 — active low, positive direction
    input  wire btn_ccw,    // pin 87 — active low, negative direction
    output wire servo_pwm   // pin 71 — servo signal
);

// ── PWM period — 50 Hz, 20 ms @ 27 MHz ───────────────────────
localparam PERIODO   = 540_000;  // 20 ms

// ── Servo pulse range (MG90S: 1.0 ms – 2.0 ms) ───────────────
localparam PW_MIN    =  27_000;  // 1.0 ms — servo 0°
localparam PW_MID    =  40_500;  // 1.5 ms — servo 90°  (start)
localparam PW_MAX    =  54_000;  // 2.0 ms — servo 180°

// ── Ramp: cycles added/removed per 20 ms tick ────────────────
// 540 → full range in ~1 s   (responsive)
// 270 → full range in ~2 s   (smoother)
localparam RAMP_STEP =     540;

// ── 2-stage button synchronizer (activo bajo → presionado = 1)
reg cw_s0,  cw_s1;
reg ccw_s0, ccw_s1;
always @(posedge clk) begin
    cw_s0  <= ~btn_cw;   cw_s1  <= cw_s0;
    ccw_s0 <= ~btn_ccw;  ccw_s1 <= ccw_s0;
end

// ── PWM counter ───────────────────────────────────────────────
reg [19:0] contador = 20'd0;
always @(posedge clk)
    contador <= (contador >= PERIODO - 1) ? 20'd0 : contador + 1'b1;

// Fires once per 20 ms period — ramp update tick
wire ramp_tick = (contador == 20'd0);

// ── Smooth ramp controller ────────────────────────────────────
reg [19:0] pulse_width = PW_MID;

always @(posedge clk) begin
    if (ramp_tick) begin
        if (cw_s1 && !ccw_s1) begin
            // Ramp up — clamp at PW_MAX
            pulse_width <= ((PW_MAX - pulse_width) >= RAMP_STEP) ?
                            pulse_width + RAMP_STEP : PW_MAX;

        end else if (ccw_s1 && !cw_s1) begin
            // Ramp down — clamp at PW_MIN
            pulse_width <= ((pulse_width - PW_MIN) >= RAMP_STEP) ?
                            pulse_width - RAMP_STEP : PW_MIN;
        end
        // both or neither → hold position (no change)
    end
end

// ── PWM output ────────────────────────────────────────────────
assign servo_pwm = (contador < pulse_width);

endmodule
