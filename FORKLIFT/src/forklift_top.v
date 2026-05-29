// ============================================================
//  forklift_top.v  —  MG90S servo ±90° via S1 / S2
//  Tang Nano 20K (GW2AR-18, 27 MHz)
//  S1 (pin 87) held → ramps toward +90°
//  S2 (pin 88) held → ramps toward –90°
//  released         → holds position
// ============================================================
module forklift_top (
    input  wire clk,        // pin 4  — 27 MHz
    input  wire btn_s1,     // pin 87 — active low, S1 → +90°
    input  wire btn_s2,     // pin 88 — active low, S2 → –90°
    output wire servo_pwm   // pin 71 — servo signal
);

// 50 Hz PWM period (20 ms at 27 MHz)
localparam PERIODO   = 540_000;

// MG90S pulse range: 1.0 ms = –90°, 1.5 ms = 0° (center), 2.0 ms = +90°
localparam PW_MIN    =  27_000;  // 1.0 ms
localparam PW_MID    =  40_500;  // 1.5 ms  ← servo starts here
localparam PW_MAX    =  54_000;  // 2.0 ms

// Ramp step per 20 ms tick — full ±90° range in ~1 s
localparam RAMP_STEP =     540;

// 2-stage synchronizer (active-low buttons → pressed = 1)
reg s1_r0, s1_r1;
reg s2_r0, s2_r1;
always @(posedge clk) begin
    s1_r0 <= ~btn_s1;  s1_r1 <= s1_r0;
    s2_r0 <= ~btn_s2;  s2_r1 <= s2_r0;
end

// PWM counter
reg [19:0] cnt = 20'd0;
always @(posedge clk)
    cnt <= (cnt >= PERIODO - 1) ? 20'd0 : cnt + 1'b1;

wire tick = (cnt == 20'd0);  // one pulse per 20 ms period

// Servo position register — starts at center (0°)
reg [19:0] pw = PW_MID;

always @(posedge clk) begin
    if (tick) begin
        if (s1_r1 && !s2_r1) begin
            // S1: ramp toward +90°
            if (pw <= PW_MAX - RAMP_STEP)
                pw <= pw + RAMP_STEP;
            else
                pw <= PW_MAX;
        end else if (s2_r1 && !s1_r1) begin
            // S2: ramp toward –90°
            if (pw >= PW_MIN + RAMP_STEP)
                pw <= pw - RAMP_STEP;
            else
                pw <= PW_MIN;
        end
        // both or neither → hold position
    end
end

// PWM output
assign servo_pwm = (cnt < pw);

endmodule
