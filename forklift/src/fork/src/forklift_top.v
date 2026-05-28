// ============================================================
//  forklift_top.v  —  Servo button control
//  Tang Nano 20K (GW2AR-18, 27 MHz)
//  Tower Pro MG90S — full range 500 µs to 2500 µs
//  S1 (btn_cw,  pin 88, active-low): hold → clockwise
//  S2 (btn_ccw, pin 87, active-low): hold → counter-clockwise
//  Release: holds current position
// ============================================================
module forklift_top (
    input  wire clk,
    input  wire btn_cw,
    input  wire btn_ccw,
    output wire servo_pwm
);

// Power-on reset (~4.8 ms)
reg [16:0] por_cnt = 17'd0;
reg        rst_n   = 1'b0;
always @(posedge clk) begin
    if (por_cnt[16]) rst_n <= 1'b1;
    else begin rst_n <= 1'b0; por_cnt <= por_cnt + 1'b1; end
end

// 50 Hz PWM — 540,000 cycles @ 27 MHz
// MG90S full mechanical range:
//   500 µs  = 13,500 cycles  →  0°
//   1500 µs = 40,500 cycles  →  90° (centre)
//   2500 µs = 67,500 cycles  →  180°
localparam [19:0] PERIOD = 20'd539_999;
localparam [19:0] PW_MIN = 20'd13_500;
localparam [19:0] PW_MAX = 20'd67_500;
localparam [19:0] CENTER = 20'd40_500;
localparam [19:0] STEP   = 20'd270;

// PWM counter
reg [19:0] pwm_cnt = 20'd0;
wire period_end = (pwm_cnt == PERIOD);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) pwm_cnt <= 20'd0;
    else        pwm_cnt <= period_end ? 20'd0 : pwm_cnt + 1'b1;
end

// Servo position — starts at 90°
reg [19:0] pulse_width = CENTER;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pulse_width <= CENTER;
    end else if (period_end) begin
        if (!btn_cw && btn_ccw) begin           // S1 pressed, S2 not — move CW
            if (pulse_width < PW_MAX)
                pulse_width <= pulse_width + STEP;
        end else if (btn_cw && !btn_ccw) begin  // S2 pressed, S1 not — move CCW
            if (pulse_width > PW_MIN)
                pulse_width <= pulse_width - STEP;
        end
        // both or neither pressed — hold position
    end
end

assign servo_pwm = (pwm_cnt < pulse_width);

endmodule
