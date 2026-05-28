// ============================================================
//  forklift_top.v  —  Tower Pro MG90S diagnosis (tope a tope)
//  Tang Nano 20K (GW2AR-18, 27 MHz)
// ============================================================
module forklift_top (
    input  wire clk,
    output wire servo_pwm
);
// ── Power-on reset (~4.8 ms) ─────────────────────────────────
reg [16:0] por_cnt = 17'd0;
reg        rst_n   = 1'b0;
always @(posedge clk) begin
    if (por_cnt[16]) rst_n <= 1'b1;
    else begin
        rst_n   <= 1'b0;
        por_cnt <= por_cnt + 1'b1;
    end
end
// ── Servo PWM — 50 Hz, 20 ms period @ 27 MHz ─────────────────
localparam [19:0] PERIOD     = 20'd539_999;
localparam [19:0] PW_OUT_0   = 20'd27_000;  // 1.00 ms → 0°
localparam [19:0] PW_OUT_360 = 20'd54_000;  // 2.00 ms → 180°
localparam [26:0] HOLD       = 27'd27_000_000;  // 1 s por paso
// ── PWM counter ───────────────────────────────────────────────
reg [19:0] pwm_cnt = 20'd0;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) pwm_cnt <= 20'd0;
    else        pwm_cnt <= (pwm_cnt == PERIOD) ? 20'd0 : pwm_cnt + 1'b1;
end
// ── Position sequencer ────────────────────────────────────────
reg [ 0:0] seq_step    = 1'd0;
reg [26:0] seq_cnt     = 27'd0;
reg [19:0] pulse_width = PW_OUT_0;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        seq_step    <= 1'd0;
        seq_cnt     <= 27'd0;
        pulse_width <= PW_OUT_0;
    end else begin
        if (seq_cnt == HOLD - 1) begin
            seq_cnt  <= 27'd0;
            seq_step <= ~seq_step;
        end else begin
            seq_cnt <= seq_cnt + 1'b1;
        end
        case (seq_step)
            1'd0: pulse_width <= PW_OUT_0;
            1'd1: pulse_width <= PW_OUT_360;
            default: pulse_width <= PW_OUT_0;
        endcase
    end
end
// ── PWM output ────────────────────────────────────────────────
assign servo_pwm = (pwm_cnt < pulse_width);
endmodule