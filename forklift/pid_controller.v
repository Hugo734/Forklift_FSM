// ============================================================
//  pid_controller.v  —  Fixed-point PID  (Q8 gains)
//
//  Position PID: output = Kp*e + Ki*sum(e) + Kd*de/dt
//
//  Fixed-point format:
//    Gains Kp, Ki, Kd are Q8 (8 fractional bits)
//    e.g. Kp=50 → real gain = 50/256 ≈ 0.195
//
//  Update rate: every PID_DIV clocks (~1ms @ 27MHz)
//  Output: signed 12-bit PWM duty (-4095 to +4095)
//    Positive → forward (fork UP)
//    Negative → reverse (fork DOWN)
//
//  Anti-windup: integrator clamps at ±IMAX
// ============================================================
module pid_controller #(
    parameter PID_DIV  = 27_000,    // 1ms update rate @ 27MHz
    parameter IMAX     = 32'd500_000 // integrator clamp (Q8 units)
)(
    input  wire        clk,
    input  wire        rst_n,

    // Gains (Q8 fixed-point)
    input  wire [7:0]  kp,
    input  wire [7:0]  ki,
    input  wire [7:0]  kd,

    // Position
    input  wire signed [31:0] current_pos,
    input  wire signed [15:0] target_pos,

    // Output: signed PWM magnitude + direction
    output reg  signed [12:0] pid_out,  // -4095..+4095
    output reg                pid_valid // 1-clk pulse on new output
);

// ── PID tick: every 1ms ───────────────────────────────────────
reg [17:0] div_cnt;
reg        pid_tick;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        div_cnt  <= 0;
        pid_tick <= 0;
    end else begin
        pid_tick <= 0;
        if (div_cnt >= PID_DIV - 1) begin
            div_cnt  <= 0;
            pid_tick <= 1;
        end else begin
            div_cnt <= div_cnt + 18'd1;
        end
    end
end

// ── PID state ─────────────────────────────────────────────────
reg signed [31:0] integral;
reg signed [31:0] prev_error;
reg signed [31:0] error;

// Extended precision for PID math
reg signed [47:0] p_term;
reg signed [47:0] i_term;
reg signed [47:0] d_term;
reg signed [47:0] pid_sum;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        integral   <= 0;
        prev_error <= 0;
        pid_out    <= 0;
        pid_valid  <= 0;
    end else begin
        pid_valid <= 0;

        if (pid_tick) begin
            // Error: target - current (extend target to 32-bit)
            error = {{16{target_pos[15]}}, target_pos} - current_pos;

            // P term: Kp * e  (Q8)
            p_term = $signed({40'b0, kp}) * error;

            // I term: Ki * sum(e)  with anti-windup clamp
            integral = integral + error;
            if (integral >  $signed(IMAX)) integral =  $signed(IMAX);
            if (integral < -$signed(IMAX)) integral = -$signed(IMAX);
            i_term = $signed({40'b0, ki}) * integral;

            // D term: Kd * (e - e_prev)
            d_term = $signed({40'b0, kd}) * (error - prev_error);
            prev_error = error;

            // Sum and scale down by Q8 (>>8)
            pid_sum = (p_term + i_term + d_term) >>> 8;

            // Clamp to 13-bit signed output (-4095..+4095)
            if      (pid_sum >  13'sd4095) pid_out <= 13'sd4095;
            else if (pid_sum < -13'sd4095) pid_out <= -13'sd4095;
            else                           pid_out <= pid_sum[12:0];

            pid_valid <= 1;
        end
    end
end

endmodule
