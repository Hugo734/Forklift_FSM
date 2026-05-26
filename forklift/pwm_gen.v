// ============================================================
//  pwm_gen.v  —  12-bit PWM + L298N direction logic
//
//  Input:  signed 13-bit pid_out (-4095..+4095)
//  Output: ENA (PWM), IN1, IN2
//
//  L298N truth table:
//  IN1=1 IN2=0 → forward (fork UP)
//  IN1=0 IN2=1 → reverse (fork DOWN)
//  IN1=0 IN2=0 → brake (both low)
//
//  Dead-band: if |pid_out| < DEADBAND, motor brakes
//  This prevents hunting around the setpoint
//
//  Soft-start ramp: PWM duty ramps up over RAMP_CYCLES
//  to prevent current spikes and mechanical shock
// ============================================================
module pwm_gen #(
    parameter DEADBAND   = 13'd20,   // counts below which we brake
    parameter RAMP_STEP  = 12'd4     // duty increment per PWM cycle
)(
    input  wire        clk,
    input  wire        rst_n,

    // From PID (signed: positive=up, negative=down)
    input  wire signed [12:0] pid_out,

    // Soft limit override — cuts motor immediately
    input  wire        limit_hit,

    // L298N outputs
    output reg         ena,
    output reg         in1,
    output reg         in2
);

// ── 12-bit PWM counter @ 27MHz → ~6.6kHz ─────────────────────
reg [11:0] pwm_cnt;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) pwm_cnt <= 0;
    else        pwm_cnt <= pwm_cnt + 12'd1;
end

// ── Extract magnitude and direction from PID output ──────────
wire        pid_dir  = pid_out[12];           // 1=negative=down
wire [11:0] pid_mag  = pid_dir
                       ? (~pid_out[11:0] + 12'd1) // absolute value
                       :   pid_out[11:0];

// ── Soft-start ramp ───────────────────────────────────────────
reg [11:0] ramp_duty;
reg        prev_dir;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        ramp_duty <= 0;
        prev_dir  <= 0;
    end else begin
        // Reset ramp on direction change
        if (pid_dir != prev_dir) begin
            ramp_duty <= 0;
            prev_dir  <= pid_dir;
        end else if (pwm_cnt == 12'hFFF) begin
            // Update ramp once per PWM cycle
            if (ramp_duty < pid_mag) begin
                if (pid_mag - ramp_duty < RAMP_STEP)
                    ramp_duty <= pid_mag;
                else
                    ramp_duty <= ramp_duty + RAMP_STEP;
            end else if (ramp_duty > pid_mag) begin
                if (ramp_duty - pid_mag < RAMP_STEP)
                    ramp_duty <= pid_mag;
                else
                    ramp_duty <= ramp_duty - RAMP_STEP;
            end
        end
    end
end

// ── PWM comparison and output ─────────────────────────────────
wire pwm_sig = (pwm_cnt < ramp_duty);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        ena <= 0; in1 <= 0; in2 <= 0;
    end else begin
        if (limit_hit || (pid_mag < DEADBAND[11:0])) begin
            // Brake: cut motor
            ena <= 0; in1 <= 0; in2 <= 0;
        end else if (!pid_dir) begin
            // Fork UP (forward)
            in1 <= 1; in2 <= 0; ena <= pwm_sig;
        end else begin
            // Fork DOWN (reverse)
            in1 <= 0; in2 <= 1; ena <= pwm_sig;
        end
    end
end

endmodule
