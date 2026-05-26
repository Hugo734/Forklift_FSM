// ============================================================
//  forklift_top.v  —  Scale forklift motor controller
//  Tang Nano 20K (GW2AR-18, 27MHz)
//  Includes UART debug output of encoder position @ 115200
// ============================================================
module forklift_top (
    input  wire clk,

    input  wire enc_a,      // pin 25
    input  wire enc_b,      // pin 26

    input  wire spi_clk,    // pin 32
    input  wire spi_cs_n,   // pin 33
    input  wire spi_mosi,   // pin 34
    output wire spi_miso,   // pin 35

    output wire ena,        // pin 72
    output wire in1,        // pin 28
    output wire in2,        // pin 29

    output wire uart_tx_pin // pin 17 → TX del Tang Nano (via BL616)
);

// ── Power-on reset ────────────────────────────────────────────
reg [16:0] por_cnt;
reg        rst_n;
always @(posedge clk) begin
    if (por_cnt[16]) rst_n <= 1'b1;
    else begin
        rst_n   <= 1'b0;
        por_cnt <= por_cnt + 1'b1;
    end
end

// ── Internal signals ──────────────────────────────────────────
wire signed [31:0] position;
wire               pos_valid;
wire signed [15:0] target_pos;
wire        [7:0]  kp, ki, kd;
wire signed [15:0] limit_hi, limit_lo;
wire               reset_pos_cmd;
wire               calibrate_mode;
wire               new_cmd;
wire signed [12:0] pid_out;
wire               pid_valid;
wire               limit_hit;
wire               at_top, at_bottom;

wire pos_rst_n = rst_n && !reset_pos_cmd;

// ── Auto jog test: 2s UP then 2s DOWN, repeating ─────────────
localparam JOG_CYCLES = 26'd54_000_000; // 2s @ 27MHz
reg [25:0] jog_cnt;
reg        jog_dir; // 0 = UP, 1 = DOWN

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        jog_cnt <= 0;
        jog_dir <= 0;
    end else begin
        if (jog_cnt >= JOG_CYCLES - 1) begin
            jog_cnt <= 0;
            jog_dir <= ~jog_dir;
        end else begin
            jog_cnt <= jog_cnt + 1'b1;
        end
    end
end

wire signed [12:0] motor_cmd = jog_dir ? -13'sd2000 : 13'sd2000;

// ── Quadrature decoder ────────────────────────────────────────
quad_decoder u_quad (
    .clk      (clk),
    .rst_n    (pos_rst_n),
    .enc_a    (enc_a),
    .enc_b    (enc_b),
    .position (position),
    .valid    (pos_valid)
);

// ── SPI slave ─────────────────────────────────────────────────
spi_slave u_spi (
    .clk            (clk),
    .rst_n          (rst_n),
    .sclk           (spi_clk),
    .cs_n           (spi_cs_n),
    .mosi           (spi_mosi),
    .miso           (spi_miso),
    .pos_readback   (position[15:0]),
    .target_pos     (target_pos),
    .kp             (kp),
    .ki             (ki),
    .kd             (kd),
    .limit_hi       (limit_hi),
    .limit_lo       (limit_lo),
    .reset_pos      (reset_pos_cmd),
    .calibrate_mode (calibrate_mode),
    .new_cmd        (new_cmd)
);

// ── PID controller ────────────────────────────────────────────
pid_controller u_pid (
    .clk         (clk),
    .rst_n       (rst_n),
    .kp          (kp),
    .ki          (ki),
    .kd          (kd),
    .current_pos (position),
    .target_pos  (target_pos),
    .pid_out     (pid_out),
    .pid_valid   (pid_valid)
);

// ── Soft limits ───────────────────────────────────────────────
soft_limits u_limits (
    .clk       (clk),
    .rst_n     (rst_n),
    .position  (position),
    .pid_out   (motor_cmd),
    .limit_hi  (limit_hi),
    .limit_lo  (limit_lo),
    .limit_hit (limit_hit),
    .at_top    (at_top),
    .at_bottom (at_bottom)
);

// ── PWM generator ─────────────────────────────────────────────
pwm_gen u_pwm (
    .clk       (clk),
    .rst_n     (rst_n),
    .pid_out   (motor_cmd),
    .limit_hit (1'b0),
    .ena       (ena),
    .in1       (in1),
    .in2       (in2)
);

// ── UART debug: transmite posicion cada 100ms ─────────────────
uart_tx u_uart (
    .clk      (clk),
    .rst_n    (rst_n),
    .position (position),
    .tx       (uart_tx_pin)
);

endmodule
