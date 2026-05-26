// ============================================================
//  soft_limits.v  —  Software travel limits
//
//  Enforces min/max position counts — no physical switches needed
//
//  Behavior:
//  - If position >= limit_hi AND motor trying to go UP  → cut
//  - If position <= limit_lo AND motor trying to go DOWN → cut
//  - LED indicators for top/bottom limit active
//
//  The limits are set via SPI and stored in registers.
//  Default: 0 (bottom) to COUNTS_PER_REV (top)
//
//  COUNTS_PER_REV = 3 PPR * 4 (quadrature) * GEAR_RATIO
//  Set GEAR_RATIO via parameter, tune after calibration.
// ============================================================
module soft_limits #(
    parameter GEAR_RATIO      = 100,         // tune after measuring
    parameter PPR             = 3,           // pulses per rev (motor)
    parameter QUAD_MUL        = 4,           // 4x quadrature
    parameter COUNTS_PER_REV  = PPR * QUAD_MUL * GEAR_RATIO  // = 1200 @ 100:1
)(
    input  wire        clk,
    input  wire        rst_n,

    // Current position from decoder
    input  wire signed [31:0] position,

    // PID output direction
    input  wire signed [12:0] pid_out,

    // Configurable limits from SPI
    input  wire signed [15:0] limit_hi,
    input  wire signed [15:0] limit_lo,

    // Output
    output reg         limit_hit,   // cut motor immediately
    output reg         at_top,      // status LEDs / readback
    output reg         at_bottom
);

wire signed [31:0] pos_hi = {{16{limit_hi[15]}}, limit_hi};
wire signed [31:0] pos_lo = {{16{limit_lo[15]}}, limit_lo};

wire going_up   = !pid_out[12] && (pid_out[11:0] > 0);
wire going_down =  pid_out[12] && (pid_out[11:0] > 0);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        limit_hit <= 0;
        at_top    <= 0;
        at_bottom <= 0;
    end else begin
        at_top    <= (position >= pos_hi);
        at_bottom <= (position <= pos_lo);

        // Cut motor if at limit AND trying to go further that way
        limit_hit <= (position >= pos_hi && going_up) ||
                     (position <= pos_lo && going_down);
    end
end

endmodule
