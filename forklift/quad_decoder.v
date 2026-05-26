// ============================================================
//  quad_decoder.v  —  Quadrature encoder decoder
//  Motor: GBMQ-GM12BY20  3PPR
//
//  Quadrature decoding: 4x resolution
//  3 PPR * 4 edges * GEAR_RATIO = counts per output shaft rev
//
//  Position is a signed 32-bit counter
//  Positive = forward (fork UP)
//  Negative = reverse (fork DOWN)
// ============================================================
module quad_decoder #(
    parameter FILTER_DEPTH = 4      // debounce flip-flop stages
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enc_a,
    input  wire        enc_b,
    output reg  signed [31:0] position,   // absolute count
    output reg         valid              // pulses 1 clk when position updates
);

// ── Sample tick: 100 kHz (every 270 clocks @ 27MHz) ──────────
localparam SAMPLE_DIV = 270;
reg [8:0]  samp_cnt;
reg        samp_tick;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        samp_cnt  <= 0;
        samp_tick <= 0;
    end else begin
        samp_tick <= 0;
        if (samp_cnt >= SAMPLE_DIV - 1) begin
            samp_cnt  <= 0;
            samp_tick <= 1;
        end else begin
            samp_cnt <= samp_cnt + 9'd1;
        end
    end
end

// ── 4-stage filter for enc_a and enc_b ───────────────────────
reg [FILTER_DEPTH-1:0] filt_a, filt_b;
reg a_clean, b_clean;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        filt_a  <= 0; filt_b  <= 0;
        a_clean <= 0; b_clean <= 0;
    end else if (samp_tick) begin
        filt_a <= {filt_a[FILTER_DEPTH-2:0], enc_a};
        filt_b <= {filt_b[FILTER_DEPTH-2:0], enc_b};
        if (filt_a == {FILTER_DEPTH{1'b1}}) a_clean <= 1;
        if (filt_a == {FILTER_DEPTH{1'b0}}) a_clean <= 0;
        if (filt_b == {FILTER_DEPTH{1'b1}}) b_clean <= 1;
        if (filt_b == {FILTER_DEPTH{1'b0}}) b_clean <= 0;
    end
end

// ── Quadrature state machine ──────────────────────────────────
// Standard Gray-code quadrature decoder
// State = {a_prev, b_prev, a_clean, b_clean}
// +1 on forward transitions, -1 on reverse transitions
reg a_prev, b_prev;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        a_prev   <= 0;
        b_prev   <= 0;
        position <= 0;
        valid    <= 0;
    end else begin
        valid <= 0;
        if (samp_tick) begin
            a_prev <= a_clean;
            b_prev <= b_clean;

            // 4x quadrature decode table
            // {a_prev, b_prev, a_now, b_now}
            case ({a_prev, b_prev, a_clean, b_clean})
                // Forward (fork UP)
                4'b0001: begin position <= position + 1; valid <= 1; end
                4'b0111: begin position <= position + 1; valid <= 1; end
                4'b1110: begin position <= position + 1; valid <= 1; end
                4'b1000: begin position <= position + 1; valid <= 1; end
                // Reverse (fork DOWN)
                4'b0010: begin position <= position - 1; valid <= 1; end
                4'b1011: begin position <= position - 1; valid <= 1; end
                4'b1101: begin position <= position - 1; valid <= 1; end
                4'b0100: begin position <= position - 1; valid <= 1; end
                // No change or error — ignore
                default: ;
            endcase
        end
    end
end

endmodule
