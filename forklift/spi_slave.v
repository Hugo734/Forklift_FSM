// ============================================================
//  spi_slave.v  —  SPI Mode 0 slave (CPOL=0, CPHA=0)
//
//  Protocol (24-bit frame from Jetson Nano):
//  [23:16] CMD byte
//  [15:0]  VALUE (signed 16-bit)
//
//  Commands:
//  CMD 0x01 → SET_TARGET   : set target position (counts)
//  CMD 0x02 → SET_KP       : set Kp gain (fixed-point Q8)
//  CMD 0x03 → SET_KI       : set Ki gain (fixed-point Q8)
//  CMD 0x04 → SET_KD       : set Kd gain (fixed-point Q8)
//  CMD 0x05 → SET_LIMIT_HI : set upper soft limit (counts)
//  CMD 0x06 → SET_LIMIT_LO : set lower soft limit (counts)
//  CMD 0x07 → RESET_POS    : zero the position counter
//  CMD 0x08 → CALIBRATE    : enter calibration mode (read-back)
//
//  MISO returns current position (16-bit) during each frame
// ============================================================
module spi_slave (
    input  wire        clk,
    input  wire        rst_n,

    // SPI pins
    input  wire        sclk,
    input  wire        cs_n,       // active low
    input  wire        mosi,
    output reg         miso,

    // Current position to send back (MISO)
    input  wire signed [15:0] pos_readback,

    // Decoded outputs
    output reg  signed [15:0] target_pos,
    output reg         [7:0]  kp,
    output reg         [7:0]  ki,
    output reg         [7:0]  kd,
    output reg  signed [15:0] limit_hi,
    output reg  signed [15:0] limit_lo,
    output reg                reset_pos,
    output reg                calibrate_mode,
    output reg                new_cmd         // 1-clk pulse on valid command
);

// ── Synchronize SPI signals into FPGA clock domain ───────────
reg [2:0] sclk_r, cs_r;
reg [1:0] mosi_r;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sclk_r <= 3'b000;
        cs_r   <= 3'b111;
        mosi_r <= 2'b00;
    end else begin
        sclk_r <= {sclk_r[1:0], sclk};
        cs_r   <= {cs_r[1:0],   cs_n};
        mosi_r <= {mosi_r[0],   mosi};
    end
end

// Rising/falling edge detection on SCLK
wire sclk_rise = (sclk_r[2:1] == 2'b01);
wire sclk_fall = (sclk_r[2:1] == 2'b10);
wire cs_active = ~cs_r[1];

// ── Shift register: 24-bit frame ─────────────────────────────
reg [4:0]  bit_cnt;       // 0..23
reg [23:0] rx_shift;
reg [15:0] tx_shift;
reg        frame_done;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        bit_cnt    <= 0;
        rx_shift   <= 0;
        tx_shift   <= 0;
        miso       <= 0;
        frame_done <= 0;
    end else begin
        frame_done <= 0;

        if (!cs_active) begin
            // CS deasserted — load MISO with readback for next frame
            bit_cnt  <= 0;
            tx_shift <= pos_readback;
        end else begin
            // Sample MOSI on rising SCLK (Mode 0)
            if (sclk_rise) begin
                rx_shift <= {rx_shift[22:0], mosi_r[1]};
                if (bit_cnt == 5'd23) begin
                    frame_done <= 1;
                    bit_cnt    <= 0;
                end else begin
                    bit_cnt <= bit_cnt + 5'd1;
                end
            end

            // Shift MISO on falling SCLK
            if (sclk_fall) begin
                miso     <= tx_shift[15];
                tx_shift <= {tx_shift[14:0], 1'b0};
            end
        end
    end
end

// ── Command decoder ───────────────────────────────────────────
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        target_pos     <= 0;
        kp             <= 8'd50;   // reasonable defaults
        ki             <= 8'd5;
        kd             <= 8'd10;
        limit_hi       <= 16'd1200; // 1 output rev @ 100:1, 3PPR, 4x
        limit_lo       <= 16'd0;
        reset_pos      <= 0;
        calibrate_mode <= 0;
        new_cmd        <= 0;
    end else begin
        reset_pos <= 0;
        new_cmd   <= 0;

        if (frame_done) begin
            new_cmd <= 1;
            case (rx_shift[23:16])
                8'h01: target_pos     <= rx_shift[15:0];
                8'h02: kp             <= rx_shift[7:0];
                8'h03: ki             <= rx_shift[7:0];
                8'h04: kd             <= rx_shift[7:0];
                8'h05: limit_hi       <= rx_shift[15:0];
                8'h06: limit_lo       <= rx_shift[15:0];
                8'h07: reset_pos      <= 1'b1;
                8'h08: calibrate_mode <= rx_shift[0];
                default: new_cmd      <= 0; // unknown cmd
            endcase
        end
    end
end

endmodule
