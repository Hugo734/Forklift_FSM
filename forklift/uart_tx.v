// ============================================================
//  uart_tx.v  —  Simple UART transmitter
//  115200 baud @ 27MHz
//  Transmite posicion del encoder cada ~100ms
// ============================================================
module uart_tx #(
    parameter CLK_HZ  = 27_000_000,
    parameter BAUD    = 115200,
    parameter CLKS_PER_BIT = CLK_HZ / BAUD  // = 234
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire signed [31:0] position,  // del quad_decoder
    output reg         tx                // pin UART TX del Tang Nano
);

// ── Transmit cada 100ms ───────────────────────────────────────
localparam REPORT_DIV = 27_000_000 / 10; // 100ms
reg [24:0] report_cnt;
reg        send_tick;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        report_cnt <= 0;
        send_tick  <= 0;
    end else begin
        send_tick <= 0;
        if (report_cnt >= REPORT_DIV - 1) begin
            report_cnt <= 0;
            send_tick  <= 1;
        end else begin
            report_cnt <= report_cnt + 25'd1;
        end
    end
end

// ── Formatear posicion como ASCII: "POS:XXXXX\r\n" ──────────
// Maximo 12 bytes: "POS:" + 6 digits + sign + "\r\n"
reg [7:0]  tx_buf [0:11];
reg [3:0]  tx_len;
reg [3:0]  tx_idx;
reg [8:0]  bit_cnt;   // contador de bits dentro del byte
reg [3:0]  bit_idx;   // bit actual (0=start, 1-8=data, 9=stop)
reg        busy;

// Conversion de posicion a ASCII
reg signed [31:0] pos_latch;
reg [31:0] pos_abs;
reg        pos_neg;
reg [7:0]  digits [0:5];
reg [2:0]  ndigits;
integer i;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx      <= 1; // idle high
        busy    <= 0;
        tx_idx  <= 0;
        bit_cnt <= 0;
        bit_idx <= 0;
        tx_len  <= 0;
    end else begin

        // Latch y formatear cuando llega send_tick y no estamos ocupados
        if (send_tick && !busy) begin
            pos_latch = position;
            pos_neg   = pos_latch[31]; // signo
            pos_abs   = pos_neg ? (~pos_latch + 1) : pos_latch;

            // Convertir a digitos ASCII (hasta 6 digitos)
            ndigits = 0;
            if (pos_abs == 0) begin
                digits[0] = 8'h30; // '0'
                ndigits   = 1;
            end else begin
                for (i = 0; i < 6; i = i + 1) begin
                    if (pos_abs > 0) begin
                        digits[i] = 8'h30 + (pos_abs % 10);
                        pos_abs   = pos_abs / 10;
                        ndigits   = ndigits + 1;
                    end
                end
            end

            // Construir buffer: "POS:" + signo + digitos invertidos + "\r\n"
            tx_buf[0] = 8'h50; // 'P'
            tx_buf[1] = 8'h4F; // 'O'
            tx_buf[2] = 8'h53; // 'S'
            tx_buf[3] = 8'h3A; // ':'
            tx_buf[4] = pos_neg ? 8'h2D : 8'h2B; // '-' or '+'

            // digitos en orden correcto (estaban invertidos)
            for (i = 0; i < 6; i = i + 1) begin
                if (i < ndigits)
                    tx_buf[5 + (ndigits - 1 - i)] = digits[i];
            end

            tx_len  = 5 + ndigits;
            tx_buf[tx_len]     = 8'h0D; // '\r'
            tx_buf[tx_len + 1] = 8'h0A; // '\n'
            tx_len  = tx_len + 2;

            tx_idx  <= 0;
            bit_idx <= 0;
            bit_cnt <= 0;
            busy    <= 1;
        end

        // Transmitir byte a byte
        if (busy) begin
            if (bit_cnt >= CLKS_PER_BIT - 1) begin
                bit_cnt <= 0;
                bit_idx <= bit_idx + 1;

                if (bit_idx == 0) begin
                    tx <= 0; // start bit
                end else if (bit_idx >= 1 && bit_idx <= 8) begin
                    tx <= tx_buf[tx_idx][bit_idx - 1]; // data bits
                end else begin
                    tx <= 1; // stop bit
                    if (tx_idx >= tx_len - 1) begin
                        busy    <= 0; // fin de trama
                        tx_idx  <= 0;
                    end else begin
                        tx_idx  <= tx_idx + 1;
                        bit_idx <= 0;
                    end
                end
            end else begin
                bit_cnt <= bit_cnt + 9'd1;
            end
        end
    end
end

endmodule
