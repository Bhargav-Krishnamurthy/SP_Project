`timescale 1ns / 1ps

// RealDigital Boolean board top-level.
// Board: xc7s50csga324-1, 100 MHz oscillator on F14.
// btn[0] = start (press once)
// btn[1] = reset (hold while resetting)
// led[0] = busy (processing + UART transfer)
// led[1] = done (latched after complete UART transfer)
// led[2] = UART streaming active
// UART_txd = 115200 baud, 8-N-1, binary RGB stream.
//
// The image is held in ONE 24-bit BRAM and overwritten in place during
// the final pass. This avoids the 384 KiB dual-image storage problem.
module underwater_fpga_top (
    input  wire        clk,
    input  wire [3:0]  btn,
    output wire [15:0] led,
    output wire        UART_txd
);

    // Two-flop synchronizers for the active-high push buttons.
    // Initial values are supported by the Xilinx FPGA configuration flow.
    reg [1:0] start_sync = 2'b00;
    reg [1:0] reset_sync = 2'b00;
    reg       start_d    = 1'b0;

    always @(posedge clk) begin
        start_sync <= {start_sync[0], btn[0]};
        reset_sync <= {reset_sync[0], btn[1]};
        start_d    <= start_sync[1];
    end

    wire start_pulse = start_sync[1] & ~start_d;
    wire rst         = reset_sync[1];

    wire busy;
    wire done;
    wire streaming;

    assign led[0]    = busy;
    assign led[1]    = done;
    assign led[2]    = streaming;
    assign led[15:3] = 13'b0;

    wb_image_uart_engine #(
        .WIDTH(256),
        .HEIGHT(256),
        .ADDR_WIDTH(16),
        .CLK_HZ(100000000),
        .UART_BAUD(115200)
    ) engine (
        .clk(clk),
        .rst(rst),
        .start(start_pulse),
        .busy(busy),
        .done(done),
        .streaming(streaming),
        .uart_txd(UART_txd)
    );

endmodule


module wb_image_uart_engine #(
    parameter integer WIDTH      = 256,
    parameter integer HEIGHT     = 256,
    parameter integer ADDR_WIDTH = 16,
    parameter integer CLK_HZ     = 100000000,
    parameter integer UART_BAUD  = 115200
)(
    input  wire clk,
    input  wire rst,
    input  wire start,
    output reg  busy,
    output reg  done,
    output reg  streaming,
    output wire uart_txd
);

    localparam integer DEPTH = WIDTH * HEIGHT;

    localparam [4:0]
        IDLE       = 5'd0,
        P1_REQ     = 5'd1,
        P1_WAIT    = 5'd2,
        P1_USE     = 5'd3,
        P1_CALC    = 5'd4,
        P2_REQ     = 5'd5,
        P2_WAIT    = 5'd6,
        P2_USE     = 5'd7,
        P2_CALC    = 5'd8,
        P3_REQ     = 5'd9,
        P3_WAIT    = 5'd10,
        P3_USE     = 5'd11,
        P3_WRITE   = 5'd12,
        U_REQ      = 5'd13,
        U_WAIT     = 5'd14,
        U_LATCH    = 5'd15,
        U_R_START  = 5'd16,
        U_R_WAIT   = 5'd17,
        U_G_START  = 5'd18,
        U_G_WAIT   = 5'd19,
        U_B_START  = 5'd20,
        U_B_WAIT   = 5'd21,
        FINISHED   = 5'd22;

    reg [4:0] state;

    // One 24-bit image BRAM only. Initialized by image.mem.
    // Include image.mem in the Vivado project as a memory initialization file.
    (* ram_style = "block" *)
    reg [23:0] image_mem [0:DEPTH-1];

    reg [ADDR_WIDTH-1:0] pixel_index;
    reg [ADDR_WIDTH-1:0] rd_addr;
    reg [ADDR_WIDTH-1:0] write_addr;
    reg [23:0] rd_data;
    reg [23:0] write_data;

    reg [31:0] sum_r, sum_g, sum_b;
    reg [31:0] sum_rc, sum_gc, sum_bc;

    reg [7:0] mean_r, mean_g, mean_b;
    reg [7:0] mean_rc, mean_gc, mean_bc;
    reg [7:0] target_mean;

    reg [ADDR_WIDTH-1:0] uart_index;
    reg [23:0] uart_pixel;

    reg [7:0] tx_data;
    reg       tx_start;
    wire      tx_busy;
    wire      tx_done;

    integer init_i;

    initial begin
        $readmemh("image.mem", image_mem);
    end

    // Synchronous BRAM read port and independent write port.
    // During P3_WRITE, rd_addr is deliberately different from write_addr.
    // The final pixel sets rd_addr to zero before its write, avoiding
    // same-address read/write ambiguity on the final in-place update.
    always @(posedge clk) begin
        rd_data <= image_mem[rd_addr];

        if (state == P3_WRITE)
            image_mem[write_addr] <= write_data;
    end

    // Red compensation:
    // Rc = R + ((meanG - meanR) * (255-R) * G) / 65025
    // Signed 32-bit intermediates are enough for this bounded expression.
    function [7:0] compensate_red;
        input [7:0] R;
        input [7:0] G;
        input [7:0] meanR;
        input [7:0] meanG;
        reg signed [31:0] delta;
        reg signed [31:0] a, bb, cc;
        reg signed [31:0] numerator;
        reg signed [31:0] correction;
        reg signed [31:0] result;
        begin
            delta = $signed({1'b0, meanG}) - $signed({1'b0, meanR});
            a = delta;
            bb = 255 - R;
            cc = G;
            numerator = a * bb * cc;
            correction = numerator / 65025;
            result = R + correction;

            if (result < 0)
                compensate_red = 8'd0;
            else if (result > 255)
                compensate_red = 8'd255;
            else
                compensate_red = result[7:0];
        end
    endfunction

    // Blue compensation:
    // Bc = B + ((meanG - meanB) * (255-B) * G) / 65025
    function [7:0] compensate_blue;
        input [7:0] B;
        input [7:0] G;
        input [7:0] meanB;
        input [7:0] meanG;
        reg signed [31:0] delta;
        reg signed [31:0] a, bb, cc;
        reg signed [31:0] numerator;
        reg signed [31:0] correction;
        reg signed [31:0] result;
        begin
            delta = $signed({1'b0, meanG}) - $signed({1'b0, meanB});
            a = delta;
            bb = 255 - B;
            cc = G;
            numerator = a * bb * cc;
            correction = numerator / 65025;
            result = B + correction;

            if (result < 0)
                compensate_blue = 8'd0;
            else if (result > 255)
                compensate_blue = 8'd255;
            else
                compensate_blue = result[7:0];
        end
    endfunction

    // Integer Gray-World normalization, matching the software reference:
    // output = pixel * target_mean / channel_mean, truncating toward zero.
    function [7:0] gray_world;
        input [7:0] pixel;
        input [7:0] channel_mean;
        input [7:0] target;
        reg [15:0] product;
        reg [31:0] numerator;
        reg [31:0] result;
        begin
            if (channel_mean == 0) begin
                gray_world = 8'd0;
            end else begin
                // Explicitly widen operands so the 8x8 product is not
                // accidentally truncated by Verilog expression sizing.
                product = {8'd0, pixel} * {8'd0, target};
                numerator = {16'd0, product};
                result = numerator / channel_mean;
                if (result > 255)
                    gray_world = 8'd255;
                else
                    gray_world = result[7:0];
            end
        end
    endfunction

    // UART transmitter: 8-N-1.
    uart_tx #(
        .CLK_HZ(CLK_HZ),
        .BAUD(UART_BAUD)
    ) uart_tx_i (
        .clk(clk),
        .rst(rst),
        .start(tx_start),
        .data(tx_data),
        .tx(uart_txd),
        .busy(tx_busy),
        .done(tx_done)
    );

    always @(posedge clk) begin
        if (rst) begin
            state       <= IDLE;
            pixel_index <= 0;
            rd_addr     <= 0;
            write_addr  <= 0;
            write_data  <= 0;

            busy        <= 0;
            done        <= 0;
            streaming   <= 0;

            sum_r  <= 0; sum_g  <= 0; sum_b  <= 0;
            sum_rc <= 0; sum_gc <= 0; sum_bc <= 0;

            mean_r  <= 0; mean_g  <= 0; mean_b  <= 0;
            mean_rc <= 0; mean_gc <= 0; mean_bc <= 0;
            target_mean <= 0;

            uart_index <= 0;
            uart_pixel <= 0;
            tx_data    <= 0;
            tx_start   <= 0;
        end else begin
            // Default low; a transmit request is a one-clock pulse.
            tx_start <= 0;

            case (state)
                IDLE: begin
                    busy      <= 0;
                    streaming <= 0;

                    if (start) begin
                        busy <= 1;
                        done <= 0;
                        pixel_index <= 0;

                        sum_r  <= 0; sum_g  <= 0; sum_b  <= 0;
                        sum_rc <= 0; sum_gc <= 0; sum_bc <= 0;

                        state <= P1_REQ;
                    end
                end

                // PASS 1: original image channel means.
                P1_REQ: begin
                    rd_addr <= pixel_index;
                    state <= P1_WAIT;
                end

                P1_WAIT: begin
                    state <= P1_USE;
                end

                P1_USE: begin
                    sum_r <= sum_r + rd_data[23:16];
                    sum_g <= sum_g + rd_data[15:8];
                    sum_b <= sum_b + rd_data[7:0];

                    if (pixel_index == DEPTH-1)
                        state <= P1_CALC;
                    else begin
                        pixel_index <= pixel_index + 1'b1;
                        state <= P1_REQ;
                    end
                end

                P1_CALC: begin
                    mean_r <= sum_r / DEPTH;
                    mean_g <= sum_g / DEPTH;
                    mean_b <= sum_b / DEPTH;

                    pixel_index <= 0;
                    sum_rc <= 0;
                    sum_gc <= 0;
                    sum_bc <= 0;
                    state <= P2_REQ;
                end

                // PASS 2: means after red/blue compensation.
                P2_REQ: begin
                    rd_addr <= pixel_index;
                    state <= P2_WAIT;
                end

                P2_WAIT: begin
                    state <= P2_USE;
                end

                P2_USE: begin
                    sum_rc <= sum_rc + compensate_red(
                        rd_data[23:16], rd_data[15:8], mean_r, mean_g
                    );
                    sum_gc <= sum_gc + rd_data[15:8];
                    sum_bc <= sum_bc + compensate_blue(
                        rd_data[7:0], rd_data[15:8], mean_b, mean_g
                    );

                    if (pixel_index == DEPTH-1)
                        state <= P2_CALC;
                    else begin
                        pixel_index <= pixel_index + 1'b1;
                        state <= P2_REQ;
                    end
                end

                P2_CALC: begin
                    mean_rc <= sum_rc / DEPTH;
                    mean_gc <= sum_gc / DEPTH;
                    mean_bc <= sum_bc / DEPTH;
                    target_mean <= (sum_rc + sum_gc + sum_bc) / (3 * DEPTH);

                    pixel_index <= 0;
                    state <= P3_REQ;
                end

                // PASS 3: read original pixel, calculate output, then
                // write the pixel back one cycle later to the same memory.
                P3_REQ: begin
                    rd_addr <= pixel_index;
                    state <= P3_WAIT;
                end

                P3_WAIT: begin
                    state <= P3_USE;
                end

                P3_USE: begin
                    write_addr <= pixel_index;
                    write_data <= {
                        gray_world(
                            compensate_red(
                                rd_data[23:16], rd_data[15:8], mean_r, mean_g
                            ),
                            mean_rc, target_mean
                        ),
                        gray_world(rd_data[15:8], mean_gc, target_mean),
                        gray_world(
                            compensate_blue(
                                rd_data[7:0], rd_data[15:8], mean_b, mean_g
                            ),
                            mean_bc, target_mean
                        )
                    };

                    // Read the next address during the following write
                    // cycle. For the last pixel, choose address zero so
                    // the final write is not a same-address collision.
                    if (pixel_index == DEPTH-1)
                        rd_addr <= 0;
                    else
                        rd_addr <= pixel_index + 1'b1;

                    state <= P3_WRITE;
                end

                P3_WRITE: begin
                    if (pixel_index == DEPTH-1) begin
                        state <= U_REQ;
                        uart_index <= 0;
                        streaming <= 1;
                    end else begin
                        pixel_index <= pixel_index + 1'b1;
                        state <= P3_USE;
                    end
                end

                // UART dump: raw RGB bytes, R then G then B, for each pixel.
                // Host must open the serial port before pressing start.
                U_REQ: begin
                    rd_addr <= uart_index;
                    state <= U_WAIT;
                end

                U_WAIT: begin
                    state <= U_LATCH;
                end

                U_LATCH: begin
                    uart_pixel <= rd_data;
                    state <= U_R_START;
                end

                U_R_START: begin
                    if (!tx_busy) begin
                        tx_data <= uart_pixel[23:16];
                        tx_start <= 1;
                        state <= U_R_WAIT;
                    end
                end

                U_R_WAIT: begin
                    if (tx_done)
                        state <= U_G_START;
                end

                U_G_START: begin
                    if (!tx_busy) begin
                        tx_data <= uart_pixel[15:8];
                        tx_start <= 1;
                        state <= U_G_WAIT;
                    end
                end

                U_G_WAIT: begin
                    if (tx_done)
                        state <= U_B_START;
                end

                U_B_START: begin
                    if (!tx_busy) begin
                        tx_data <= uart_pixel[7:0];
                        tx_start <= 1;
                        state <= U_B_WAIT;
                    end
                end

                U_B_WAIT: begin
                    if (tx_done) begin
                        if (uart_index == DEPTH-1)
                            state <= FINISHED;
                        else begin
                            uart_index <= uart_index + 1'b1;
                            state <= U_REQ;
                        end
                    end
                end

                FINISHED: begin
                    busy <= 0;
                    streaming <= 0;
                    done <= 1;
                    state <= IDLE;
                end

                default: begin
                    state <= IDLE;
                    busy <= 0;
                    streaming <= 0;
                end
            endcase
        end
    end

endmodule


module uart_tx #(
    parameter integer CLK_HZ = 100000000,
    parameter integer BAUD   = 115200
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       start,
    input  wire [7:0] data,
    output reg        tx,
    output reg        busy,
    output reg        done
);

    localparam integer CLKS_PER_BIT = (CLK_HZ + BAUD/2) / BAUD;

    localparam [1:0]
        TX_IDLE  = 2'd0,
        TX_START = 2'd1,
        TX_DATA  = 2'd2,
        TX_STOP  = 2'd3;

    reg [1:0] state;
    reg [31:0] clk_count;
    reg [2:0] bit_index;
    reg [7:0] data_latched;

    always @(posedge clk) begin
        if (rst) begin
            state       <= TX_IDLE;
            clk_count   <= 0;
            bit_index   <= 0;
            data_latched <= 0;
            tx          <= 1'b1;
            busy        <= 1'b0;
            done        <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                TX_IDLE: begin
                    tx <= 1'b1;
                    busy <= 1'b0;
                    clk_count <= 0;

                    if (start) begin
                        data_latched <= data;
                        tx <= 1'b0;
                        busy <= 1'b1;
                        state <= TX_START;
                        clk_count <= 0;
                    end
                end

                TX_START: begin
                    if (clk_count == CLKS_PER_BIT-1) begin
                        clk_count <= 0;
                        bit_index <= 0;
                        tx <= data_latched[0];
                        state <= TX_DATA;
                    end else
                        clk_count <= clk_count + 1'b1;
                end

                TX_DATA: begin
                    if (clk_count == CLKS_PER_BIT-1) begin
                        clk_count <= 0;

                        if (bit_index == 3'd7) begin
                            tx <= 1'b1;
                            state <= TX_STOP;
                        end else begin
                            bit_index <= bit_index + 1'b1;
                            tx <= data_latched[bit_index + 1'b1];
                        end
                    end else
                        clk_count <= clk_count + 1'b1;
                end

                TX_STOP: begin
                    if (clk_count == CLKS_PER_BIT-1) begin
                        clk_count <= 0;
                        tx <= 1'b1;
                        busy <= 1'b0;
                        done <= 1'b1;
                        state <= TX_IDLE;
                    end else
                        clk_count <= clk_count + 1'b1;
                end

                default: begin
                    state <= TX_IDLE;
                    tx <= 1'b1;
                    busy <= 1'b0;
                end
            endcase
        end
    end

endmodule
