`timescale 1ns / 1ps

module bram_white_balance #(
    parameter WIDTH      = 256,
    parameter HEIGHT     = 256,
    parameter ADDR_WIDTH = 16
)(
    input  wire clk,
    input  wire rst,
    input  wire start,

    output reg  busy,
    output reg  done
);

    localparam integer DEPTH = WIDTH * HEIGHT;

    // FSM
    localparam [3:0]
        IDLE     = 4'd0,
        P1_REQ   = 4'd1,
        P1_WAIT  = 4'd2,
        P1_USE   = 4'd3,
        P1_CALC  = 4'd4,
        P2_REQ   = 4'd5,
        P2_WAIT  = 4'd6,
        P2_USE   = 4'd7,
        P2_CALC  = 4'd8,
        P3_REQ   = 4'd9,
        P3_WAIT  = 4'd10,
        P3_USE   = 4'd11,
        FINISHED = 4'd12;

    reg [3:0] state;

    // Input BRAM: one 24-bit RGB pixel per address.
    (* ram_style = "block" *)
    reg [23:0] input_mem [0:DEPTH-1];

    // Output BRAM: final corrected RGB image.
    (* ram_style = "block" *)
    reg [23:0] output_mem [0:DEPTH-1];

    reg [ADDR_WIDTH-1:0] pixel_index;
    reg [ADDR_WIDTH-1:0] rd_addr;

    // Synchronous BRAM read register.
    reg [23:0] rd_data;

    // 32-bit channel sums.
    reg [31:0] sum_r, sum_g, sum_b;
    reg [31:0] sum_rc, sum_gc, sum_bc;

    // 8-bit floor-rounded channel means.
    reg [7:0] mean_r, mean_g, mean_b;
    reg [7:0] mean_rc, mean_gc, mean_bc;
    reg [7:0] target_mean;

    // Current pixel and compensated channels.
    reg [7:0] r, g, b;
    reg [7:0] rc, bc;

    integer i;

    // Input initialization.
    initial begin
        $readmemh("image.mem", input_mem);
    end

    // Synchronous input BRAM read.
    always @(posedge clk) begin
        if (rst)
            rd_data <= 24'd0;
        else
            rd_data <= input_mem[rd_addr];
    end

    // ---------------------------------------------------------
    // Red compensation: paper Equation (4).
    // Rc = R + (meanG-meanR)*(255-R)*G / 65025
    // ---------------------------------------------------------
    function [7:0] compensate_red;
        input [7:0] R;
        input [7:0] G;
        input [7:0] meanR;
        input [7:0] meanG;

        reg signed [31:0] delta;
        reg signed [63:0] a, bb, cc;
        reg signed [63:0] numerator;
        reg signed [63:0] correction;
        reg signed [63:0] result;

        begin
            delta = $signed({1'b0, meanG})
                  - $signed({1'b0, meanR});

            a  = delta;
            bb = 255 - R;
            cc = G;

            numerator  = a * bb * cc;
            correction = numerator / 65025;
            result     = R + correction;

            if (result < 0)
                compensate_red = 8'd0;
            else if (result > 255)
                compensate_red = 8'd255;
            else
                compensate_red = result[7:0];
        end
    endfunction

    // ---------------------------------------------------------
    // Blue compensation: paper Equation (5).
    // Bc = B + (meanG-meanB)*(255-B)*G / 65025
    // ---------------------------------------------------------
    function [7:0] compensate_blue;
        input [7:0] B;
        input [7:0] G;
        input [7:0] meanB;
        input [7:0] meanG;

        reg signed [31:0] delta;
        reg signed [63:0] a, bb, cc;
        reg signed [63:0] numerator;
        reg signed [63:0] correction;
        reg signed [63:0] result;

        begin
            delta = $signed({1'b0, meanG})
                  - $signed({1'b0, meanB});

            a  = delta;
            bb = 255 - B;
            cc = G;

            numerator  = a * bb * cc;
            correction = numerator / 65025;
            result     = B + correction;

            if (result < 0)
                compensate_blue = 8'd0;
            else if (result > 255)
                compensate_blue = 8'd255;
            else
                compensate_blue = result[7:0];
        end
    endfunction

    // ---------------------------------------------------------
    // Gray-World channel normalization.
    // Yout = Yin * target_mean / channel_mean
    // ---------------------------------------------------------
    function [7:0] gray_world;
        input [7:0] pixel;
        input [7:0] channel_mean;
        input [7:0] target;

        reg [31:0] numerator;
        reg [31:0] result;

        begin
            if (channel_mean == 0) begin
                gray_world = 8'd0;
            end
            else begin
                numerator = pixel * target;
                result = numerator / channel_mean;

                if (result > 255)
                    gray_world = 8'd255;
                else
                    gray_world = result[7:0];
            end
        end
    endfunction

    // ---------------------------------------------------------
    // Main processing controller.
    // ---------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state       <= IDLE;
            pixel_index <= 0;
            rd_addr     <= 0;

            busy        <= 0;
            done        <= 0;

            sum_r  <= 0;
            sum_g  <= 0;
            sum_b  <= 0;
            sum_rc <= 0;
            sum_gc <= 0;
            sum_bc <= 0;

            mean_r  <= 0;
            mean_g  <= 0;
            mean_b  <= 0;
            mean_rc <= 0;
            mean_gc <= 0;
            mean_bc <= 0;

            target_mean <= 0;
            r <= 0;
            g <= 0;
            b <= 0;
            rc <= 0;
            bc <= 0;
        end
        else begin
            // done is a one-cycle pulse.
            done <= 0;

            case (state)

                IDLE: begin
                    busy <= 0;

                    if (start) begin
                        busy <= 1;
                        pixel_index <= 0;

                        sum_r  <= 0;
                        sum_g  <= 0;
                        sum_b  <= 0;
                        sum_rc <= 0;
                        sum_gc <= 0;
                        sum_bc <= 0;

                        state <= P1_REQ;
                    end
                end

                // PASS 1: statistics of original RGB image.
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

                    if (pixel_index == DEPTH-1) begin
                        state <= P1_CALC;
                    end
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
                    state <= P2_REQ;
                end

                // PASS 2: calculate means after both
                // red and blue compensation.
                P2_REQ: begin
                    rd_addr <= pixel_index;
                    state <= P2_WAIT;
                end

                P2_WAIT: begin
                    state <= P2_USE;
                end

                P2_USE: begin
                    r = rd_data[23:16];
                    g = rd_data[15:8];
                    b = rd_data[7:0];

                    rc = compensate_red(r, g, mean_r, mean_g);
                    bc = compensate_blue(b, g, mean_b, mean_g);

                    sum_rc <= sum_rc + rc;
                    sum_gc <= sum_gc + g;
                    sum_bc <= sum_bc + bc;

                    if (pixel_index == DEPTH-1) begin
                        state <= P2_CALC;
                    end
                    else begin
                        pixel_index <= pixel_index + 1'b1;
                        state <= P2_REQ;
                    end
                end

                P2_CALC: begin
                    mean_rc <= sum_rc / DEPTH;
                    mean_gc <= sum_gc / DEPTH;
                    mean_bc <= sum_bc / DEPTH;

                    target_mean <=
                        (sum_rc + sum_gc + sum_bc) /
                        (3 * DEPTH);

                    pixel_index <= 0;
                    state <= P3_REQ;
                end

                // PASS 3: reconstruct compensated channels,
                // apply Gray-World and write output BRAM.
                P3_REQ: begin
                    rd_addr <= pixel_index;
                    state <= P3_WAIT;
                end

                P3_WAIT: begin
                    state <= P3_USE;
                end

                P3_USE: begin
                    r = rd_data[23:16];
                    g = rd_data[15:8];
                    b = rd_data[7:0];

                    rc = compensate_red(r, g, mean_r, mean_g);
                    bc = compensate_blue(b, g, mean_b, mean_g);

                    output_mem[pixel_index] <= {
                        gray_world(rc, mean_rc, target_mean),
                        gray_world(g,  mean_gc, target_mean),
                        gray_world(bc, mean_bc, target_mean)
                    };

                    if (pixel_index == DEPTH-1) begin
                        state <= FINISHED;
                    end
                    else begin
                        pixel_index <= pixel_index + 1'b1;
                        state <= P3_REQ;
                    end
                end

                FINISHED: begin
                    busy  <= 0;
                    done  <= 1;
                    state <= IDLE;
                end

                default: begin
                    busy  <= 0;
                    state <= IDLE;
                end

            endcase
        end
    end

endmodule
