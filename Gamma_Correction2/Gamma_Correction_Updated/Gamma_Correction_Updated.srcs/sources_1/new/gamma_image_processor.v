
`timescale 1ns / 1ps

module gamma_image_processor (
    input  wire       clk,
    input  wire       rst,
    input  wire       start,

    output reg  [7:0] out_data,
    output reg        out_valid,
    output reg        done
);

    // 256 x 256 pixels x 3 RGB channels
    localparam integer TOTAL_BYTES = 256 * 256 * 3;
    localparam [17:0] LAST_ADDR = TOTAL_BYTES - 1;

    // Image and gamma lookup memories
    reg [7:0] image_mem [0:TOTAL_BYTES-1];
    reg [7:0] gamma_mem [0:255];

    // Current image byte and address
    reg [17:0] addr;
    reg [7:0] pixel_reg;

    // FSM states
    localparam [2:0]
        IDLE       = 3'd0,
        READ_IMAGE = 3'd1,
        APPLY_LUT  = 3'd2,
        FINISHED   = 3'd3;

    reg [2:0] state;

    // Load the existing .mem files at simulation startup
    initial begin
        $readmemh("image_rgb_channels.mem", image_mem);
        $readmemh("gamma_lut.mem", gamma_mem);
    end

    // Process one RGB channel intensity at a time
    always @(posedge clk) begin
        if (rst) begin
            addr      <= 18'd0;
            pixel_reg <= 8'd0;
            out_data  <= 8'd0;
            out_valid <= 1'b0;
            done      <= 1'b0;
            state     <= IDLE;
        end else begin
            // Default: out_valid is a one-cycle pulse
            out_valid <= 1'b0;

            case (state)

                IDLE: begin
                    if (start) begin
                        addr  <= 18'd0;
                        done  <= 1'b0;
                        state <= READ_IMAGE;
                    end
                end

                READ_IMAGE: begin
                    pixel_reg <= image_mem[addr];
                    state     <= APPLY_LUT;
                end

                APPLY_LUT: begin
                    // Replace input intensity with its LUT value
                    out_data  <= gamma_mem[pixel_reg];
                    out_valid <= 1'b1;

                    if (addr == LAST_ADDR) begin
                        done  <= 1'b1;
                        state <= FINISHED;
                    end else begin
                        addr  <= addr + 1'b1;
                        state <= READ_IMAGE;
                    end
                end

                FINISHED: begin
                    done  <= 1'b1;
                end

                default: begin
                    state <= IDLE;
                end

            endcase
        end
    end

endmodule
