`timescale 1ns / 1ps

module bram_image_reader #(
    parameter WIDTH  = 256,
    parameter HEIGHT = 256,
    parameter ADDR_WIDTH = 16
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        start,

    output reg  [23:0] pixel_out,
    output reg         pixel_valid,
    output reg         done
);

    localparam DEPTH = WIDTH * HEIGHT;

    // 24-bit RGB pixel at each address
    (* ram_style = "block" *)
    reg [23:0] image_mem [0:DEPTH-1];

    reg [ADDR_WIDTH-1:0] addr;
    reg reading;

    // Initialize memory from the generated file
    initial begin
        $readmemh("image.mem", image_mem);
    end

    // Synchronous memory read
    always @(posedge clk) begin
        if (rst) begin
            addr        <= 0;
            pixel_out   <= 0;
            pixel_valid <= 0;
            reading     <= 0;
            done        <= 0;
        end else begin
            pixel_valid <= 0;

            if (start && !reading && !done) begin
                addr    <= 0;
                reading <= 1;
            end
            else if (reading) begin
                pixel_out   <= image_mem[addr];
                pixel_valid <= 1;

                if (addr == DEPTH-1) begin
                    reading <= 0;
                    done    <= 1;
                end
                else begin
                    addr <= addr + 1'b1;
                end
            end
        end
    end

endmodule

