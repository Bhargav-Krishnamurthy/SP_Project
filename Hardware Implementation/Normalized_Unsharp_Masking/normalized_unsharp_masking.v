`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/09/2026 04:26:12 PM
// Design Name: 
// Module Name: normalized_unsharp_masking
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module normalized_unsharp_masking#(
    parameter BIT_WIDTH = 8
)(
    input  wire       clk,
    input  wire [BIT_WIDTH-1:0] win_r [0:2][0:2],
    input  wire [BIT_WIDTH-1:0] win_g [0:2][0:2],
    input  wire [BIT_WIDTH-1:0] win_b [0:2][0:2],
    output reg  [BIT_WIDTH-1:0] i2_r,
    output reg  [BIT_WIDTH-1:0] i2_g,
    output reg  [BIT_WIDTH-1:0] i2_b
); 

    // Separable Gaussian Blur (Kernel sum / 16)
    wire [11:0] blur_r = (win_r[0][0] + (win_r[0][1]<<1) + win_r[0][2] +
                          (win_r[1][0]<<1) + (win_r[1][1]<<2) + (win_r[1][2]<<1) +
                          win_r[2][0] + (win_r[2][1]<<1) + win_r[2][2]) >> 4;

    wire [11:0] blur_g = (win_g[0][0] + (win_g[0][1]<<1) + win_g[0][2] +
                          (win_g[1][0]<<1) + (win_g[1][1]<<2) + (win_g[1][2]<<1) +
                          win_g[2][0] + (win_g[2][1]<<1) + win_g[2][2]) >> 4;

    wire [11:0] blur_b = (win_b[0][0] + (win_b[0][1]<<1) + win_b[0][2] +
                          (win_b[1][0]<<1) + (win_b[1][1]<<2) + (win_b[1][2]<<1) +
                          win_b[2][0] + (win_b[2][1]<<1) + win_b[2][2]) >> 4; 
                          
    // High-pass difference with a signed shift offset (+128)
    wire [8:0] hp_r = 9'd128 + win_r[1][1] - blur_r[7:0];
    wire [8:0] hp_g = 9'd128 + win_g[1][1] - blur_g[7:0];
    wire [8:0] hp_b = 9'd128 + win_b[1][1] - blur_b[7:0];

    always @(posedge clk) begin
        i2_r <= (win_r[1][1] + hp_r[7:0]) >> 1;
        i2_g <= (win_g[1][1] + hp_g[7:0]) >> 1;
        i2_b <= (win_b[1][1] + hp_b[7:0]) >> 1;
    end

endmodule
