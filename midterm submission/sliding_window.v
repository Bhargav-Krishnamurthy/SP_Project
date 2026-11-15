`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/22/2026 11:44:28 PM
// Design Name: 
// Module Name: sliding_window
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


module sliding_window#(

    parameter IMG_WIDTH  = 256,
    parameter BIT_WIDTH = 8
    
    )(
    
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    input  wire [23:0]  pixel_in,
    output reg  [BIT_WIDTH-1:0]  win_r [0:2][0:2],  // 3x3
    output reg  [BIT_WIDTH-1:0]  win_g [0:2][0:2],
    output reg  [BIT_WIDTH-1:0]  win_b [0:2][0:2],
    output reg  [8:0]  col_cnt,
    output reg  [8:0]  row_cnt
    
    ); 
    
    reg [BIT_WIDTH-1:0] line_buf_0 [0:IMG_WIDTH-1];
    reg [BIT_WIDTH-1:0] line_buf_1 [0:IMG_WIDTH-1]; 
    
    always @(posedge clk or negedge rst_n) begin 
        if(!rst_n) begin 
            row_cnt <= 0;
            col_cnt <= 0;
        end
        
        else if (enable) begin 
            if(col_cnt==IMG_WIDTH-1) begin 
                col_cnt<=0;
                row_cnt <= (row_cnt==IMG_WIDTH-1)?0:row_cnt + 1'b1;
            end 
            
            else begin 
                col_cnt <= col_cnt +1'b1;
            end 
            
            //shift internal line buffers 
            line_buf_0[col_cnt] <= pixel_in;
            line_buf_1[col_cnt] <= line_buf_0[col_cnt]; 
            
            // 3x3 Window Updating - Red
            win_r[0][0] <= win_r[0][1]; win_r[0][1] <= win_r[0][2]; win_r[0][2] <= pixel_in[23:16];
            win_r[1][0] <= win_r[1][1]; win_r[1][1] <= win_r[1][2]; win_r[1][2] <= line_buf_0[col_cnt][23:16];
            win_r[2][0] <= win_r[2][1]; win_r[2][1] <= win_r[2][2]; win_r[2][2] <= line_buf_1[col_cnt][23:16];

            // 3x3 Window Updating - Green
            win_g[0][0] <= win_g[0][1]; win_g[0][1] <= win_g[0][2]; win_g[0][2] <= pixel_in[15:8];
            win_g[1][0] <= win_g[1][1]; win_g[1][1] <= win_g[1][2]; win_g[1][2] <= line_buf_0[col_cnt][15:8];
            win_g[2][0] <= win_g[2][1]; win_g[2][1] <= win_g[2][2]; win_g[2][2] <= line_buf_1[col_cnt][15:8];

            // 3x3 Window Updating - Blue
            win_b[0][0] <= win_b[0][1]; win_b[0][1] <= win_b[0][2]; win_b[0][2] <= pixel_in[7:0];
            win_b[1][0] <= win_b[1][1]; win_b[1][1] <= win_b[1][2]; win_b[1][2] <= line_buf_0[col_cnt][7:0];
            win_b[2][0] <= win_b[2][1]; win_b[2][1] <= win_b[2][2]; win_b[2][2] <= line_buf_1[col_cnt][7:0];
            
        end      
    
    end
    
    
    
endmodule
