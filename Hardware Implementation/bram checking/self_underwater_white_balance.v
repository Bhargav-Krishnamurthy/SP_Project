`timescale 1ns / 1ps

module balancer #(
    parameter LENGTH = 256,
    parameter WIDTH = 256,
    parameter DEPTH = 256*256,
    parameter ADDR_LENGTH = 16
) (
    input wire clk,
    input wire reset,
    input wire start,
    output reg busy,
    output reg done
);

localparam [3:0]
    IDLE = 4'b0000,
    READY_1 = 4'b0001,
    MEAN_CALC_1 = 4'b0010,
    MEAN_DONE_1 = 4'b0011,
    READY_2 = 4'b0100,
    CORRECTION = 4'b0101,
    CORRECTION_DONE = 4'b0110,
    READY_3 = 4'b0111,
    MEAN_CALC_2 = 4'b1000,
    MEAN_DONE_2 = 4'b1001,
    READY_4 = 4'b1010,
    GRAY_WORLD = 4'b1011,
    GRAY_WORLD_DONE = 4'b1100,
    DONE = 4'b1101;

reg [3:0] state;

//FORMATTING THE MEMORY
(* ram_style = "block" *)
reg [23:0] memory [0:DEPTH-1];
reg [ADDR_LENGTH-1:0] address;
reg [23:0] mem_dout;
reg [23:0] mem_din;
reg mem_we;

reg [23:0] pixel_value;
reg [23:0] temp_pixel_value;

reg [31:0] red_sum;
reg [31:0] green_sum;
reg [31:0] blue_sum;

reg [7:0] red_avg;
reg [7:0] green_avg;
reg [7:0] blue_avg;

reg [31:0] corrected_red_sum;
reg [31:0] corrected_green_sum;
reg [31:0] corrected_blue_sum;

reg [7:0] corrected_red_avg;
reg [7:0] corrected_green_avg;
reg [7:0] corrected_blue_avg;

reg[7:0] target_avg;

integer i;

initial begin
        $readmemh("image.mem", memory);
    end

function [7:0] compensate_red;
        input [7:0] red;
        input [7:0] green;
        input [7:0] red_avg;
        input [7:0] green_avg;
        reg signed [8:0] delta;
        reg signed [31:0] numerator;
        reg signed [31:0] correction;  
        reg signed [31:0] result;

        begin

            delta = $signed({1'b0, green_avg}) - $signed({1'b0, red_avg});

            numerator  = delta * $signed({1'b0, 8'd255 - red}) * $signed({1'b0, green});
            correction = numerator >>> 16;
            result     = $signed({1'b0, red}) + correction;

            if (result < 0)
                compensate_red = 8'd0;
            else if (result > 255)
                compensate_red = 8'd255;
            else
                compensate_red = result[7:0];
        end
endfunction

function [7:0] compensate_blue;
        input [7:0] blue;
        input [7:0] green;
        input [7:0] blue_avg;
        input [7:0] green_avg;
        reg signed [8:0] delta;
        reg signed [31:0] numerator;
        reg signed [31:0] correction; 
        reg signed [31:0] result;  

        
        begin
            delta = $signed({1'b0, green_avg}) - $signed({1'b0, blue_avg});

            numerator  = delta * (255 - blue) * green;
            correction = numerator>>16;
            result     = blue + correction;

            if (result < 0)
                compensate_blue = 8'd0;
            else if (result > 255)
                compensate_blue = 8'd255;
            else
                compensate_blue = result[7:0];
        end
endfunction

function [7:0] grey_world;
        input [7:0] channel_to_be_compensated;     
        input [7:0] that_channels_average;
        input [7:0] target_mean;

        reg [15:0] temp_mult;
        reg[7:0] result;
        tempmult = channel_to_be_compensated*target_mean;

        result = temp_mult/that_channels_average;
        if (result<0)begin
            grey_world = 8'b0;
        end
        else begin
            grey_world = result[7:0];
        end
endfunction

always@(posedge clk)begin

    if(reset) begin
        pixel_value<=0;
        temp_pixel_value<=0;

        red_sum<=0;
        green_sum<=0;
        blue_sum<=0;

        red_avg<=0;
        green_avg<=0;
        blue_avg<=0;

        corrected_red_sum<=0;
        corrected_green_sum<=0;
        corrected_blue_sum<=0;

        corrected_red_avg<=0;
        corrected_green_avg<=0;
        corrected_blue_avg<=0;

        target_avg<=0;

        state<=IDLE;
        address<=0;
        busy<=0;
        done<=0;
    end

    else begin
        case(state)

            IDLE: begin
                busy<=0;
                if(start) begin
                    busy<=1;
                    red_sum<=0;
                    green_sum<=0;
                    blue_sum<=0;
                    red_avg<=0;
                    green_avg<=0;
                    blue_avg<=0;
                    corrected_red_sum<=0;
                    corrected_green_sum<=0;
                    corrected_blue_sum<=0;
                    corrected_red_avg<=0;
                    corrected_green_avg<=0;
                    corrected_blue_avg<=0;

                    target_avg<=0;

                    state<=READY_1;
                end
            end

            READY_1: begin
                address<=0;
                state<=MEAN_CALC_1;
            end

            MEAN_CALC_1: begin
                pixel_value<=memory[address];
                red_sum<=red_sum+pixel_value[23:16];
                green_sum<=green_sum+pixel_value[15:8];
                blue_sum<=blue_sum+pixel_value[7:0];

                if(address==DEPTH-1) begin
                    red_avg<=red_sum/DEPTH;
                    green_avg<=green_sum/DEPTH;
                    blue_avg<=blue_sum/DEPTH;
                    state<=MEAN_DONE_1;
                end
                else begin
                    address<=address+1;
                    state<=MEAN_CALC_1;
                end
            end

            MEAN_DONE_1: begin
                state<=READY_2;
            end

            READY_2: begin
                address<=0;
                state<=CORRECTION;
            end
            
            CORRECTION: begin
                pixel_value<=memory[address];
                temp_pixel_value[23:16]<=compensate_red(pixel_value[23:16], pixel_value[15:8], red_avg, green_avg);
                temp_pixel_value[15:8]<=pixel_value[15:8];
                temp_pixel_value[7:0]<=compensate_blue(pixel_value[7:0], pixel_value[15:8], blue_avg, green_avg);

                memory[address]<=temp_pixel_value;

                corrected_red_sum<=corrected_red_sum+temp_pixel_value[23:16];
                corrected_green_sum<=corrected_green_sum+temp_pixel_value[15:8];
                corrected_blue_sum<=corrected_blue_sum+temp_pixel_value[7:0];

                if(address==DEPTH-1) begin
                    corrected_red_avg<=corrected_red_sum/DEPTH;
                    corrected_green_avg<=corrected_green_sum/DEPTH;
                    corrected_blue_avg<=corrected_blue_sum/DEPTH;
                    state<=CORRECTION_DONE;
                end
                else begin
                    address<=address+1;
                    state<=CORRECTION;
                end
            end

            CORRECTION_DONE: begin
                done<=1;
                state<=READY_3;
            end

            READY_3: begin
                address<=0;
                state<=MEAN_CALC_2;
            end

            MEAN_CALC_2: begin
                pixel_value<=memory[address];
                corrected_red_sum<=corrected_red_sum+pixel_value[23:16];
                corrected_green_sum<=corrected_green_sum+pixel_value[15:8];
                corrected_blue_sum<=corrected_blue_sum+pixel_value[7:0];

                if(address==DEPTH-1) begin
                    corrected_red_avg<=corrected_red_sum/DEPTH;
                    corrected_green_avg<=corrected_green_sum/DEPTH;
                    corrected_blue_avg<=corrected_blue_sum/DEPTH;
                    state<=MEAN_DONE_2;
                end
                else begin
                    address<=address+1;
                    state<=MEAN_CALC_2;
                end
            end
            MEAN_DONE_2: begin
                target_avg<=(corrected_red_avg+corrected_green_avg+corrected_blue_avg)/3;
                state<=READY_4;
            end

            READY_4: begin
                address<=0;
                state<=GRAY_WORLD;
            end

            GRAY_WORLD: begin
                pixel_value<=memory[address];
                temp_pixel_value[23:16]<=grey_world(pixel_value[23:16], corrected_red_avg, target_avg);
                temp_pixel_value[15:8]<=grey_world(pixel_value[15:8], corrected_green_avg, target_avg);
                temp_pixel_value[7:0]<=grey_world(pixel_value[7:0], corrected_blue_avg, target_avg);

                memory[address]<=temp_pixel_value;

                if(address==DEPTH-1) begin
                    state<=GRAY_WORLD_DONE;
                end
                else begin
                    address<=address+1;
                    state<=GRAY_WORLD;
                end
            end
            GRAY_WORLD_DONE: begin
                done<=1;
                state<=DONE;
            end

            DONE: begin
                busy<=0;
                state<=IDLE;
            end

            default: begin
                busy  <= 0;
                state <= IDLE;
                end
        endcase
    end
end

endmodule

