
`timescale 1ns / 1ps

module image_gamma_uart_top #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer BAUD_RATE   = 115200
)(
    input  wire clk_100MHz,
    input  wire rst,          // Active-high reset
    output wire uart_tx,
    output reg  done
);

    // 256 x 256 pixels x 3 color channels
    localparam integer TOTAL_BYTES = 196608;
    localparam [17:0] LAST_ADDR = TOTAL_BYTES - 1;

    // Initialized image and gamma memories
    (* ram_style = "block" *)
    reg [7:0] image_mem [0:TOTAL_BYTES-1];

    (* ram_style = "block" *)
    reg [7:0] gamma_mem [0:255];

    initial begin
        $readmemh("image_rgb_channels.mem", image_mem);
        $readmemh("gamma_lut.mem", gamma_mem);
    end

    reg [17:0] addr;
    reg [7:0] image_pixel;
    reg [7:0] corrected_pixel;

    reg [2:0] state;
    reg [7:0] uart_data;
    reg uart_start;

    wire uart_busy;
    wire uart_done;

    localparam [2:0]
        READ_IMAGE  = 3'd0,
        READ_GAMMA  = 3'd1,
        START_UART  = 3'd2,
        WAIT_UART   = 3'd3,
        FINISHED    = 3'd4;

    // One image intensity is processed at a time.
    // Both memories are read synchronously.
    always @(posedge clk_100MHz) begin
        if (rst) begin
            addr            <= 0;
            image_pixel     <= 0;
            corrected_pixel <= 0;
            uart_data       <= 0;
            uart_start      <= 0;
            done            <= 0;
            state           <= READ_IMAGE;
        end else begin
            uart_start <= 0;

            case (state)

                READ_IMAGE: begin
                    image_pixel <= image_mem[addr];
                    state <= READ_GAMMA;
                end

                READ_GAMMA: begin
                    corrected_pixel <= gamma_mem[image_pixel];
                    state <= START_UART;
                end

                START_UART: begin
                    if (!uart_busy) begin
                        uart_data  <= corrected_pixel;
                        uart_start <= 1;
                        state      <= WAIT_UART;
                    end
                end

                WAIT_UART: begin
                    if (uart_done) begin
                        if (addr == LAST_ADDR) begin
                            done  <= 1;
                            state <= FINISHED;
                        end else begin
                            addr  <= addr + 1'b1;
                            state <= READ_IMAGE;
                        end
                    end
                end

                FINISHED: begin
                    done <= 1;
                end

                default: begin
                    state <= READ_IMAGE;
                end

            endcase
        end
    end

    uart_tx_module #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .BAUD_RATE(BAUD_RATE)
    ) uart_inst (
        .clk(clk_100MHz),
        .rst(rst),
        .start(uart_start),
        .data_in(uart_data),
        .tx(uart_tx),
        .busy(uart_busy),
        .done(uart_done)
    );

endmodule


module uart_tx_module #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer BAUD_RATE   = 115200
)(
    input  wire clk,
    input  wire rst,
    input  wire start,
    input  wire [7:0] data_in,
    output reg tx,
    output reg busy,
    output reg done
);

    localparam integer CLKS_PER_BIT = CLK_FREQ_HZ / BAUD_RATE;

    reg [31:0] baud_count;
    reg [3:0] bit_index;
    reg [9:0] shift_reg;

    always @(posedge clk) begin
        if (rst) begin
            tx         <= 1'b1;
            busy       <= 1'b0;
            done       <= 1'b0;
            baud_count <= 0;
            bit_index  <= 0;
            shift_reg  <= 10'h3FF;
        end else begin
            done <= 1'b0;

            if (!busy) begin
                tx <= 1'b1;

                if (start) begin
                    // UART frame: start bit, 8 data bits, stop bit
                    shift_reg  <= {1'b1, data_in, 1'b0};
                    tx         <= 1'b0;
                    busy       <= 1'b1;
                    bit_index  <= 0;
                    baud_count <= CLKS_PER_BIT - 1;
                end
            end else begin
                if (baud_count != 0) begin
                    baud_count <= baud_count - 1'b1;
                end else begin
                    baud_count <= CLKS_PER_BIT - 1;

                    if (bit_index == 8) begin
                        // Stop bit has completed
                        tx   <= 1'b1;
                        busy <= 1'b0;
                        done <= 1'b1;
                    end else begin
                        tx        <= shift_reg[1];
                        shift_reg <= {1'b1, shift_reg[9:1]};
                        bit_index <= bit_index + 1'b1;
                    end
                end
            end
        end
    end

endmodule
