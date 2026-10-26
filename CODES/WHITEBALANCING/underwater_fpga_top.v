`timescale 1ns / 1ps

// RealDigital Boolean board top-level.
// Board: xc7s50csga324-1, 100 MHz oscillator on F14.
// btn[0] = start (press once)
// btn[1] = reset (hold while resetting)
// led[0] = busy (processing + UART transfer)
// led[1] = done (latched after complete UART transfer)
// led[2] = UART streaming active
// UART_txd = binary RGB stream, 8-N-1 (R, G, B per pixel).
//
// One 24-bit BRAM holds the image and is overwritten in place in pass 3.
//
// NOTE: btn is [1:0] on purpose. The XDC only constrains btn[0] and btn[1].
// A [3:0] port with 2 unconstrained bits makes write_bitstream fail DRC
// (NSTD-1 / UCIO-1).
module underwater_fpga_top (
    input  wire        clk,
    input  wire [1:0]  btn,
    output wire [15:0] led,
    output wire        UART_txd
);

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

    wire busy, done, streaming;

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


// ---------------------------------------------------------------------------
// Engine. WIDTH*HEIGHT must equal 2**ADDR_WIDTH.
// Speed is not important here (a few tens of ms of compute), so all the
// math is done with short multi-cycle units. That keeps every register-to-
// register path small enough for 100 MHz.
// ---------------------------------------------------------------------------
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
        IDLE      = 5'd0,
        P1_REQ    = 5'd1,
        P1_WAIT   = 5'd2,
        P1_USE    = 5'd3,
        P1_CALC   = 5'd4,
        P2_REQ    = 5'd5,
        P2_WAIT   = 5'd6,
        P2_LATCH  = 5'd7,
        P2_GO     = 5'd8,
        P2_COMP   = 5'd9,
        P2_CALC   = 5'd10,
        P2_CALC2  = 5'd11,
        P3_REQ    = 5'd12,
        P3_WAIT   = 5'd13,
        P3_LATCH  = 5'd14,
        P3_GO     = 5'd15,
        P3_COMP   = 5'd16,
        P3_GGO    = 5'd17,
        P3_GCOMP  = 5'd18,
        P3_WRITE  = 5'd19,
        U_REQ     = 5'd20,
        U_WAIT    = 5'd21,
        U_LATCH   = 5'd22,
        U_R_START = 5'd23,
        U_R_WAIT  = 5'd24,
        U_G_START = 5'd25,
        U_G_WAIT  = 5'd26,
        U_B_START = 5'd27,
        U_B_WAIT  = 5'd28,
        FINISHED  = 5'd29;
    reg [4:0] state;

    // ---------------- image memory (one BRAM, read port + write port) -----
    (* ram_style = "block" *)
    reg [23:0] image_mem [0:DEPTH-1];

    reg [ADDR_WIDTH-1:0] pixel_index;
    reg [ADDR_WIDTH-1:0] rd_addr;
    reg [ADDR_WIDTH-1:0] write_addr;
    reg [23:0] rd_data;
    reg [23:0] write_data;

    initial begin
        $readmemh("image.mem", image_mem);
    end

    always @(posedge clk) begin
        rd_data <= image_mem[rd_addr];
        if (state == P3_WRITE)
            image_mem[write_addr] <= write_data;
    end

    // ---------------- statistics ------------------------------------------
    reg [31:0] sum_r, sum_g, sum_b;
    reg [31:0] sum_rc, sum_gc, sum_bc;
    reg [7:0]  mean_r, mean_g, mean_b;
    reg [7:0]  mean_rc, mean_gc, mean_bc;
    reg [7:0]  target_mean;
    reg [31:0] tsum;

    reg [7:0]  pix_r, pix_g, pix_b;
    reg [7:0]  rc_val, bc_val;

    // ---------------- red / blue compensation units -----------------------
    reg  c_start;
    wire [7:0] rc_y, bc_y;
    wire rc_done, bc_done;

    comp_ch comp_r (.clk(clk), .start(c_start), .x(pix_r), .g(pix_g),
                    .mean_x(mean_r), .mean_g(mean_g), .y(rc_y), .done(rc_done));
    comp_ch comp_b (.clk(clk), .start(c_start), .x(pix_b), .g(pix_g),
                    .mean_x(mean_b), .mean_g(mean_g), .y(bc_y), .done(bc_done));

    // ---------------- gray-world units ------------------------------------
    reg  g_start;
    wire [7:0] gr_y, gg_y, gb_y;
    wire gr_done, gg_done, gb_done;

    gw_ch gw_r (.clk(clk), .start(g_start), .pixel(rc_val), .mean(mean_rc),
                .target(target_mean), .y(gr_y), .done(gr_done));
    gw_ch gw_g (.clk(clk), .start(g_start), .pixel(pix_g),  .mean(mean_gc),
                .target(target_mean), .y(gg_y), .done(gg_done));
    gw_ch gw_b (.clk(clk), .start(g_start), .pixel(bc_val), .mean(mean_bc),
                .target(target_mean), .y(gb_y), .done(gb_done));

    // ---------------- UART ------------------------------------------------
    reg [ADDR_WIDTH-1:0] uart_index;
    reg [23:0] uart_pixel;
    reg [7:0]  tx_data;
    reg        tx_start;
    wire       tx_busy;
    wire       tx_done;

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(UART_BAUD)) uart_tx_i (
        .clk(clk), .rst(rst), .start(tx_start), .data(tx_data),
        .tx(uart_txd), .busy(tx_busy), .done(tx_done)
    );

    // ---------------- main FSM --------------------------------------------
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
            tsum       <= 0;
            pix_r <= 0; pix_g <= 0; pix_b <= 0;
            rc_val <= 0; bc_val <= 0;
            c_start <= 0;
            g_start <= 0;
            uart_index <= 0;
            uart_pixel <= 0;
            tx_data    <= 0;
            tx_start   <= 0;
        end else begin
            // one-clock pulses
            tx_start <= 0;
            c_start  <= 0;
            g_start  <= 0;

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

                // ---------- PASS 1: channel means of the original image ----
                P1_REQ:  begin rd_addr <= pixel_index; state <= P1_WAIT; end
                P1_WAIT: state <= P1_USE;
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
                    state <= P2_REQ;
                end

                // ---------- PASS 2: means after red/blue compensation -----
                P2_REQ:  begin rd_addr <= pixel_index; state <= P2_WAIT; end
                P2_WAIT: state <= P2_LATCH;
                P2_LATCH: begin
                    pix_r <= rd_data[23:16];
                    pix_g <= rd_data[15:8];
                    pix_b <= rd_data[7:0];
                    state <= P2_GO;
                end
                P2_GO: begin
                    c_start <= 1;
                    state   <= P2_COMP;
                end
                P2_COMP: begin
                    if (rc_done && bc_done) begin
                        sum_rc <= sum_rc + rc_y;
                        sum_gc <= sum_gc + pix_g;
                        sum_bc <= sum_bc + bc_y;
                        if (pixel_index == DEPTH-1)
                            state <= P2_CALC;
                        else begin
                            pixel_index <= pixel_index + 1'b1;
                            state <= P2_REQ;
                        end
                    end
                end
                P2_CALC: begin
                    mean_rc <= sum_rc / DEPTH;
                    mean_gc <= sum_gc / DEPTH;
                    mean_bc <= sum_bc / DEPTH;
                    tsum    <= (sum_rc + sum_gc + sum_bc) / DEPTH;
                    state   <= P2_CALC2;
                end
                P2_CALC2: begin
                    // floor(S/(3*DEPTH)) == floor(floor(S/DEPTH)/3)
                    target_mean <= tsum / 3;
                    pixel_index <= 0;
                    state <= P3_REQ;
                end

                // ---------- PASS 3: final per-pixel result, written in place
                P3_REQ:  begin rd_addr <= pixel_index; state <= P3_WAIT; end
                P3_WAIT: state <= P3_LATCH;
                P3_LATCH: begin
                    pix_r <= rd_data[23:16];
                    pix_g <= rd_data[15:8];
                    pix_b <= rd_data[7:0];
                    state <= P3_GO;
                end
                P3_GO: begin
                    c_start <= 1;
                    state   <= P3_COMP;
                end
                P3_COMP: begin
                    if (rc_done && bc_done) begin
                        rc_val <= rc_y;
                        bc_val <= bc_y;
                        state  <= P3_GGO;
                    end
                end
                P3_GGO: begin
                    g_start <= 1;
                    state   <= P3_GCOMP;
                end
                P3_GCOMP: begin
                    if (gr_done && gg_done && gb_done) begin
                        write_addr <= pixel_index;
                        write_data <= {gr_y, gg_y, gb_y};
                        state      <= P3_WRITE;
                    end
                end
                P3_WRITE: begin
                    // BRAM write happens on this clock edge (state==P3_WRITE)
                    if (pixel_index == DEPTH-1) begin
                        uart_index <= 0;
                        streaming  <= 1;
                        state      <= U_REQ;      // <-- was missing: FSM hung here
                    end else begin
                        pixel_index <= pixel_index + 1'b1;
                        state <= P3_REQ;
                    end
                end

                // ---------- UART dump: R, G, B for every pixel ------------
                U_REQ:   begin rd_addr <= uart_index; state <= U_WAIT; end
                U_WAIT:  state <= U_LATCH;
                U_LATCH: begin uart_pixel <= rd_data; state <= U_R_START; end

                U_R_START: if (!tx_busy) begin
                    tx_data <= uart_pixel[23:16]; tx_start <= 1; state <= U_R_WAIT;
                end
                U_R_WAIT: if (tx_done) state <= U_G_START;

                U_G_START: if (!tx_busy) begin
                    tx_data <= uart_pixel[15:8]; tx_start <= 1; state <= U_G_WAIT;
                end
                U_G_WAIT: if (tx_done) state <= U_B_START;

                U_B_START: if (!tx_busy) begin
                    tx_data <= uart_pixel[7:0]; tx_start <= 1; state <= U_B_WAIT;
                end
                U_B_WAIT: if (tx_done) begin
                    if (uart_index == DEPTH-1)
                        state <= FINISHED;
                    else begin
                        uart_index <= uart_index + 1'b1;
                        state <= U_REQ;
                    end
                end

                FINISHED: begin
                    busy      <= 0;
                    streaming <= 0;
                    done      <= 1;
                    state     <= IDLE;
                end

                default: begin
                    state     <= IDLE;
                    busy      <= 0;
                    streaming <= 0;
                end
            endcase
        end
    end

endmodule


// ---------------------------------------------------------------------------
// Serial restoring divider (unsigned). NW+1 clocks after start, done pulses.
// ---------------------------------------------------------------------------
module div_seq #(
    parameter integer NW = 24,
    parameter integer DW = 17
)(
    input  wire          clk,
    input  wire          start,
    input  wire [NW-1:0] num,
    input  wire [DW-1:0] den,
    output reg  [NW-1:0] quo,
    output reg           done
);
    reg [NW-1:0] n;
    reg [DW-1:0] d;
    reg [DW:0]   rem;
    reg [DW:0]   rem_sh;
    reg [7:0]    cnt;
    reg          running = 1'b0;

    initial begin quo = 0; done = 0; end

    always @(posedge clk) begin
        done <= 1'b0;
        if (start) begin
            n       <= num;
            d       <= den;
            rem     <= 0;
            quo     <= 0;
            cnt     <= NW;
            running <= 1'b1;
        end else if (running) begin
            rem_sh = {rem[DW-1:0], n[NW-1]};
            if (rem_sh >= {1'b0, d}) begin
                rem <= rem_sh - {1'b0, d};
                quo <= {quo[NW-2:0], 1'b1};
            end else begin
                rem <= rem_sh;
                quo <= {quo[NW-2:0], 1'b0};
            end
            n   <= {n[NW-2:0], 1'b0};
            cnt <= cnt - 1'b1;
            if (cnt == 8'd1) begin
                running <= 1'b0;
                done    <= 1'b1;
            end
        end
    end
endmodule


// ---------------------------------------------------------------------------
// y = clamp( x + trunc( (meanG-meanX) * (255-x) * g / 65025 ), 0, 255 )
// Same math as the original compensate_red / compensate_blue, split over
// several clocks (each multiplier gets its own register stage).
// ---------------------------------------------------------------------------
module comp_ch (
    input  wire       clk,
    input  wire       start,
    input  wire [7:0] x,
    input  wire [7:0] g,
    input  wire [7:0] mean_x,
    input  wire [7:0] mean_g,
    output reg  [7:0] y,
    output reg        done
);
    localparam [2:0] S_IDLE = 0, S_M1 = 1, S_M2 = 2, S_ABS = 3,
                     S_DIV  = 4, S_WAIT = 5, S_OUT = 6;
    reg [2:0] st = S_IDLE;

    reg [7:0]  x_l;
    reg signed [9:0]  delta, inv, gs;
    reg signed [17:0] t1;
    reg signed [25:0] t2;
    reg        neg;
    reg [23:0] mag;
    reg        dv_start;
    wire [23:0] dv_q;
    wire        dv_done;

    div_seq #(.NW(24), .DW(17)) div_i (
        .clk(clk), .start(dv_start), .num(mag), .den(17'd65025),
        .quo(dv_q), .done(dv_done)
    );

    reg signed [10:0] res;

    initial begin y = 0; done = 0; dv_start = 0; end

    always @(posedge clk) begin
        done     <= 1'b0;
        dv_start <= 1'b0;
        case (st)
            S_IDLE: if (start) begin
                x_l   <= x;
                delta <= $signed({2'b00, mean_g}) - $signed({2'b00, mean_x});
                inv   <= $signed({2'b00, 8'd255 - x});
                gs    <= $signed({2'b00, g});
                st    <= S_M1;
            end
            S_M1: begin t1 <= delta * inv; st <= S_M2; end
            S_M2: begin t2 <= t1 * gs;     st <= S_ABS; end
            S_ABS: begin
                neg <= t2[25];
                mag <= t2[25] ? (-t2) : t2;
                st  <= S_DIV;
            end
            S_DIV: begin dv_start <= 1'b1; st <= S_WAIT; end
            S_WAIT: if (dv_done) begin
                if (neg) res <= $signed({3'b000, x_l}) - $signed({3'b000, dv_q[7:0]});
                else     res <= $signed({3'b000, x_l}) + $signed({3'b000, dv_q[7:0]});
                st <= S_OUT;
            end
            S_OUT: begin
                if (res < 0)        y <= 8'd0;
                else if (res > 255) y <= 8'd255;
                else                y <= res[7:0];
                done <= 1'b1;
                st   <= S_IDLE;
            end
            default: st <= S_IDLE;
        endcase
    end
endmodule


// ---------------------------------------------------------------------------
// y = min(255, pixel * target / mean), 0 if mean == 0 (same as original).
// ---------------------------------------------------------------------------
module gw_ch (
    input  wire       clk,
    input  wire       start,
    input  wire [7:0] pixel,
    input  wire [7:0] mean,
    input  wire [7:0] target,
    output reg  [7:0] y,
    output reg        done
);
    localparam [1:0] S_IDLE = 0, S_CHK = 1, S_WAIT = 2;
    reg [1:0]  st = S_IDLE;
    reg [15:0] prod;
    reg [7:0]  mean_l;
    reg        dv_start;
    wire [15:0] dv_q;
    wire        dv_done;

    div_seq #(.NW(16), .DW(8)) div_i (
        .clk(clk), .start(dv_start), .num(prod), .den(mean_l),
        .quo(dv_q), .done(dv_done)
    );

    initial begin y = 0; done = 0; dv_start = 0; end

    always @(posedge clk) begin
        done     <= 1'b0;
        dv_start <= 1'b0;
        case (st)
            S_IDLE: if (start) begin
                prod   <= pixel * target;
                mean_l <= mean;
                st     <= S_CHK;
            end
            S_CHK: begin
                if (mean_l == 8'd0) begin
                    y    <= 8'd0;
                    done <= 1'b1;
                    st   <= S_IDLE;
                end else begin
                    dv_start <= 1'b1;
                    st       <= S_WAIT;
                end
            end
            S_WAIT: if (dv_done) begin
                y    <= (dv_q > 16'd255) ? 8'd255 : dv_q[7:0];
                done <= 1'b1;
                st   <= S_IDLE;
            end
            default: st <= S_IDLE;
        endcase
    end
endmodule


// ---------------------------------------------------------------------------
// UART transmitter 8-N-1 (unchanged logic from your version).
// ---------------------------------------------------------------------------
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

    reg [1:0]  state = TX_IDLE;
    reg [31:0] clk_count;
    reg [2:0]  bit_index;
    reg [7:0]  data_latched;

    initial begin tx = 1'b1; busy = 1'b0; done = 1'b0; end

    always @(posedge clk) begin
        if (rst) begin
            state        <= TX_IDLE;
            clk_count    <= 0;
            bit_index    <= 0;
            data_latched <= 0;
            tx           <= 1'b1;
            busy         <= 1'b0;
            done         <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                TX_IDLE: begin
                    tx        <= 1'b1;
                    busy      <= 1'b0;
                    clk_count <= 0;
                    if (start) begin
                        data_latched <= data;
                        tx    <= 1'b0;
                        busy  <= 1'b1;
                        state <= TX_START;
                    end
                end

                TX_START: begin
                    if (clk_count == CLKS_PER_BIT-1) begin
                        clk_count <= 0;
                        bit_index <= 0;
                        tx        <= data_latched[0];
                        state     <= TX_DATA;
                    end else
                        clk_count <= clk_count + 1'b1;
                end

                TX_DATA: begin
                    if (clk_count == CLKS_PER_BIT-1) begin
                        clk_count <= 0;
                        if (bit_index == 3'd7) begin
                            tx    <= 1'b1;
                            state <= TX_STOP;
                        end else begin
                            bit_index <= bit_index + 1'b1;
                            tx        <= data_latched[bit_index + 1'b1];
                        end
                    end else
                        clk_count <= clk_count + 1'b1;
                end

                TX_STOP: begin
                    if (clk_count == CLKS_PER_BIT-1) begin
                        clk_count <= 0;
                        tx    <= 1'b1;
                        busy  <= 1'b0;
                        done  <= 1'b1;
                        state <= TX_IDLE;
                    end else
                        clk_count <= clk_count + 1'b1;
                end

                default: begin
                    state <= TX_IDLE;
                    tx    <= 1'b1;
                    busy  <= 1'b0;
                end
            endcase
        end
    end

endmodule
