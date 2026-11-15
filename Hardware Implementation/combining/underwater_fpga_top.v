`timescale 1ns / 1ps

// RealDigital Boolean board top-level.
// Board: xc7s50csga324-1, 100 MHz oscillator on F14.
// btn[0] = start (press once; press again to RE-SEND the results)
// btn[1] = reset
// led[0] = busy, led[1] = done, led[2] = UART streaming
// led[4:3] = stream stage while streaming: 00 white-balanced, 01 gamma, 10 unsharp
//
// One run = three UART images, back to back, each preceded by a 4-byte header:
//   header = A5 5A <stage> 00        stage: 00 = white balanced
//                                           01 = gamma corrected  (gamma_lut.mem)
//                                           02 = normalized unsharp masking
//   body   = WIDTH*HEIGHT pixels, R,G,B bytes per pixel, row-major, 8-N-1.
//
// The white-balanced image is written in place into the single 24-bit BRAM and
// is NEVER modified afterwards, so gamma and unsharp masking are both computed
// from the white-balanced image (exactly as in Fig. 1 of the paper).
//
// NOTE: btn is [1:0] on purpose (the XDC only constrains btn[0], btn[1]).
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

    wire       busy, done, streaming;
    wire [1:0] stage;

    assign led[0]     = busy;
    assign led[1]     = done;
    assign led[2]     = streaming;
    assign led[4:3]   = stage;
    assign led[15:5]  = 11'b0;

    wb_image_uart_engine #(
        .WIDTH(256),
        .HEIGHT(256),
        .ADDR_WIDTH(16),
        .CLK_HZ(100000000),
        .UART_BAUD(115200)          // must match capture_all.py (default 115200)
    ) engine (
        .clk(clk),
        .rst(rst),
        .start(start_pulse),
        .busy(busy),
        .done(done),
        .streaming(streaming),
        .stage_out(stage),
        .uart_txd(UART_txd)
    );

endmodule


// ---------------------------------------------------------------------------
// Engine. WIDTH*HEIGHT must equal 2**ADDR_WIDTH.
//
//  P1..P3  : your original white-balance passes (unchanged), result written
//            in place into image_mem.
//  MM_*    : unsharp pre-pass. For every pixel/channel D = I - blur3x3(I);
//            find the global min / max of D (needed by the paper's N{.}).
//  stage 0 : stream white-balanced image
//  stage 1 : stream gamma_lut[ white-balanced ]
//  stage 2 : stream S = (I + N{I - G*I}) / 2   (Eq. 6)
//              blur  = (1 2 1 / 2 4 2 / 1 2 1) >> 4, borders replicated
//              N{D}  = floor( (D - Dmin) * 255 / (Dmax - Dmin) )
//              S     = (I + N) >> 1          (if Dmax == Dmin: S = I)
// ---------------------------------------------------------------------------
module wb_image_uart_engine #(
    parameter integer WIDTH      = 256,
    parameter integer HEIGHT     = 256,
    parameter integer ADDR_WIDTH = 16,
    parameter integer CLK_HZ     = 100000000,
    parameter integer UART_BAUD  = 115200
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       start,
    output reg        busy,
    output reg        done,
    output reg        streaming,
    output wire [1:0] stage_out,
    output wire       uart_txd
);

    localparam integer DEPTH = WIDTH * HEIGHT;
    localparam [ADDR_WIDTH-1:0] W_ADDR = WIDTH;

    localparam [5:0]
        IDLE      = 6'd0,
        P1_REQ    = 6'd1,
        P1_WAIT   = 6'd2,
        P1_USE    = 6'd3,
        P1_CALC   = 6'd4,
        P2_REQ    = 6'd5,
        P2_WAIT   = 6'd6,
        P2_LATCH  = 6'd7,
        P2_GO     = 6'd8,
        P2_COMP   = 6'd9,
        P2_CALC   = 6'd10,
        P2_CALC2  = 6'd11,
        P3_REQ    = 6'd12,
        P3_WAIT   = 6'd13,
        P3_LATCH  = 6'd14,
        P3_GO     = 6'd15,
        P3_COMP   = 6'd16,
        P3_GGO    = 6'd17,
        P3_GCOMP  = 6'd18,
        P3_WRITE  = 6'd19,
        FINISHED  = 6'd20,
        // 3x3 neighbourhood fetch (shared by pre-pass and stage 2)
        MM_INIT   = 6'd21,
        NB_INIT   = 6'd22,
        NB_ADDR   = 6'd23,
        NB_REQ    = 6'd24,
        NB_WAIT   = 6'd25,
        NB_USE    = 6'd26,
        NB_DONE   = 6'd27,
        // pre-pass min/max
        MM_R      = 6'd28,
        MM_G      = 6'd29,
        MM_B      = 6'd30,
        MM_NEXT   = 6'd31,
        MM_FIN    = 6'd32,
        // streaming
        STG_BEGIN = 6'd33,
        H_START   = 6'd34,
        H_WAIT    = 6'd35,
        PX_REQ    = 6'd36,
        PX_WAIT   = 6'd37,
        PX_LATCH  = 6'd38,
        G_W1      = 6'd39,
        G_W2      = 6'd40,
        UN_SUB    = 6'd41,
        UN_MUL    = 6'd42,
        UN_GO     = 6'd43,
        UN_WAIT   = 6'd44,
        UN_FIN    = 6'd45,
        TX_START  = 6'd46,
        TX_WAIT   = 6'd47,
        PX_NEXT   = 6'd48;
    reg [5:0] state;

    // Set once the white-balanced image sits in image_mem. Deliberately NOT
    // reset by rst (image_mem is overwritten in place, so running the white
    // balance a second time would double-process it). A later start just
    // re-streams the results.
    reg wb_ready = 1'b0;

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

    // ---------------- gamma LUTs (your gamma_lut.v, 1-clock latency) ------
    reg  [23:0] raw_pix;
    wire [7:0]  gam_r, gam_g, gam_b;

    gamma_lut gl_r (.clk(clk), .pixel_in(raw_pix[23:16]), .pixel_out(gam_r));
    gamma_lut gl_g (.clk(clk), .pixel_in(raw_pix[15:8]),  .pixel_out(gam_g));
    gamma_lut gl_b (.clk(clk), .pixel_in(raw_pix[7:0]),   .pixel_out(gam_b));

    // ---------------- 3x3 neighbourhood / unsharp datapath ----------------
    reg [ADDR_WIDTH-1:0] x_cnt, y_cnt;      // coordinates of the current pixel
    reg [ADDR_WIDTH-1:0] xx_r, yy_r;        // clamped neighbour coordinates
    reg [2:0]  nb_w_r;
    reg [3:0]  k;                           // neighbour index 0..8
    reg        nb_ctx;                      // 0 = min/max pre-pass, 1 = stage 2 output

    reg [11:0] acc_r, acc_g, acc_b;         // weighted sums (max 4080)
    reg [7:0]  ctr_r, ctr_g, ctr_b;         // centre pixel
    reg signed [8:0] d_r, d_g, d_b;         // I - blur, -255..255
    reg signed [8:0] dmin, dmax;
    reg [8:0]  range_v;                     // dmax - dmin, 0..510

    reg [8:0]  dd_r, dd_g, dd_b;            // D - dmin, 0..510
    reg [16:0] num_r, num_g, num_b;         // (D - dmin) * 255
    reg        dv_start;
    wire [16:0] dvr_q, dvg_q, dvb_q;
    wire        dvr_done, dvg_done, dvb_done;

    div_seq #(.NW(17), .DW(9)) un_div_r (.clk(clk), .start(dv_start), .num(num_r),
                                         .den(range_v), .quo(dvr_q), .done(dvr_done));
    div_seq #(.NW(17), .DW(9)) un_div_g (.clk(clk), .start(dv_start), .num(num_g),
                                         .den(range_v), .quo(dvg_q), .done(dvg_done));
    div_seq #(.NW(17), .DW(9)) un_div_b (.clk(clk), .start(dv_start), .num(num_b),
                                         .den(range_v), .quo(dvb_q), .done(dvb_done));

    // neighbour k -> (dy, dx, weight) of the 1-2-1 / 2-4-2 / 1-2-1 kernel
    reg signed [1:0] nb_dy, nb_dx;
    reg        [2:0] nb_w;
    always @* begin
        case (k)
            4'd0:    begin nb_dy = -2'sd1; nb_dx = -2'sd1; nb_w = 3'd1; end
            4'd1:    begin nb_dy = -2'sd1; nb_dx =  2'sd0; nb_w = 3'd2; end
            4'd2:    begin nb_dy = -2'sd1; nb_dx =  2'sd1; nb_w = 3'd1; end
            4'd3:    begin nb_dy =  2'sd0; nb_dx = -2'sd1; nb_w = 3'd2; end
            4'd4:    begin nb_dy =  2'sd0; nb_dx =  2'sd0; nb_w = 3'd4; end
            4'd5:    begin nb_dy =  2'sd0; nb_dx =  2'sd1; nb_w = 3'd2; end
            4'd6:    begin nb_dy =  2'sd1; nb_dx = -2'sd1; nb_w = 3'd1; end
            4'd7:    begin nb_dy =  2'sd1; nb_dx =  2'sd0; nb_w = 3'd2; end
            default: begin nb_dy =  2'sd1; nb_dx =  2'sd1; nb_w = 3'd1; end
        endcase
    end

    // border-replicated neighbour coordinates
    wire [ADDR_WIDTH-1:0] x_m1 = (x_cnt == 0)        ? x_cnt : x_cnt - 1'b1;
    wire [ADDR_WIDTH-1:0] x_p1 = (x_cnt == WIDTH-1)  ? x_cnt : x_cnt + 1'b1;
    wire [ADDR_WIDTH-1:0] y_m1 = (y_cnt == 0)        ? y_cnt : y_cnt - 1'b1;
    wire [ADDR_WIDTH-1:0] y_p1 = (y_cnt == HEIGHT-1) ? y_cnt : y_cnt + 1'b1;

    // unsharp result: S = (I + N) >> 1
    wire [8:0] s_r_sum = {1'b0, ctr_r} + {1'b0, dvr_q[7:0]};
    wire [8:0] s_g_sum = {1'b0, ctr_g} + {1'b0, dvg_q[7:0]};
    wire [8:0] s_b_sum = {1'b0, ctr_b} + {1'b0, dvb_q[7:0]};

    // ---------------- UART ------------------------------------------------
    reg [23:0] out_pixel;
    reg [1:0]  stage;
    reg [1:0]  hdr_cnt;
    reg [1:0]  tx_seq;
    reg [7:0]  tx_data;
    reg        tx_start;
    wire       tx_busy;
    wire       tx_done;

    assign stage_out = stage;

    wire [7:0] hdr_byte = (hdr_cnt == 2'd0) ? 8'hA5 :
                          (hdr_cnt == 2'd1) ? 8'h5A :
                          (hdr_cnt == 2'd2) ? {6'b0, stage} : 8'h00;
    wire [7:0] px_byte  = (tx_seq == 2'd0) ? out_pixel[23:16] :
                          (tx_seq == 2'd1) ? out_pixel[15:8]  : out_pixel[7:0];

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
            tx_data    <= 0;
            tx_start   <= 0;
            raw_pix    <= 0;
            x_cnt <= 0; y_cnt <= 0; xx_r <= 0; yy_r <= 0;
            nb_w_r <= 0; k <= 0; nb_ctx <= 0;
            acc_r <= 0; acc_g <= 0; acc_b <= 0;
            ctr_r <= 0; ctr_g <= 0; ctr_b <= 0;
            d_r <= 0; d_g <= 0; d_b <= 0;
            dmin <= 0; dmax <= 0; range_v <= 0;
            dd_r <= 0; dd_g <= 0; dd_b <= 0;
            num_r <= 0; num_g <= 0; num_b <= 0;
            dv_start <= 0;
            out_pixel <= 0;
            stage <= 0;
            hdr_cnt <= 0;
            tx_seq <= 0;
            // wb_ready intentionally not reset
        end else begin
            // one-clock pulses
            tx_start <= 0;
            c_start  <= 0;
            g_start  <= 0;
            dv_start <= 0;

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
                        if (wb_ready) state <= MM_INIT;   // just re-stream
                        else          state <= P1_REQ;
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
                        wb_ready <= 1'b1;
                        state    <= MM_INIT;
                    end else begin
                        pixel_index <= pixel_index + 1'b1;
                        state <= P3_REQ;
                    end
                end

                // ---------- unsharp pre-pass: global min / max of D --------
                MM_INIT: begin
                    dmin        <= 9'sd255;
                    dmax        <= -9'sd255;
                    pixel_index <= 0;
                    x_cnt       <= 0;
                    y_cnt       <= 0;
                    nb_ctx      <= 1'b0;
                    state       <= NB_INIT;
                end

                // ---------- 3x3 neighbourhood fetch -------------------------
                NB_INIT: begin
                    k     <= 0;
                    acc_r <= 0; acc_g <= 0; acc_b <= 0;
                    state <= NB_ADDR;
                end
                NB_ADDR: begin
                    xx_r   <= nb_dx[1] ? x_m1 : (nb_dx[0] ? x_p1 : x_cnt);
                    yy_r   <= nb_dy[1] ? y_m1 : (nb_dy[0] ? y_p1 : y_cnt);
                    nb_w_r <= nb_w;
                    state  <= NB_REQ;
                end
                NB_REQ: begin
                    rd_addr <= (yy_r * W_ADDR) + xx_r;
                    state   <= NB_WAIT;
                end
                NB_WAIT: state <= NB_USE;
                NB_USE: begin
                    acc_r <= acc_r + (rd_data[23:16] * nb_w_r);
                    acc_g <= acc_g + (rd_data[15:8]  * nb_w_r);
                    acc_b <= acc_b + (rd_data[7:0]   * nb_w_r);
                    if (k == 4'd4) begin
                        ctr_r <= rd_data[23:16];
                        ctr_g <= rd_data[15:8];
                        ctr_b <= rd_data[7:0];
                    end
                    if (k == 4'd8)
                        state <= NB_DONE;
                    else begin
                        k     <= k + 1'b1;
                        state <= NB_ADDR;
                    end
                end
                NB_DONE: begin
                    d_r <= {1'b0, ctr_r} - {1'b0, acc_r[11:4]};
                    d_g <= {1'b0, ctr_g} - {1'b0, acc_g[11:4]};
                    d_b <= {1'b0, ctr_b} - {1'b0, acc_b[11:4]};
                    state <= nb_ctx ? UN_SUB : MM_R;
                end

                MM_R: begin
                    if (d_r < dmin) dmin <= d_r;
                    if (d_r > dmax) dmax <= d_r;
                    state <= MM_G;
                end
                MM_G: begin
                    if (d_g < dmin) dmin <= d_g;
                    if (d_g > dmax) dmax <= d_g;
                    state <= MM_B;
                end
                MM_B: begin
                    if (d_b < dmin) dmin <= d_b;
                    if (d_b > dmax) dmax <= d_b;
                    state <= MM_NEXT;
                end
                MM_NEXT: begin
                    if (pixel_index == DEPTH-1)
                        state <= MM_FIN;
                    else begin
                        pixel_index <= pixel_index + 1'b1;
                        if (x_cnt == WIDTH-1) begin
                            x_cnt <= 0;
                            y_cnt <= y_cnt + 1'b1;
                        end else
                            x_cnt <= x_cnt + 1'b1;
                        state <= NB_INIT;
                    end
                end
                MM_FIN: begin
                    range_v <= dmax - dmin;
                    state   <= STG_BEGIN;
                end

                // ---------- UART streaming of the three images -------------
                STG_BEGIN: begin
                    stage     <= 2'd0;
                    hdr_cnt   <= 2'd0;
                    streaming <= 1'b1;
                    state     <= H_START;
                end

                H_START: if (!tx_busy) begin
                    tx_data  <= hdr_byte;
                    tx_start <= 1'b1;
                    state    <= H_WAIT;
                end
                H_WAIT: if (tx_done) begin
                    if (hdr_cnt == 2'd3) begin
                        hdr_cnt     <= 2'd0;
                        pixel_index <= 0;
                        x_cnt       <= 0;
                        y_cnt       <= 0;
                        nb_ctx      <= 1'b1;
                        state       <= (stage == 2'd2) ? NB_INIT : PX_REQ;
                    end else begin
                        hdr_cnt <= hdr_cnt + 1'b1;
                        state   <= H_START;
                    end
                end

                // stage 0 / 1: read white-balanced pixel
                PX_REQ:  begin rd_addr <= pixel_index; state <= PX_WAIT; end
                PX_WAIT: state <= PX_LATCH;
                PX_LATCH: begin
                    raw_pix <= rd_data;
                    if (stage == 2'd0) begin
                        out_pixel <= rd_data;
                        tx_seq    <= 2'd0;
                        state     <= TX_START;
                    end else
                        state <= G_W1;
                end
                G_W1: state <= G_W2;                 // LUT output registers
                G_W2: begin
                    out_pixel <= {gam_r, gam_g, gam_b};
                    tx_seq    <= 2'd0;
                    state     <= TX_START;
                end

                // stage 2: normalized unsharp masking of the pixel
                UN_SUB: begin
                    dd_r <= d_r - dmin;
                    dd_g <= d_g - dmin;
                    dd_b <= d_b - dmin;
                    state <= UN_MUL;
                end
                UN_MUL: begin
                    num_r <= {dd_r, 8'b0} - dd_r;     // dd * 255
                    num_g <= {dd_g, 8'b0} - dd_g;
                    num_b <= {dd_b, 8'b0} - dd_b;
                    state <= (range_v == 9'd0) ? UN_FIN : UN_GO;
                end
                UN_GO: begin
                    dv_start <= 1'b1;
                    state    <= UN_WAIT;
                end
                UN_WAIT: if (dvr_done && dvg_done && dvb_done) state <= UN_FIN;
                UN_FIN: begin
                    if (range_v == 9'd0)
                        out_pixel <= {ctr_r, ctr_g, ctr_b};   // flat image: S = I
                    else
                        out_pixel <= {s_r_sum[8:1], s_g_sum[8:1], s_b_sum[8:1]};
                    tx_seq <= 2'd0;
                    state  <= TX_START;
                end

                // send R, G, B of out_pixel
                TX_START: if (!tx_busy) begin
                    tx_data  <= px_byte;
                    tx_start <= 1'b1;
                    state    <= TX_WAIT;
                end
                TX_WAIT: if (tx_done) begin
                    if (tx_seq == 2'd2)
                        state <= PX_NEXT;
                    else begin
                        tx_seq <= tx_seq + 1'b1;
                        state  <= TX_START;
                    end
                end

                PX_NEXT: begin
                    if (pixel_index == DEPTH-1) begin
                        if (stage == 2'd2)
                            state <= FINISHED;
                        else begin
                            stage   <= stage + 1'b1;
                            hdr_cnt <= 2'd0;
                            state   <= H_START;
                        end
                    end else begin
                        pixel_index <= pixel_index + 1'b1;
                        if (x_cnt == WIDTH-1) begin
                            x_cnt <= 0;
                            y_cnt <= y_cnt + 1'b1;
                        end else
                            x_cnt <= x_cnt + 1'b1;
                        state <= (stage == 2'd2) ? NB_INIT : PX_REQ;
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
