`timescale 1ns / 1ps

// RealDigital Boolean board top-level for complete Underwater Image Color
// Correction & Single-Scale Fusion Engine.
// Board: xc7s50csga324-1, 100 MHz oscillator on F14.
// btn[0] = start (press once; press again to RE-SEND the results)
// btn[1] = reset
// led[0] = busy, led[1] = done, led[2] = UART streaming
// led[5:3] = stream stage while streaming:
//            000 = white-balanced
//            001 = gamma-corrected (Input 1)
//            010 = normalized unsharp masking (Input 2)
//            011 = single-scale fused output
//            100 = normalized fusion weights (R = W1_norm, G = W2_norm, B = 0)
// led[8:6] = compute phase indicator
//
// One run = five UART images, back to back, each preceded by a 4-byte header:
//   header = A5 5A <stage> 00
//   body   = WIDTH*HEIGHT pixels, R,G,B bytes per pixel, row-major, 8-N-1.
//
// The white-balanced image is written in place into the single 24-bit BRAM and
// is NEVER modified afterwards. Gamma, unsharp masking, quality weights
// (Laplacian, Saliency, Saturation), and single-scale fusion are all computed
// on the fly from the white-balanced BRAM.
//
// NOTE: btn is [1:0] on purpose (the XDC only constrains btn[0] and btn[1]).
module underwater_fusion_top #(
    parameter [1:0] BETA_MODE = 2'b01  // 00: beta=0.5, 01: beta=1.0 (default), 10: beta=1.5, 11: beta=2.0
)(
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
    wire [2:0] stage;
    wire [2:0] phase;

    assign led[0]    = busy;
    assign led[1]    = done;
    assign led[2]    = streaming;
    assign led[5:3]  = stage;
    assign led[8:6]  = phase;
    assign led[15:9] = 7'b0;

    underwater_fusion_engine #(
        .WIDTH(256),
        .HEIGHT(256),
        .ADDR_WIDTH(16),
        .CLK_HZ(100000000),
        .UART_BAUD(115200),
        .STRETCH_PCT(0),
        .BETA_MODE(BETA_MODE)
    ) engine (
        .clk(clk),
        .rst(rst),
        .start(start_pulse),
        .busy(busy),
        .done(done),
        .streaming(streaming),
        .stage_out(stage),
        .phase_out(phase),
        .uart_txd(UART_txd)
    );

endmodule


// ---------------------------------------------------------------------------
// Complete Multi-Pass + Single-Scale Fusion Engine.
// WIDTH*HEIGHT must equal 2**ADDR_WIDTH.
//
// Passes:
//  P1..P3  : Underwater White Balance (red/blue compensation + Gray World),
//            result written in place into image_mem.
//  SLM_*   : Saliency Lab mean pre-pass: global mean CIE Lab of blurred I1, I2.
//  WMM_*   : Weight map min/max pre-pass: global min/max of W_L, W_S, W_Sat
//            for both Input 1 (gamma) and Input 2 (enhanced unsharp).
//  FMM_*   : Fused image histogram & percentile/min-max linear stretch bounds.
//  Streaming stages 0..4:
//   stage 0: white-balanced image
//   stage 1: gamma_lut[ white-balanced ] (Input 1)
//   stage 2: enhanced unsharp masking    (Input 2: S = clip(I + beta*(I - G*I)))
//   stage 3: single-scale fusion R = Wbar1 * I1 + Wbar2 * I2 (linearly stretched)
//   stage 4: normalized weights  {Wbar1, Wbar2, 0}
// ---------------------------------------------------------------------------
module underwater_fusion_engine #(
    parameter integer WIDTH       = 256,
    parameter integer HEIGHT      = 256,
    parameter integer ADDR_WIDTH  = 16,
    parameter integer CLK_HZ      = 100000000,
    parameter integer UART_BAUD   = 115200,
    parameter integer STRETCH_PCT = 0,
    parameter [1:0]   BETA_MODE   = 2'b01  // 00: 0.5, 01: 1.0 (default), 10: 1.5, 11: 2.0
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       start,
    output reg        busy,
    output reg        done,
    output reg        streaming,
    output wire [2:0] stage_out,
    output reg  [2:0] phase_out,
    output wire       uart_txd
);

    localparam integer DEPTH = WIDTH * HEIGHT;
    localparam [ADDR_WIDTH-1:0] W_ADDR = WIDTH;
    localparam [16:0] PCT_LO = (STRETCH_PCT == 0) ? 17'd1 : ((DEPTH * STRETCH_PCT) / 100);
    localparam [16:0] PCT_HI = (STRETCH_PCT == 0) ? DEPTH : (DEPTH - ((DEPTH * STRETCH_PCT) / 100));

    localparam [1:0]
        CTX_UNSHARP = 2'd1,
        CTX_FUSION  = 2'd2;

    localparam [1:0]
        FMODE_SLM    = 2'd0,
        FMODE_WMM    = 2'd1,
        FMODE_STREAM = 2'd2,
        FMODE_FMM    = 2'd3;

    localparam [6:0]
        IDLE         = 7'd0,
        P1_REQ       = 7'd1,
        P1_WAIT      = 7'd2,
        P1_USE       = 7'd3,
        P1_CALC      = 7'd4,
        P2_REQ       = 7'd5,
        P2_WAIT      = 7'd6,
        P2_LATCH     = 7'd7,
        P2_GO        = 7'd8,
        P2_COMP      = 7'd9,
        P2_CALC      = 7'd10,
        P2_CALC2     = 7'd11,
        P3_REQ       = 7'd12,
        P3_WAIT      = 7'd13,
        P3_LATCH     = 7'd14,
        P3_GO        = 7'd15,
        P3_COMP      = 7'd16,
        P3_GGO       = 7'd17,
        P3_GCOMP     = 7'd18,
        P3_WRITE     = 7'd19,
        FINISHED     = 7'd20,
        // 3x3 inner neighbourhood fetch
        NB_INIT      = 7'd22,
        NB_ADDR      = 7'd23,
        NB_REQ       = 7'd24,
        NB_WAIT      = 7'd25,
        NB_USE       = 7'd26,
        NB_DONE      = 7'd27,
        // streaming header + stage 0/1/2
        STG_BEGIN    = 7'd33,
        H_START      = 7'd34,
        H_WAIT       = 7'd35,
        PX_REQ       = 7'd36,
        PX_WAIT      = 7'd37,
        PX_LATCH     = 7'd38,
        G_W1         = 7'd39,
        G_W2         = 7'd40,
        UN_FIN       = 7'd45,
        TX_START     = 7'd46,
        TX_WAIT      = 7'd47,
        PX_NEXT      = 7'd48,
        // saliency Lab mean + weight min/max + outer 3x3 fusion
        SLM_INIT     = 7'd49,
        SLM_FIN      = 7'd50,
        WMM_INIT     = 7'd51,
        WMM_FIN      = 7'd52,
        OUTER_INIT   = 7'd53,
        OUTER_ADDR   = 7'd54,
        OUTER_ACC1   = 7'd55,
        OUTER_ACC2   = 7'd56,
        LAB_GO       = 7'd57,
        LAB_WAIT     = 7'd58,
        WT_DIFF      = 7'd59,
        WT_SQ1       = 7'd60,
        WT_SQ2       = 7'd61,
        WT_SQ3       = 7'd62,
        WT_SQRT_GO   = 7'd63,
        WT_SQRT_WAIT = 7'd64,
        WN_PREP      = 7'd65,
        WN_GO        = 7'd66,
        WN_WAIT      = 7'd67,
        FUS_PREP     = 7'd68,
        FUS_GO       = 7'd69,
        FUS_WAIT     = 7'd70,
        FUS_MUL      = 7'd71,
        FUS_ADD      = 7'd72,
        // final image linear stretch histogram pass & dividers
        FMM_INIT     = 7'd73,
        FMM_CLR      = 7'd74,
        FMM_HIST_RD  = 7'd75,
        FMM_HIST_WR  = 7'd76,
        FMM_CDF_RD   = 7'd77,
        FMM_CDF_USE  = 7'd78,
        FMM_FIN      = 7'd79,
        STR_PREP     = 7'd80,
        STR_GO       = 7'd81,
        STR_WAIT     = 7'd82;

    reg [6:0] state = IDLE;

    // Set once the white-balanced image sits in image_mem. Deliberately NOT
    // reset by rst so re-running does not double-process image_mem.
    reg wb_ready = 1'b0;

    // ---------------- image memory (single 24-bit BRAM) -------------------
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

    // ---------------- white balance statistics ----------------------------
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
    reg  [2:0] gw_done_mask;
    wire [7:0] gr_y, gg_y, gb_y;
    wire gr_done, gg_done, gb_done;
    wire [2:0] gw_done_next = gw_done_mask | {gr_done, gg_done, gb_done};

    gw_ch gw_r (.clk(clk), .start(g_start), .pixel(rc_val), .mean(mean_rc),
                .target(target_mean), .y(gr_y), .done(gr_done));
    gw_ch gw_g (.clk(clk), .start(g_start), .pixel(pix_g),  .mean(mean_gc),
                .target(target_mean), .y(gg_y), .done(gg_done));
    gw_ch gw_b (.clk(clk), .start(g_start), .pixel(bc_val), .mean(mean_bc),
                .target(target_mean), .y(gb_y), .done(gb_done));

    // ---------------- gamma LUTs (1-clock latency) ------------------------
    reg  [23:0] raw_pix;
    wire [7:0]  gam_r, gam_g, gam_b;

    gamma_lut gl_r (.clk(clk), .pixel_in(raw_pix[23:16]), .pixel_out(gam_r));
    gamma_lut gl_g (.clk(clk), .pixel_in(raw_pix[15:8]),  .pixel_out(gam_g));
    gamma_lut gl_b (.clk(clk), .pixel_in(raw_pix[7:0]),   .pixel_out(gam_b));

    // ---------------- 3x3 inner & outer neighbourhood ---------------------
    reg [ADDR_WIDTH-1:0] x_cnt, y_cnt;      // current outer pixel
    reg [ADDR_WIDTH-1:0] ux, uy;            // centre pixel for inner 3x3 unsharp
    reg [ADDR_WIDTH-1:0] xx_r, yy_r;        // clamped inner neighbour coords
    reg [2:0]  nb_w_r;
    reg [3:0]  k;                           // inner neighbour index 0..8
    reg [1:0]  nb_ctx;                      // CTX_MM, CTX_UNSHARP, CTX_FUSION
    reg [1:0]  fusion_mode;                 // FMODE_SLM, FMODE_WMM, FMODE_STREAM

    reg [3:0]  m;                           // outer neighbour index 0..8
    reg [2:0]  om_w_r;

    reg [11:0] acc_r, acc_g, acc_b;         // inner blur sums (max 4080)
    reg [7:0]  ctr_r, ctr_g, ctr_b;         // inner centre pixel

    // Inner neighbour offset / Gaussian weight
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

    // Outer neighbour offset / Gaussian weight
    reg signed [1:0] om_dy, om_dx;
    reg        [2:0] om_w;
    always @* begin
        case (m)
            4'd0:    begin om_dy = -2'sd1; om_dx = -2'sd1; om_w = 3'd1; end
            4'd1:    begin om_dy = -2'sd1; om_dx =  2'sd0; om_w = 3'd2; end
            4'd2:    begin om_dy = -2'sd1; om_dx =  2'sd1; om_w = 3'd1; end
            4'd3:    begin om_dy =  2'sd0; om_dx = -2'sd1; om_w = 3'd2; end
            4'd4:    begin om_dy =  2'sd0; om_dx =  2'sd0; om_w = 3'd4; end
            4'd5:    begin om_dy =  2'sd0; om_dx =  2'sd1; om_w = 3'd2; end
            4'd6:    begin om_dy =  2'sd1; om_dx = -2'sd1; om_w = 3'd1; end
            4'd7:    begin om_dy =  2'sd1; om_dx =  2'sd0; om_w = 3'd2; end
            default: begin om_dy =  2'sd1; om_dx =  2'sd1; om_w = 3'd1; end
        endcase
    end

    // Border-replicated outer & inner neighbour coordinates
    wire [ADDR_WIDTH-1:0] x_m1  = (x_cnt == 0)        ? x_cnt : x_cnt - 1'b1;
    wire [ADDR_WIDTH-1:0] x_p1  = (x_cnt == WIDTH-1)  ? x_cnt : x_cnt + 1'b1;
    wire [ADDR_WIDTH-1:0] y_m1  = (y_cnt == 0)        ? y_cnt : y_cnt - 1'b1;
    wire [ADDR_WIDTH-1:0] y_p1  = (y_cnt == HEIGHT-1) ? y_cnt : y_cnt + 1'b1;

    wire [ADDR_WIDTH-1:0] ux_m1 = (ux == 0)           ? ux    : ux - 1'b1;
    wire [ADDR_WIDTH-1:0] ux_p1 = (ux == WIDTH-1)     ? ux    : ux + 1'b1;
    wire [ADDR_WIDTH-1:0] uy_m1 = (uy == 0)           ? uy    : uy - 1'b1;
    wire [ADDR_WIDTH-1:0] uy_p1 = (uy == HEIGHT-1)    ? uy    : uy + 1'b1;

    // Enhanced Unsharp Masking: S = clip(I + beta*(I - G*I), 0, 255)
    wire signed [9:0] d_r_s = $signed({1'b0, ctr_r}) - $signed({1'b0, acc_r[11:4]});
    wire signed [9:0] d_g_s = $signed({1'b0, ctr_g}) - $signed({1'b0, acc_g[11:4]});
    wire signed [9:0] d_b_s = $signed({1'b0, ctr_b}) - $signed({1'b0, acc_b[11:4]});

    reg signed [10:0] d_term_r, d_term_g, d_term_b;
    always @* begin
        case (BETA_MODE)
            2'b00: begin // beta = 0.5
                d_term_r = {{2{d_r_s[9]}}, d_r_s[9:1]};
                d_term_g = {{2{d_g_s[9]}}, d_g_s[9:1]};
                d_term_b = {{2{d_b_s[9]}}, d_b_s[9:1]};
            end
            2'b10: begin // beta = 1.5
                d_term_r = $signed({{1{d_r_s[9]}}, d_r_s}) + $signed({{2{d_r_s[9]}}, d_r_s[9:1]});
                d_term_g = $signed({{1{d_g_s[9]}}, d_g_s}) + $signed({{2{d_g_s[9]}}, d_g_s[9:1]});
                d_term_b = $signed({{1{d_b_s[9]}}, d_b_s}) + $signed({{2{d_b_s[9]}}, d_b_s[9:1]});
            end
            2'b11: begin // beta = 2.0
                d_term_r = $signed({d_r_s, 1'b0});
                d_term_g = $signed({d_g_s, 1'b0});
                d_term_b = $signed({d_b_s, 1'b0});
            end
            default: begin // beta = 1.0 (default)
                d_term_r = {{1{d_r_s[9]}}, d_r_s};
                d_term_g = {{1{d_g_s[9]}}, d_g_s};
                d_term_b = {{1{d_b_s[9]}}, d_b_s};
            end
        endcase
    end

    wire signed [11:0] val_r = $signed({3'b000, ctr_r}) + $signed({{1{d_term_r[10]}}, d_term_r});
    wire signed [11:0] val_g = $signed({3'b000, ctr_g}) + $signed({{1{d_term_g[10]}}, d_term_g});
    wire signed [11:0] val_b = $signed({3'b000, ctr_b}) + $signed({{1{d_term_b[10]}}, d_term_b});

    wire [7:0] sharp_r = val_r[11] ? 8'd0 : (val_r[10:8] != 3'b000 ? 8'd255 : val_r[7:0]);
    wire [7:0] sharp_g = val_g[11] ? 8'd0 : (val_g[10:8] != 3'b000 ? 8'd255 : val_g[7:0]);
    wire [7:0] sharp_b = val_b[11] ? 8'd0 : (val_b[10:8] != 3'b000 ? 8'd255 : val_b[7:0]);

    // ---------------- Weight & Single-Scale Fusion datapath ---------------
    reg [23:0] i1_nb, i2_nb;
    reg [23:0] i1_ctr, i2_ctr;
    reg [11:0] blur1_r, blur1_g, blur1_b;
    reg [11:0] blur2_r, blur2_g, blur2_b;
    reg [15:0] lum1_p, lum2_p;
    reg signed [15:0] lap1_acc, lap2_acc;
    wire [15:0] wL1_abs = lap1_acc[15] ? (-lap1_acc) : lap1_acc;
    wire [15:0] wL2_abs = lap2_acc[15] ? (-lap2_acc) : lap2_acc;
    reg [11:0] wL1, wL2;
    reg [15:0] lc1_sum, lc2_sum;

    // CIE Lab converters (for Input 1 and Input 2 blurred pixels)
    reg         lab_start;
    wire [9:0]  lab1_L, lab2_L;
    wire signed [12:0] lab1_a, lab1_b, lab2_a, lab2_b;
    wire        lab1_done, lab2_done;

    rgb_to_lab_fp lab1_i (
        .clk(clk), .start(lab_start),
        .r_in(blur1_r[11:4]), .g_in(blur1_g[11:4]), .b_in(blur1_b[11:4]),
        .l_out(lab1_L), .a_out(lab1_a), .b_out(lab1_b), .done(lab1_done)
    );
    rgb_to_lab_fp lab2_i (
        .clk(clk), .start(lab_start),
        .r_in(blur2_r[11:4]), .g_in(blur2_g[11:4]), .b_in(blur2_b[11:4]),
        .l_out(lab2_L), .a_out(lab2_a), .b_out(lab2_b), .done(lab2_done)
    );

    // Saliency Lab accumulators & global means
    reg [31:0]        sum_L1, sum_L2;
    reg signed [31:0] sum_a1, sum_b1, sum_a2, sum_b2;
    reg [9:0]         mean_L1, mean_L2;
    reg signed [12:0] mean_a1, mean_b1, mean_a2, mean_b2;

    // Saliency & Saturation squared-distance pipelines (signed so squares work!)
    reg signed [10:0] dL1, dL2;
    reg signed [13:0] da1, db1, da2, db2;
    reg signed [8:0]  dr1, dg1, dbb1, dr2, dg2, dbb2;
    reg signed [27:0] sq_dL1, sq_da1, sq_db1, sq_dL2, sq_da2, sq_db2;
    reg signed [19:0] sq_sr1, sq_sg1, sq_sb1, sq_sr2, sq_sg2, sq_sb2;

    reg [23:0] sal_rad1, sal_rad2;
    reg [16:0] sat_sum1, sat_sum2;
    reg [31:0] sat_mul1, sat_mul2;
    reg [19:0] sat_rad1, sat_rad2;

    reg         sqrt_start;
    reg  [3:0]  sqrt_done_mask;
    wire [11:0] sal1_root, sal2_root, sat1_root, sat2_root;
    wire        sal1_sq_done, sal2_sq_done, sat1_sq_done, sat2_sq_done;
    wire [3:0]  sqrt_done_next = sqrt_done_mask | {sal1_sq_done, sal2_sq_done, sat1_sq_done, sat2_sq_done};

    sqrt_seq #(.NW(24)) sq_sal1 (.clk(clk), .start(sqrt_start), .num(sal_rad1),
                                 .root(sal1_root), .done(sal1_sq_done));
    sqrt_seq #(.NW(24)) sq_sal2 (.clk(clk), .start(sqrt_start), .num(sal_rad2),
                                 .root(sal2_root), .done(sal2_sq_done));
    sqrt_seq #(.NW(24)) sq_sat1 (.clk(clk), .start(sqrt_start), .num({4'b0000, sat_rad1}),
                                 .root(sat1_root), .done(sat1_sq_done));
    sqrt_seq #(.NW(24)) sq_sat2 (.clk(clk), .start(sqrt_start), .num({4'b0000, sat_rad2}),
                                 .root(sat2_root), .done(sat2_sq_done));

    // Raw 12-bit weights and global min/max/range
    reg [11:0] wS1, wS2, wSat1, wSat2;
    reg [11:0] wL1_min, wL1_max, wL1_rng;
    reg [11:0] wS1_min, wS1_max, wS1_rng;
    reg [11:0] wSat1_min, wSat1_max, wSat1_rng;
    reg [11:0] wL2_min, wL2_max, wL2_rng;
    reg [11:0] wS2_min, wS2_max, wS2_rng;
    reg [11:0] wSat2_min, wSat2_max, wSat2_rng;

    // 6 parallel normalizers to [0..255]
    reg [19:0] wn_num_L1, wn_num_S1, wn_num_Sat1;
    reg [19:0] wn_num_L2, wn_num_S2, wn_num_Sat2;
    reg        wn_dv_start;
    wire [19:0] wn_q_L1, wn_q_S1, wn_q_Sat1, wn_q_L2, wn_q_S2, wn_q_Sat2;
    wire        wn_d_L1, wn_d_S1, wn_d_Sat1, wn_d_L2, wn_d_S2, wn_d_Sat2;

    div_seq #(.NW(20), .DW(12)) div_wn_L1   (.clk(clk), .start(wn_dv_start), .num(wn_num_L1),
                                             .den(wL1_rng == 12'd0 ? 12'd1 : wL1_rng), .quo(wn_q_L1), .done(wn_d_L1));
    div_seq #(.NW(20), .DW(12)) div_wn_S1   (.clk(clk), .start(wn_dv_start), .num(wn_num_S1),
                                             .den(wS1_rng == 12'd0 ? 12'd1 : wS1_rng), .quo(wn_q_S1), .done(wn_d_S1));
    div_seq #(.NW(20), .DW(12)) div_wn_Sat1 (.clk(clk), .start(wn_dv_start), .num(wn_num_Sat1),
                                             .den(wSat1_rng == 12'd0 ? 12'd1 : wSat1_rng), .quo(wn_q_Sat1), .done(wn_d_Sat1));
    div_seq #(.NW(20), .DW(12)) div_wn_L2   (.clk(clk), .start(wn_dv_start), .num(wn_num_L2),
                                             .den(wL2_rng == 12'd0 ? 12'd1 : wL2_rng), .quo(wn_q_L2), .done(wn_d_L2));
    div_seq #(.NW(20), .DW(12)) div_wn_S2   (.clk(clk), .start(wn_dv_start), .num(wn_num_S2),
                                             .den(wS2_rng == 12'd0 ? 12'd1 : wS2_rng), .quo(wn_q_S2), .done(wn_d_S2));
    div_seq #(.NW(20), .DW(12)) div_wn_Sat2 (.clk(clk), .start(wn_dv_start), .num(wn_num_Sat2),
                                             .den(wSat2_rng == 12'd0 ? 12'd1 : wSat2_rng), .quo(wn_q_Sat2), .done(wn_d_Sat2));

    wire [7:0] nwL1   = (wL1_rng   == 12'd0) ? 8'd0 : wn_q_L1[7:0];
    wire [7:0] nwS1   = (wS1_rng   == 12'd0) ? 8'd0 : wn_q_S1[7:0];
    wire [7:0] nwSat1 = (wSat1_rng == 12'd0) ? 8'd0 : wn_q_Sat1[7:0];
    wire [7:0] nwL2   = (wL2_rng   == 12'd0) ? 8'd0 : wn_q_L2[7:0];
    wire [7:0] nwS2   = (wS2_rng   == 12'd0) ? 8'd0 : wn_q_S2[7:0];
    wire [7:0] nwSat2 = (wSat2_rng == 12'd0) ? 8'd0 : wn_q_Sat2[7:0];

    reg [9:0]  w1_agg, w2_agg;
    reg [17:0] fus_num;
    reg [10:0] fus_den;
    reg        fus_dv_start;
    wire [17:0] fus_dv_q;
    wire        fus_dv_done;

    div_seq #(.NW(18), .DW(11)) div_fus (
        .clk(clk), .start(fus_dv_start), .num(fus_num), .den(fus_den),
        .quo(fus_dv_q), .done(fus_dv_done)
    );

    reg [8:0]  nw1_reg, nw2_reg;
    reg [16:0] fm1_r, fm2_r, fm1_g, fm2_g, fm1_b, fm2_b;
    wire [16:0] fsum_r = fm1_r + fm2_r + 17'd128;
    wire [16:0] fsum_g = fm1_g + fm2_g + 17'd128;
    wire [16:0] fsum_b = fm1_b + fm2_b + 17'd128;
    wire [7:0]  fused_r_w = fsum_r[16] ? 8'd255 : fsum_r[15:8];
    wire [7:0]  fused_g_w = fsum_g[16] ? 8'd255 : fsum_g[15:8];
    wire [7:0]  fused_b_w = fsum_b[16] ? 8'd255 : fsum_b[15:8];

    // ---------------- Final fused linear stretching (histogram CDF) -------
    reg [16:0] hist_r [0:255];
    reg [16:0] hist_g [0:255];
    reg [16:0] hist_b [0:255];
    reg [16:0] hval_r, hval_g, hval_b;
    reg [7:0]  f_r_q, f_g_q, f_b_q;
    reg [7:0]  hist_idx;
    reg [7:0]  h_addr_r, h_addr_g, h_addr_b;

    always @* begin
        if (state == FMM_HIST_RD || state == FMM_HIST_WR) begin
            h_addr_r = f_r_q;
            h_addr_g = f_g_q;
            h_addr_b = f_b_q;
        end else begin
            h_addr_r = hist_idx;
            h_addr_g = hist_idx;
            h_addr_b = hist_idx;
        end
    end

    always @(posedge clk) begin
        hval_r <= hist_r[h_addr_r];
        hval_g <= hist_g[h_addr_g];
        hval_b <= hist_b[h_addr_b];
        if (state == FMM_CLR) begin
            hist_r[h_addr_r] <= 17'd0;
            hist_g[h_addr_g] <= 17'd0;
            hist_b[h_addr_b] <= 17'd0;
        end else if (state == FMM_HIST_WR) begin
            hist_r[h_addr_r] <= hval_r + 1'b1;
            hist_g[h_addr_g] <= hval_g + 1'b1;
            hist_b[h_addr_b] <= hval_b + 1'b1;
        end
    end

    reg [16:0] cdf_r, cdf_g, cdf_b;
    wire [16:0] cdf_r_next = cdf_r + hval_r;
    wire [16:0] cdf_g_next = cdf_g + hval_g;
    wire [16:0] cdf_b_next = cdf_b + hval_b;
    reg        found_lo_r, found_hi_r, found_lo_g, found_hi_g, found_lo_b, found_hi_b;
    reg [7:0]  f_lo_r, f_hi_r, f_rng_r;
    reg [7:0]  f_lo_g, f_hi_g, f_rng_g;
    reg [7:0]  f_lo_b, f_hi_b, f_rng_b;
    reg        f_flat_r, f_flat_g, f_flat_b;

    wire [7:0] fs_cl_r = (f_r_q <= f_lo_r) ? 8'd0 : ((f_r_q >= f_hi_r) ? f_rng_r : (f_r_q - f_lo_r));
    wire [7:0] fs_cl_g = (f_g_q <= f_lo_g) ? 8'd0 : ((f_g_q >= f_hi_g) ? f_rng_g : (f_g_q - f_lo_g));
    wire [7:0] fs_cl_b = (f_b_q <= f_lo_b) ? 8'd0 : ((f_b_q >= f_hi_b) ? f_rng_b : (f_b_q - f_lo_b));

    reg [15:0] str_num_r, str_num_g, str_num_b;
    reg        str_dv_start;
    wire [15:0] str_q_r, str_q_g, str_q_b;
    wire        str_d_r, str_d_g, str_d_b;

    div_seq #(.NW(16), .DW(8)) str_div_r (.clk(clk), .start(str_dv_start), .num(str_num_r),
                                          .den(f_rng_r), .quo(str_q_r), .done(str_d_r));
    div_seq #(.NW(16), .DW(8)) str_div_g (.clk(clk), .start(str_dv_start), .num(str_num_g),
                                          .den(f_rng_g), .quo(str_q_g), .done(str_d_g));
    div_seq #(.NW(16), .DW(8)) str_div_b (.clk(clk), .start(str_dv_start), .num(str_num_b),
                                          .den(f_rng_b), .quo(str_q_b), .done(str_d_b));

    // ---------------- UART ------------------------------------------------
    reg [23:0] out_pixel;
    reg [2:0]  stage;
    reg [1:0]  hdr_cnt;
    reg [1:0]  tx_seq;
    reg [7:0]  tx_data;
    reg        tx_start;
    wire       tx_busy;
    wire       tx_done;

    assign stage_out = stage;

    wire [7:0] hdr_byte = (hdr_cnt == 2'd0) ? 8'hA5 :
                          (hdr_cnt == 2'd1) ? 8'h5A :
                          (hdr_cnt == 2'd2) ? {5'b0, stage} : 8'h00;
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
            phase_out   <= 3'd0;
            sum_r  <= 0; sum_g  <= 0; sum_b  <= 0;
            sum_rc <= 0; sum_gc <= 0; sum_bc <= 0;
            mean_r  <= 0; mean_g  <= 0; mean_b  <= 0;
            mean_rc <= 0; mean_gc <= 0; mean_bc <= 0;
            target_mean <= 0;
            tsum       <= 0;
            pix_r <= 0; pix_g <= 0; pix_b <= 0;
            rc_val <= 0; bc_val <= 0;
            c_start <= 0;
            g_start <= 0; gw_done_mask <= 3'b000;
            tx_data    <= 0;
            tx_start   <= 0;
            raw_pix    <= 0;
            x_cnt <= 0; y_cnt <= 0; ux <= 0; uy <= 0;
            xx_r <= 0; yy_r <= 0;
            nb_w_r <= 0; k <= 0; nb_ctx <= 0; fusion_mode <= 0;
            m <= 0; om_w_r <= 0;
            acc_r <= 0; acc_g <= 0; acc_b <= 0;
            ctr_r <= 0; ctr_g <= 0; ctr_b <= 0;
            i1_nb <= 0; i2_nb <= 0; i1_ctr <= 0; i2_ctr <= 0;
            blur1_r <= 0; blur1_g <= 0; blur1_b <= 0;
            blur2_r <= 0; blur2_g <= 0; blur2_b <= 0;
            lum1_p <= 0; lum2_p <= 0;
            lap1_acc <= 0; lap2_acc <= 0;
            wL1 <= 0; wL2 <= 0;
            lc1_sum <= 0; lc2_sum <= 0;
            lab_start <= 0;
            sum_L1 <= 0; sum_L2 <= 0;
            sum_a1 <= 0; sum_b1 <= 0; sum_a2 <= 0; sum_b2 <= 0;
            mean_L1 <= 0; mean_L2 <= 0;
            mean_a1 <= 0; mean_b1 <= 0; mean_a2 <= 0; mean_b2 <= 0;
            dL1 <= 0; dL2 <= 0; da1 <= 0; db1 <= 0; da2 <= 0; db2 <= 0;
            dr1 <= 0; dg1 <= 0; dbb1 <= 0; dr2 <= 0; dg2 <= 0; dbb2 <= 0;
            sq_dL1 <= 0; sq_da1 <= 0; sq_db1 <= 0;
            sq_dL2 <= 0; sq_da2 <= 0; sq_db2 <= 0;
            sq_sr1 <= 0; sq_sg1 <= 0; sq_sb1 <= 0;
            sq_sr2 <= 0; sq_sg2 <= 0; sq_sb2 <= 0;
            sal_rad1 <= 0; sal_rad2 <= 0;
            sat_sum1 <= 0; sat_sum2 <= 0;
            sat_mul1 <= 0; sat_mul2 <= 0;
            sat_rad1 <= 0; sat_rad2 <= 0;
            sqrt_start <= 0; sqrt_done_mask <= 4'b0000;
            wS1 <= 0; wS2 <= 0; wSat1 <= 0; wSat2 <= 0;
            wL1_min <= 0; wL1_max <= 0; wL1_rng <= 0;
            wS1_min <= 0; wS1_max <= 0; wS1_rng <= 0;
            wSat1_min <= 0; wSat1_max <= 0; wSat1_rng <= 0;
            wL2_min <= 0; wL2_max <= 0; wL2_rng <= 0;
            wS2_min <= 0; wS2_max <= 0; wS2_rng <= 0;
            wSat2_min <= 0; wSat2_max <= 0; wSat2_rng <= 0;
            wn_num_L1 <= 0; wn_num_S1 <= 0; wn_num_Sat1 <= 0;
            wn_num_L2 <= 0; wn_num_S2 <= 0; wn_num_Sat2 <= 0;
            wn_dv_start <= 0;
            w1_agg <= 0; w2_agg <= 0;
            fus_num <= 0; fus_den <= 0; fus_dv_start <= 0;
            nw1_reg <= 0; nw2_reg <= 0;
            fm1_r <= 0; fm2_r <= 0; fm1_g <= 0; fm2_g <= 0; fm1_b <= 0; fm2_b <= 0;
            f_r_q <= 0; f_g_q <= 0; f_b_q <= 0; hist_idx <= 0;
            cdf_r <= 0; cdf_g <= 0; cdf_b <= 0;
            found_lo_r <= 0; found_hi_r <= 0;
            found_lo_g <= 0; found_hi_g <= 0;
            found_lo_b <= 0; found_hi_b <= 0;
            f_lo_r <= 0; f_hi_r <= 0; f_rng_r <= 8'd1; f_flat_r <= 0;
            f_lo_g <= 0; f_hi_g <= 0; f_rng_g <= 8'd1; f_flat_g <= 0;
            f_lo_b <= 0; f_hi_b <= 0; f_rng_b <= 8'd1; f_flat_b <= 0;
            str_num_r <= 0; str_num_g <= 0; str_num_b <= 0;
            str_dv_start <= 0;
            out_pixel <= 0;
            stage <= 0;
            hdr_cnt <= 0;
            tx_seq <= 0;
            // wb_ready intentionally not reset
        end else begin
            // one-clock pulses
            tx_start     <= 0;
            c_start      <= 0;
            g_start      <= 0;
            lab_start    <= 0;
            sqrt_start   <= 0;
            wn_dv_start  <= 0;
            fus_dv_start <= 0;
            str_dv_start <= 0;

            case (state)
                IDLE: begin
                    busy      <= 0;
                    streaming <= 0;
                    phase_out <= 3'd0;
                    if (start) begin
                        busy <= 1;
                        done <= 0;
                        pixel_index <= 0;
                        sum_r  <= 0; sum_g  <= 0; sum_b  <= 0;
                        sum_rc <= 0; sum_gc <= 0; sum_bc <= 0;
                        if (wb_ready) begin
                            phase_out <= 3'd4;
                            state     <= SLM_INIT;
                        end else begin
                            phase_out <= 3'd1;
                            state     <= P1_REQ;
                        end
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
                    phase_out <= 3'd2;
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
                    target_mean <= tsum[10:0] / 3;
                    pixel_index <= 0;
                    phase_out <= 3'd3;
                    state <= P3_REQ;
                end

                // ---------- PASS 3: final per-pixel WB written in place ---
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
                    g_start      <= 1;
                    gw_done_mask <= 3'b000;
                    state        <= P3_GCOMP;
                end
                P3_GCOMP: begin
                    gw_done_mask <= gw_done_next;
                    if (gw_done_next == 3'b111) begin
                        write_addr <= pixel_index;
                        write_data <= {gr_y, gg_y, gb_y};
                        state      <= P3_WRITE;
                    end
                end
                P3_WRITE: begin
                    if (pixel_index == DEPTH-1) begin
                        wb_ready  <= 1'b1;
                        phase_out <= 3'd4;
                        state     <= SLM_INIT;
                    end else begin
                        pixel_index <= pixel_index + 1'b1;
                        state <= P3_REQ;
                    end
                end

                // ---------- 3x3 inner neighbourhood fetch around (ux, uy) -
                NB_INIT: begin
                    k     <= 0;
                    acc_r <= 0; acc_g <= 0; acc_b <= 0;
                    state <= NB_ADDR;
                end
                NB_ADDR: begin
                    xx_r   <= nb_dx[1] ? ux_m1 : (nb_dx[0] ? ux_p1 : ux);
                    yy_r   <= nb_dy[1] ? uy_m1 : (nb_dy[0] ? uy_p1 : uy);
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
                        ctr_r   <= rd_data[23:16];
                        ctr_g   <= rd_data[15:8];
                        ctr_b   <= rd_data[7:0];
                        raw_pix <= rd_data;
                    end
                    if (k == 4'd8)
                        state <= NB_DONE;
                    else begin
                        k     <= k + 1'b1;
                        state <= NB_ADDR;
                    end
                end
                NB_DONE: begin
                    state <= UN_FIN;
                end

                // ---------- PASS 4: Saliency Lab global mean --------------
                SLM_INIT: begin
                    sum_L1      <= 32'd0; sum_L2 <= 32'd0;
                    sum_a1      <= 32'sd0; sum_b1 <= 32'sd0;
                    sum_a2      <= 32'sd0; sum_b2 <= 32'sd0;
                    pixel_index <= 0;
                    x_cnt       <= 0;
                    y_cnt       <= 0;
                    nb_ctx      <= CTX_FUSION;
                    fusion_mode <= FMODE_SLM;
                    state       <= OUTER_INIT;
                end
                SLM_FIN: begin
                    mean_L1   <= sum_L1[ADDR_WIDTH+9  : ADDR_WIDTH];
                    mean_a1   <= sum_a1[ADDR_WIDTH+12 : ADDR_WIDTH];
                    mean_b1   <= sum_b1[ADDR_WIDTH+12 : ADDR_WIDTH];
                    mean_L2   <= sum_L2[ADDR_WIDTH+9  : ADDR_WIDTH];
                    mean_a2   <= sum_a2[ADDR_WIDTH+12 : ADDR_WIDTH];
                    mean_b2   <= sum_b2[ADDR_WIDTH+12 : ADDR_WIDTH];
                    phase_out <= 3'd6;
                    state     <= WMM_INIT;
                end

                // ---------- PASS 6: Global min/max of all 6 weight maps ---
                WMM_INIT: begin
                    wL1_min   <= 12'hFFF; wL1_max   <= 12'd0;
                    wS1_min   <= 12'hFFF; wS1_max   <= 12'd0;
                    wSat1_min <= 12'hFFF; wSat1_max <= 12'd0;
                    wL2_min   <= 12'hFFF; wL2_max   <= 12'd0;
                    wS2_min   <= 12'hFFF; wS2_max   <= 12'd0;
                    wSat2_min <= 12'hFFF; wSat2_max <= 12'd0;
                    pixel_index <= 0;
                    x_cnt       <= 0;
                    y_cnt       <= 0;
                    nb_ctx      <= CTX_FUSION;
                    fusion_mode <= FMODE_WMM;
                    state       <= OUTER_INIT;
                end
                WMM_FIN: begin
                    wL1_rng   <= wL1_max   - wL1_min;
                    wS1_rng   <= wS1_max   - wS1_min;
                    wSat1_rng <= wSat1_max - wSat1_min;
                    wL2_rng   <= wL2_max   - wL2_min;
                    wS2_rng   <= wS2_max   - wS2_min;
                    wSat2_rng <= wSat2_max - wSat2_min;
                    phase_out <= 3'd7;
                    state     <= FMM_INIT;
                end

                // ---------- PASS 7: Fused histogram & stretch bounds ------
                FMM_INIT: begin
                    hist_idx <= 8'd0;
                    state    <= FMM_CLR;
                end
                FMM_CLR: begin
                    if (hist_idx == 8'd255) begin
                        pixel_index <= 0;
                        x_cnt       <= 0;
                        y_cnt       <= 0;
                        nb_ctx      <= CTX_FUSION;
                        fusion_mode <= FMODE_FMM;
                        state       <= OUTER_INIT;
                    end else begin
                        hist_idx <= hist_idx + 1'b1;
                    end
                end
                FMM_HIST_RD: state <= FMM_HIST_WR;
                FMM_HIST_WR: begin
                    if (pixel_index == DEPTH-1) begin
                        hist_idx   <= 8'd0;
                        cdf_r      <= 17'd0; cdf_g <= 17'd0; cdf_b <= 17'd0;
                        found_lo_r <= 1'b0; found_hi_r <= 1'b0;
                        found_lo_g <= 1'b0; found_hi_g <= 1'b0;
                        found_lo_b <= 1'b0; found_hi_b <= 1'b0;
                        state      <= FMM_CDF_RD;
                    end else begin
                        pixel_index <= pixel_index + 1'b1;
                        if (x_cnt == WIDTH-1) begin
                            x_cnt <= 0;
                            y_cnt <= y_cnt + 1'b1;
                        end else
                            x_cnt <= x_cnt + 1'b1;
                        state <= OUTER_INIT;
                    end
                end
                FMM_CDF_RD: state <= FMM_CDF_USE;
                FMM_CDF_USE: begin
                    cdf_r <= cdf_r_next;
                    cdf_g <= cdf_g_next;
                    cdf_b <= cdf_b_next;
                    if (!found_lo_r && (cdf_r_next >= PCT_LO)) begin f_lo_r <= hist_idx; found_lo_r <= 1'b1; end
                    if (!found_hi_r && (cdf_r_next >= PCT_HI)) begin f_hi_r <= hist_idx; found_hi_r <= 1'b1; end
                    if (!found_lo_g && (cdf_g_next >= PCT_LO)) begin f_lo_g <= hist_idx; found_lo_g <= 1'b1; end
                    if (!found_hi_g && (cdf_g_next >= PCT_HI)) begin f_hi_g <= hist_idx; found_hi_g <= 1'b1; end
                    if (!found_lo_b && (cdf_b_next >= PCT_LO)) begin f_lo_b <= hist_idx; found_lo_b <= 1'b1; end
                    if (!found_hi_b && (cdf_b_next >= PCT_HI)) begin f_hi_b <= hist_idx; found_hi_b <= 1'b1; end
                    if (hist_idx == 8'd255)
                        state <= FMM_FIN;
                    else begin
                        hist_idx <= hist_idx + 1'b1;
                        state    <= FMM_CDF_RD;
                    end
                end
                FMM_FIN: begin
                    f_flat_r  <= (f_hi_r <= f_lo_r);
                    f_flat_g  <= (f_hi_g <= f_lo_g);
                    f_flat_b  <= (f_hi_b <= f_lo_b);
                    f_rng_r   <= (f_hi_r > f_lo_r) ? (f_hi_r - f_lo_r) : 8'd1;
                    f_rng_g   <= (f_hi_g > f_lo_g) ? (f_hi_g - f_lo_g) : 8'd1;
                    f_rng_b   <= (f_hi_b > f_lo_b) ? (f_hi_b - f_lo_b) : 8'd1;
                    phase_out <= 3'd0;
                    state     <= STG_BEGIN;
                end

                // ---------- Outer 3x3 loop for I1 & I2 weights / fusion ---
                OUTER_INIT: begin
                    m        <= 0;
                    blur1_r  <= 0; blur1_g <= 0; blur1_b <= 0;
                    blur2_r  <= 0; blur2_g <= 0; blur2_b <= 0;
                    lap1_acc <= 0; lap2_acc <= 0;
                    state    <= OUTER_ADDR;
                end
                OUTER_ADDR: begin
                    ux     <= om_dx[1] ? x_m1 : (om_dx[0] ? x_p1 : x_cnt);
                    uy     <= om_dy[1] ? y_m1 : (om_dy[0] ? y_p1 : y_cnt);
                    om_w_r <= om_w;
                    state  <= NB_INIT;
                end
                OUTER_ACC1: begin
                    if (m == 4'd4) begin
                        i1_ctr <= i1_nb;
                        i2_ctr <= i2_nb;
                    end
                    blur1_r <= blur1_r + (i1_nb[23:16] * om_w_r);
                    blur1_g <= blur1_g + (i1_nb[15:8]  * om_w_r);
                    blur1_b <= blur1_b + (i1_nb[7:0]   * om_w_r);
                    blur2_r <= blur2_r + (i2_nb[23:16] * om_w_r);
                    blur2_g <= blur2_g + (i2_nb[15:8]  * om_w_r);
                    blur2_b <= blur2_b + (i2_nb[7:0]   * om_w_r);
                    lum1_p  <= (16'd54 * i1_nb[23:16]) + (16'd183 * i1_nb[15:8]) + (16'd19 * i1_nb[7:0]);
                    lum2_p  <= (16'd54 * i2_nb[23:16]) + (16'd183 * i2_nb[15:8]) + (16'd19 * i2_nb[7:0]);
                    state   <= OUTER_ACC2;
                end
                OUTER_ACC2: begin
                    if (m == 4'd0 || m == 4'd2 || m == 4'd6 || m == 4'd8) begin
                        lap1_acc <= lap1_acc + $signed({3'b000, lum1_p[15:4], 1'b0});
                        lap2_acc <= lap2_acc + $signed({3'b000, lum2_p[15:4], 1'b0});
                    end else if (m == 4'd4) begin
                        lap1_acc <= lap1_acc - $signed({1'b0, lum1_p[15:4], 3'b000});
                        lap2_acc <= lap2_acc - $signed({1'b0, lum2_p[15:4], 3'b000});
                    end
                    if (m == 4'd8)
                        state <= LAB_GO;
                    else begin
                        m     <= m + 1'b1;
                        state <= OUTER_ADDR;
                    end
                end

                LAB_GO: begin
                    lab_start <= 1'b1;
                    wL1       <= wL1_abs[14:3];
                    wL2       <= wL2_abs[14:3];
                    lc1_sum   <= (16'd54 * i1_ctr[23:16]) + (16'd183 * i1_ctr[15:8]) + (16'd19 * i1_ctr[7:0]) + 16'd128;
                    lc2_sum   <= (16'd54 * i2_ctr[23:16]) + (16'd183 * i2_ctr[15:8]) + (16'd19 * i2_ctr[7:0]) + 16'd128;
                    state     <= LAB_WAIT;
                end
                LAB_WAIT: if (lab1_done && lab2_done) begin
                    if (fusion_mode == FMODE_SLM) begin
                        sum_L1 <= sum_L1 + {22'd0, lab1_L};
                        sum_a1 <= sum_a1 + {{19{lab1_a[12]}}, lab1_a};
                        sum_b1 <= sum_b1 + {{19{lab1_b[12]}}, lab1_b};
                        sum_L2 <= sum_L2 + {22'd0, lab2_L};
                        sum_a2 <= sum_a2 + {{19{lab2_a[12]}}, lab2_a};
                        sum_b2 <= sum_b2 + {{19{lab2_b[12]}}, lab2_b};
                        if (pixel_index == DEPTH-1)
                            state <= SLM_FIN;
                        else begin
                            pixel_index <= pixel_index + 1'b1;
                            if (x_cnt == WIDTH-1) begin
                                x_cnt <= 0;
                                y_cnt <= y_cnt + 1'b1;
                            end else
                                x_cnt <= x_cnt + 1'b1;
                            state <= OUTER_INIT;
                        end
                    end else begin
                        state <= WT_DIFF;
                    end
                end

                WT_DIFF: begin
                    dL1  <= $signed({1'b0, lab1_L}) - $signed({1'b0, mean_L1});
                    da1  <= $signed({lab1_a[12], lab1_a}) - $signed({mean_a1[12], mean_a1});
                    db1  <= $signed({lab1_b[12], lab1_b}) - $signed({mean_b1[12], mean_b1});
                    dL2  <= $signed({1'b0, lab2_L}) - $signed({1'b0, mean_L2});
                    da2  <= $signed({lab2_a[12], lab2_a}) - $signed({mean_a2[12], mean_a2});
                    db2  <= $signed({lab2_b[12], lab2_b}) - $signed({mean_b2[12], mean_b2});
                    dr1  <= $signed({1'b0, i1_ctr[23:16]}) - $signed({1'b0, lc1_sum[15:8]});
                    dg1  <= $signed({1'b0, i1_ctr[15:8]})  - $signed({1'b0, lc1_sum[15:8]});
                    dbb1 <= $signed({1'b0, i1_ctr[7:0]})   - $signed({1'b0, lc1_sum[15:8]});
                    dr2  <= $signed({1'b0, i2_ctr[23:16]}) - $signed({1'b0, lc2_sum[15:8]});
                    dg2  <= $signed({1'b0, i2_ctr[15:8]})  - $signed({1'b0, lc2_sum[15:8]});
                    dbb2 <= $signed({1'b0, i2_ctr[7:0]})   - $signed({1'b0, lc2_sum[15:8]});
                    state <= WT_SQ1;
                end
                WT_SQ1: begin
                    sq_dL1 <= dL1 * dL1; sq_da1 <= da1 * da1; sq_db1 <= db1 * db1;
                    sq_dL2 <= dL2 * dL2; sq_da2 <= da2 * da2; sq_db2 <= db2 * db2;
                    sq_sr1 <= dr1 * dr1; sq_sg1 <= dg1 * dg1; sq_sb1 <= dbb1 * dbb1;
                    sq_sr2 <= dr2 * dr2; sq_sg2 <= dg2 * dg2; sq_sb2 <= dbb2 * dbb2;
                    state  <= WT_SQ2;
                end
                WT_SQ2: begin
                    sal_rad1 <= sq_dL1[23:0] + sq_da1[23:0] + sq_db1[23:0];
                    sal_rad2 <= sq_dL2[23:0] + sq_da2[23:0] + sq_db2[23:0];
                    sat_sum1 <= sq_sr1[16:0] + sq_sg1[16:0] + sq_sb1[16:0];
                    sat_sum2 <= sq_sr2[16:0] + sq_sg2[16:0] + sq_sb2[16:0];
                    state    <= WT_SQ3;
                end
                WT_SQ3: begin
                    sat_mul1 <= sat_sum1 * 15'd21845;
                    sat_mul2 <= sat_sum2 * 15'd21845;
                    state    <= WT_SQRT_GO;
                end
                WT_SQRT_GO: begin
                    sat_rad1       <= sat_mul1[31:12];
                    sat_rad2       <= sat_mul2[31:12];
                    sqrt_done_mask <= 4'b0000;
                    sqrt_start     <= 1'b1;
                    state          <= WT_SQRT_WAIT;
                end
                WT_SQRT_WAIT: begin
                    sqrt_done_mask <= sqrt_done_next;
                    if (sqrt_done_next == 4'b1111) begin
                        wS1   <= sal1_root;
                        wS2   <= sal2_root;
                        wSat1 <= sat1_root;
                        wSat2 <= sat2_root;
                        if (fusion_mode == FMODE_WMM) begin
                            if (wL1 < wL1_min)   wL1_min <= wL1;
                            if (wL1 > wL1_max)   wL1_max <= wL1;
                            if (sal1_root < wS1_min) wS1_min <= sal1_root;
                            if (sal1_root > wS1_max) wS1_max <= sal1_root;
                            if (sat1_root < wSat1_min) wSat1_min <= sat1_root;
                            if (sat1_root > wSat1_max) wSat1_max <= sat1_root;

                            if (wL2 < wL2_min)   wL2_min <= wL2;
                            if (wL2 > wL2_max)   wL2_max <= wL2;
                            if (sal2_root < wS2_min) wS2_min <= sal2_root;
                            if (sal2_root > wS2_max) wS2_max <= sal2_root;
                            if (sat2_root < wSat2_min) wSat2_min <= sat2_root;
                            if (sat2_root > wSat2_max) wSat2_max <= sat2_root;

                            if (pixel_index == DEPTH-1)
                                state <= WMM_FIN;
                            else begin
                                pixel_index <= pixel_index + 1'b1;
                                if (x_cnt == WIDTH-1) begin
                                    x_cnt <= 0;
                                    y_cnt <= y_cnt + 1'b1;
                                end else
                                    x_cnt <= x_cnt + 1'b1;
                                state <= OUTER_INIT;
                            end
                        end else begin
                            state <= WN_PREP;
                        end
                    end
                end

                WN_PREP: begin
                    wn_num_L1   <= (wL1   - wL1_min)   * 8'd255;
                    wn_num_S1   <= (wS1   - wS1_min)   * 8'd255;
                    wn_num_Sat1 <= (wSat1 - wSat1_min) * 8'd255;
                    wn_num_L2   <= (wL2   - wL2_min)   * 8'd255;
                    wn_num_S2   <= (wS2   - wS2_min)   * 8'd255;
                    wn_num_Sat2 <= (wSat2 - wSat2_min) * 8'd255;
                    state <= WN_GO;
                end
                WN_GO: begin
                    wn_dv_start <= 1'b1;
                    state       <= WN_WAIT;
                end
                WN_WAIT: if (wn_d_L1 && wn_d_S1 && wn_d_Sat1 && wn_d_L2 && wn_d_S2 && wn_d_Sat2) begin
                    w1_agg <= {2'b00, nwL1} + {2'b00, nwS1} + {2'b00, nwSat1};
                    w2_agg <= {2'b00, nwL2} + {2'b00, nwS2} + {2'b00, nwSat2};
                    state  <= FUS_PREP;
                end

                FUS_PREP: begin
                    fus_num <= ({1'b0, w1_agg} + 11'd26) << 8;
                    fus_den <= {1'b0, w1_agg} + {1'b0, w2_agg} + 11'd52;
                    state   <= FUS_GO;
                end
                FUS_GO: begin
                    fus_dv_start <= 1'b1;
                    state        <= FUS_WAIT;
                end
                FUS_WAIT: if (fus_dv_done) begin
                    nw1_reg <= fus_dv_q[8:0];
                    nw2_reg <= 9'd256 - fus_dv_q[8:0];
                    state   <= FUS_MUL;
                end
                FUS_MUL: begin
                    fm1_r <= nw1_reg * i1_ctr[23:16];
                    fm2_r <= nw2_reg * i2_ctr[23:16];
                    fm1_g <= nw1_reg * i1_ctr[15:8];
                    fm2_g <= nw2_reg * i2_ctr[15:8];
                    fm1_b <= nw1_reg * i1_ctr[7:0];
                    fm2_b <= nw2_reg * i2_ctr[7:0];
                    state <= FUS_ADD;
                end
                FUS_ADD: begin
                    if (fusion_mode == FMODE_FMM) begin
                        f_r_q <= fused_r_w;
                        f_g_q <= fused_g_w;
                        f_b_q <= fused_b_w;
                        state <= FMM_HIST_RD;
                    end else if (stage == 3'd3) begin
                        f_r_q <= fused_r_w;
                        f_g_q <= fused_g_w;
                        f_b_q <= fused_b_w;
                        state <= STR_PREP;
                    end else begin
                        out_pixel <= {
                            nw1_reg[7:0],
                            (nw2_reg[8] ? 8'd255 : nw2_reg[7:0]),
                            8'd0
                        };
                        tx_seq <= 2'd0;
                        state  <= TX_START;
                    end
                end
                STR_PREP: begin
                    str_num_r <= {fs_cl_r, 8'b0} - fs_cl_r;
                    str_num_g <= {fs_cl_g, 8'b0} - fs_cl_g;
                    str_num_b <= {fs_cl_b, 8'b0} - fs_cl_b;
                    state     <= STR_GO;
                end
                STR_GO: begin
                    str_dv_start <= 1'b1;
                    state        <= STR_WAIT;
                end
                STR_WAIT: if (str_d_r && str_d_g && str_d_b) begin
                    out_pixel <= {
                        (f_flat_r ? f_r_q : str_q_r[7:0]),
                        (f_flat_g ? f_g_q : str_q_g[7:0]),
                        (f_flat_b ? f_b_q : str_q_b[7:0])
                    };
                    tx_seq <= 2'd0;
                    state  <= TX_START;
                end

                // ---------- UART streaming of all 5 stages -----------------
                STG_BEGIN: begin
                    stage     <= 3'd0;
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
                        ux          <= 0;
                        uy          <= 0;
                        if (stage == 3'd2) begin
                            nb_ctx <= CTX_UNSHARP;
                            state  <= NB_INIT;
                        end else if (stage >= 3'd3) begin
                            nb_ctx      <= CTX_FUSION;
                            fusion_mode <= FMODE_STREAM;
                            state       <= OUTER_INIT;
                        end else begin
                            state <= PX_REQ;
                        end
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
                    if (stage == 3'd0) begin
                        out_pixel <= rd_data;
                        tx_seq    <= 2'd0;
                        state     <= TX_START;
                    end else
                        state <= G_W1;
                end
                G_W1: state <= G_W2;
                G_W2: begin
                    out_pixel <= {gam_r, gam_g, gam_b};
                    tx_seq    <= 2'd0;
                    state     <= TX_START;
                end

                // inner unsharp masking computation (shared by stage 2 and CTX_FUSION)
                UN_FIN: begin
                    if (nb_ctx == CTX_FUSION) begin
                        i1_nb <= {gam_r, gam_g, gam_b};
                        i2_nb <= {sharp_r, sharp_g, sharp_b};
                        state <= OUTER_ACC1;
                    end else begin
                        out_pixel <= {sharp_r, sharp_g, sharp_b};
                        tx_seq    <= 2'd0;
                        state     <= TX_START;
                    end
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
                        if (stage == 3'd4)
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
                            ux    <= 0;
                            y_cnt <= y_cnt + 1'b1;
                            uy    <= y_cnt + 1'b1;
                        end else begin
                            x_cnt <= x_cnt + 1'b1;
                            ux    <= x_cnt + 1'b1;
                            uy    <= y_cnt;
                        end
                        if (stage == 3'd2)
                            state <= NB_INIT;
                        else if (stage >= 3'd3)
                            state <= OUTER_INIT;
                        else
                            state <= PX_REQ;
                    end
                end

                FINISHED: begin
                    busy      <= 0;
                    streaming <= 0;
                    done      <= 1;
                    phase_out <= 3'd0;
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
// y = min(255, pixel * target / mean), 0 if mean == 0.
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
// UART transmitter 8-N-1.
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
