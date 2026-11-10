`timescale 1ns/1ps
// Testbench for underwater_fusion_engine.
// Runs with fast UART (25 Mbaud), captures all 5 streams:
//   0: uart_wb.hex
//   1: uart_gamma.hex
//   2: uart_unsharp.hex
//   3: uart_fused.hex
//   4: uart_weights.hex
// Quick run: override W=32, H=32, AW=10 with a 1024-line image.mem.
module tb_underwater_fusion;
  parameter W = 256, H = 256, AW = 16;
  localparam DEPTH  = W * H;
  localparam FRAME  = DEPTH * 3 + 4;
  localparam BIT_NS = 40;

  reg clk = 0, rst = 1, start = 0;
  wire busy, done, streaming, txd;
  wire [2:0] stage, phase;

  underwater_fusion_engine #(
    .WIDTH(W), .HEIGHT(H), .ADDR_WIDTH(AW),
    .CLK_HZ(100000000), .UART_BAUD(25000000)
  ) dut (
    .clk(clk), .rst(rst), .start(start),
    .busy(busy), .done(done), .streaming(streaming),
    .stage_out(stage), .phase_out(phase), .uart_txd(txd)
  );

  always #5 clk = ~clk;

  reg [7:0] bytes [0:5*FRAME-1];
  integer nb = 0;

  always @(negedge txd) begin : rx
    integer idx;
    reg [7:0] b;
    #(BIT_NS / 2);
    for (idx = 0; idx < 8; idx = idx + 1) begin
      #(BIT_NS);
      b[idx] = txd;
    end
    #(BIT_NS);
    if (nb < 5 * FRAME) bytes[nb] = b;
    nb = nb + 1;
  end

  integer f, i, s, errs;
  initial begin
    errs = 0;
    repeat (4) @(negedge clk);
    rst = 0;
    @(negedge clk); start = 1;
    @(negedge clk); start = 0;
    wait (done === 1'b1);
    #200;

    for (s = 0; s < 5; s = s + 1) begin
      if (bytes[s*FRAME] !== 8'hA5 || bytes[s*FRAME+1] !== 8'h5A ||
          bytes[s*FRAME+2] !== s   || bytes[s*FRAME+3] !== 8'h00) begin
        $display("BAD HEADER stage %0d: %02x %02x %02x %02x",
                 s, bytes[s*FRAME], bytes[s*FRAME+1], bytes[s*FRAME+2], bytes[s*FRAME+3]);
        errs = errs + 1;
      end
      case (s)
        0: f = $fopen("uart_wb.hex",      "w");
        1: f = $fopen("uart_gamma.hex",   "w");
        2: f = $fopen("uart_unsharp.hex", "w");
        3: f = $fopen("uart_fused.hex",   "w");
        4: f = $fopen("uart_weights.hex", "w");
      endcase
      for (i = 0; i < DEPTH; i = i + 1)
        $fdisplay(f, "%02x%02x%02x",
                  bytes[s*FRAME+4+3*i],
                  bytes[s*FRAME+4+3*i+1],
                  bytes[s*FRAME+4+3*i+2]);
      $fclose(f);
    end
    $display("DONE at %0t ns, uart bytes = %0d (expected %0d), header errors = %0d",
             $time, nb, 5 * FRAME, errs);
    $finish;
  end

  initial begin
    #2000000000;
    $display("TIMEOUT state=%0d phase=%0d stage=%0d pixel_index=%0d",
             dut.state, dut.phase_out, dut.stage, dut.pixel_index);
    $finish;
  end
endmodule
