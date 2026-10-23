
`timescale 1ns / 1ps

module tb_bram_white_balance;

    reg clk;
    reg rst;
    reg start;

    wire busy;
    wire done;

    integer outfile;
    integer count;
    integer i;

    localparam WIDTH  = 256;
    localparam HEIGHT = 256;
    localparam DEPTH  = WIDTH * HEIGHT;

    bram_white_balance #(
        .WIDTH(WIDTH),
        .HEIGHT(HEIGHT),
        .ADDR_WIDTH(16)
    ) dut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .busy(busy),
        .done(done)
    );

    always #5 clk = ~clk;

    initial begin
        // Initialize ALL testbench variables.
        clk     = 0;
        rst     = 1;
        start   = 0;
        count   = 0;
        i       = 0;
        outfile = 0;

        // Open output file immediately.
        outfile = $fopen("white_balance_output.mem", "w");

        if (outfile == 0) begin
            $display("ERROR: Cannot open output file.");
            $finish;
        end

        $display("Output file opened successfully.");

        // Apply reset.
        repeat (4) @(negedge clk);
        rst = 0;

        // Give reset one full clock before starting.
        @(negedge clk);
        start = 1;

        @(negedge clk);
        start = 0;

        $display("Processing started at %0t ns", $time);

        // Wait until the DUT finishes all processing passes.
        wait (done === 1'b1);

        $display("Processing completed at %0t ns", $time);

        // Export output BRAM.
        // Wait until the next falling edge to avoid races.
        @(negedge clk);

        for (i = 0; i < DEPTH; i = i + 1) begin
            $fdisplay(outfile, "%06h", dut.output_mem[i]);
            count = count + 1;

            if (count <= 5)
                $display(
                    "Pixel %0d = %06h",
                    count - 1,
                    dut.output_mem[i]
                );

            if ((count % 16384) == 0)
                $display("Pixels exported: %0d", count);
        end

        $fflush(outfile);
        $fclose(outfile);
        outfile = 0;

        $display("------------------------------------");
        $display("WHITE BALANCING COMPLETED");
        $display("Pixels exported: %0d", count);
        $display("Output: white_balance_output.mem");
        $display("------------------------------------");

        $finish;
    end

    // Monitor controller progress.
    always @(posedge clk) begin
        if (busy && (dut.pixel_index == 0))
            $display(
                "Time=%0t ns, state=%0d, busy=%b",
                $time, dut.state, busy
            );
    end

    // A 20 ms simulation timeout.
    initial begin
        #20000000;
        $display("ERROR: Timeout before completion.");
        $display(
            "state=%0d, busy=%b, done=%b, pixel_index=%0d",
            dut.state, busy, done, dut.pixel_index
        );
        $finish;
    end

endmodule
