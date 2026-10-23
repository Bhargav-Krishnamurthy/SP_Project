
`timescale 1ns / 1ps

module tb_gamma_lut;

    reg clk;
    reg [7:0] pixel_in;
    wire [7:0] pixel_out;

    integer i;
    integer errors;

    // Instantiate the gamma LUT
    gamma_lut dut (
        .clk(clk),
        .pixel_in(pixel_in),
        .pixel_out(pixel_out)
    );

    // 100 MHz clock: 10 ns period
    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    // Test all 256 input intensity values
    initial begin
        pixel_in = 8'd0;
        errors = 0;

        $display("======================================");
        $display("       GAMMA LUT TEST STARTED");
        $display("======================================");
        $display("Input Intensity | Output Intensity");
        $display("--------------------------------------");

        // Allow memory initialization and first read
        @(negedge clk);

        for (i = 0; i < 256; i = i + 1) begin
            pixel_in = i;

            // Wait for the synchronous memory read
            @(posedge clk);
            #1;

            $display("Input = %0d, Output = %0d",
                     pixel_in, pixel_out);

            // Detect uninitialized or unknown output
            if ((^pixel_out) === 1'bx) begin
                $display("ERROR: Unknown output for input %0d", i);
                errors = errors + 1;
            end

            // Check endpoints
            if (i == 0 && pixel_out !== 8'd0) begin
                $display("ERROR: Input 0 should produce 0");
                errors = errors + 1;
            end

            if (i == 255 && pixel_out !== 8'd255) begin
                $display("ERROR: Input 255 should produce 255");
                errors = errors + 1;
            end

            // Set up the next input away from the active edge
            @(negedge clk);
        end

        $display("--------------------------------------");

        if (errors == 0)
            $display("TEST PASSED: No errors detected.");
        else
            $display("TEST FAILED: %0d errors detected.", errors);

        $display("======================================");

        $finish;
    end

endmodule
