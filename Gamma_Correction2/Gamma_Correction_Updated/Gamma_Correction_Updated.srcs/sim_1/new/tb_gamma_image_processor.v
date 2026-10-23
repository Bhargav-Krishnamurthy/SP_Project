
`timescale 1ns / 1ps

module tb_gamma_image_processor;

    localparam integer TOTAL_BYTES = 256 * 256 * 3;

    reg clk = 1'b0;
    reg rst = 1'b1;
    reg start = 1'b0;

    wire [7:0] out_data;
    wire out_valid;
    wire done;

    integer output_file;
    integer output_count;
    integer cycle_count;

    gamma_image_processor dut (
        .clk       (clk),
        .rst       (rst),
        .start     (start),
        .out_data  (out_data),
        .out_valid (out_valid),
        .done      (done)
    );

    // 100 MHz clock
    always #5 clk = ~clk;

    initial begin
        output_count = 0;
        cycle_count  = 0;

        output_file = $fopen("gamma_corrected_rgb.mem", "w");

        if (output_file == 0) begin
            $display("ERROR: Cannot open output file.");
            $finish;
        end

        // Reset
        repeat (5) @(negedge clk);
        rst = 1'b0;

        // Start processing
        @(negedge clk);
        start = 1'b1;

        @(negedge clk);
        start = 1'b0;

        $display("Processing 256x256 RGB image...");

        // Capture exactly TOTAL_BYTES output values.
        while (output_count < TOTAL_BYTES) begin
            @(negedge clk);
            cycle_count = cycle_count + 1;

            if (out_valid) begin
                $fwrite(output_file, "%02X\n", out_data);
                output_count = output_count + 1;

                if ((output_count % 16384) == 0)
                    $display("Processed %0d / %0d bytes",
                             output_count, TOTAL_BYTES);
            end

            if (cycle_count > 500000) begin
                $display("ERROR: Timeout waiting for output.");
                $display("Bytes received: %0d", output_count);
                $fclose(output_file);
                $finish;
            end
        end

        // Close only after all bytes have been captured.
        $fflush(output_file);
        $fclose(output_file);

        if (done !== 1'b1) begin
            $display("ERROR: All bytes captured but done is not asserted.");
            $finish;
        end

        $display("======================================");
        $display("SIMULATION PASSED");
        $display("Output bytes: %0d", output_count);
        $display("Output file: gamma_corrected_rgb.mem");
        $display("======================================");

        $finish;
    end

endmodule
