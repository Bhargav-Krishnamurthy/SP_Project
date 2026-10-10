
`timescale 1ns / 1ps

module tb_bram_image_reader;

    reg clk = 0;
    reg rst = 1;
    reg start = 0;

    wire [23:0] pixel_out;
    wire pixel_valid;
    wire done;

    integer outfile;
    integer count;

    bram_image_reader dut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .pixel_out(pixel_out),
        .pixel_valid(pixel_valid),
        .done(done)
    );

    always #5 clk = ~clk;

    initial begin
        count = 0;

        // Open the output file
        outfile = $fopen("readback.mem", "w");

        if (outfile == 0) begin
            $display("ERROR: fopen failed for readback.mem");
            $finish;
        end

        $display("SUCCESS: readback.mem opened.");

        // Apply reset
        repeat (4) @(negedge clk);
        rst = 0;

        // Start the reader
        @(negedge clk);
        start = 1;

        @(negedge clk);
        start = 0;

        // Collect each valid output pixel
        while (count < 65536) begin
            @(negedge clk);

            if (pixel_valid) begin
                $fdisplay(outfile, "%06h", pixel_out);
                count = count + 1;

                // Print the first few pixels for debugging
                if (count <= 5)
                    $display(
                        "Pixel %0d = %06h",
                        count - 1, pixel_out
                    );

                // Periodically report progress
                if ((count % 16384) == 0)
                    $display("Pixels written: %0d", count);
            end
        end

        // Flush and close the file
        $fflush(outfile);
        $fclose(outfile);

        $display("SUCCESS: Wrote %0d pixels.", count);
        $display("Readback complete.");

        $finish;
    end

endmodule
