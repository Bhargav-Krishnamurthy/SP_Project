
module gamma_lut (

    input  wire       clk,
    input  wire [7:0] pixel_in,
    output reg  [7:0] pixel_out
);

    reg [7:0] lut [0:255];

    initial begin
        $readmemh("gamma_lut.mem", lut);
    end

    // Synchronous lookup: output updates on the clock edge.
    always @(posedge clk) begin
        pixel_out <= lut[pixel_in];
    end

endmodule
