`default_nettype none

/**
 * Data Memory (Pipelined, Synchronous Read)
 *
 * This version correctly implements a SYNCHRONOUS read,
 * which is required by your hart.v pipeline specification.
 * Data is available on the clock edge *after* the read is requested.
 */
module Data_Memory (
    // Inputs
    input  wire        clk,
    input  wire        rst,
    input  wire        WE,   // Write Enable (from hart's o_dmem_wen)
    input  wire        RE,   // Read Enable  (from hart's o_dmem_ren)
    input  wire [31:0] A,    // Address      (from hart's o_dmem_addr)
    input  wire [31:0] WD,   // Write Data   (from hart's o_dmem_wdata)

    // Output
    output reg  [31:0] RD    // Read Data    (to hart's i_dmem_rdata)
);

    // 1024 entries, 32-bits each
    reg [31:0] mem [1023:0];

    // --- Synchronous Write ---
    // Writes happen on the clock edge
    always @ (posedge clk)
    begin
        if (WE) begin
            mem[A] <= WD;
        end
    end

    // --- Synchronous Read ---
    // The read happens in two steps:
    // 1. (Combinational) Get the data from the array: `mem[A]`
    // 2. (Synchronous) On the next clock edge, put that data into the
    //    output register `RD` if Read Enable (RE) is high.
    always @ (posedge clk)
    begin
        if (rst) begin
            RD <= 32'h00000000;
        end
        else if (RE) begin
            RD <= mem[A]; // Data will appear on RD next cycle
        end
    end

    // Initial block for simulation
    initial begin
        mem[0] = 32'h00000000;
    end

endmodule

`default_nettype wire