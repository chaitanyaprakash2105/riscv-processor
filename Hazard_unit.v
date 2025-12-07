/**
 * Hazard Detection Unit (Verilog-2001 Style)
 *
 * This version uses Verilog-2001 ANSI-style ports and always @*
 * to be "modern" without requiring a SystemVerilog compiler.
 */

`default_nettype none

module hazard_unit (
    // Inputs (ANSI-style port list)
    // --- FIX: Added 'wire' keyword to all inputs ---
    input  wire        rst,       // Active-high reset
    input  wire        RegWriteM, // RegWrite signal in Memory stage
    input  wire        RegWriteW, // RegWrite signal in Writeback stage
    input  wire [4:0]  RD_M,      // Destination Register address in M stage
    input  wire [4:0]  RD_W,      // Destination Register address in W stage
    input  wire [4:0]  Rs1_E,     // Source Register 1 address in E stage
    input  wire [4:0]  Rs2_E,     // Source Register 2 address in E stage

    // Outputs (ANSI-style port list)
    // 'reg' is an explicit type, so these are already correct
    output reg [1:0]  ForwardAE, // Forwarding control for ALU input A
    output reg [1:0]  ForwardBE  // Forwarding control for ALU input B
);

    // --- Parameters ---
    // Use 'parameter' instead of 'localparam'
    parameter [1:0] FWD_NONE = 2'b00; // No forwarding
    parameter [1:0] FWD_WB   = 2'b01; // Forward from Writeback stage
    parameter [1:0] FWD_MEM  = 2'b10; // Forward from Memory stage
    parameter [4:0] ZERO_REG = 5'b0;  // The zero register (x0)

    // Use always @* for combinational logic
    always @* begin
        // --- Defaults (No Hazard) ---
        ForwardAE = FWD_NONE;
        ForwardBE = FWD_NONE;

        if (rst) begin
            // On active-high reset, clear forwarding
            ForwardAE = FWD_NONE;
            ForwardBE = FWD_NONE;
        end else begin
            
            // --- Logic for ForwardAE (Rs1) ---
            // Priority: Check MEM stage first (closest data)
            if (RegWriteM && (RD_M != ZERO_REG) && (RD_M == Rs1_E)) begin
                ForwardAE = FWD_MEM;
            end
            // Check WB stage only if no match from MEM
            else if (RegWriteW && (RD_W != ZERO_REG) && (RD_W == Rs1_E)) begin
                ForwardAE = FWD_WB;
            end

            // --- Logic for ForwardBE (Rs2) ---
            // Priority: Check MEM stage first
            if (RegWriteM && (RD_M != ZERO_REG) && (RD_M == Rs2_E)) begin
                ForwardBE = FWD_MEM;
            end
            // Check WB stage only if no match from MEM
            else if (RegWriteW && (RD_W != ZERO_REG) && (RD_W == Rs2_E)) begin
                ForwardBE = FWD_WB;
            end
        end
    end

endmodule

`default_nettype wire