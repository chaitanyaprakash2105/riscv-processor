/**
 * Register File Module
 *
 * Implements 32 general-purpose 32-bit registers for a RISC-V CPU.
 *
 * Features:
 * - Synchronous write on the positive clock edge.
 * - Asynchronous (combinational) read.
 * - Active-high reset initializes all registers to 0.
 * - Write to register x0 (address 0) is ignored.
 * - Read from register x0 (address 0) always returns 0.
 * - Includes decode-stage forwarding (bypass) logic to resolve
 * read-after-write hazards from the Writeback stage.
 */

`default_nettype none

module Register_File(
    // --- FIX: Added 'wire' to all port declarations ---
    input  wire        clk, // Clock
    input  wire        rst, // Active-high reset
    
    // Write Port (from Writeback Stage)
    input  wire        WE3, // Write Enable
    input  wire [4:0]  A3,  // Write Address
    input  wire [31:0] WD3, // Write Data
    
    // Read Port 1 (for Decode Stage Rs1)
    input  wire [4:0]  A1,  // Read Address 1
    output wire [31:0] RD1, // Read Data 1
    
    // Read Port 2 (for Decode Stage Rs2)
    input  wire [4:0]  A2,  // Read Address 2
    output wire [31:0] RD2  // Read Data 2
);

    // 32 registers, 32-bits each
    reg [31:0] Register [31:0];

    // --- Synchronous Write Logic ---
    // Resets all registers on active-high reset
    // Writes to the specified register (A3) if enabled (WE3)
    integer i;
    always @ (posedge clk or posedge rst)
    begin
        if (rst == 1'b1) begin // Active-high reset
            // Loop to reset all registers
            for (i = 0; i < 32; i = i + 1) begin
                Register[i] <= 32'h00000000;
            end
        end
        else begin
            // Normal write logic
            // Check for write enable AND address is not x0
            if (WE3 & (A3 != 5'h00)) begin
                Register[A3] <= WD3;
            end
        end
    end

    // --- Combinational Read Logic with Bypass ---
    //
    // This logic implements:
    // 1. Check for x0: If read address is 0, output 0.
    // 2. Check for Bypass: ELSE IF the WB stage is writing (WE3) to the
    //    register we are trying to read (A3 == A1), forward the WB data (WD3).
    // 3. Read from RegFile: ELSE, read the value from the register array.
    
    assign RD1 = (A1 == 5'h00) ? 32'h00000000 :        // Check x0
                 (WE3 && (A3 == A1)) ? WD3 :          // Check Bypass
                 Register[A1];                       // Read from file

    assign RD2 = (A2 == 5'h00) ? 32'h00000000 :        // Check x0
                 (WE3 && (A3 == A2)) ? WD3 :          // Check Bypass
                 Register[A2];                       // Read from file

endmodule

`default_nettype wire