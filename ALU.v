`default_nettype none

/**
 * ALU (Arithmetic Logic Unit) - Full RV32I
 *
 * This version adds a 'funct7_bit' input to correctly
 * distinguish SRA from SRL.
 */
module ALU (
    // Inputs
    input  wire [31:0] A,
    input  wire [31:0] B,
    input  wire [2:0]  ALUControl,
    input  wire        funct7_bit, // <-- NEW INPUT (connect to InstrE[30])
    input  wire        funct3_bit0, // <-- ADD THIS NEW INPUT (for SLT/SLTU)    
    // Outputs
    output reg        Carry,
    output reg        OverFlow,
    output reg        Zero,
    output reg        Negative,
    output reg [31:0] Result
);

    // --- Internal Wires ---
    wire [31:0] B_inv    = ~B;
    wire [32:0] sum_temp;
    wire [4:0]  shamt    = B[4:0]; // Shift amount
    
    wire C_in = (ALUControl == 3'b001) | // SUB
                (ALUControl == 3'b111) | // SLT
                (ALUControl == 3'b100);  // SLTU
                
    assign sum_temp = A + (C_in ? B_inv : B) + C_in;

    wire v_bit = (A[31] == (C_in ? B_inv[31] : B[31])) && (A[31] != sum_temp[31]);
    wire n_bit = sum_temp[31];
    
    // --- MODIFIED LINE ---
    // Use the new input port. This is 1 for SRA/SRAI.
    wire is_SRA = funct7_bit; 

    
    always @(*) begin
        // --- Set default values ---
        Result   = 32'h00000000;
        Carry    = 1'b0;
        OverFlow = 1'b0; 

        case (ALUControl)
            3'b000: begin // ADD
                Result = sum_temp[31:0];
            end
            
            3'b001: begin // SUB
                Result = sum_temp[31:0];
            end
            
            3'b010: begin // AND
                Result = A & B;
            end
            
            3'b011: begin // OR
                Result = A | B;
            end
            
            3'b100: begin // XOR
                Result = A ^ B;
            end
            
            3'b101: begin // SLL (Shift Left Logical)
                Result = A << shamt;
            end
            
            3'b110: begin // SRL / SRA (Shift Right)
                if (is_SRA) // <-- This now works for R-type and I-type
                    Result = $signed(A) >>> shamt; // SRA/SRAI
                else
                    Result = A >> shamt;           // SRL/SRLI (logical)
            end
            
            // --- HERE IS THE OTHER FIX ---
            3'b111: begin // SLT / SLTU (Set Less Than)
                if (funct3_bit0) // This is 1 for SLTU
                    Result = (~sum_temp[32]) ? 32'd1 : 32'd0; // Unsigned (check borrow)
                else // This is 0 for SLT
                    Result = (n_bit ^ v_bit) ? 32'd1 : 32'd0; // Signed (check N^V)
            end
            
            default: begin
                Result = 32'h00000000;
            end
        endcase
        
        // --- Flag Logic ---
        if (ALUControl == 3'b000 || ALUControl == 3'b001 || ALUControl == 3'b111 || ALUControl == 3'b100) begin
            Carry    = sum_temp[32];
            OverFlow = v_bit;
        end
        
        Zero     = (Result == 32'h00000000);
        Negative = Result[31];
    end
endmodule

`default_nettype wire
