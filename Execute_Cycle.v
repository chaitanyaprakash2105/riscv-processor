`default_nettype none

module execute_cycle(
    // --- Inputs ---
    input  wire        clk,
    input  wire        rst,
    input  wire        RegWriteE,
    input  wire        ALUSrcE,
    input  wire        MemWriteE,
    input  wire [1:0]  ResultSrcE,
    input  wire        BranchE,
    input  wire        JumpE,
    input  wire [2:0]  ALUControlE,
    input  wire [31:0] RD1_E,
    input  wire [31:0] RD2_E,
    input  wire [31:0] Imm_Ext_E,
    input  wire [4:0]  RD_E,
    input  wire [31:0] PCE,
    input  wire [31:0] PCPlus4E,
    input  wire [31:0] ResultW,
    input  wire [1:0]  ForwardA_E,
    input  wire [1:0]  ForwardB_E,
    input  wire [31:0] InstrE,
    input  wire        ValidE,
    input  wire [4:0]  RS1_E,
    input  wire [4:0]  RS2_E,
    input wire    Stall,

    // --- Outputs ---
    output wire        PCSrcE,
    output wire [31:0] PCTargetE,
    output wire        RegWriteM,
    output wire        MemWriteM,
    output wire [1:0]  ResultSrcM,
    output wire        JumpM,
    output wire [4:0]  RD_M, 
    output wire [31:0] PCPlus4M,
    output wire [31:0] WriteDataM,
    output wire [31:0] ALU_ResultM,
    output wire        MemReadM,
    output wire [3:0]  dmem_mask_M,
    output wire        ValidM,
    // Retire Outputs
    output wire [31:0] InstrM,
    output wire [31:0] PC_M,
    output wire [4:0]  RS1_M,
    output wire [4:0]  RS2_M,
    output wire [31:0] RD1_M,
    output wire [31:0] RD2_M,
    // NEW Retire Outputs
    output wire        PCSrcM,
    output wire [31:0] PCTargetM
);

    // Wires
    wire [31:0] Src_A, Src_B_interim, Src_B;
    wire [31:0] ResultE; // This is the ALU result
    wire        ZeroE;

    // EX/MEM Registers
    reg        RegWriteE_r, MemWriteE_r, JumpE_r;
    reg [1:0]  ResultSrcE_r;
    reg [4:0]  RD_E_r;
    reg [31:0] PCPlus4E_r, RD2_E_r, ResultE_r;
    reg        MemReadE_r;
    reg [3:0]  dmem_mask_E_r; // This register holds the mask for the MEM stage
    reg        ValidM_r;

    // Retire Registers
    reg [31:0] InstrE_r;
    reg [31:0] PCE_r;
    reg [4:0]  RS1_E_r, RS2_E_r;
    reg [31:0] RD1_E_r, RD2_E_r_retire;
    reg [3:0] dmem_mask_logic; // The new combinational mask
    // NEW Retire Registers
    reg        PCSrcE_r;
    reg [31:0] PCTargetE_r;

    wire        NegativeE;  
    wire        CarryE;     

    // Modules
    wire [31:0] Src_A_forwarded; 
    Mux_3_by_1 srca_mux ( .a(RD1_E), .b(ResultW), .c(ALU_ResultM), .s(ForwardA_E), .d(Src_A_forwarded) );     
    Mux_3_by_1 srcb_mux ( .a(RD2_E), .b(ResultW), .c(ALU_ResultM), .s(ForwardB_E), .d(Src_B_interim) );
    Mux alu_src_mux ( .a(Src_B_interim), .b(Imm_Ext_E), .s(ALUSrcE), .c(Src_B) );
    wire OverFlowE;
    ALU alu ( .A(Src_A), .B(Src_B), .Result(ResultE), .ALUControl(ALUControlE), .OverFlow(OverFlowE), .Carry(CarryE), .Zero(ZeroE), .Negative(NegativeE), .funct7_bit(InstrE[30]), .funct3_bit0(InstrE[12]) );

    wire [31:0] branch_jal_target;
    PC_Adder branch_adder ( .a(PCE), .b(Imm_Ext_E), .c(branch_jal_target) );

    wire is_jalr = JumpE & (InstrE[6:0] == 7'b1100111);
    // RISC-V requires JALR target to be aligned (LSB forced to 0)
    assign PCTargetE = is_jalr ? (ResultE & ~32'h1) : branch_jal_target;

    wire is_LUI_final = (InstrE[6:0] == 7'b0110111);
    wire is_AUIPC = (InstrE[6:0] == 7'b0010111);
    assign Src_A = is_LUI_final ? 32'h00000000 : 
                   is_AUIPC     ? PCE :
                   Src_A_forwarded;

    // Register Logic
    always @(posedge clk or posedge rst) begin
        if(rst == 1'b1) begin
            RegWriteE_r    <= 1'b0; 
            MemWriteE_r    <= 1'b0; 
            ResultSrcE_r   <= 2'b00;
            JumpE_r        <= 1'b0;
            RD_E_r         <= 5'h00;
            PCPlus4E_r     <= 32'h0; 
            RD2_E_r        <= 32'h0; 
            ResultE_r      <= 32'h0;
            MemReadE_r     <= 1'b0;
            dmem_mask_E_r  <= 4'b0;
            InstrE_r       <= 32'h0;
            PCE_r          <= 32'h0;
            RS1_E_r        <= 5'h0;
            RS2_E_r        <= 5'h0;
            RD1_E_r        <= 32'h0;
            RD2_E_r_retire <= 32'h0;
            PCSrcE_r       <= 1'b0;
            PCTargetE_r    <= 32'h0;
            ValidM_r       <= 1'b0;
        end
        else if (Stall) begin
            RegWriteE_r    <= RegWriteE_r;
            MemWriteE_r    <= MemWriteE_r;
            ResultSrcE_r   <= ResultSrcE_r;
            JumpE_r        <= JumpE_r;
            RD_E_r         <= RD_E_r;
            PCPlus4E_r     <= PCPlus4E_r;
            RD2_E_r        <= RD2_E_r;
            ResultE_r      <= ResultE_r;
            MemReadE_r     <= MemReadE_r;
            dmem_mask_E_r  <= dmem_mask_E_r;
            InstrE_r       <= InstrE_r;
            PCE_r          <= PCE_r;
            RS1_E_r        <= RS1_E_r;
            RS2_E_r        <= RS2_E_r;
            RD1_E_r        <= RD1_E_r;
            RD2_E_r_retire <= RD2_E_r_retire;
            PCSrcE_r       <= PCSrcE_r;
            PCTargetE_r    <= PCTargetE_r;
            ValidM_r       <= ValidM_r;
        end
        else begin
            RegWriteE_r    <= RegWriteE; 
            MemWriteE_r    <= MemWriteE; 
            ResultSrcE_r   <= ResultSrcE;
            JumpE_r        <= JumpE;
            RD_E_r         <= RD_E;
            PCPlus4E_r     <= PCPlus4E; 
            RD2_E_r        <= Src_B_interim; // Forwarded rs2 for WriteDataM
            ResultE_r      <= ResultE;
            MemReadE_r     <= (ResultSrcE == 2'b01); 
            InstrE_r       <= InstrE;
            PCE_r          <= PCE;
            RS1_E_r        <= RS1_E;
            RS2_E_r        <= RS2_E;
            RD1_E_r        <= Src_A;
            RD2_E_r_retire <= Src_B; // Original rs2
            PCSrcE_r       <= PCSrcE;
            PCTargetE_r    <= PCTargetE;
            ValidM_r       <= ValidE;
            // *** MODIFICATION 3: Register the new combinational mask ***
            dmem_mask_E_r  <= dmem_mask_logic; 
        end
    end

    // --- Branch Logic ---
    wire [2:0] funct3_E = InstrE[14:12];
    localparam F3_BEQ  = 3'b000;
    localparam F3_BNE  = 3'b001;
    localparam F3_BLT  = 3'b100;
    localparam F3_BGE  = 3'b101;
    localparam F3_BLTU = 3'b110;
    localparam F3_BGEU = 3'b111;
    wire is_beq  = BranchE & (funct3_E == F3_BEQ);
    wire is_bne  = BranchE & (funct3_E == F3_BNE);
    wire is_blt  = BranchE & (funct3_E == F3_BLT);
    wire is_bge  = BranchE & (funct3_E == F3_BGE);
    wire is_bltu = BranchE & (funct3_E == F3_BLTU);
    wire is_bgeu = BranchE & (funct3_E == F3_BGEU);
    wire signed_lt  = (NegativeE ^ OverFlowE);
    wire signed_ge  = ~signed_lt;
    wire branch_taken = 
        (is_beq  &  ZeroE)     |
        (is_bne  & ~ZeroE)     |
        (is_blt  &  signed_lt) |
        (is_bge  &  signed_ge) |
        (is_bltu & ~CarryE)    |
        (is_bgeu &  CarryE);   
    // Only redirect PC when the EX-stage instruction is valid
    // and the pipeline is not stalled (e.g., on an I/D cache miss).
    // This prevents losing taken branches that resolve while the
    // front-end is stalled.
    assign PCSrcE = ValidE & (branch_taken | JumpE) & ~Stall;

    
    // ---  Add Mask Generation Logic ---

    wire [1:0] addr_lsb = ResultE[1:0];   // LSBs of the calculated ALU address
    wire [2:0] funct3   = InstrE[14:12];  // Get funct3 from the instruction
    wire       is_load  = (ResultSrcE == 2'b01);
    wire       is_store = MemWriteE;

    always @(*) begin
        // Default to no mask (for non-memory ops)
        dmem_mask_logic = 4'b0000;

        if (is_load || is_store) begin
            case (funct3[1:0]) // Check type: Byte, Half, or Word
                
                // Byte access (LB, LBU, SB)
                2'b00: begin
                    case (addr_lsb)
                        2'b00:   dmem_mask_logic = 4'b0001; // addr ...0
                        2'b01:   dmem_mask_logic = 4'b0010; // addr ...1
                        2'b10:   dmem_mask_logic = 4'b0100; // addr ...2
                        2'b11:   dmem_mask_logic = 4'b1000; // addr ...3
                        default: dmem_mask_logic = 4'b0000;
                    endcase
                end

                // Halfword access (LH, LHU, SH)
                2'b01: begin
                    // This logic assumes aligned accesses (addr LSB is 0 or 2)
                    case (addr_lsb[1]) // Check 0x...0 vs 0x...2
                        1'b0:    dmem_mask_logic = 4'b0011; // addr ...0
                        1'b1:    dmem_mask_logic = 4'b1100; // addr ...2
                        default: dmem_mask_logic = 4'b0000;
                    endcase
                end

                // Word access (LW, SW)
                2'b10: begin
                    // This logic assumes aligned access (addr LSBs are 00)
                    dmem_mask_logic = 4'b1111;
                end

                default: begin
                    dmem_mask_logic = 4'b0000;
                end
            endcase
        end
    end
    // --- End of Mask Generation Logic ---

    
    assign RegWriteM   = RegWriteE_r;
    assign MemWriteM   = MemWriteE_r;
    assign ResultSrcM  = ResultSrcE_r;
    assign JumpM       = JumpE_r;
    assign RD_M        = RD_E_r;
    assign PCPlus4M    = PCPlus4E_r;
    assign WriteDataM  = RD2_E_r;
    assign ALU_ResultM = ResultE_r;
    assign MemReadM    = MemReadE_r;
    assign dmem_mask_M = dmem_mask_E_r; // This now outputs the correctly generated mask
    assign InstrM      = InstrE_r;
    assign PC_M        = PCE_r;
    assign RS1_M       = RS1_E_r;
    assign RS2_M       = RS2_E_r;
    assign RD1_M       = RD1_E_r;
    assign RD2_M       = RD2_E_r_retire;
    assign PCSrcM      = PCSrcE_r;
    assign PCTargetM   = PCTargetE_r;
    assign ValidM      = ValidM_r;

endmodule

`default_nettype wire
