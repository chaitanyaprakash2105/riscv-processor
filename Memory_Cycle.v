`default_nettype none

module memory_cycle(
    // --- Global ---
    input  wire        clk,
    input  wire        rst,
    // --- Inputs from Execute (EX/MEM) ---
    input  wire        RegWriteM,
    input  wire        MemWriteM,
    input  wire        MemReadM,
    input  wire [3:0]  dmem_mask_M,
    input  wire [1:0]  ResultSrcM,
    input  wire        JumpM,
    input  wire [4:0]  RD_M, 
    input  wire [31:0] PCPlus4M,
    input  wire [31:0] WriteDataM,
    input  wire [31:0] ALU_ResultM,
    input  wire [31:0] InstrM,
    input  wire [31:0] PC_M,
    input  wire [4:0]  RS1_M,
    input  wire [4:0]  RS2_M,
    input  wire [31:0] RD1_M,
    input  wire [31:0] RD2_M,
    input  wire        PCSrcM,
    input  wire [31:0] PCTargetM,
    input  wire        ValidM,
    input wire    Stall,
    // --- Input from hart (dmem) ---
    input  wire [31:0] i_dmem_rdata, // <-- This port is now unused, but fine to leave
    // --- Outputs to Writeback (MEM/WB) ---
    output wire        RegWriteW,
    output wire [1:0]  ResultSrcW,
    output wire        JumpW,
    output wire [4:0]  RD_W,
    output wire [31:0] PCPlus4W,
    output wire [31:0] ALU_ResultW,
    output wire [31:0] ReadDataW,    // <-- This is now tied off (driven in hart.v)
    output wire [31:0] InstrW,
    output wire [31:0] PC_W,
    output wire [4:0]  RS1_W,
    output wire [31:0] RawReadDataW, // <-- This is now tied off (driven in hart.v)
    output wire [4:0]  RS2_W,
    output wire [31:0] RD1_W,
    output wire [31:0] RD2_W,
    output wire        MemReadW,
    output wire        MemWriteW,
    output wire [3:0]  dmem_mask_W,
    output wire [31:0] WriteDataW,
    output wire        PCSrcW,
    output wire [31:0] PCTargetW,
    output wire        ValidW,
    // --- Outputs to hart (dmem) ---
    output wire [31:0] o_dmem_addr,
    output wire [31:0] o_dmem_wdata,
    output wire        o_dmem_wen,
    output wire        o_dmem_ren,
    output wire [3:0]  o_dmem_mask
);
    
    // MEM/WB Registers
    reg  RegWriteM_r, JumpM_r;
    reg  [1:0] ResultSrcM_r;
    reg  [4:0] RD_M_r;
    reg  [31:0] PCPlus4M_r, ALU_ResultM_r;
    reg  [31:0] InstrM_r;
    reg  [31:0] PC_M_r;
    reg  [4:0] RS1_M_r, RS2_M_r;
    reg  [31:0] RD1_M_r, RD2_M_r;
    reg  MemReadM_r, MemWriteM_r;
    reg  [3:0] dmem_mask_M_r;
    reg  [31:0] WriteDataM_r;
    reg  PCSrcM_r;
    reg  [31:0] PCTargetM_r;
    reg  ValidW_r;

    // Connect to hart's d-mem interface
    assign o_dmem_addr  = {ALU_ResultM[31:2], 2'b00};

    // Get the LSBs of the calculated address
    wire [1:0] addr_lsb;
    assign addr_lsb = ALU_ResultM[1:0];
    
    // Shift the store data to the correct byte lanes
    wire [31:0] store_data_shifted;
    assign store_data_shifted = WriteDataM << (addr_lsb * 8);

    // Output the *shifted* data to memory
    assign o_dmem_wdata = store_data_shifted;


    assign o_dmem_wen   = MemWriteM;
    assign o_dmem_ren   = MemReadM;
    assign o_dmem_mask  = dmem_mask_M;

    // MEM/WB Pipeline Register Logic
    always @(posedge clk or posedge rst) begin
        if (rst == 1'b1) begin
            RegWriteM_r   <= 1'b0; 
            ResultSrcM_r  <= 2'b00;
            JumpM_r       <= 1'b0;
            RD_M_r        <= 5'h00;
            PCPlus4M_r    <= 32'h0; 
            ALU_ResultM_r <= 32'h0; 
            InstrM_r      <= 32'h0;
            PC_M_r        <= 32'h0;
            RS1_M_r       <= 5'h0;
            RS2_M_r       <= 5'h0;
            RD1_M_r       <= 32'h0;
            RD2_M_r       <= 32'h0;
            MemReadM_r    <= 1'b0;
            MemWriteM_r   <= 1'b0;
            dmem_mask_M_r <= 4'h0;
            WriteDataM_r  <= 32'h0;
            PCSrcM_r      <= 1'b0;
            PCTargetM_r   <= 32'h0;
            ValidW_r      <= 1'b0;
        end
        else if (Stall) begin
            RegWriteM_r   <= RegWriteM_r;
            ResultSrcM_r  <= ResultSrcM_r;
            JumpM_r       <= JumpM_r;
            RD_M_r        <= RD_M_r;
            PCPlus4M_r    <= PCPlus4M_r;
            ALU_ResultM_r <= ALU_ResultM_r;
            InstrM_r      <= InstrM_r;
            PC_M_r        <= PC_M_r;
            RS1_M_r       <= RS1_M_r;
            RS2_M_r       <= RS2_M_r;
            RD1_M_r       <= RD1_M_r;
            RD2_M_r       <= RD2_M_r;
            MemReadM_r    <= MemReadM_r;
            MemWriteM_r   <= MemWriteM_r;
            dmem_mask_M_r <= dmem_mask_M_r;
            WriteDataM_r  <= WriteDataM_r;
            PCSrcM_r      <= PCSrcM_r;
            PCTargetM_r   <= PCTargetM_r;
            ValidW_r      <= ValidW_r;
        end
        else begin
            RegWriteM_r   <= RegWriteM; 
            ResultSrcM_r  <= ResultSrcM;
            JumpM_r       <= JumpM;
            RD_M_r        <= RD_M;
            PCPlus4M_r    <= PCPlus4M; 
            ALU_ResultM_r <= ALU_ResultM; 
            InstrM_r      <= InstrM;
            PC_M_r        <= PC_M;
            RS1_M_r       <= RS1_M;
            RS2_M_r       <= RS2_M;
            RD1_M_r       <= RD1_M;
            RD2_M_r       <= RD2_M;
            MemReadM_r    <= MemReadM;
            MemWriteM_r   <= MemWriteM;
            dmem_mask_M_r <= dmem_mask_M;
            WriteDataM_r  <= WriteDataM;
            PCSrcM_r      <= PCSrcM;
            PCTargetM_r   <= PCTargetM;
            ValidW_r      <= ValidM;
        end
    end 

    
    // Outputs to Writeback Stage
    assign RegWriteW   = RegWriteM_r;
    assign ResultSrcW  = ResultSrcM_r;
    assign JumpW       = JumpM_r;
    assign RD_W        = RD_M_r;
    assign PCPlus4W    = PCPlus4M_r;
    assign ALU_ResultW = ALU_ResultM_r;
    assign InstrW      = InstrM_r;
    assign PC_W        = PC_M_r;
    assign RS1_W       = RS1_M_r;
    assign RS2_W       = RS2_M_r;
    assign RD1_W       = RD1_M_r;
    assign RD2_W       = RD2_M_r;
    assign MemReadW    = MemReadM_r;
    assign MemWriteW   = MemWriteM_r;
    assign dmem_mask_W = dmem_mask_M_r;
    assign WriteDataW  = WriteDataM_r;
    assign PCSrcW      = PCSrcM_r;
    assign PCTargetW   = PCTargetM_r;
    assign ValidW      = ValidW_r;

    // Tie off unused outputs (these are now handled in hart.v)
    assign ReadDataW    = 32'h0;
    assign RawReadDataW = 32'h0;

endmodule
