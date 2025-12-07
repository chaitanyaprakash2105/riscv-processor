`default_nettype none

module hart #(
    // After reset, the program counter (PC) should be initialized to this
    // address and start executing instructions from there.
    parameter RESET_ADDR = 32'h00000000
) (
    // Global clock.
    input  wire        i_clk,
    // Synchronous active-high reset.
    input  wire        i_rst,
    // Instruction fetch goes through a read only instruction memory (imem)
    // port. The port accepts a 32-bit address (e.g. from the program counter)
    // per cycle and combinationally returns a 32-bit instruction word. This
    // is not representative of a realistic memory interface; it has been
    // modeled as more similar to a DFF or SRAM to simplify phase 3. In
    // later phases, you will replace this with a more realistic memory.
    //
    // 32-bit read address for the instruction memory. This is expected to be
    // 4 byte aligned - that is, the two LSBs should be zero.
    output wire [31:0] o_imem_raddr,
    // Instruction word fetched from memory, available synchronously after
    // the next clock edge.
    // NOTE: This is different from the previous phase. To accomodate a
    // multi-cycle pipelined design, the instruction memory read is
    // now synchronous.
    input  wire [31:0] i_imem_rdata,
    // Data memory accesses go through a separate read/write data memory (dmem)
    // that is shared between read (load) and write (stored). The port accepts
    // a 32-bit address, read or write enable, and mask (explained below) each
    // cycle. Reads are combinational - values are available immediately after
    // updating the address and asserting read enable. Writes occur on (and
    // are visible at) the next clock edge.
    //
    // Read/write address for the data memory. This should be 32-bit aligned
    // (i.e. the two LSB should be zero). See `o_dmem_mask` for how to perform
    // half-word and byte accesses at unaligned addresses.
    output wire [31:0] o_dmem_addr,
    // When asserted, the memory will perform a read at the aligned address
    // specified by `i_addr` and return the 32-bit word at that address
    // immediately (i.e. combinationally). It is illegal to assert this and
    // `o_dmem_wen` on the same cycle.
    output wire        o_dmem_ren,
    // When asserted, the memory will perform a write to the aligned address
    // `o_dmem_addr`. When asserted, the memory will write the bytes in
    // `o_dmem_wdata` (specified by the mask) to memory at the specified
    // address on the next rising clock edge. It is illegal to assert this and
    // `o_dmem_ren` on the same cycle.
    output wire        o_dmem_wen,
    // The 32-bit word to write to memory when `o_dmem_wen` is asserted. When
    // write enable is asserted, the byte lanes specified by the mask will be
    // written to the memory word at the aligned address at the next rising
    // clock edge. The other byte lanes of the word will be unaffected.
    output wire [31:0] o_dmem_wdata,
    // The dmem interface expects word (32 bit) aligned addresses. However,
    // WISC-25 supports byte and half-word loads and stores at unaligned and
    // 16-bit aligned addresses, respectively. To support this, the access
    // mask specifies which bytes within the 32-bit word are actually read
    // from or written to memory.
    //
    // To perform a half-word read at address 0x00001002, align `o_dmem_addr`
    // to 0x00001000, assert `o_dmem_ren`, and set the mask to 0b1100 to
    // indicate that only the upper two bytes should be read. Only the upper
    // two bytes of `i_dmem_rdata` can be assumed to have valid data; to
    // calculate the final value of the `lh[u]` instruction, shift the rdata
    // word right by 16 bits and sign/zero extend as appropriate.
    //
    // To perform a byte write at address 0x00002003, align `o_dmem_addr` to
    // `0x00002000`, assert `o_dmem_wen`, and set the mask to 0b1000 to
    // indicate that only the upper byte should be written. On the next clock
    // cycle, the upper byte of `o_dmem_wdata` will be written to memory, with
    // the other three bytes of the aligned word unaffected. Remember to shift
    // the value of the `sb` instruction left by 24 bits to place it in the
    // appropriate byte lane.
    output wire [ 3:0] o_dmem_mask,
    // The 32-bit word read from data memory. When `o_dmem_ren` is asserted,
    // after the next clock edge, this will reflect the contents of memory
    // at the specified address, for the bytes enabled by the mask. When
    // read enable is not asserted, or for bytes not set in the mask, the
    // value is undefined.
    // NOTE: This is different from the previous phase. To accomodate a
    // multi-cycle pipelined design, the data memory read is
    // now synchronous.
    input  wire [31:0] i_dmem_rdata,
	// The output `retire` interface is used to signal to the testbench that
    // the CPU has completed and retired an instruction. A single cycle
    // implementation will assert this every cycle; however, a pipelined
    // implementation that needs to stall (due to internal hazards or waiting
    // on memory accesses) will not assert the signal on cycles where the
    // instruction in the writeback stage is not retiring.
    //
    // Asserted when an instruction is being retired this cycle. If this is
    // not asserted, the other retire signals are ignored and may be left invalid.
    output wire        o_retire_valid,
    // The 32 bit instruction word of the instrution being retired. This
    // should be the unmodified instruction word fetched from instruction
    // memory.
    output wire [31:0] o_retire_inst,
    // Asserted if the instruction produced a trap, due to an illegal
    // instruction, unaligned data memory access, or unaligned instruction
    // address on a taken branch or jump.
    output wire        o_retire_trap,
    // Asserted if the instruction is an `ebreak` instruction used to halt the
    // processor. This is used for debugging and testing purposes to end
    // a program.
    output wire        o_retire_halt,
    // The first register address read by the instruction being retired. If
    // the instruction does not read from a register (like `lui`), this
    // should be 5'd0.
    output wire [ 4:0] o_retire_rs1_raddr,
    // The second register address read by the instruction being retired. If
    // the instruction does not read from a second register (like `addi`), this
    // should be 5'd0.
    output wire [ 4:0] o_retire_rs2_raddr,
    // The first source register data read from the register file (in the
    // decode stage) for the instruction being retired. If rs1 is 5'd0, this
    // should also be 32'd0.
    output wire [31:0] o_retire_rs1_rdata,
    // The second source register data read from the register file (in the
    // decode stage) for the instruction being retired. If rs2 is 5'd0, this
    // should also be 32'd0.
    output wire [31:0] o_retire_rs2_rdata,
    // The destination register address written by the instruction being
    // retired. If the instruction does not write to a register (like `sw`),
    // this should be 5'd0.
    output wire [ 4:0] o_retire_rd_waddr,
    // The destination register data written to the register file in the
    // writeback stage by this instruction. If rd is 5'd0, this field is
    // ignored and can be treated as a don't care.
    output wire [31:0] o_retire_rd_wdata,
    // The following data memory retire interface is used to record the
    // memory transactions completed by the instruction being retired.
    // As such, it mirrors the transactions happening on the main data
    // memory interface (o_dmem_* and i_dmem_*) but is delayed to match
    // the retirement of the instruction. You can hook this up by just
    // registering the main dmem interface signals into the writeback
    // stage of your pipeline.
    //
    // All these fields are don't-care for instructions that do not
    // access data memory (o_retire_dmem_ren and o_retire_dmem_wen
    // not asserted).
    // NOTE: This interface is new for phase 5 in order to account for
    // the delay between data memory accesses and instruction retire.
    //
    // The 32-bit data memory address accessed by the instruction.
    output wire [31:0] o_retire_dmem_addr,
    // The byte masked used for the data memory access.
    output wire [ 3:0] o_retire_dmem_mask,
    // Asserted if the instruction performed a read (load) from data memory.
    output wire        o_retire_dmem_ren,
    // Asserted if the instruction performed a write (store) to data memory.
    output wire        o_retire_dmem_wen,
    // The 32-bit data read from memory by a load instruction.
    output wire [31:0] o_retire_dmem_rdata,
    // The 32-bit data written to memory by a store instruction.
    output wire [31:0] o_retire_dmem_wdata,
    // The current program counter of the instruction being retired - i.e.
    // the instruction memory address that the instruction was fetched from.
    output wire [31:0] o_retire_pc,
    // the next program counter after the instruction is retired. For most
    // instructions, this is `o_retire_pc + 4`, but must be the branch or jump
    // target for *taken* branches and jumps.
    output wire [31:0] o_retire_next_pc,
    input wire i_imem_ready,
    input wire i_imem_valid,
    input wire i_dmem_ready,
    input wire i_dmem_valid,
    output wire o_imem_ren    

`ifdef RISCV_FORMAL
    ,`RVFI_OUTPUTS,
`endif
);
    // Fill in your implementation here.

	wire [31:0] cpu_dmem_addr;
    wire [31:0] cpu_dmem_wdata;
    wire        cpu_dmem_ren;
    wire        cpu_dmem_wen;
    wire [3:0]  cpu_dmem_mask;
    wire [31:0] cpu_dmem_rdata;
    wire [31:0] cpu_imem_raddr; 
    wire [31:0] cpu_imem_rdata; // Data returning from I-Cache to Fetch Stage
    	
    // --- Pipeline Control Wires ---
    wire [1:0] ResultSrcE, ResultSrcM, ResultSrcW;
    wire       JumpE, JumpM, JumpW;
    wire       PCSrcE, PCSrcM, PCSrcW;
    wire       RegWriteE, RegWriteM, RegWriteW;
    wire       ALUSrcE;
    wire       MemWriteE, MemWriteM, MemWriteW;
    wire       MemReadM, MemReadW;
    wire       BranchE;
    wire [2:0] ALUControlE;
    wire [31:0] ReadDataW_from_mem_mod, RawReadDataW_from_mem_mod;
    
    // --- Pipeline Data Wires ---
    wire [4:0]  RD_E, RD_M, RDW;
    wire [4:0]  RS1_E, RS1_M, RS1_W;
    wire [4:0]  RS2_E, RS2_M, RS2_W;
    wire [31:0] PCTargetE, PCTargetM, PCTargetW;
    wire [31:0] InstrD, InstrE, InstrM, InstrW;
    wire ValidE;
    wire [31:0] PCD, PCE, PC_M, PC_W;
    wire [31:0] PCPlus4D, PCPlus4E, PCPlus4M, PCPlus4W;
    wire [31:0] ResultW;
    wire [31:0] RD1_E, RD1_M, RD1_W;
    wire [31:0] RD2_E, RD2_M, RD2_W;
    wire [31:0] Imm_Ext_E;
    wire ValidM;
    wire ValidW;
    wire [31:0] WriteDataM, WriteDataW; // WriteDataW is for retire
    wire [31:0] ALU_ResultM, ALU_ResultW;
    // wire [31:0] ReadDataW;
    // wire [31:0] RawReadDataW;
    wire [3:0]  dmem_mask_M, dmem_mask_W;
    // Early redirect from Decode
    wire        PCSrcD_dec;   // raw from decode (JAL only)
    wire [31:0] PCTargetD;
    // always @(posedge i_clk)
    //     $display("clk=%0t rst=%b", $time, i_clk);

    // --- Forwarding Wires ---
    wire [1:0] ForwardBE, ForwardAE;
    wire stall_cpu;

    // Load-use hazard detection (stall + bubble into EX) without case-equality.
    // Use X-safe if/else with == so unknowns fall to the safe default 0.
    reg        is_load_E_safe;
    wire [6:0] opD              = InstrD[6:0];
    reg        uses_rs1_safe;
    reg        uses_rs2_safe;
    reg        rs1_dep_safe;
    reg        rs2_dep_safe;
    reg        rd_nonzero_safe;

    always @* begin
        // Default-safe values
        is_load_E_safe  = 1'b0;
        uses_rs1_safe   = 1'b0;
        uses_rs2_safe   = 1'b0;
        rs1_dep_safe    = 1'b0;
        rs2_dep_safe    = 1'b0;
        rd_nonzero_safe = 1'b1; // assume nonzero unless positively known zero

        // is_load_E_safe: ResultSrcE == 2'b01
        if (ResultSrcE == 2'b01) is_load_E_safe = 1'b1; else is_load_E_safe = 1'b0;

        // uses_rs1_safe: match specific opcodes
        if      (opD == 7'b0110011) uses_rs1_safe = 1'b1; // R-type
        else if (opD == 7'b0010011) uses_rs1_safe = 1'b1; // OP-IMM
        else if (opD == 7'b0000011) uses_rs1_safe = 1'b1; // LOAD
        else if (opD == 7'b0100011) uses_rs1_safe = 1'b1; // STORE
        else if (opD == 7'b1100011) uses_rs1_safe = 1'b1; // BRANCH
        else if (opD == 7'b1100111) uses_rs1_safe = 1'b1; // JALR
        else                        uses_rs1_safe = 1'b0;

        // uses_rs2_safe: match specific opcodes
        if      (opD == 7'b0110011) uses_rs2_safe = 1'b1; // R-type
        else if (opD == 7'b0100011) uses_rs2_safe = 1'b1; // STORE
        else if (opD == 7'b1100011) uses_rs2_safe = 1'b1; // BRANCH
        else                        uses_rs2_safe = 1'b0;

        // rd_nonzero_safe: treat unknown as nonzero (safe for hazard detect)
        if (RD_E == 5'd0) rd_nonzero_safe = 1'b0; else rd_nonzero_safe = 1'b1;

        // rs1/rs2 dependency checks: unknown compare resolves to else => 0
        if (uses_rs1_safe && (InstrD[19:15] == RD_E)) rs1_dep_safe = 1'b1; else rs1_dep_safe = 1'b0;
        if (uses_rs2_safe && (InstrD[24:20] == RD_E)) rs2_dep_safe = 1'b1; else rs2_dep_safe = 1'b0;
    end
    wire       load_use_hazard  = is_load_E_safe && rd_nonzero_safe && (rs1_dep_safe || rs2_dep_safe);
    wire       StallF = load_use_hazard | stall_cpu;
    wire       StallD = load_use_hazard | stall_cpu;

    // Module Initiation
    // Fetch Stage
    fetch_cycle Fetch (
        .clk(i_clk), 
        .rst(i_rst), 
        .PCSrcE(PCSrcE),
        .PCTargetE(PCTargetE),
        .StallF(StallF),
        .StallD(StallD),
        .i_imem_rdata(cpu_imem_rdata),
        .o_imem_raddr(cpu_imem_raddr),
        .InstrD(InstrD), 
        .PCD(PCD), 
        .PCPlus4D(PCPlus4D)
    );
    // FlushE: flush ID/EX on taken branch/jump or load-use hazard
    wire FlushE = PCSrcE | load_use_hazard;

    // Decode Stage
    decode_cycle Decode (
        .clk(i_clk), 
        .rst(i_rst), 
        .InstrD(InstrD), 
        .PCSrcE(FlushE), // Combined flush for ID/EX bubble
        .PCD(PCD),
        .Stall(stall_cpu), 
        .PCPlus4D(PCPlus4D), 
        .RegWriteW(RegWriteW), 
        .RDW(RDW), 
        .ResultW(ResultW), 
        .RegWriteE(RegWriteE), 
        .ALUSrcE(ALUSrcE), 
        .MemWriteE(MemWriteE), 
        .ResultSrcE(ResultSrcE),
        .BranchE(BranchE),   
        .JumpE(JumpE),
        .ALUControlE(ALUControlE), 
        .RD1_E(RD1_E), 
        .RD2_E(RD2_E), 
        .Imm_Ext_E(Imm_Ext_E), 
        .RD_E(RD_E), 
        .PCE(PCE), 
        .PCPlus4E(PCPlus4E),
        .RS1_E(RS1_E),
        .RS2_E(RS2_E),
        .InstrE(InstrE),
        .ValidE(ValidE),
        .PCSrcD(),      // JAL early redirect not used in this design
        .PCTargetD()
    );

    // Execute Stage
    execute_cycle Execute (
        .clk(i_clk), 
        .rst(i_rst), 
        // Inputs from Decode
        .RegWriteE(RegWriteE), 
        .ALUSrcE(ALUSrcE), 
        .MemWriteE(MemWriteE), 
        .ResultSrcE(ResultSrcE),
        .BranchE(BranchE), 
        .ValidE(ValidE),
        .JumpE(JumpE),
        .ALUControlE(ALUControlE), 
        .RD1_E(RD1_E), 
        .RD2_E(RD2_E),
        .Stall(stall_cpu),  
        .Imm_Ext_E(Imm_Ext_E),
        .RD_E(RD_E), 
        .PCE(PCE), 
        .PCPlus4E(PCPlus4E), 
        .InstrE(InstrE),
        .RS1_E(RS1_E), 
        .RS2_E(RS2_E),
        // Inputs from WB
        .ResultW(ResultW),
        .ForwardA_E(ForwardAE),
        .ForwardB_E(ForwardBE),
        // Outputs to Fetch
        .PCSrcE(PCSrcE), 
        .PCTargetE(PCTargetE), 
        // Outputs to Memory
        .RegWriteM(RegWriteM), 
        .MemWriteM(MemWriteM), 
        .ResultSrcM(ResultSrcM),
        .JumpM(JumpM),
        .RD_M(RD_M), 
        .ValidM(ValidM),
        .PCPlus4M(PCPlus4M), 
        .WriteDataM(WriteDataM), 
        .ALU_ResultM(ALU_ResultM),
        .MemReadM(MemReadM),
        .dmem_mask_M(dmem_mask_M),
        // Retire Outputs to Memory
        .InstrM(InstrM),
        .PC_M(PC_M),
        .RS1_M(RS1_M),
        .RS2_M(RS2_M),
        .RD1_M(RD1_M),
        .RD2_M(RD2_M),
        .PCSrcM(PCSrcM),      // <-- Must add to execute_cycle.v
        .PCTargetM(PCTargetM) // <-- Must add to execute_cycle.v
    );
    
    // --- Store-to-Load Forwarding Logic ---

    wire load_after_store_hazard ;
    assign load_after_store_hazard = MemReadM    && // lw in MEM stage
                                     MemWriteW   && // sw in WB stage
                                     (ALU_ResultM == ALU_ResultW); // Addr match


    wire [31:0] dmem_rdata_forwarded;

    assign dmem_rdata_forwarded[ 7: 0] = (load_after_store_hazard && dmem_mask_W[0]) ? WriteDataW[ 7: 0] : cpu_dmem_rdata[ 7: 0];
    assign dmem_rdata_forwarded[15: 8] = (load_after_store_hazard && dmem_mask_W[1]) ? WriteDataW[15: 8] : cpu_dmem_rdata[15: 8];
    assign dmem_rdata_forwarded[23:16] = (load_after_store_hazard && dmem_mask_W[2]) ? WriteDataW[23:16] : cpu_dmem_rdata[23:16];
    assign dmem_rdata_forwarded[31:24] = (load_after_store_hazard && dmem_mask_W[3]) ? WriteDataW[31:24] : cpu_dmem_rdata[31:24];
    
    // Memory Stage
    memory_cycle Memory (
        .clk(i_clk), 
        .rst(i_rst), 
        // Inputs from Execute
        .RegWriteM(RegWriteM), 
        .MemWriteM(MemWriteM), 
        .MemReadM(MemReadM),
        .ValidM(ValidM),
        .dmem_mask_M(dmem_mask_M), 
        .ResultSrcM(ResultSrcM),
        .JumpM(JumpM),
        .RawReadDataW(RawReadDataW_from_mem_mod),
        .RD_M(RD_M), 
        .Stall(stall_cpu), 
        .PCPlus4M(PCPlus4M), 
        .WriteDataM(WriteDataM), 
        .ALU_ResultM(ALU_ResultM), 
        // Retire Inputs from Execute
        .InstrM(InstrM), 
        .PC_M(PC_M), 
        .RS1_M(RS1_M), 
        .RS2_M(RS2_M),
        .RD1_M(RD1_M), 
        .RD2_M(RD2_M),
        .PCSrcM(PCSrcM),      // <-- Must add to memory_cycle.v
        .PCTargetM(PCTargetM),// <-- Must add to memory_cycle.v
        // DMEM interface
        .i_dmem_rdata(cpu_dmem_rdata), //cc - cache change i_dmem_rdata-> cpu_dmem_rdata
        .o_dmem_addr(cpu_dmem_addr), //cc - cache change i_dmem_addr-> cpu_dmem_addr
        .o_dmem_wdata(cpu_dmem_wdata), //cc o_dmem_wdata -> cpu_dmem_wdata
        .o_dmem_wen(cpu_dmem_wen), //cc o_dmem_wen -> cpu_dmem_wen 
        .o_dmem_ren(cpu_dmem_ren), //cc o_dmem_ren -> cpu_dmem_ren
        .o_dmem_mask(cpu_dmem_mask), //cc o_dmem_mask -> cpu_dmem_mask
        // Outputs to Writeback
        .RegWriteW(RegWriteW), 
        .ResultSrcW(ResultSrcW),
        .JumpW(JumpW),
        .RD_W(RDW), 
        .PCPlus4W(PCPlus4W), 
        .ALU_ResultW(ALU_ResultW), 
        .ReadDataW(ReadDataW_from_mem_mod),
        // Retire Outputs to Writeback
        .InstrW(InstrW), 
        .ValidW(ValidW),
        .PC_W(PC_W), 
        .RS1_W(RS1_W), 
        .RS2_W(RS2_W),
        .RD1_W(RD1_W), 
        .RD2_W(RD2_W),
        .MemReadW(MemReadW), 
        .MemWriteW(MemWriteW), 
        .dmem_mask_W(dmem_mask_W),
        .WriteDataW(WriteDataW),
        .PCSrcW(PCSrcW),      // <-- Must add to memory_cycle.v
        .PCTargetW(PCTargetW) // <-- Must add to memory_cycle.v
    );

    wire [1:0] addr_lsb_W = ALU_ResultW[1:0];
    wire [2:0] funct3_W   = InstrW[14:12];
    wire       MemRead_W  = MemReadW; // Use the pipelined MemRead signal

    wire [31:0] load_data_aligned_W;
    reg  [31:0] load_data_extended_W; // This is the final data

    // Use the forwarded data if forwarding, otherwise use raw memory data.
    // Latch at MEM/WB boundary so WB sees the data for its own load, not the
    // following memory access.
    reg [31:0] dmem_data_in_wb;
    always @(posedge i_clk or posedge i_rst) begin
        if (i_rst)
            dmem_data_in_wb <= 32'h0;
        else if (!stall_cpu)
            dmem_data_in_wb <= dmem_rdata_forwarded;
    end
    
    assign load_data_aligned_W = (dmem_data_in_wb >> (addr_lsb_W * 8));

    always @(*) begin
        load_data_extended_W = load_data_aligned_W;
        if (MemRead_W) begin
            case (funct3_W)
                3'b000: load_data_extended_W = $signed(load_data_aligned_W[7:0]);
                3'b001: load_data_extended_W = $signed(load_data_aligned_W[15:0]);
                3'b010: load_data_extended_W = load_data_aligned_W;
                3'b100: load_data_extended_W = {24'b0, load_data_aligned_W[7:0]};
                3'b101: load_data_extended_W = {16'b0, load_data_aligned_W[15:0]};
                default: load_data_extended_W = load_data_aligned_W;
            endcase
        end
    end
    // --- End of New Logic ---

    // Write Back Stage
    writeback_cycle WriteBack (
        .clk(i_clk), // <-- Fixed typo
        .rst(i_rst), // <-- Fixed typo
        .ResultSrcW(ResultSrcW),
        .JumpW(JumpW),
        .PCPlus4W(PCPlus4W), 
        .ALU_ResultW(ALU_ResultW), 
        .ReadDataW(load_data_extended_W), 
        .ResultW(ResultW)
    );

    // // Hazard Unit
    hazard_unit FWD_Unit (
        .rst(i_rst),          // <-- See critical note on reset polarity below
        .RegWriteM(RegWriteM), 
        .RegWriteW(RegWriteW), 
        .RD_M(RD_M), 
        .RD_W(RDW),         // Connects to the RDW wire in hart.v
        .Rs1_E(RS1_E), 
        .Rs2_E(RS2_E), 
        .ForwardAE(ForwardAE), 
        .ForwardBE(ForwardBE)
    );

    //Instantiation of cache interface

    //   wire        i_imem_ready;
    //  wire [31:0] o_mem_addr,
    //  wire        o_mem_ren,
    //  wire        o_mem_wen,
    //  wire [31:0] o_mem_wdata,
    //  wire [31:0] i_mem_rdata,
    // input wire        i_imem_valid;
    //  wire        o_busy,
    //  wire [31:0] i_req_addr,
    //  wire        i_req_ren,
    //  wire        i_req_wen,
    //  wire [ 3:0] i_req_mask,
    //  wire [31:0] i_req_wdata,
    //  wire [31:0] o_res_rdata,
    
    // ========================================================================
    // --- INTERNAL CACHE INTERFACE WIRES ---
    // ========================================================================

    // 1. Global Stall Logic
    // If either cache is busy (fetching from memory), we freeze the whole CPU.
    wire icache_busy;
    wire dcache_busy;
    // For now, all stages share the same global stall. In later phases,
    // these aliases let us split I-cache vs D-cache and per-stage stalls
    // without changing external behavior.
    wire stall_icache;
    wire stall_dcache;
    wire stall_if;
    wire stall_id;
    wire stall_ex;
    wire stall_mem;
    wire stall_wb;

    assign stall_icache = icache_busy;
    assign stall_dcache = dcache_busy;

    // Phase 1: keep all stages using the same combined stall, matching
    // the original design semantics.
    assign stall_if  = stall_icache | stall_dcache;
    assign stall_id  = stall_icache | stall_dcache;
    assign stall_ex  = stall_icache | stall_dcache;
    assign stall_mem = stall_icache | stall_dcache;
    assign stall_wb  = stall_icache | stall_dcache;

    assign stall_cpu = icache_busy | dcache_busy;

    // 2. I-Cache Interface Wires (Between Fetch Stage and I-Cache)
    // The Fetch stage will drive 'cpu_imem_addr' instead of 'o_imem_raddr'


    // 3. D-Cache Interface Wires (Between Memory Stage and D-Cache)
    // The Memory stage will drive these instead of the top-level ports
 // Data returning from D-Cache to Mem Stage

    // 4. Dummy signals for unused Cache inputs
    // The I-Cache doesn't write, so we need to tie its write ports to 0.
    wire [31:0] zero_wdata = 32'h0;
    wire [3:0]  zero_mask  = 4'h0;
    // wire i_dmem_ready;
    // wire i_dmem_valid;


    // ========================================================================
    // --- INSTANTIATION: INSTRUCTION CACHE ---
    // ========================================================================
    cache I_Cache (
        .i_clk          (i_clk),
        .i_rst          (i_rst),
        
        // --- Memory Side (To Top-Level Ports) ---
        .i_mem_ready    (i_imem_ready),
        .i_mem_valid    (i_imem_valid),
        .i_mem_rdata    (i_imem_rdata),
        .o_mem_addr     (o_imem_raddr),
        .o_mem_wdata    (),              // I-Cache never writes to memory
        .o_mem_ren      (o_imem_ren),              // Usually implied by address change for Imem, or unconnected
        .o_mem_wen      (),              // I-Cache never writes
        
        // --- CPU Side (To Fetch_Cycle via Internal Wires) ---
        .o_busy         (icache_busy),
        .i_req_addr     (cpu_imem_raddr), // From Fetch Stage
        .i_req_ren      (1'b1),          // CPU always wants to read instructions
        .i_req_wen      (1'b0),          // CPU never writes instructions
        .i_req_mask     (4'b1111),       // Full word access
        .i_req_wdata    (32'b0),         // No write data
        .o_res_rdata    (cpu_imem_rdata) // To Fetch Stage
    );

    // ========================================================================
    // --- INSTANTIATION: DATA CACHE ---
    // ========================================================================
    cache D_Cache (
        .i_clk          (i_clk),
        .i_rst          (i_rst),
        
        // --- Memory Side (To Top-Level Ports) ---
        .i_mem_ready    (i_dmem_ready),
        .i_mem_valid    (i_dmem_valid),
        .i_mem_rdata    (i_dmem_rdata),
        .o_mem_addr     (o_dmem_addr),
        .o_mem_wdata    (o_dmem_wdata),
        .o_mem_ren      (o_dmem_ren),
        .o_mem_wen      (o_dmem_wen),
        
        // --- CPU Side (To Memory_Cycle via Internal Wires) ---
        .o_busy         (dcache_busy),
        .i_req_addr     (cpu_dmem_addr),  // From Memory Stage
        .i_req_ren      (cpu_dmem_ren),   // From Memory Stage
        .i_req_wen      (cpu_dmem_wen),   // From Memory Stage
        .i_req_mask     (cpu_dmem_mask),  // From Memory Stage
        .i_req_wdata    (cpu_dmem_wdata), // From Memory Stage
        .o_res_rdata    (cpu_dmem_rdata)  // To Memory Stage
    );

    // --- Assigning all retire signals ---
    // Gate the very first retire after reset to avoid duplicate-first
    // trace artifacts across differing TB reset phasing.
    // reg retire_enabled;
    // always @(posedge i_clk or posedge i_rst) begin
    //     if (i_rst)
    //         retire_enabled <= 1'b0;
    //     else if (ValidW)
    //         retire_enabled <= 1'b1;
    // end
    // assign o_retire_valid = retire_enabled ? ValidW : 1'b0; 
    assign o_retire_valid = ValidW && !stall_cpu;
    // Check for ebreak (0x00100073). Opcode=SYSTEM, funct12=1
    // This is a simple implementation. A full implementation would pipe a 
    // "Halt" signal from the control unit.
    assign o_retire_halt = (InstrW == 32'h00100073);
    
    // Trap is tied low. You would add logic for illegal instructions,
    // unaligned accesses, etc., and pipeline that signal.
    assign o_retire_trap = 1'b0; 

    // Pipelined instruction and PC
    assign o_retire_inst = InstrW;
    assign o_retire_pc   = PC_W;
    
    // Pipelined register addresses and data (from Decode)
    assign o_retire_rs1_raddr = RS1_W;
    assign o_retire_rs2_raddr = RS2_W;
    assign o_retire_rs1_rdata = RD1_W;
    assign o_retire_rs2_rdata = MemWriteW ? WriteDataW : RD2_W;
    
    // Register writeback data (from Writeback)
    assign o_retire_rd_waddr = RegWriteW ? RDW : 5'd0;
    assign o_retire_rd_wdata = ResultW;
    
    // Data memory transaction details (pipelined from Memory)
    // Use the aligned address to mirror dmem port behavior
    assign o_retire_dmem_addr  = {ALU_ResultW[31:2], 2'b00};
    assign o_retire_dmem_mask  = dmem_mask_W;
    assign o_retire_dmem_ren   = MemReadW;
    assign o_retire_dmem_wen   = MemWriteW;
    assign o_retire_dmem_rdata = dmem_data_in_wb;                    // Aligned word read
    assign o_retire_dmem_wdata = (WriteDataW << (addr_lsb_W * 8));   // Shifted to byte lanes
    
    // Next PC logic
    assign o_retire_next_pc = PCSrcW ? PCTargetW : PCPlus4W;

endmodule

`default_nettype wire
