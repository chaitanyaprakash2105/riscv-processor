`default_nettype none

module cache (
    // Global clock.
    input  wire        i_clk,
    // Synchronous active-high reset.
    input  wire        i_rst,
    // External memory interface. See hart interface for details. This
    // interface is nearly identical to the phase 5 memory interface, with the
    // exception that the byte mask (`o_mem_mask`) has been removed. This is
    // no longer needed as the cache will only access the memory at word
    // granularity, and implement masking internally.
    input  wire        i_mem_ready,
    output wire [31:0] o_mem_addr,
    output wire        o_mem_ren,
    output wire        o_mem_wen,
    output wire [31:0] o_mem_wdata,
    input  wire [31:0] i_mem_rdata,
    input  wire        i_mem_valid,
    // Interface to CPU hart. This is nearly identical to the phase 5 hart memory
    // interface, but includes a stall signal (`o_busy`), and the input/output
    // polarities are swapped for obvious reasons.
    //
    // The CPU should use this as a stall signal for both instruction fetch
    // (IF) and memory (MEM) stages, from the instruction or data cache
    // respectively. If a memory request is made (`i_req_ren` for instruction
    // cache, or either `i_req_ren` or `i_req_wen` for data cache), this
    // should be asserted *combinationally* if the request results in a cache
    // miss.
    //
    // In case of a cache miss, the CPU must stall the respective pipeline
    // stage and deassert ren/wen on subsequent cycles, until the cache
    // deasserts `o_busy` to indicate it has serviced the cache miss. However,
    // the CPU must keep the other request lines constant. For example, the
    // CPU should not change the request address while stalling.
    output wire        o_busy,
    // 32-bit read/write address to access from the cache. This should be
    // 32-bit aligned (i.e. the two LSBs should be zero). See `i_req_mask` for
    // how to perform half-word and byte accesses to unaligned addresses.
    input  wire [31:0] i_req_addr,
    // When asserted, the cache should perform a read at the aligned address
    // specified by `i_req_addr` and return the 32-bit word at that address,
    // either immediately (i.e. combinationally) on a cache hit, or
    // synchronously on a cache miss. It is illegal to assert this and
    // `i_dmem_wen` on the same cycle.
    input  wire        i_req_ren,
    // When asserted, the cache should perform a write at the aligned address
    // specified by `i_req_addr` with the 32-bit word provided in
    // `o_req_wdata` (specified by the mask). This is necessarily synchronous,
    // but may either happen on the next clock edge (on a cache hit) or after
    // multiple cycles of latency (cache miss). As the cache is write-through
    // and write-allocate, writes must be applied to both the cache and
    // underlying memory.
    // It is illegal to assert this and `i_dmem_ren` on the same cycle.
    input  wire        i_req_wen,
    // The memory interface expects word (32 bit) aligned addresses. However,
    // WISC-25 supports byte and half-word loads and stores at unaligned and
    // 16-bit aligned addresses, respectively. To support this, the access
    // mask specifies which bytes within the 32-bit word are actually read
    // from or written to memory.
    input  wire [ 3:0] i_req_mask,
    // The 32-bit word to write to memory, if the request is a write
    // (i_req_wen is asserted). Only the bytes corresponding to set bits in
    // the mask should be written into the cache (and to backing memory).
    input  wire [31:0] i_req_wdata,
    // THe 32-bit data word read from memory on a read request.
    output wire [31:0] o_res_rdata
);
    // These parameters are equivalent to those provided in the project
    // 6 specification. Feel free to use them, but hardcoding these numbers
    // rather than using the localparams is also permitted, as long as the
    // same values are used (and consistent with the project specification).
    //
    // 32 sets * 2 ways per set * 16 bytes per way = 1K cache
    localparam O = 4;            // 4 bit offset => 16 byte cache line
    localparam S = 5;            // 5 bit set index => 32 sets
    localparam DEPTH = 2 ** S;   // 32 sets
    localparam W = 2;            // 2 way set associative, NMRU
    localparam T = 32 - O - S;   // 23 bit tag
    localparam D = 2 ** O / 4;   // 16 bytes per line / 4 bytes per word = 4 words per line

    // The following memory arrays model the cache structure. As this is
    // an internal implementation detail, you are *free* to modify these
    // arrays as you please.

    // Backing memory, modeled as two separate ways.
    reg [   31:0] datas0 [DEPTH - 1:0][D - 1:0];
    reg [   31:0] datas1 [DEPTH - 1:0][D - 1:0];
    reg [T - 1:0] tags0  [DEPTH - 1:0];
    reg [T - 1:0] tags1  [DEPTH - 1:0];
    reg [1:0] valid [DEPTH - 1:0];
    reg       lru   [DEPTH - 1:0];

    // Fill in your implementation here.

    // FSM logic 
    // IDLE: Idle state, gets a i_req_ren or i_req_wen, if its a hit, return data and do nothing
    // WRITE_MEM: If its a write hit, it goes to this state to write the data back into the memory
    // REFILL: If it's a miss, we shift to this state, now we go and fetch the data from memory, o_busy signal is asserted.
    //         After refill, we go back to idle state, this time its a hit, we resume operations likewise
    // WRITE_DONE: Deassert o_busy, go to IDLE state

    localparam IDLE = 2'b00;   
    localparam WRITE_MEM = 2'b01;  
    localparam REFILL = 2'b10;

    reg [1:0] state;
    reg [1:0] refill_count;
    reg [2:0] send_count;
    reg       way_to_evict;
    
    // NEW: Register to remember if we are refilling due to a Write Miss
    reg       pending_write; 
    
    integer i;

    wire [T-1:0] tag = i_req_addr[31:9];
    wire [S-1:0] idx = i_req_addr[8:4];
    wire [1:0]   wsel = i_req_addr[3:2];

    wire hit0 = valid[idx][0] && (tags0[idx] == tag);
    wire hit1 = valid[idx][1] && (tags1[idx] == tag);
    wire hit = hit0 || hit1;

    always @(posedge i_clk, posedge i_rst) begin
        if(i_rst) begin
            state <= IDLE;
            refill_count <= 0;
            send_count <= 0;
            way_to_evict <= 0;
            pending_write <= 0;
            for(i=0; i<DEPTH; i=i+1) begin
                valid[i] <= 2'b00;
                lru[i]   <= 1'b0;
            end
        end else begin
            case (state)
                IDLE: begin
                    if ((i_req_ren || i_req_wen) && !hit) begin
                        // True miss: choose a victim way:
                        //   - Prefer any invalid way (cold-fill), else evict the
                        //     *least* recently used way.
                        state         <= REFILL;
                        if (!valid[idx][0])
                            way_to_evict <= 1'b0;
                        else if (!valid[idx][1])
                            way_to_evict <= 1'b1;
                        else
                            // lru[idx] encodes the most recently used way:
                            //   0 => way 0 was MRU, evict way 1
                            //   1 => way 1 was MRU, evict way 0
                            way_to_evict <= (lru[idx] ? 1'b0 : 1'b1);
                        // We treat the miss-detection cycle as the first read
                        // request for word index 0, so count it here. This
                        // removes one extra stall cycle per miss compared to
                        // starting all requests in REFILL.
                        send_count    <= 2'd1;
                        refill_count  <= 0;
                        pending_write <= i_req_wen; 
                    end 
                    else if (hit) begin
                        // HIT: update MRU information for either read or write.
                        if (hit0)      lru[idx] <= 1'b0; // way 0 was most recently used
                        else if (hit1) lru[idx] <= 1'b1; // way 1 was most recently used

                        // WRITE HIT: update the cache line in-place, NO stall, NO miss.
                        if (i_req_wen) begin
                            if (hit0) begin
                                // way 0 hit
                                if (i_req_mask[0]) datas0[idx][wsel][ 7: 0] <= i_req_wdata[ 7: 0];
                                if (i_req_mask[1]) datas0[idx][wsel][15: 8] <= i_req_wdata[15: 8];
                                if (i_req_mask[2]) datas0[idx][wsel][23:16] <= i_req_wdata[23:16];
                                if (i_req_mask[3]) datas0[idx][wsel][31:24] <= i_req_wdata[31:24];
                            end else begin
                                // way 1 hit
                                if (i_req_mask[0]) datas1[idx][wsel][ 7: 0] <= i_req_wdata[ 7: 0];
                                if (i_req_mask[1]) datas1[idx][wsel][15: 8] <= i_req_wdata[15: 8];
                                if (i_req_mask[2]) datas1[idx][wsel][23:16] <= i_req_wdata[23:16];
                                if (i_req_mask[3]) datas1[idx][wsel][31:24] <= i_req_wdata[31:24];
                            end
                            pending_write <= 0;
                            // state stays IDLE, so o_busy stays 0 => testbench prints "Write HIT"
                        end
                    end
                end

                WRITE_MEM: begin
                    if(i_mem_ready) begin
                        // Update MRU based on which way now contains the line.
                        if (hit0)      lru[idx] <= 1'b0;
                        else if (hit1) lru[idx] <= 1'b1;
                        
                        // Because input addresses/data are constant during stall,
                        // we can use i_req_wdata/mask here safely.
                        if (hit0 || (pending_write && way_to_evict == 0)) begin
                            if (i_req_mask[0]) datas0[idx][wsel][ 7: 0] <= i_req_wdata[ 7: 0];
                            if (i_req_mask[1]) datas0[idx][wsel][15: 8] <= i_req_wdata[15: 8];
                            if (i_req_mask[2]) datas0[idx][wsel][23:16] <= i_req_wdata[23:16];
                            if (i_req_mask[3]) datas0[idx][wsel][31:24] <= i_req_wdata[31:24];
                        end else begin 
                            if (i_req_mask[0]) datas1[idx][wsel][ 7: 0] <= i_req_wdata[ 7: 0];
                            if (i_req_mask[1]) datas1[idx][wsel][15: 8] <= i_req_wdata[15: 8];
                            if (i_req_mask[2]) datas1[idx][wsel][23:16] <= i_req_wdata[23:16];
                            if (i_req_mask[3]) datas1[idx][wsel][31:24] <= i_req_wdata[31:24];
                        end
                        state <= IDLE;
                    end
                end

                REFILL: begin
                    if (i_mem_ready && send_count < 4) begin
                        send_count <= send_count + 1;
                    end

                    if(i_mem_valid) begin 
                        if(way_to_evict == 0) datas0[idx][refill_count] <= i_mem_rdata;
                        else                  datas1[idx][refill_count] <= i_mem_rdata;

                        refill_count <= refill_count + 1;

                        if(refill_count == 2'd3) begin
                            if(way_to_evict == 0) begin
                                tags0[idx]    <= tag;
                                valid[idx][0] <= 1'b1;
                                lru[idx]      <= 1'b0;  // way 0 just became MRU
                            end else begin
                                tags1[idx]    <= tag;
                                valid[idx][1] <= 1'b1;
                                lru[idx]      <= 1'b1;  // way 1 just became MRU
                            end
                            
                            // NEW: If this was a write miss, go to WRITE_MEM now!
                            // The CPU has dropped 'wen', so we must force the state.
                            if (pending_write) 
                                state <= WRITE_MEM;
                            else
                                state <= IDLE;
                        end
                    end
                end
            endcase
        end
    end
    wire is_idle_miss;
    assign is_idle_miss = (state == IDLE) && (i_req_ren || i_req_wen) && !hit;
    // wire is_write_stall;
    // assign is_write_stall = (state == IDLE) && (i_req_wen && hit && !i_mem_ready);

    assign o_busy = (state == REFILL) || (state == WRITE_MEM) || is_idle_miss;
    
    // On a miss we start the line refill immediately in the miss-detection
    // cycle (is_idle_miss), requesting the first word (index 0). Subsequent
    // words use send_count as the word index.
    wire [1:0] refill_index = (state == REFILL) ? send_count[1:0] : 2'd0;

    assign o_mem_addr = (state == REFILL || is_idle_miss)
                        ? {i_req_addr[31:4], refill_index, 2'b00}
                        : i_req_addr;

    assign o_mem_ren = ((state == REFILL) && (send_count < 4)) || is_idle_miss;
    assign o_mem_wen = (state == WRITE_MEM) || (state == IDLE && i_req_wen && hit);
    assign o_mem_wdata = i_req_wdata;
    
    assign o_res_rdata = (hit0) ? datas0[idx][wsel] : 
                         (hit1) ? datas1[idx][wsel] : 32'd0;
endmodule

`default_nettype wire
