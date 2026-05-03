// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// N-entry write-back forwarding buffer for single-port SRAMs.
//
// Same port list as `sram_forwarding_buffer` (single-entry) with two
// additional outputs used by the access controller to loosen its
// speculative-writeback gating:
//   - `buf_has_free_clean_o`: at least one entry is invalid or clean;
//      the next allocation will not force a writeback.
//   - `buf_near_full_o`    : N-1 or more entries are dirty; the next
//      allocation will need to evict a dirty entry unless drained first.
//
// Allocation priority (populate / full-line replace):
//   1. Same-address entry (reuse, preserves part tracking).
//   2. Invalid entry.
//   3. LRU-clean entry.
//   4. LRU entry (requires prior writeback by caller).
//
// Writeback selection: LRU if dirty, else the first dirty entry.
// Pseudo-LRU: whenever an entry is touched, the other becomes LRU.
//
// Invariants:
//   - At any time at most one valid entry holds a given address.
//   - A pending SRAM read populates exactly one entry (sram_rd_target_q).

`include "common_cells/registers.svh"
module sram_forwarding_buffer_multi #(
    parameter int unsigned Depth           = 512,
    parameter int unsigned NumWordsPerLine = 2,
    parameter int unsigned WordWidth       = 32,
    parameter int unsigned ByteWidth       = 8,
    parameter bit          Enable          = 1'b1,
    parameter int unsigned PartSplit       = 1,
    parameter int unsigned NumEntries      = 2,
    // -- Derived (do not override) --
    localparam int unsigned DataWidth      = WordWidth * NumWordsPerLine,
    localparam int unsigned MaskBits       = DataWidth / ByteWidth,
    localparam int unsigned PartIdxWidth   = (PartSplit > 1) ? $clog2(PartSplit) : 1,
    localparam int unsigned PartMaskBits   = MaskBits / ((PartSplit > 1) ? PartSplit : 1),
    localparam int unsigned EntryIdxWidth  = (NumEntries > 1) ? $clog2(NumEntries) : 1,
    localparam type         data_t         = logic [DataWidth-1:0],
    localparam type         mask_t         = logic [MaskBits-1:0],
    localparam type         addr_t         = logic [$clog2(Depth)-1:0]
)(
    input  logic   clk_i,
    input  logic   rst_ni,

    input  addr_t  rd_addr_i,
    input  logic   rd_valid_i,
    input  logic   rd_ready_i,
    input  logic [PartIdxWidth-1:0] rd_part_idx_i,
    input  logic                    rd_all_parts_i,

    input  addr_t  wr_addr_i,
    input  data_t  wr_data_i,
    input  mask_t  wr_mask_i,
    input  logic   wr_req_i,

    input  logic   sram_rd_issued_i,
    input  data_t  sram_rdata_i,
    input  logic   sram_wr_req_i,
    input  addr_t  sram_wr_addr_i,

    output logic   rd_hit_comb_o,
    output logic   wr_hit_comb_o,
    // 1 iff this absorption leaves the targeted entry holding the WHOLE line.
    output logic   wr_full_coverage_o,

    output logic   wb_needed_o,
    output addr_t  wb_addr_o,
    output data_t  wb_data_o,
    output mask_t  wb_mask_o,
    input  logic   wb_done_i,

    // Phase 3 advisory (single-entry (D) only; accepted-but-unused here
    // to keep port lists aligned across the two buffer flavours).
    input  logic   wr_target_valid_i,

    output data_t  fwd_rdata_o,
    output logic   fwd_hit_o,

    // Multi-entry state exposed to the access controller for SpecWb gating.
    output logic   buf_has_free_clean_o,   // at least one entry is invalid or clean
    output logic   buf_near_full_o,        // popcount(dirty) >= NumEntries-1

    output logic [31:0] stat_rd_hit_o,
    output logic [31:0] stat_rd_miss_o,
    output logic [31:0] stat_wr_merge_o,
    output logic [31:0] stat_wr_inval_o,
    output logic [31:0] stat_rd_total_o,
    output logic [31:0] stat_wr_total_o,
    output logic [31:0] stat_sram_rd_o,
    output logic [31:0] stat_wb_o
);

    // ==================================================================
    //                           State
    // ==================================================================
    // All packed: outer dimension = NumEntries, inner = each field's width.
    data_t                   [NumEntries-1:0] buf_data_q;
    addr_t                   [NumEntries-1:0] buf_addr_q;
    logic                    [NumEntries-1:0] buf_valid_q;
    logic                    [NumEntries-1:0] buf_dirty_q;
    logic [NumEntries-1:0][PartIdxWidth-1:0]  buf_part_idx_q;
    logic                    [NumEntries-1:0] buf_all_parts_q;

    // Pseudo-LRU victim pointer (for N=2 a single bit; for larger N,
    // tree-PLRU is a natural extension).
    logic [EntryIdxWidth-1:0] lru_q;

    // In-flight SRAM read tracking (one outstanding, downstream single-port).
    logic                          sram_rd_pend_q;
    addr_t                         sram_rd_addr_q;
    logic [PartIdxWidth-1:0]       sram_rd_part_idx_q;
    logic                          sram_rd_all_parts_q;
    logic [EntryIdxWidth-1:0]      sram_rd_target_q;

    // Registered read-hit (match SRAM 1-cycle latency).
    logic  buf_rd_hit_q;
    data_t buf_rd_data_q;

    // Stats
    logic [31:0] stat_rd_hit, stat_rd_miss;
    logic [31:0] stat_wr_merge, stat_wr_inval;
    logic [31:0] stat_rd_total, stat_wr_total;
    logic [31:0] stat_sram_rd, stat_wb;

    assign stat_rd_hit_o   = stat_rd_hit;
    assign stat_rd_miss_o  = stat_rd_miss;
    assign stat_wr_merge_o = stat_wr_merge;
    assign stat_wr_inval_o = stat_wr_inval;
    assign stat_rd_total_o = stat_rd_total;
    assign stat_wr_total_o = stat_wr_total;
    assign stat_sram_rd_o  = stat_sram_rd;
    assign stat_wb_o       = stat_wb;

    // ==================================================================
    //                      Combinational per-entry
    // ==================================================================
    logic [NumEntries-1:0] rd_part_match;
    logic [NumEntries-1:0] wr_parts_covered;
    logic [NumEntries-1:0] rd_hit_per_entry;
    logic [NumEntries-1:0] wr_buf_hit_entry;

    always_comb begin
        for (int e = 0; e < NumEntries; e++) begin
            // Read part match
            if (PartSplit <= 1) begin
                rd_part_match[e] = 1'b1;
            end else if (buf_all_parts_q[e]) begin
                rd_part_match[e] = 1'b1;
            end else if (rd_all_parts_i) begin
                rd_part_match[e] = 1'b0;
            end else begin
                rd_part_match[e] = (buf_part_idx_q[e] == rd_part_idx_i);
            end
            // Write parts covered (for this entry's buffer state)
            wr_parts_covered[e] = 1'b1;
            if (PartSplit > 1 && !buf_all_parts_q[e]) begin
                for (int p = 0; p < PartSplit; p++) begin
                    if (|wr_mask_i[p*PartMaskBits +: PartMaskBits] &&
                        (p[PartIdxWidth-1:0] != buf_part_idx_q[e]))
                        wr_parts_covered[e] = 1'b0;
                end
            end
        end
    end

    // Write parts covered against pending SRAM read (concurrent merge)
    logic wr_parts_covered_concurrent;
    always_comb begin
        wr_parts_covered_concurrent = 1'b1;
        if (PartSplit > 1 && !sram_rd_all_parts_q) begin
            for (int p = 0; p < PartSplit; p++) begin
                if (|wr_mask_i[p*PartMaskBits +: PartMaskBits] &&
                    (p[PartIdxWidth-1:0] != sram_rd_part_idx_q))
                    wr_parts_covered_concurrent = 1'b0;
            end
        end
    end

    logic has_wr_data;
    assign has_wr_data = |wr_mask_i;

    logic wr_full_line;
    assign wr_full_line = &wr_mask_i;

    // Per-entry hit
    always_comb begin
        for (int e = 0; e < NumEntries; e++) begin
            rd_hit_per_entry[e] = buf_valid_q[e]
                                & (buf_addr_q[e] == rd_addr_i)
                                & rd_part_match[e]
                                & !sram_rd_pend_q;
            wr_buf_hit_entry[e] = buf_valid_q[e]
                                & (buf_addr_q[e] == wr_addr_i)
                                & wr_parts_covered[e]
                                & has_wr_data
                                & !sram_rd_pend_q;
        end
    end

    // OR-reduce for any hit
    logic rd_hit_any;
    logic wr_buf_hit_any;
    always_comb begin
        rd_hit_any     = 1'b0;
        wr_buf_hit_any = 1'b0;
        for (int e = 0; e < NumEntries; e++) begin
            rd_hit_any     |= rd_hit_per_entry[e];
            wr_buf_hit_any |= wr_buf_hit_entry[e];
        end
    end

    // One-hot to binary (priority-first) for target indices.
    logic [EntryIdxWidth-1:0] rd_hit_target;
    logic [EntryIdxWidth-1:0] wr_buf_hit_target;
    always_comb begin
        rd_hit_target     = '0;
        wr_buf_hit_target = '0;
        for (int e = 0; e < NumEntries; e++) begin
            if (rd_hit_per_entry[e])  rd_hit_target     = EntryIdxWidth'(e);
            if (wr_buf_hit_entry[e])  wr_buf_hit_target = EntryIdxWidth'(e);
        end
    end

    // Read hit data mux
    data_t rd_hit_data;
    always_comb begin
        rd_hit_data = '0;
        for (int e = 0; e < NumEntries; e++) begin
            if (rd_hit_per_entry[e]) rd_hit_data = buf_data_q[e];
        end
    end

    // ==================================================================
    //                   Allocation target selection
    // ==================================================================
    // A suitable victim:
    //   1. Existing entry with matching address (reuse).
    //   2. Invalid entry.
    //   3. LRU entry if clean (or any clean entry).
    // When all entries are dirty with different addresses, no victim is
    // suitable -- the caller (access controller) must drain via writeback
    // first.  The flag `buf_has_free_clean_o` exposes this condition.

    // -- Populate target (used at sram_rd_issued_i time) --
    logic [EntryIdxWidth-1:0] populate_target;
    logic                     populate_found;
    always_comb begin
        populate_target = lru_q;
        populate_found  = 1'b0;
        // priority 1: existing addr match
        for (int e = 0; e < NumEntries; e++) begin
            if (buf_valid_q[e] && (buf_addr_q[e] == rd_addr_i) && !populate_found) begin
                populate_target = EntryIdxWidth'(e);
                populate_found  = 1'b1;
            end
        end
        if (!populate_found) begin
            // priority 2: invalid
            for (int e = 0; e < NumEntries; e++) begin
                if (!buf_valid_q[e] && !populate_found) begin
                    populate_target = EntryIdxWidth'(e);
                    populate_found  = 1'b1;
                end
            end
        end
        if (!populate_found) begin
            // priority 3: clean, prefer LRU
            if (!buf_dirty_q[lru_q]) begin
                populate_target = lru_q;
                populate_found  = 1'b1;
            end else begin
                for (int e = 0; e < NumEntries; e++) begin
                    if (!buf_dirty_q[e] && !populate_found) begin
                        populate_target = EntryIdxWidth'(e);
                        populate_found  = 1'b1;
                    end
                end
            end
        end
        // Fall-through: stays at lru_q (caller must have drained).
    end

    // -- Full-line write target --
    logic [EntryIdxWidth-1:0] wr_full_target;
    logic                     wr_full_victim_exists;
    always_comb begin
        wr_full_target        = lru_q;
        wr_full_victim_exists = 1'b0;
        // priority 1: same addr
        for (int e = 0; e < NumEntries; e++) begin
            if (buf_valid_q[e] && (buf_addr_q[e] == wr_addr_i) && !wr_full_victim_exists) begin
                wr_full_target        = EntryIdxWidth'(e);
                wr_full_victim_exists = 1'b1;
            end
        end
        if (!wr_full_victim_exists) begin
            // priority 2: invalid
            for (int e = 0; e < NumEntries; e++) begin
                if (!buf_valid_q[e] && !wr_full_victim_exists) begin
                    wr_full_target        = EntryIdxWidth'(e);
                    wr_full_victim_exists = 1'b1;
                end
            end
        end
        if (!wr_full_victim_exists) begin
            // priority 3: LRU clean, else any clean
            if (!buf_dirty_q[lru_q]) begin
                wr_full_target        = lru_q;
                wr_full_victim_exists = 1'b1;
            end else begin
                for (int e = 0; e < NumEntries; e++) begin
                    if (!buf_dirty_q[e] && !wr_full_victim_exists) begin
                        wr_full_target        = EntryIdxWidth'(e);
                        wr_full_victim_exists = 1'b1;
                    end
                end
            end
        end
    end

    // Concurrent hit (against pending SRAM read) -- lands in sram_rd_target_q
    logic wr_concurrent_hit;
    assign wr_concurrent_hit = sram_rd_pend_q & (sram_rd_addr_q == wr_addr_i)
                             & wr_parts_covered_concurrent & has_wr_data;

    logic wr_full_hit;
    assign wr_full_hit = wr_full_line & has_wr_data & !sram_rd_pend_q
                       & wr_full_victim_exists;

    assign rd_hit_comb_o = Enable & rd_hit_any;
    assign wr_hit_comb_o = Enable & (wr_buf_hit_any | wr_concurrent_hit | wr_full_hit);

    // wr_full_coverage_o: AFTER this absorption, targeted entry covers all parts.
    //   - wr_full_hit always sets buf_all_parts_q[target]=1 next cycle.
    //   - wr_buf_hit on an entry whose buf_all_parts_q is already 1 stays 1.
    //   - wr_concurrent_hit when the in-flight SRAM read is full-line populates all parts.
    logic wr_buf_hit_target_all_parts;
    assign wr_buf_hit_target_all_parts = wr_buf_hit_any & buf_all_parts_q[wr_buf_hit_target];
    assign wr_full_coverage_o = Enable
        & ( wr_full_hit
          | wr_buf_hit_target_all_parts
          | (wr_concurrent_hit & sram_rd_all_parts_q));

    // ==================================================================
    //                    Writeback selection
    // ==================================================================
    logic any_dirty;
    always_comb begin
        any_dirty = 1'b0;
        for (int e = 0; e < NumEntries; e++) any_dirty |= buf_dirty_q[e];
    end

    logic [EntryIdxWidth-1:0] wb_sel;
    always_comb begin
        wb_sel = lru_q;
        if (!buf_dirty_q[lru_q]) begin
            for (int e = 0; e < NumEntries; e++) begin
                if (buf_dirty_q[e]) wb_sel = EntryIdxWidth'(e);
            end
        end
    end

    assign wb_needed_o = Enable & any_dirty;
    assign wb_addr_o   = buf_addr_q[wb_sel];
    assign wb_data_o   = buf_data_q[wb_sel];

    always_comb begin
        wb_mask_o = '1;
        if (PartSplit > 1 && !buf_all_parts_q[wb_sel]) begin
            wb_mask_o = '0;
            for (int p = 0; p < PartSplit; p++) begin
                if (p[PartIdxWidth-1:0] == buf_part_idx_q[wb_sel])
                    wb_mask_o[p*PartMaskBits +: PartMaskBits] = '1;
            end
        end
    end

    // ==================================================================
    //          SpecWb gating signals for the access controller
    // ==================================================================
    // buf_has_free_clean: at least one entry is NOT valid-dirty.
    // buf_near_full    : popcount(dirty) >= NumEntries-1.
    logic [EntryIdxWidth:0] dirty_count;
    always_comb begin
        dirty_count = '0;
        for (int e = 0; e < NumEntries; e++) begin
            if (buf_valid_q[e] && buf_dirty_q[e])
                dirty_count = dirty_count + 1;
        end
    end
    assign buf_has_free_clean_o = Enable & (dirty_count < NumEntries);
    assign buf_near_full_o      = Enable & (dirty_count >= (NumEntries-1));

    // ==================================================================
    //                         Sequential
    // ==================================================================
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            for (int e = 0; e < NumEntries; e++) begin
                buf_valid_q     [e] <= 1'b0;
                buf_data_q      [e] <= '0;
                buf_addr_q      [e] <= '0;
                buf_dirty_q     [e] <= 1'b0;
                buf_part_idx_q  [e] <= '0;
                buf_all_parts_q [e] <= 1'b0;
            end
            lru_q                 <= '0;
            sram_rd_pend_q        <= 1'b0;
            sram_rd_addr_q        <= '0;
            sram_rd_part_idx_q    <= '0;
            sram_rd_all_parts_q   <= 1'b0;
            sram_rd_target_q      <= '0;
            buf_rd_hit_q          <= 1'b0;
            buf_rd_data_q         <= '0;
            stat_rd_hit           <= '0;
            stat_rd_miss          <= '0;
            stat_wr_merge         <= '0;
            stat_wr_inval         <= '0;
            stat_rd_total         <= '0;
            stat_wr_total         <= '0;
            stat_sram_rd          <= '0;
            stat_wb               <= '0;
        end else begin
            // Stats
            if (sram_rd_issued_i) stat_sram_rd <= stat_sram_rd + 1;
            if (wr_req_i && has_wr_data) stat_wr_total <= stat_wr_total + 1;
            if (wb_done_i) stat_wb <= stat_wb + 1;

            // Track pending SRAM read (one outstanding).
            sram_rd_pend_q <= sram_rd_issued_i;
            if (sram_rd_issued_i) begin
                sram_rd_addr_q      <= rd_addr_i;
                sram_rd_part_idx_q  <= rd_part_idx_i;
                sram_rd_all_parts_q <= rd_all_parts_i;
                sram_rd_target_q    <= populate_target;
            end

            // --- Populate path takes priority over write merge ---
            if (sram_rd_pend_q) begin
                // Always update tracked metadata on populate target.
                buf_valid_q    [sram_rd_target_q] <= 1'b1;
                buf_addr_q     [sram_rd_target_q] <= sram_rd_addr_q;
                buf_part_idx_q [sram_rd_target_q] <= sram_rd_part_idx_q;
                buf_all_parts_q[sram_rd_target_q] <= sram_rd_all_parts_q;

                if (wr_req_i && has_wr_data && (wr_addr_i == sram_rd_addr_q)
                    && wr_parts_covered_concurrent) begin
                    // Concurrent safe merge.
                    for (int b = 0; b < MaskBits; b++) begin
                        if (wr_mask_i[b])
                            buf_data_q[sram_rd_target_q][b*ByteWidth +: ByteWidth] <=
                                wr_data_i[b*ByteWidth +: ByteWidth];
                        else
                            buf_data_q[sram_rd_target_q][b*ByteWidth +: ByteWidth] <=
                                sram_rdata_i[b*ByteWidth +: ByteWidth];
                    end
                    if (Enable) buf_dirty_q[sram_rd_target_q] <= 1'b1;
                    stat_wr_merge <= stat_wr_merge + 1;
                end else if (wr_req_i && has_wr_data
                             && (wr_addr_i == sram_rd_addr_q)) begin
                    // Unsafe parts merge: populate then invalidate.
                    buf_data_q [sram_rd_target_q] <= sram_rdata_i;
                    buf_dirty_q[sram_rd_target_q] <= 1'b0;
                    buf_valid_q[sram_rd_target_q] <= 1'b0;
                    stat_wr_inval <= stat_wr_inval + 1;
                end else begin
                    buf_data_q [sram_rd_target_q] <= sram_rdata_i;
                    buf_dirty_q[sram_rd_target_q] <= 1'b0;
                end

                // Promote populated entry to MRU.  For N=1, lru_q stays 0.
                if (NumEntries > 1) begin
                    lru_q <= ~sram_rd_target_q[0];
                end
            end else begin
                // --- Full-line write absorb ---
                if (wr_req_i && wr_full_hit) begin
                    buf_data_q     [wr_full_target] <= wr_data_i;
                    buf_addr_q     [wr_full_target] <= wr_addr_i;
                    buf_valid_q    [wr_full_target] <= 1'b1;
                    buf_all_parts_q[wr_full_target] <= 1'b1;
                    buf_part_idx_q [wr_full_target] <= '0;
                    if (Enable) buf_dirty_q[wr_full_target] <= 1'b1;
                    stat_wr_merge <= stat_wr_merge + 1;
                    if (NumEntries > 1) begin
                        lru_q <= ~wr_full_target[0];
                    end
                end
                // --- Buffered write merge ---
                else if (wr_req_i && wr_buf_hit_any) begin
                    for (int b = 0; b < MaskBits; b++)
                        if (wr_mask_i[b])
                            buf_data_q[wr_buf_hit_target][b*ByteWidth +: ByteWidth] <=
                                wr_data_i[b*ByteWidth +: ByteWidth];
                    if (Enable) buf_dirty_q[wr_buf_hit_target] <= 1'b1;
                    stat_wr_merge <= stat_wr_merge + 1;
                    if (NumEntries > 1) begin
                        lru_q <= ~wr_buf_hit_target[0];
                    end
                end

                // --- Different-addr write, clean entry: invalidate ---
                // Mirrors the single-entry rule.  Sweeps across all entries.
                if (wr_req_i && !wr_full_hit && has_wr_data) begin
                    for (int e = 0; e < NumEntries; e++) begin
                        if (buf_valid_q[e] && (wr_addr_i != buf_addr_q[e])
                            && !buf_dirty_q[e]) begin
                            buf_valid_q[e] <= 1'b0;
                            stat_wr_inval <= stat_wr_inval + 1;
                        end
                    end
                end

                // --- Same-addr uncovered-parts clean write: invalidate ---
                if (wr_req_i && !wr_full_hit && has_wr_data) begin
                    for (int e = 0; e < NumEntries; e++) begin
                        if (buf_valid_q[e] && (wr_addr_i == buf_addr_q[e])
                            && !wr_parts_covered[e] && !buf_dirty_q[e]) begin
                            buf_valid_q[e] <= 1'b0;
                            stat_wr_inval <= stat_wr_inval + 1;
                        end
                    end
                end

                // --- SRAM write to a clean matching entry: invalidate ---
                if (sram_wr_req_i) begin
                    for (int e = 0; e < NumEntries; e++) begin
                        if (buf_valid_q[e] && (sram_wr_addr_i == buf_addr_q[e])
                            && !buf_dirty_q[e]) begin
                            buf_valid_q[e] <= 1'b0;
                        end
                    end
                end

                // --- Writeback done: clear dirty on wb_sel ---
                if (wb_done_i) begin
                    buf_dirty_q[wb_sel] <= 1'b0;
                end
            end

            // --- Registered read hit ---
            if (rd_valid_i & rd_ready_i) begin
                stat_rd_total <= stat_rd_total + 1;
                if (rd_hit_any) begin
                    buf_rd_hit_q  <= 1'b1;
                    buf_rd_data_q <= rd_hit_data;
                    stat_rd_hit   <= stat_rd_hit + 1;
                end else begin
                    buf_rd_hit_q <= 1'b0;
                    stat_rd_miss <= stat_rd_miss + 1;
                end
            end else begin
                buf_rd_hit_q <= 1'b0;
            end
        end
    end

    // Output mux
    assign fwd_rdata_o = (Enable && buf_rd_hit_q) ? buf_rd_data_q : sram_rdata_i;
    assign fwd_hit_o   = Enable & buf_rd_hit_q;

`ifndef TARGET_SYNTHESIS
    final begin
        if (stat_rd_total > 0 || stat_wr_total > 0) begin
            $display("[FWD_BUF_M %m] Enable=%0d NumEntries=%0d PartSplit=%0d | RD: total=%0d hit=%0d miss=%0d (hit_rate=%0.1f%%) | WR: total=%0d merge=%0d inval=%0d (absorb_rate=%0.1f%%) | sram_rd=%0d wb=%0d",
                Enable, NumEntries, PartSplit,
                stat_rd_total, stat_rd_hit, stat_rd_miss,
                (stat_rd_total > 0) ? 100.0 * real'(stat_rd_hit) / real'(stat_rd_total) : 0.0,
                stat_wr_total, stat_wr_merge, stat_wr_inval,
                (stat_wr_total > 0) ? 100.0 * real'(stat_wr_merge) / real'(stat_wr_total) : 0.0,
                stat_sram_rd, stat_wb);
        end
    end
`endif

endmodule : sram_forwarding_buffer_multi
