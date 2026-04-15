// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// 1-entry write-back forwarding buffer for single-port SRAMs.
//
// Sits between an access controller and SRAM banks.
// Caches one SRAM row in registers.
//
// When Enable==1 (active mode):
//   - Matching reads return buffer data; SRAM read is suppressed.
//   - Matching writes merge into the buffer; SRAM write is suppressed.
//   - Dirty buffer data is written back on address change (eviction).
//   - Combinational hit outputs allow the access controller to gate
//     downstream SRAM operations in the same cycle.
//   - Part-aware: with PartSplit > 1, the buffer tracks which part
//     of the cache line was cached and only reports hits for that part.
//
// When Enable==0 (passthrough mode):
//   - All combinational outputs are tied low (no hits, no writeback).
//   - The output always returns SRAM data.
//   - Buffer registers still update (not removed by optimizer).

`include "common_cells/registers.svh"
module sram_forwarding_buffer #(
    /// SRAM depth (number of rows)
    parameter int unsigned Depth          = 512,
    /// Number of data words per row
    parameter int unsigned NumWordsPerLine = 2,
    /// Width of each data word in bits
    parameter int unsigned WordWidth      = 32,
    /// Width of each byte-enable granule in bits
    parameter int unsigned ByteWidth      = 8,
    /// Enable active forwarding (1=active write-back, 0=passthrough)
    parameter bit          Enable         = 1'b1,
    /// Number of parts per cache line (1 = no part gating).
    /// When > 1, the buffer tracks which part is cached and only
    /// reports hits for that part.
    parameter int unsigned PartSplit      = 1,
    // -- Derived parameters (do not override) --
    localparam int unsigned DataWidth     = WordWidth * NumWordsPerLine,
    localparam int unsigned MaskBits      = DataWidth / ByteWidth,
    localparam int unsigned PartIdxWidth  = (PartSplit > 1) ? $clog2(PartSplit) : 1,
    localparam int unsigned PartMaskBits  = MaskBits / ((PartSplit > 1) ? PartSplit : 1),
    localparam type         data_t        = logic [DataWidth-1:0],
    localparam type         mask_t        = logic [MaskBits-1:0],
    localparam type         addr_t        = logic [$clog2(Depth)-1:0]
)(
    input  logic   clk_i,
    input  logic   rst_ni,

    // -- Upstream read channel --
    input  addr_t  rd_addr_i,
    input  logic   rd_valid_i,     // upstream read valid
    input  logic   rd_ready_i,     // upstream read accepted (hit or SRAM ready)
    input  logic [PartIdxWidth-1:0] rd_part_idx_i,   // which part is being read
    input  logic                    rd_all_parts_i,   // 1 = full-line read

    // -- Upstream write channel --
    input  addr_t  wr_addr_i,
    input  data_t  wr_data_i,
    input  mask_t  wr_mask_i,
    input  logic   wr_req_i,       // upstream write request (1-cycle pulse)

    // -- SRAM tracking --
    input  logic   sram_rd_issued_i,  // downstream read valid & ready (actual SRAM read)
    input  data_t  sram_rdata_i,      // SRAM read data (1 cycle after sram_rd_issued)
    input  logic   sram_wr_req_i,     // downstream write accepted by SRAM
    input  addr_t  sram_wr_addr_i,    // downstream write address

    // -- Combinational hit outputs (active mode only) --
    output logic   rd_hit_comb_o,  // buffer can serve this read (suppress SRAM read)
    output logic   wr_hit_comb_o,  // buffer can absorb this write (suppress SRAM write)

    // -- Writeback interface --
    output logic   wb_needed_o,    // buffer dirty, may need writeback before miss
    output addr_t  wb_addr_o,      // writeback address
    output data_t  wb_data_o,      // writeback data
    output mask_t  wb_mask_o,      // writeback byte mask (only cached parts)
    input  logic   wb_done_i,      // writeback completed (clear dirty)

    // -- Forwarded read data output --
    output data_t  fwd_rdata_o,    // replaces upstream read data on hit
    output logic   fwd_hit_o,      // 1 = data came from buffer (debug)

    // -- Statistics outputs (always present for observability) --
    output logic [31:0] stat_rd_hit_o,
    output logic [31:0] stat_rd_miss_o,
    output logic [31:0] stat_wr_merge_o,
    output logic [31:0] stat_wr_inval_o,
    output logic [31:0] stat_rd_total_o,   // total read acceptances
    output logic [31:0] stat_wr_total_o,   // total write requests with data
    output logic [31:0] stat_sram_rd_o,    // SRAM reads issued
    output logic [31:0] stat_wb_o          // writeback completions
);

    // -- Buffer state --
    data_t  buf_data_q;
    addr_t  buf_addr_q;
    logic   buf_valid_q;
    logic   buf_dirty_q;

    // -- Part tracking --
    logic [PartIdxWidth-1:0] buf_part_idx_q;   // which part is cached
    logic                    buf_all_parts_q;   // all parts cached

    // -- In-flight SRAM read tracking --
    logic   sram_rd_pend_q;
    addr_t  sram_rd_addr_q;
    logic [PartIdxWidth-1:0] sram_rd_part_idx_q;
    logic                    sram_rd_all_parts_q;

    // -- Registered hit output (1-cycle latency to match SRAM) --
    logic   buf_rd_hit_q;
    data_t  buf_rd_data_q;

    // -- Statistics counters --
    logic [31:0] stat_rd_hit;
    logic [31:0] stat_rd_miss;
    logic [31:0] stat_wr_merge;
    logic [31:0] stat_wr_inval;
    logic [31:0] stat_rd_total;
    logic [31:0] stat_wr_total;
    logic [31:0] stat_sram_rd;
    logic [31:0] stat_wb;

    assign stat_rd_hit_o   = stat_rd_hit;
    assign stat_rd_miss_o  = stat_rd_miss;
    assign stat_wr_merge_o = stat_wr_merge;
    assign stat_wr_inval_o = stat_wr_inval;
    assign stat_rd_total_o = stat_rd_total;
    assign stat_wr_total_o = stat_wr_total;
    assign stat_sram_rd_o  = stat_sram_rd;
    assign stat_wb_o       = stat_wb;

    // -- Read part match --
    // Hit only when the requested part is actually cached.
    logic rd_part_match;
    always_comb begin
        if (PartSplit <= 1)
            rd_part_match = 1'b1;
        else if (buf_all_parts_q)
            rd_part_match = 1'b1;
        else if (rd_all_parts_i)
            rd_part_match = 1'b0;  // full read but buffer has 1 part
        else
            rd_part_match = (buf_part_idx_q == rd_part_idx_i);
    end

    // -- Write parts coverage (normal: buffer state) --
    logic wr_parts_covered;
    always_comb begin
        wr_parts_covered = 1'b1;
        if (PartSplit > 1 && !buf_all_parts_q) begin
            for (int p = 0; p < PartSplit; p++) begin
                if (|wr_mask_i[p*PartMaskBits +: PartMaskBits] &&
                    (p[PartIdxWidth-1:0] != buf_part_idx_q))
                    wr_parts_covered = 1'b0;
            end
        end
    end

    // -- Write parts coverage (concurrent: SRAM read arriving) --
    // Used when sram_rd_pend_q=1 and the SRAM read is for the write address.
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

    // -- Combinational hit checks --
    // Read hit: suppress while SRAM read pending and the pending read
    // is for a different address (buffer about to be overwritten).
    assign rd_hit_comb_o = Enable & buf_valid_q & (buf_addr_q == rd_addr_i)
                         & rd_part_match & !sram_rd_pend_q;

    // Write hit: buffer has the address, write has actual data to merge,
    // and not during SRAM populate.
    logic has_wr_data;
    assign has_wr_data = |wr_mask_i;
    assign wr_hit_comb_o = Enable & buf_valid_q & (buf_addr_q == wr_addr_i)
                         & wr_parts_covered & has_wr_data & !sram_rd_pend_q;

    // -- Writeback outputs --
    assign wb_needed_o = Enable & buf_dirty_q;
    assign wb_addr_o   = buf_addr_q;
    assign wb_data_o   = buf_data_q;

    // Writeback mask: only write back the cached part's bytes.
    always_comb begin
        wb_mask_o = '1;
        if (PartSplit > 1 && !buf_all_parts_q) begin
            wb_mask_o = '0;
            for (int p = 0; p < PartSplit; p++) begin
                if (p[PartIdxWidth-1:0] == buf_part_idx_q)
                    wb_mask_o[p*PartMaskBits +: PartMaskBits] = '1;
            end
        end
    end

    // -- Internal merge gate --
    // Buffer only merges when the write is actually absorbed (address match,
    // all written parts are cached, and write mask is non-zero).
    logic can_merge;
    assign can_merge = buf_valid_q & (wr_addr_i == buf_addr_q) & wr_parts_covered & has_wr_data;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            buf_valid_q         <= 1'b0;
            buf_data_q          <= '0;
            buf_addr_q          <= '0;
            buf_dirty_q         <= 1'b0;
            buf_part_idx_q      <= '0;
            buf_all_parts_q     <= 1'b0;
            sram_rd_pend_q      <= 1'b0;
            sram_rd_addr_q      <= '0;
            sram_rd_part_idx_q  <= '0;
            sram_rd_all_parts_q <= 1'b0;
            buf_rd_hit_q        <= 1'b0;
            buf_rd_data_q       <= '0;
            stat_rd_hit         <= '0;
            stat_rd_miss        <= '0;
            stat_wr_merge       <= '0;
            stat_wr_inval       <= '0;
            stat_rd_total       <= '0;
            stat_wr_total       <= '0;
            stat_sram_rd        <= '0;
            stat_wb             <= '0;
        end else begin
            // -- Aggregate counters --
            if (sram_rd_issued_i)
                stat_sram_rd <= stat_sram_rd + 1;
            if (wr_req_i && has_wr_data)
                stat_wr_total <= stat_wr_total + 1;
            if (wb_done_i)
                stat_wb <= stat_wb + 1;

            // -- Track SRAM reads in flight --
            sram_rd_pend_q <= sram_rd_issued_i;
            if (sram_rd_issued_i) begin
                sram_rd_addr_q      <= rd_addr_i;
                sram_rd_part_idx_q  <= rd_part_idx_i;
                sram_rd_all_parts_q <= rd_all_parts_i;
            end

            // -- SRAM populate has priority over write merge --
            if (sram_rd_pend_q) begin
                buf_valid_q     <= 1'b1;
                buf_addr_q      <= sram_rd_addr_q;
                buf_part_idx_q  <= sram_rd_part_idx_q;
                buf_all_parts_q <= sram_rd_all_parts_q;
                // Concurrent write merge: safe when the SRAM read covers
                // all parts or PartSplit==1 (no zero-fill issue).
                if (wr_req_i && (wr_addr_i == sram_rd_addr_q)
                    && (PartSplit <= 1 || sram_rd_all_parts_q)) begin
                    for (int b = 0; b < MaskBits; b++) begin
                        if (wr_mask_i[b])
                            buf_data_q[b*ByteWidth +: ByteWidth] <=
                                wr_data_i[b*ByteWidth +: ByteWidth];
                        else
                            buf_data_q[b*ByteWidth +: ByteWidth] <=
                                sram_rdata_i[b*ByteWidth +: ByteWidth];
                    end
                    if (Enable) buf_dirty_q <= 1'b1;
                    stat_wr_merge <= stat_wr_merge + 1;
                end else if (wr_req_i && has_wr_data
                             && (wr_addr_i == sram_rd_addr_q)) begin
                    // Write to same address but merge unsafe (partial
                    // parts).  The write goes to SRAM, making the buffer
                    // stale.  Populate then invalidate.
                    buf_data_q  <= sram_rdata_i;
                    buf_dirty_q <= 1'b0;
                    buf_valid_q <= 1'b0;
                    stat_wr_inval <= stat_wr_inval + 1;
                end else begin
                    buf_data_q  <= sram_rdata_i;
                    buf_dirty_q <= 1'b0;
                end
            end else begin
                // -- Write merge: same address, parts covered --
                if (wr_req_i && can_merge) begin
                    for (int b = 0; b < MaskBits; b++)
                        if (wr_mask_i[b])
                            buf_data_q[b*ByteWidth +: ByteWidth] <=
                                wr_data_i[b*ByteWidth +: ByteWidth];
                    if (Enable) buf_dirty_q <= 1'b1;
                    stat_wr_merge <= stat_wr_merge + 1;
                end

                // -- Write to different address, buffer clean: invalidate --
                if (wr_req_i && buf_valid_q && (wr_addr_i != buf_addr_q)
                    && !buf_dirty_q) begin
                    buf_valid_q <= 1'b0;
                    stat_wr_inval <= stat_wr_inval + 1;
                end

                // -- Write to same address, parts NOT covered, clean:
                //    invalidate (SRAM will have newer data) --
                if (wr_req_i && buf_valid_q && (wr_addr_i == buf_addr_q)
                    && !wr_parts_covered && !buf_dirty_q) begin
                    buf_valid_q <= 1'b0;
                    stat_wr_inval <= stat_wr_inval + 1;
                end

                // -- SRAM write to buffer address: invalidate --
                // Catches stall-resent writes that the buffer doesn't see
                // as upstream write pulses.
                if (sram_wr_req_i && buf_valid_q
                    && (sram_wr_addr_i == buf_addr_q) && !buf_dirty_q) begin
                    buf_valid_q <= 1'b0;
                end

                // -- Writeback done: clear dirty --
                if (wb_done_i)
                    buf_dirty_q <= 1'b0;
            end

            // -- Read hit detection (registered for 1-cycle latency) --
            if (rd_valid_i & rd_ready_i) begin
                stat_rd_total <= stat_rd_total + 1;
                if (buf_valid_q && (buf_addr_q == rd_addr_i)
                    && rd_part_match && !sram_rd_pend_q) begin
                    buf_rd_hit_q  <= 1'b1;
                    buf_rd_data_q <= buf_data_q;
                    stat_rd_hit <= stat_rd_hit + 1;
                end else begin
                    buf_rd_hit_q <= 1'b0;
                    stat_rd_miss <= stat_rd_miss + 1;
                end
            end else begin
                buf_rd_hit_q <= 1'b0;
            end
        end
    end

    // -- Output mux --
    assign fwd_rdata_o = (Enable && buf_rd_hit_q) ? buf_rd_data_q : sram_rdata_i;
    assign fwd_hit_o   = Enable & buf_rd_hit_q;

`ifndef TARGET_SYNTHESIS
    final begin
        if (stat_rd_total > 0 || stat_wr_total > 0) begin
            $display("[FWD_BUF %m] Enable=%0d PartSplit=%0d | RD: total=%0d hit=%0d miss=%0d (hit_rate=%0.1f%%) | WR: total=%0d merge=%0d inval=%0d (absorb_rate=%0.1f%%) | sram_rd=%0d wb=%0d",
                Enable, PartSplit,
                stat_rd_total, stat_rd_hit, stat_rd_miss,
                (stat_rd_total > 0) ? 100.0 * real'(stat_rd_hit) / real'(stat_rd_total) : 0.0,
                stat_wr_total, stat_wr_merge, stat_wr_inval,
                (stat_wr_total > 0) ? 100.0 * real'(stat_wr_merge) / real'(stat_wr_total) : 0.0,
                stat_sram_rd, stat_wb);
        end
    end
`endif

endmodule : sram_forwarding_buffer
