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
//   - Part-aware: with PartSplit > 1, the buffer tracks WHICH parts
//     of the cache line are cached (bitmap), and reports hits only
//     when ALL requested parts are present.  SRAM populates are
//     ADDITIVE: a same-line populate ORs the new parts into the
//     bitmap and updates only the byte lanes for the newly-arrived
//     parts, preserving previously-cached parts.
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
    /// When > 1, the buffer tracks per-part validity as a bitmap
    /// and supports incremental same-line accumulation.
    parameter int unsigned PartSplit      = 1,
    /// Read-after-write forwarding within the buffer.
    ///   0 (default): when a read AND write to the same line both hit the
    ///                buffer in the same cycle, the read sees the
    ///                **pre-write** value of buf_data_q (read-before-write
    ///                semantics, the standard for non-blocking flop
    ///                updates).
    ///   1: the read response is computed from the **post-write** merged
    ///      value -- the bytes the write touched are visible in the same
    ///      cycle's read response.  Adds a wr_data->buf_rd_data_q
    ///      combinational mux (byte-mask wide) before the response register;
    ///      mildly increases the cycle's critical path.
    parameter bit          EnableRawForwarding = 1'b0,
    /// (b3) Inflight-populate concurrent-write merge.
    ///   0 (default): when a read targets the in-flight SRAM read addr
    ///                (rd_inflight_hit candidate) AND a same-cycle write
    ///                targets that same addr, rd_inflight_hit is
    ///                suppressed; the access ctrl issues a fresh SRAM
    ///                read and pseudo_dual_port resolves the R+W via
    ///                WR_SAME_ADDR forwarding (or returns pre-write
    ///                state if the write is absorbed by the buffer).
    ///   1: rd_inflight_hit fires even with the concurrent same-addr
    ///                write.  The buffer captures sram_rdata_i with the
    ///                write's bytes overlaid on wr_mask -- post-write
    ///                semantics, byte-granular, matching the value
    ///                buf_data_q will hold at posedge T+1 (via the (D) /
    ///                REPLACE-with-merge populate path).  Saves the
    ///                redundant SRAM read in this scenario.
    parameter bit          EnableInflightWriteMerge = 1'b0,
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

    // -- Coverage hint for hazard bypass --
    // 1 iff this absorption will leave the buffer holding the WHOLE line.
    // Cache core uses this -- not wr_hit_comb_o -- to safely bypass the
    // bank-write/upstream-read same-line hazard, because partial-coverage
    // absorptions leave OTHER parts in SRAM, where a subsequent same-line
    // read for a different part would silently get stale data.
    output logic   wr_full_coverage_o,

    // -- Writeback interface --
    output logic   wb_needed_o,    // buffer dirty, may need writeback before miss
    output addr_t  wb_addr_o,      // writeback address
    output data_t  wb_data_o,      // writeback data
    output mask_t  wb_mask_o,      // writeback byte mask (only cached parts)
    input  logic   wb_done_i,      // writeback completed (clear dirty)

    // -- Phase 3 advisory: target line is currently in VALID state at the
    //    cache controller.  Used to gate the ACCUMULATE-CONCURRENT-MERGE
    //    branch so it doesn't fire when the cache controller's status array
    //    is mid-protocol (PEND).  Tied to 1 in modules that don't supply
    //    the signal -- behavior reverts to baseline ACCUMULATE-anywhere.
    input  logic   wr_target_valid_i,

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

    // -- Part tracking (BITMAP) --
    // Bit p == 1 means part p of the line is currently cached in buf_data_q.
    // PartSplit==1 collapses to a single bit (always meaning "the only part").
    logic [PartSplit-1:0]    buf_parts_valid_q;
    // Derived: convenience wire used by SVAs and external observers.
    logic                    buf_all_parts_q;
    assign buf_all_parts_q = &buf_parts_valid_q;

    // -- In-flight SRAM read tracking (BITMAP) --
    logic   sram_rd_pend_q;
    addr_t  sram_rd_addr_q;
    logic [PartSplit-1:0] sram_rd_parts_q;     // which parts the in-flight SRAM read fetches

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

    // -- Helpers: input encoding -> bitmap --
    // Convert the access-controller's (rd_part_idx_i, rd_all_parts_i)
    // and (wr_mask_i) into per-part bitmaps so the rest of the buffer
    // logic operates uniformly on PartSplit-bit vectors.
    //
    // wr_parts_bm is gated by wr_req_i so that helpers derived from it
    // (wr_parts_covered, has_wr_data, wr_full_line, all wr_*_hit signals)
    // cannot fire spuriously on a cycle where the upstream is not
    // requesting a write but wr_mask_i is left at a stale non-zero value
    // from the previous request.  Otherwise the (B) PEND_DISJOINT NBA
    // branch -- which gates only on wr_buf_hit_pend_disjoint and not
    // wr_req_i -- could absorb stale wr_data into buf_data_q and mark
    // the buffer dirty on a phantom write.
    logic [PartSplit-1:0] rd_parts_bm;
    logic [PartSplit-1:0] wr_parts_bm;
    always_comb begin
        if (PartSplit <= 1) begin
            rd_parts_bm = '1;
        end else if (rd_all_parts_i) begin
            rd_parts_bm = '1;
        end else begin
            rd_parts_bm = '0;
            rd_parts_bm[rd_part_idx_i] = 1'b1;
        end
        for (int p = 0; p < PartSplit; p++)
            wr_parts_bm[p] = wr_req_i & (|wr_mask_i[p*PartMaskBits +: PartMaskBits]);
    end

    // -- Read part match: every requested part is in the buffer --
    logic rd_part_match;
    assign rd_part_match = ((rd_parts_bm & buf_parts_valid_q) == rd_parts_bm);

    // -- Write parts coverage (normal: vs. buffer state) --
    logic wr_parts_covered;
    assign wr_parts_covered =
        ((wr_parts_bm & buf_parts_valid_q) == wr_parts_bm);

    // -- Write parts coverage (concurrent: vs. in-flight SRAM read parts) --
    // Used when sram_rd_pend_q=1 and the write is for the SRAM-read address.
    logic wr_parts_covered_concurrent;
    assign wr_parts_covered_concurrent =
        ((wr_parts_bm & sram_rd_parts_q) == wr_parts_bm);

    // -- Combinational hit checks --
    // Standard buffer hit: buf has the line, parts covered, and the
    // pending SRAM read (if any) is NOT for a different address that
    // would overwrite the buffer.
    logic rd_buf_hit;
    assign rd_buf_hit = buf_valid_q & (buf_addr_q == rd_addr_i)
                      & rd_part_match & !sram_rd_pend_q;

    // -- In-flight populate match (data on sram_rdata_i this cycle) --
    // When the read addr equals the in-flight SRAM read addr and the
    // populate parts cover the requested parts, the data the read
    // wants is ALREADY combinationally on sram_rdata_i (response of
    // the previous-cycle SRAM read).  We can serve the read directly
    // from sram_rdata_i instead of issuing a redundant new SRAM read.
    //
    // Gate against a concurrent same-addr write: in that case the
    // sram_rdata_i value reflects pre-write state, and we want
    // pseudo_dual_port's WR_SAME_ADDR forwarding (used by the
    // SRAM-miss path) to provide post-write data.  Falling back to
    // the SRAM-miss path keeps RAW semantics intact without adding
    // a separate RAW merge here.
    logic rd_inflight_parts_covered;
    assign rd_inflight_parts_covered =
        ((rd_parts_bm & sram_rd_parts_q) == rd_parts_bm);
    // (b3) In-flight populate + concurrent same-addr write detector.
    // Used both to gate the rd_inflight_hit fallback (off-mode) and to
    // drive the response merge (on-mode).
    logic inflight_concurrent_wr;
    assign inflight_concurrent_wr = wr_req_i & (|wr_mask_i)
                                  & (wr_addr_i == sram_rd_addr_q);
    logic rd_inflight_hit;
    assign rd_inflight_hit = sram_rd_pend_q
                           & (sram_rd_addr_q == rd_addr_i)
                           & rd_inflight_parts_covered
                           & (EnableInflightWriteMerge | !inflight_concurrent_wr);

    assign rd_hit_comb_o = Enable & (rd_buf_hit | rd_inflight_hit);

    // Write hit: three paths --
    //   Normal:     buffer has the address (not during SRAM populate).
    //   Concurrent: SRAM read arriving for the SAME address -- the
    //               sequential merge captures write+SRAM data together.
    //   Full-line:  write mask covers all bytes -- we don't need the
    //               pre-existing SRAM data; write directly into buffer.
    //               Safe when buffer is clean or already at same address.
    // has_wr_data / wr_full_line are gated by wr_req_i so they only
    // assert on cycles when upstream is actually requesting a write.
    // wr_mask_i may hold stale non-zero values from a previous request
    // when wr_req_i=0; treating those as a fresh write absorption would
    // corrupt buf_data_q via the wr_*_hit signals (notably the (B)
    // PEND_DISJOINT NBA path that doesn't redundantly check wr_req_i).
    logic has_wr_data;
    assign has_wr_data = wr_req_i & (|wr_mask_i);

    // Full-line write detection: all bytes being written.
    logic wr_full_line;
    assign wr_full_line = wr_req_i & (&wr_mask_i);

    // -- Write hit classification --
    // wr_buf_hit fires in two cases:
    //   IDLE: no SRAM read pending; original semantics.
    //   PEND_DISJOINT: a same-line populate is in flight AND the write
    //     touches buffer-cached parts that are DISJOINT from the parts
    //     the SRAM read is fetching this cycle.  The populate updates
    //     newly-arrived parts while the absorb updates already-buffered
    //     parts -- on disjoint byte lanes -- so they coexist without
    //     conflict.  Gated to clean buffer to avoid multi-part dirty.
    logic wr_buf_hit_idle;
    logic wr_buf_hit_pend_disjoint;
    logic wr_buf_hit;
    assign wr_buf_hit_idle =
           buf_valid_q & (buf_addr_q == wr_addr_i)
         & wr_parts_covered & has_wr_data
         & !sram_rd_pend_q;
    assign wr_buf_hit_pend_disjoint =
           buf_valid_q & (buf_addr_q == wr_addr_i)
         & (buf_addr_q == sram_rd_addr_q)
         & wr_parts_covered & has_wr_data
         & sram_rd_pend_q
         & ((wr_parts_bm & sram_rd_parts_q) == '0);
    // (Note: `!buf_dirty_q` gate dropped -- absorbing a write into already-
    // buffered parts that are disjoint from the in-flight SRAM read is
    // safe regardless of dirty state.  The tile-level write-priority
    // arbiter and byte-level wb_mask_o handle multi-part dirty writeback.)
    assign wr_buf_hit = wr_buf_hit_idle | wr_buf_hit_pend_disjoint;

    logic wr_concurrent_hit;
    assign wr_concurrent_hit = sram_rd_pend_q & (sram_rd_addr_q == wr_addr_i)
                             & wr_parts_covered_concurrent & has_wr_data;
    // Full-line write can absorb directly when buffer can be safely replaced:
    //   - invalid OR clean (no writeback needed), OR
    //   - valid+dirty but same address (full write overrides dirty data).
    logic wr_full_hit;
    assign wr_full_hit = wr_full_line & has_wr_data & !sram_rd_pend_q
                       & (!buf_valid_q | !buf_dirty_q
                          | (buf_addr_q == wr_addr_i));

    assign wr_hit_comb_o = Enable & (wr_buf_hit | wr_concurrent_hit | wr_full_hit);

    // wr_full_coverage_o: AFTER this absorption, will the buffer hold
    // the FULL line?  With the bitmap encoding this is a generalized
    // check -- a partial-coverage absorption that COMPLETES the line
    // qualifies, in addition to the original "all parts at once" cases.
    logic [PartSplit-1:0] post_absorb_parts_buf;
    logic [PartSplit-1:0] post_absorb_parts_pd;
    logic [PartSplit-1:0] post_absorb_parts_cc;
    assign post_absorb_parts_buf = buf_parts_valid_q | wr_parts_bm;
    assign post_absorb_parts_pd  = buf_parts_valid_q | sram_rd_parts_q | wr_parts_bm;
    assign post_absorb_parts_cc  = sram_rd_parts_q   | wr_parts_bm;
    assign wr_full_coverage_o = Enable
        & ( wr_full_hit
          | (wr_buf_hit_idle          & (&post_absorb_parts_buf))
          | (wr_buf_hit_pend_disjoint & (&post_absorb_parts_pd))
          | (wr_concurrent_hit        & (&post_absorb_parts_cc)));

    // -- Writeback outputs --
    assign wb_needed_o = Enable & buf_dirty_q;
    assign wb_addr_o   = buf_addr_q;

    // -- WB data + mask with concurrent-absorb merge --
    // When a write absorbs into the dirty buffer in the SAME cycle that
    // a spec_wb fires, the wb data must capture the absorb's bytes so
    // SRAM stays consistent after wb_done clears dirty next cycle.
    // When buf_dirty_q AND wr_hit_comb_o, wr_addr_i == buf_addr_q is
    // guaranteed (C3 + wr_full_hit's clean-buffer gate together imply
    // every wr_hit case fires only against the buffer's own line when
    // it is dirty).  This keeps the wb path simple: we always merge
    // absorb bytes when both signals are high, knowing the addresses
    // match.
    logic                  absorb_into_dirty_buf;
    data_t                 buf_data_for_wb;
    logic [PartSplit-1:0]  wb_parts_post_absorb;

    assign absorb_into_dirty_buf = Enable & wr_req_i & wr_hit_comb_o
                                 & buf_dirty_q & (|wr_mask_i);

    always_comb begin
        buf_data_for_wb      = buf_data_q;
        wb_parts_post_absorb = buf_parts_valid_q;
        if (absorb_into_dirty_buf) begin
            for (int b = 0; b < MaskBits; b++) begin
                if (wr_mask_i[b])
                    buf_data_for_wb[b*ByteWidth +: ByteWidth] =
                        wr_data_i[b*ByteWidth +: ByteWidth];
            end
            wb_parts_post_absorb = buf_parts_valid_q | wr_parts_bm;
        end
    end

    assign wb_data_o = buf_data_for_wb;

    // Writeback mask: cached parts' bytes (extended with the absorb's
    // parts when a concurrent absorb merges into wb_data_o, so wr_full_hit
    // and wr_buf_hit_pend_disjoint that add new parts are committed too).
    always_comb begin
        if (PartSplit <= 1) begin
            wb_mask_o = '1;
        end else begin
            wb_mask_o = '0;
            for (int p = 0; p < PartSplit; p++)
                if (wb_parts_post_absorb[p])
                    wb_mask_o[p*PartMaskBits +: PartMaskBits] = '1;
        end
    end

    // -- Internal merge gate --
    // Buffer only merges when the write is actually absorbed (address match,
    // all written parts are cached, and write mask is non-zero).
    logic can_merge;
    assign can_merge = buf_valid_q & (wr_addr_i == buf_addr_q)
                     & wr_parts_covered & has_wr_data;

    // -- Post-write buf_data_q (combinational) for RAW forwarding --
    // When EnableRawForwarding=1 AND the read+write hit the buffer in the
    // same cycle on the SAME line, the read response should reflect the
    // write's bytes.  buf_data_post_write is the value buf_data_q WILL hold
    // at the next posedge for the absorption paths (wr_buf_hit_idle,
    // wr_buf_hit_pend_disjoint, wr_full_hit -- i.e., wr_hit_comb_o cases).
    //
    // We feed this into the registered read response (buf_rd_data_q) so that
    // the read sees post-write data the same cycle the read returns it.
    //
    // Only forward when:
    //   * RAW forwarding is enabled,
    //   * the buffer is going to absorb the write this cycle (wr_hit_comb_o),
    //   * the read addr matches the write addr (same line) -- guaranteed in
    //     practice when both hit, but explicit for safety.
    //
    // For non-absorbing writes (effective_write -> SRAM), the buffer is not
    // updated by this write, so buf_data_post_write reduces to buf_data_q.
    data_t buf_data_post_write;
    always_comb begin
        buf_data_post_write = buf_data_q;
        if (EnableRawForwarding && wr_req_i && has_wr_data
            && wr_hit_comb_o && (wr_addr_i == rd_addr_i)) begin
            for (int b = 0; b < MaskBits; b++) begin
                if (wr_mask_i[b])
                    buf_data_post_write[b*ByteWidth +: ByteWidth] =
                        wr_data_i[b*ByteWidth +: ByteWidth];
            end
        end
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            buf_valid_q         <= 1'b0;
            buf_data_q          <= '0;
            buf_addr_q          <= '0;
            buf_dirty_q         <= 1'b0;
            buf_parts_valid_q   <= '0;
            sram_rd_pend_q      <= 1'b0;
            sram_rd_addr_q      <= '0;
            sram_rd_parts_q     <= '0;
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
                sram_rd_addr_q  <= rd_addr_i;
                // Snapshot the parts bitmap (rd_parts_bm is already the
                // combinational mapping of rd_part_idx_i / rd_all_parts_i).
                sram_rd_parts_q <= rd_parts_bm;
            end

            // -- SRAM populate has priority over write merge --
            // Three populate paths:
            //   (A) ACCUMULATE: same-addr populate, no concurrent write
            //       to the SRAM-read address -> OR in new parts; hold
            //       bytes for already-cached parts; PRESERVE dirty
            //       state.  Works for both clean and dirty buffers --
            //       the tile arbiter handles multi-part dirty writeback
            //       in a single cycle when eviction eventually fires.
            //   (B) ACCUMULATE-PEND-DISJOINT: same-addr populate AND a
            //       concurrent write hits buffer parts disjoint from
            //       the in-flight SRAM read.  Populate updates newly-
            //       arrived parts; absorb updates the buffer parts the
            //       write targets; both happen on disjoint byte lanes.
            //   (C) REPLACE: any other case.  Original semantics.
            if (sram_rd_pend_q) begin
                if (buf_valid_q && (buf_addr_q == sram_rd_addr_q)
                    && !(wr_req_i && has_wr_data
                         && (wr_addr_i == sram_rd_addr_q))) begin
                    // ===== (A) ACCUMULATE (no write to sram_rd_addr) =====
                    // Works for clean OR dirty buffer.  Dirty bytes
                    // (in already-cached parts) are HELD; only newly-
                    // arrived parts get written from SRAM.
                    buf_parts_valid_q <= buf_parts_valid_q | sram_rd_parts_q;
                    for (int b = 0; b < MaskBits; b++) begin
                        automatic int p = b / PartMaskBits;
                        if (sram_rd_parts_q[p])
                            buf_data_q[b*ByteWidth +: ByteWidth] <=
                                sram_rdata_i[b*ByteWidth +: ByteWidth];
                    end
                    // buf_valid_q, buf_addr_q held; buf_dirty_q PRESERVED
                    // (no clear -- dirty bytes are not overwritten).
                end else if (wr_buf_hit_pend_disjoint) begin
                    // ===== (B) ACCUMULATE-PEND-DISJOINT =====
                    buf_parts_valid_q <=
                        buf_parts_valid_q | sram_rd_parts_q;
                    for (int b = 0; b < MaskBits; b++) begin
                        automatic int p = b / PartMaskBits;
                        if (sram_rd_parts_q[p]) begin
                            // Newly-populated part: take SRAM data.
                            buf_data_q[b*ByteWidth +: ByteWidth] <=
                                sram_rdata_i[b*ByteWidth +: ByteWidth];
                        end else if (wr_parts_bm[p] && wr_mask_i[b]) begin
                            // Absorbed write byte (already-buffered part,
                            // disjoint from current SRAM read).
                            buf_data_q[b*ByteWidth +: ByteWidth] <=
                                wr_data_i[b*ByteWidth +: ByteWidth];
                        end
                        // else: hold (already-cached part, not written).
                    end
                    if (Enable) buf_dirty_q <= 1'b1;
                    stat_wr_merge <= stat_wr_merge + 1;
                end else if (buf_valid_q && (buf_addr_q == sram_rd_addr_q)
                             && wr_req_i && has_wr_data
                             && (wr_addr_i == sram_rd_addr_q)
                             && wr_parts_covered_concurrent
                             && wr_target_valid_i) begin
                    // ===== (D) ACCUMULATE-CONCURRENT-MERGE =====
                    // Same-line populate AND concurrent write covered AND
                    // the line is currently in VALID state at the cache
                    // controller.  Preserve old parts (OR in new); only
                    // update bytes for newly-arriving parts, hold bytes
                    // for already-cached parts (whose data is the live,
                    // possibly-dirty buffer state).  Wr_data merges into
                    // newly-arriving parts on wr_mask bytes.
                    //
                    // The wr_target_valid_i gate (Phase 3 handshake)
                    // ensures (D) doesn't fire when the cache controller's
                    // status array is mid-protocol (PEND).  In PEND, the
                    // cache controller expects a refill to land on this
                    // way; (D)'s buffer-side absorption can leave that
                    // refill un-issued or un-matched, leading to
                    // WR_CONFLICT_STALL hangs.
                    buf_parts_valid_q <= buf_parts_valid_q | sram_rd_parts_q;
                    for (int b = 0; b < MaskBits; b++) begin
                        automatic int p = b / PartMaskBits;
                        if (sram_rd_parts_q[p] && !buf_parts_valid_q[p]) begin
                            // NEW part: take wr_data on wr_mask, sram_rdata otherwise.
                            if (wr_mask_i[b])
                                buf_data_q[b*ByteWidth +: ByteWidth] <=
                                    wr_data_i[b*ByteWidth +: ByteWidth];
                            else
                                buf_data_q[b*ByteWidth +: ByteWidth] <=
                                    sram_rdata_i[b*ByteWidth +: ByteWidth];
                        end else if (wr_parts_bm[p] && wr_mask_i[b]) begin
                            // already-cached part with write hit: take wr_data
                            buf_data_q[b*ByteWidth +: ByteWidth] <=
                                wr_data_i[b*ByteWidth +: ByteWidth];
                        end
                        // else: hold (already-cached, no write).
                    end
                    if (Enable) buf_dirty_q <= 1'b1;
                    stat_wr_merge <= stat_wr_merge + 1;
                end else begin
                    // ===== REPLACE (original semantics) =====
                    buf_valid_q       <= 1'b1;
                    buf_addr_q        <= sram_rd_addr_q;
                    buf_parts_valid_q <= sram_rd_parts_q;
                    // Concurrent write merge: safe when all written parts
                    // are covered by the SRAM-read parts.
                    if (wr_req_i && has_wr_data
                        && (wr_addr_i == sram_rd_addr_q)
                        && wr_parts_covered_concurrent) begin
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
                        buf_data_q        <= sram_rdata_i;
                        buf_dirty_q       <= 1'b0;
                        buf_valid_q       <= 1'b0;
                        buf_parts_valid_q <= '0;
                        stat_wr_inval     <= stat_wr_inval + 1;
                    end else begin
                        buf_data_q  <= sram_rdata_i;
                        buf_dirty_q <= 1'b0;
                    end
                end
            end else begin
                // -- Full-line write: populate buffer directly, no SRAM --
                // Takes priority over merge (provides all bytes).
                if (wr_req_i && wr_full_hit) begin
                    buf_data_q        <= wr_data_i;
                    buf_addr_q        <= wr_addr_i;
                    buf_valid_q       <= 1'b1;
                    buf_parts_valid_q <= '1;
                    if (Enable) buf_dirty_q <= 1'b1;
                    stat_wr_merge <= stat_wr_merge + 1;
                end
                // -- Write merge: same address, parts covered --
                else if (wr_req_i && can_merge) begin
                    // Subset OR: no change to buf_parts_valid_q because
                    // wr_parts_bm is already a subset.  Kept explicit
                    // here for clarity.
                    buf_parts_valid_q <= buf_parts_valid_q | wr_parts_bm;
                    for (int b = 0; b < MaskBits; b++)
                        if (wr_mask_i[b])
                            buf_data_q[b*ByteWidth +: ByteWidth] <=
                                wr_data_i[b*ByteWidth +: ByteWidth];
                    if (Enable) buf_dirty_q <= 1'b1;
                    stat_wr_merge <= stat_wr_merge + 1;
                end

                // -- Write to different address, buffer clean: KEEP --
                // The buffer holds (clean) data for a DIFFERENT line than
                // the write.  The buffer's data is still consistent with
                // SRAM (clean), and the new write goes to SRAM at its own
                // address without touching this buffer.  Holding the
                // entry as a "victim" lets subsequent reads/writes to the
                // OLD line still hit the buffer (re-dirty if write).
                // Eviction happens only when an actual REPLACE is forced
                // (different-line populate).

                // -- Write to same address, parts NOT covered, clean:
                //    invalidate (SRAM will have newer data) --
                // (Skipped for wr_full_hit since full-line write covers all.)
                if (wr_req_i && !wr_full_hit && buf_valid_q
                    && (wr_addr_i == buf_addr_q)
                    && !wr_parts_covered && !buf_dirty_q) begin
                    buf_valid_q       <= 1'b0;
                    buf_parts_valid_q <= '0;
                    stat_wr_inval     <= stat_wr_inval + 1;
                end

                // -- SRAM write to buffer address: invalidate --
                // Catches stall-resent writes that the buffer doesn't see
                // as upstream write pulses.
                if (sram_wr_req_i && buf_valid_q
                    && (sram_wr_addr_i == buf_addr_q) && !buf_dirty_q) begin
                    buf_valid_q       <= 1'b0;
                    buf_parts_valid_q <= '0;
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
                    // RAW forwarding: when EnableRawForwarding=1 and a
                    // concurrent write hits the buffer for the same line,
                    // buf_data_post_write reflects the merged value the
                    // buffer WILL hold next cycle.  Otherwise it equals
                    // buf_data_q (pre-write, the standard semantics).
                    buf_rd_data_q <= buf_data_post_write;
                    stat_rd_hit <= stat_rd_hit + 1;
                end else if (rd_inflight_hit) begin
                    // In-flight populate match: the data the read wants
                    // is already on sram_rdata_i this cycle (response
                    // of the previously-issued SRAM read).  Register it
                    // so next-cycle fwd_rdata_o returns it via the
                    // standard buf_rd_data_q path.  No new SRAM read is
                    // issued this cycle (rd_hit_comb_o=1 → access ctrl
                    // suppresses downstream_read_valid_o).
                    //
                    // (b3) EnableInflightWriteMerge: when a same-addr write
                    // is concurrent (inflight_concurrent_wr=1), the buffer
                    // will absorb it via the (D)/REPLACE-with-merge populate
                    // path at posedge T+1.  Overlay wr_data on wr_mask bytes
                    // into the response register so the read sees post-write
                    // semantics matching the buffer's next-cycle state.
                    buf_rd_hit_q  <= 1'b1;
                    for (int b = 0; b < MaskBits; b++) begin
                        if (EnableInflightWriteMerge && inflight_concurrent_wr
                            && wr_mask_i[b])
                            buf_rd_data_q[b*ByteWidth +: ByteWidth] <=
                                wr_data_i[b*ByteWidth +: ByteWidth];
                        else
                            buf_rd_data_q[b*ByteWidth +: ByteWidth] <=
                                sram_rdata_i[b*ByteWidth +: ByteWidth];
                    end
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

`ifndef TARGET_SYNTHESIS
    // ---------------------------------------------------------------
    // Contract assertions -- enforce the buffer<->cache-core contract.
    // (Pulled inline because `bind sram_forwarding_buffer ...` was not
    // taking effect in the integrated cachepool build, even though the
    // bind file compiled cleanly.  Inlining guarantees elaboration.)
    //
    // Each property fires `$error` at the cycle of divergence so a
    // contract violation is pinpointed instead of surfacing 80us later
    // at the cluster xbar as `Visited illegal address`.
    // ---------------------------------------------------------------

    // C1 / C4: full-coverage absorption propagates correctly.
    //   At cycle T: wr_full_coverage_o=1 AND wr_req_i=1.
    //   At cycle T+1: buf_valid_q & (buf_addr_q == wr_addr_T) & buf_all_parts_q.
    // (buf_all_parts_q is the &-reduction of buf_parts_valid_q, so this
    // continues to assert "buffer holds the WHOLE line".)
    property p_C1_full_coverage_propagates;
        addr_t saved_addr;
        @(posedge clk_i) disable iff (!rst_ni)
        (wr_full_coverage_o && wr_req_i, saved_addr = wr_addr_i)
        |=> (buf_valid_q && (buf_addr_q == saved_addr) && buf_all_parts_q);
    endproperty
    a_C1_full_coverage_propagates: assert property (p_C1_full_coverage_propagates)
        else $error("[fwd_buf C1 %m] wr_full_coverage_o asserted but next-cycle buffer is not full-line at written addr");

    // C3: never overwrite dirty data on a SRAM-read populate when the
    // pending read is for a DIFFERENT address than the dirty line.
    property p_C3_no_clobber_dirty;
        @(posedge clk_i) disable iff (!rst_ni)
        Enable -> !(sram_rd_pend_q && buf_valid_q && buf_dirty_q
                    && (sram_rd_addr_q != buf_addr_q));
    endproperty
    a_C3_no_clobber_dirty: assert property (p_C3_no_clobber_dirty)
        else $error("[fwd_buf C3 %m] SRAM-read populate would clobber dirty buffer at different addr: dirty=0x%0h pending=0x%0h",
                    buf_addr_q, sram_rd_addr_q);

    // C5: wb_done while wb_needed clears dirty next cycle (unless a
    // same-cycle write merge re-dirties, which is the only legal way
    // for buf_dirty_q to stay 1).
    property p_C5_wb_done_clears_dirty;
        @(posedge clk_i) disable iff (!rst_ni)
        (wb_needed_o && wb_done_i && !(wr_req_i && wr_hit_comb_o))
        |=> !buf_dirty_q;
    endproperty
    a_C5_wb_done_clears_dirty: assert property (p_C5_wb_done_clears_dirty)
        else $error("[fwd_buf C5 %m] wb_done_i asserted but buf_dirty_q didn't clear next cycle");

    // Sanity: wr_full_coverage_o is a strict subset of wr_hit_comb_o.
    property p_full_cov_implies_hit;
        @(posedge clk_i) disable iff (!rst_ni)
        wr_full_coverage_o |-> wr_hit_comb_o;
    endproperty
    a_full_cov_implies_hit: assert property (p_full_cov_implies_hit)
        else $error("[fwd_buf SANITY %m] wr_full_coverage_o=1 but wr_hit_comb_o=0");

    // Sanity: wb_needed_o equals (buf_valid_q & buf_dirty_q) when Enable.
    property p_wb_needed_definition;
        @(posedge clk_i) disable iff (!rst_ni)
        (wb_needed_o == (Enable & buf_valid_q & buf_dirty_q));
    endproperty
    a_wb_needed_definition: assert property (p_wb_needed_definition)
        else $error("[fwd_buf SANITY %m] wb_needed_o disagrees with buf_valid_q & buf_dirty_q");

    // BITMAP-NEW: when buffer is invalid, parts bitmap must be zero.
    property p_invalid_implies_no_parts;
        @(posedge clk_i) disable iff (!rst_ni)
        (!buf_valid_q) |-> (buf_parts_valid_q == '0);
    endproperty
    a_invalid_implies_no_parts: assert property (p_invalid_implies_no_parts)
        else $error("[fwd_buf SANITY %m] buf_valid_q=0 but buf_parts_valid_q=0x%0h", buf_parts_valid_q);

    // BITMAP-NEW: when buffer is valid, parts bitmap must be non-zero.
    property p_valid_implies_some_parts;
        @(posedge clk_i) disable iff (!rst_ni)
        (buf_valid_q) |-> (buf_parts_valid_q != '0);
    endproperty
    a_valid_implies_some_parts: assert property (p_valid_implies_some_parts)
        else $error("[fwd_buf SANITY %m] buf_valid_q=1 but buf_parts_valid_q=0");
`endif // !TARGET_SYNTHESIS

endmodule : sram_forwarding_buffer
