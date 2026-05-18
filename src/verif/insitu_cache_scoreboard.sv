// Copyright 2026 ETH Zurich and University of Bologna.
// SPDX-License-Identifier: SHL-0.51
//
// insitu_cache_scoreboard — verification IP for the L1 cache controller.
//
// This module is a PASSIVE OBSERVER bound into insitu_cache_tcdm_wrapper.
// It mirrors the cache controller's per-line state (valid / dirty / tag /
// data) by snooping commits to the cache (refills, write-hits, flush/init
// writes).  On every cycle the decoder produces a hit/miss decision, the
// scoreboard cross-checks:
//
//   1. dec_is_hit matches the scoreboard's prediction (no phantom hits).
//   2. When dec_is_hit=1, dec_cache_data equals the data the scoreboard
//      has tracked for that (depth, way).
//   3. The way the cache picked (dec_way) is a way the scoreboard
//      considers VALID + tag-matching.
//
// All checks are guarded by `ifndef TARGET_SYNTHESIS so synthesis is
// unaffected.  No outputs drive any RTL signal -- the scoreboard cannot
// alter cache behaviour.
//
// Limitations of the simple model:
//   - Does not model MSHR / READ_PEND / WRITE_PEND interleavings.  When a
//     line is mid-refill, the scoreboard treats it as INVALID (no entry)
//     until the refill commits.  The cache may legitimately return
//     dec_is_hit_pend in this window -- we ignore those cycles.
//   - Does not model the data-side forwarding buffer's own state.  The
//     scoreboard tracks what GETS WRITTEN to the SRAM, which is the
//     ground-truth that subsequent reads must observe.

`ifndef TARGET_SYNTHESIS
module insitu_cache_scoreboard
  import insitu_cache_pkg::*;
#(
    parameter int unsigned CacheBankDepth   = 256,
    parameter int unsigned SetAssociativity = 4,
    parameter int unsigned CacheLineWidth   = 512,
    parameter int unsigned MaskWidth        = CacheLineWidth / 8,
    parameter int unsigned ReqAddrWidth     = 32,
    parameter int unsigned CacheTagWidth    = 18,
    // Upstream request widths (for the req-snoop port; sized by the wrapper).
    parameter int unsigned UpstreamDataWidth = 32,
    parameter int unsigned UpstreamMaskWidth = 64,
    parameter int unsigned InfoWidth         = 1,
    parameter string       CtrlName         = "ctrl?",
    // Soft-disable individual checks if you want only one kind of report.
    parameter bit          CheckHitMiss     = 1'b1,
    parameter bit          CheckHitData     = 1'b1,
    parameter bit          CheckHitWay      = 1'b1
) (
    input  logic clk_i,
    input  logic rst_ni,

    // -- Flush / init commits (one entry per cycle when valid && ready) --
    // Writes ALL ways' meta at the given depth in one shot.
    input  logic                                                          flush_commit_valid,
    input  logic [$clog2(CacheBankDepth)-1:0]                             flush_commit_addr,
    input  cache_status_t [SetAssociativity-1:0]                          flush_commit_status,
    input  logic [SetAssociativity-1:0]                                   flush_commit_dirty,
    input  logic [SetAssociativity-1:0][CacheTagWidth-1:0]                flush_commit_tag,

    // -- Proc-side commits (refill, write-hit, LRU update) --
    // One way per pulse; updates that way's meta + data (data updated
    // per byte mask).
    input  logic                                                          proc_commit_valid,
    input  logic [$clog2(CacheBankDepth)-1:0]                             proc_commit_addr,
    input  logic [$clog2(SetAssociativity)-1:0]                           proc_commit_way,
    input  cache_status_t                                                 proc_commit_status,
    input  logic                                                          proc_commit_dirty,
    input  logic [CacheTagWidth-1:0]                                      proc_commit_tag,
    input  logic [CacheLineWidth-1:0]                                     proc_commit_data,
    input  logic [MaskWidth-1:0]                                          proc_commit_mask,

    // -- Read decision (decoder output) for cross-check --
    // dec_valid is asserted when the decoder is producing a hit/miss
    // decision for a load request (not a refill, not a write).
    input  logic                                                          dec_valid,
    input  logic [ReqAddrWidth-1:0]                                       dec_addr,
    input  logic                                                          dec_is_hit,
    input  logic [$clog2(SetAssociativity)-1:0]                           dec_way,
    input  logic [CacheLineWidth-1:0]                                     dec_data,

    // -- Upstream request snoop (post-hash, going into the cache_core) --
    // Logged when accepted (valid && ready) AND its addr's depth matches
    // the trace window.  Lets the user correlate later commits to the
    // originating request.
    input  logic                                                          upreq_valid,
    input  logic                                                          upreq_ready,
    input  logic [ReqAddrWidth-1:0]                                       upreq_addr,
    input  logic                                                          upreq_write,
    input  logic [UpstreamDataWidth-1:0]                                  upreq_wdata,
    input  logic [UpstreamMaskWidth-1:0]                                  upreq_wmask,
    input  logic [InfoWidth-1:0]                                          upreq_info,

    // -- Upstream RESPONSE snoop (= what the wrapper actually drives back) --
    // Every (valid && ready) cycle is one fired response.  For READS we
    // cross-check upresp_data against the SB's tracked line at the request's
    // addr (found via the info-keyed req tracker below).  For WRITES we
    // simply consume the matching request entry to verify the wrapper
    // returns exactly one response per accepted request.
    input  logic                                                          upresp_valid,
    input  logic                                                          upresp_ready,
    input  logic                                                          upresp_write,
    input  logic [UpstreamDataWidth-1:0]                                  upresp_data,
    input  logic [InfoWidth-1:0]                                          upresp_info
);

    localparam int unsigned ByteOfstBits = $clog2(CacheLineWidth/8);
    localparam int unsigned DepthBits    = $clog2(CacheBankDepth);
    localparam int unsigned WayBits      = (SetAssociativity > 1) ? $clog2(SetAssociativity) : 1;

    // ---------------------------------------------------------------------
    // Scoreboard state — one entry per (depth, way).
    // ---------------------------------------------------------------------
    typedef struct packed {
        logic                       valid;
        logic                       dirty;
        logic [CacheTagWidth-1:0]   tag;
    } sb_meta_t;

    sb_meta_t                  sb_meta [CacheBankDepth][SetAssociativity];
    logic [CacheLineWidth-1:0] sb_data [CacheBankDepth][SetAssociativity];

    // -- Coverage counters --
    // cov_commits[d][w]: number of proc-side commits (refill / write-hit /
    // LRU) at (d, w).
    // cov_reads[d][w] : number of read-hit responses where dec_way==w.
    int unsigned cov_commits [CacheBankDepth][SetAssociativity];
    int unsigned cov_reads   [CacheBankDepth][SetAssociativity];

    // ---------------------------------------------------------------------
    // Stats
    // ---------------------------------------------------------------------
    longint unsigned n_checked_reads     = 0;
    longint unsigned n_phantom_hits      = 0;   // cache says hit, SB has no valid line
    longint unsigned n_data_mismatches   = 0;   // cache hit but returned wrong data
    longint unsigned n_way_mismatches    = 0;   // cache picked a way SB considers invalid
    longint unsigned n_proc_commits      = 0;
    longint unsigned n_flush_commits     = 0;
    // -- End-to-end (wrapper output) checks --
    longint unsigned n_resp_total         = 0;  // every fired upstream rsp
    longint unsigned n_resp_read          = 0;
    longint unsigned n_resp_write         = 0;
    longint unsigned n_resp_stray         = 0;  // rsp arrived but no matching outstanding req
    longint unsigned n_resp_dir_mismatch  = 0;  // rsp.write != stored req.write
    longint unsigned n_resp_data_mismatch = 0;  // read rsp data != SB-tracked line slice
    longint unsigned n_resp_no_line       = 0;  // read rsp but SB has no valid line for addr (= silent refill)
    longint unsigned n_req_fires          = 0;

    // ---------------------------------------------------------------------
    // Runtime trace: dump every commit and every read at a specific
    // (depth, way) of interest.  Both default to all-ones (= no trace);
    // override at vsim time with e.g. `+sb_trace_depth=0x35 +sb_trace_way=1`
    // or with `+sb_trace_depth=0x35 +sb_trace_way=ALL` to follow all ways
    // at that set.
    // ---------------------------------------------------------------------
    int unsigned trace_depth    = '0;
    int unsigned trace_way      = '0;
    bit          trace_all_ways = 1'b0;
    bit          trace_enable   = 1'b0;   // off unless +sb_trace_depth=... provided
    bit          sb_verbose     = 1'b0;   // off unless +sb_verbose given (init + summary on PASS)

    initial begin
        string tw_str;
        trace_enable = $value$plusargs("sb_trace_depth=%h", trace_depth);
        if ($value$plusargs("sb_trace_way=%s", tw_str)) begin
            if (tw_str == "ALL" || tw_str == "all") begin
                trace_all_ways = 1'b1;
            end else begin
                void'($value$plusargs("sb_trace_way=%h", trace_way));
            end
        end
        sb_verbose = $test$plusargs("sb_verbose");
        if (sb_verbose || trace_enable) begin
            $display("[SB %m] initialised: depth=%0d ways=%0d line=%0db tag=%0db  trace_depth=0x%0h trace_way=%s  trace_enable=%0d",
                     CacheBankDepth, SetAssociativity, CacheLineWidth, CacheTagWidth,
                     trace_depth, trace_all_ways ? "ALL" : $sformatf("0x%0h", trace_way),
                     trace_enable);
        end
    end

    function automatic logic in_trace_window(
        input logic [DepthBits-1:0] d,
        input logic [WayBits-1:0]   w
    );
        return trace_enable &&
               (d == trace_depth[DepthBits-1:0]) &&
               (trace_all_ways || (w == trace_way[WayBits-1:0]));
    endfunction

    // ---------------------------------------------------------------------
    // Address decomposition (cache-local addr, AFTER xbar strip & hashing)
    //   addr layout:  [tag | depth | byte_offset]
    // ---------------------------------------------------------------------
    function automatic logic [DepthBits-1:0] addr_depth(input logic [ReqAddrWidth-1:0] addr);
        return addr[ByteOfstBits + DepthBits - 1 -: DepthBits];
    endfunction

    function automatic logic [CacheTagWidth-1:0] addr_tag(input logic [ReqAddrWidth-1:0] addr);
        return addr[ReqAddrWidth-1 -: CacheTagWidth];
    endfunction

    // Associative lookup — returns 1 if any way at the addr's depth has
    // valid==1 and tag==addr_tag.  hit_way is the first matching way.
    function automatic logic sb_find_hit(
        input  logic [ReqAddrWidth-1:0]    addr,
        output logic [WayBits-1:0]         hit_way
    );
        sb_meta_t                  m;
        logic [CacheTagWidth-1:0]  t;
        logic [DepthBits-1:0]      d;
        d = addr_depth(addr);
        t = addr_tag(addr);
        for (int w = 0; w < SetAssociativity; w++) begin
            m = sb_meta[d][w];
            if (m.valid && m.tag == t) begin
                hit_way = w[WayBits-1:0];
                return 1'b1;
            end
        end
        hit_way = '0;
        return 1'b0;
    endfunction

    // ---------------------------------------------------------------------
    // Unified state-update process.  `always` (not always_ff) keeps the
    // optimizer from flagging this as a constrained-driver block; we are
    // a passive observer, not synthesisable RTL.
    // ---------------------------------------------------------------------
    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            // Reset all lines to invalid.
            for (int d = 0; d < CacheBankDepth; d++)
                for (int w = 0; w < SetAssociativity; w++) begin
                    sb_meta[d][w]    <= '{valid: 1'b0, dirty: 1'b0, tag: '0};
                    sb_data[d][w]    <= '0;
                    cov_commits[d][w] <= 0;
                    cov_reads[d][w]   <= 0;
                end
            n_flush_commits <= 0;
            n_proc_commits  <= 0;
        end else begin
            // -- Flush / init: writes ALL ways at the given depth. --
            // For init (insn=2'b11), all flush_commit_status[w] are
            // INVALID and flush_commit_dirty[w]=0, so the line gets
            // invalidated.
            if (flush_commit_valid) begin
                for (int w = 0; w < SetAssociativity; w++) begin
                    sb_meta[flush_commit_addr][w] <= '{
                        valid: (flush_commit_status[w] == VALID),
                        dirty: flush_commit_dirty[w],
                        tag:   flush_commit_tag[w]
                    };
                end
                n_flush_commits <= n_flush_commits + 1;
                // -- Trace --
                if (trace_enable && flush_commit_addr == trace_depth[DepthBits-1:0]) begin
                    $display("[SB %m] FLUSH  t=%0t  depth=0x%0h  ways: w0={v=%0b,t=0x%0h} w1={v=%0b,t=0x%0h} w2={v=%0b,t=0x%0h} w3={v=%0b,t=0x%0h}",
                             $time, flush_commit_addr,
                             (flush_commit_status[0]==VALID), flush_commit_tag[0],
                             (flush_commit_status[1]==VALID), flush_commit_tag[1],
                             (flush_commit_status[2]==VALID), flush_commit_tag[2],
                             (flush_commit_status[3]==VALID), flush_commit_tag[3]);
                end
            end

            // -- Proc-side: refill / write-hit / LRU update.  Updates the
            // target way's meta, and bytes of its data line under the
            // commit mask.  (If both flush and proc fire same cycle, the
            // proc write wins for the proc_commit_way -- matches the
            // wrapper's proc_write_select mux precedence.) --
            if (proc_commit_valid) begin
                sb_meta[proc_commit_addr][proc_commit_way] <= '{
                    valid: (proc_commit_status == VALID),
                    dirty: proc_commit_dirty,
                    tag:   proc_commit_tag
                };
                for (int b = 0; b < MaskWidth; b++) begin
                    if (proc_commit_mask[b])
                        sb_data[proc_commit_addr][proc_commit_way][b*8 +: 8]
                            <= proc_commit_data[b*8 +: 8];
                end
                n_proc_commits <= n_proc_commits + 1;
                cov_commits[proc_commit_addr][proc_commit_way] <=
                    cov_commits[proc_commit_addr][proc_commit_way] + 1;
                // -- Trace --
                if (in_trace_window(proc_commit_addr, proc_commit_way)) begin
                    $display("[SB %m] COMMIT t=%0t  depth=0x%0h  way=%0d  status=%0d dirty=%0b tag=0x%0h  mask=0x%0h  data=0x%0h",
                             $time, proc_commit_addr, proc_commit_way,
                             proc_commit_status, proc_commit_dirty, proc_commit_tag,
                             proc_commit_mask, proc_commit_data);
                end
            end
        end
    end

    // ---------------------------------------------------------------------
    // Cross-check: decoder hit/miss decision vs scoreboard.
    // ---------------------------------------------------------------------
    // We sample at posedge: at that point both the decoder's combinational
    // output and the scoreboard's prior-cycle state are settled.  Writes
    // committed THIS cycle are visible in the scoreboard NEXT cycle
    // (matching the cache, which sees its own writes one cycle later via
    // SRAM read-out).
    logic                    sb_hit_pred;
    logic [WayBits-1:0]      sb_hit_way;

    always_comb begin
        sb_hit_pred = sb_find_hit(dec_addr, sb_hit_way);
    end

    // ---------------------------------------------------------------------
    // Upstream-request snoop: log every accepted upstream req whose depth
    // matches trace_depth.  This lets the user correlate a later commit to
    // the request that caused it (e.g. a stale-data store that triggers a
    // WRITE_PEND on the line).
    // ---------------------------------------------------------------------
    always @(posedge clk_i) begin
        if (trace_enable && rst_ni && upreq_valid && upreq_ready) begin
            if (addr_depth(upreq_addr) == trace_depth[DepthBits-1:0]) begin
                $display("[SB %m] UPREQ  t=%0t  addr=0x%0h  depth=0x%0h  tag=0x%0h  %s  wdata=0x%0h  wmask=0x%0h  info=0x%0h",
                         $time, upreq_addr,
                         addr_depth(upreq_addr), addr_tag(upreq_addr),
                         upreq_write ? "WRITE" : "READ ",
                         upreq_wdata, upreq_wmask, upreq_info);
            end
        end
    end

    always @(posedge clk_i) begin
        if (rst_ni && dec_valid) begin
            n_checked_reads <= n_checked_reads + 1;
            if (dec_is_hit) begin
                cov_reads[addr_depth(dec_addr)][dec_way] <=
                    cov_reads[addr_depth(dec_addr)][dec_way] + 1;
            end

            // -- Trace --
            if (trace_enable && addr_depth(dec_addr) == trace_depth[DepthBits-1:0]) begin
                automatic logic [CacheLineWidth-1:0] sb_d_trace;
                sb_d_trace = sb_hit_pred ? sb_data[addr_depth(dec_addr)][sb_hit_way] : '0;
                $display("[SB %m] READ   t=%0t  addr=0x%0h  depth=0x%0h  tag=0x%0h  cache:{hit=%0b way=%0d data=0x%0h}  sb:{hit=%0b way=%0d data=0x%0h}",
                         $time, dec_addr,
                         addr_depth(dec_addr), addr_tag(dec_addr),
                         dec_is_hit, dec_way, dec_data,
                         sb_hit_pred, sb_hit_way, sb_d_trace);
            end

            // -- Phantom hit: cache says hit but SB has no valid line --
            if (CheckHitMiss && dec_is_hit && !sb_hit_pred) begin
                n_phantom_hits <= n_phantom_hits + 1;
                $error("[SB %m] PHANTOM HIT  t=%0t  addr=0x%0h  depth=0x%0h  cache_way=%0d  dec_data=0x%0h\n        scoreboard has no valid line for tag=0x%0h",
                       $time, dec_addr,
                       addr_depth(dec_addr), dec_way, dec_data, addr_tag(dec_addr));
                // Dump the SB's view of all ways at this depth.
                for (int w = 0; w < SetAssociativity; w++) begin
                    $display("        way[%0d]: valid=%0b dirty=%0b tag=0x%0h",
                             w, sb_meta[addr_depth(dec_addr)][w].valid,
                             sb_meta[addr_depth(dec_addr)][w].dirty,
                             sb_meta[addr_depth(dec_addr)][w].tag);
                end
            end

            // -- Data mismatch on a hit --
            if (CheckHitData && dec_is_hit && sb_hit_pred) begin
                automatic logic [CacheLineWidth-1:0] sb_d;
                sb_d = sb_data[addr_depth(dec_addr)][sb_hit_way];
                if (dec_data !== sb_d) begin
                    n_data_mismatches <= n_data_mismatches + 1;
                    $error("[SB %m] DATA MISMATCH  t=%0t  addr=0x%0h  depth=0x%0h  way=%0d (cache picked %0d)\n        cache_data = 0x%0h\n        sb_data    = 0x%0h",
                           $time, dec_addr, addr_depth(dec_addr),
                           sb_hit_way, dec_way, dec_data, sb_d);
                end
            end

            // -- Way mismatch: cache picked a way SB doesn't consider valid --
            if (CheckHitWay && dec_is_hit && sb_hit_pred) begin
                if (!sb_meta[addr_depth(dec_addr)][dec_way].valid ||
                    sb_meta[addr_depth(dec_addr)][dec_way].tag != addr_tag(dec_addr)) begin
                    n_way_mismatches <= n_way_mismatches + 1;
                    $error("[SB %m] WAY MISMATCH  t=%0t  addr=0x%0h  depth=0x%0h\n        cache picked way=%0d (sb: valid=%0b tag=0x%0h)\n        sb expected way=%0d (valid=%0b tag=0x%0h)",
                           $time, dec_addr, addr_depth(dec_addr),
                           dec_way,    sb_meta[addr_depth(dec_addr)][dec_way].valid,
                                       sb_meta[addr_depth(dec_addr)][dec_way].tag,
                           sb_hit_way, sb_meta[addr_depth(dec_addr)][sb_hit_way].valid,
                                       sb_meta[addr_depth(dec_addr)][sb_hit_way].tag);
                end
            end
        end
    end

    // ---------------------------------------------------------------------
    // End-to-end (wrapper output) check.
    //
    // The decoder-based checks above only catch errors where the cache's
    // own internal decoder mis-decides hit/miss/way/data.  They DO NOT
    // catch errors in the data path between the cache_core and the
    // wrapper's upstream_resp port (forwarding-buffer hits, refill
    // direct-pass, resp_fifo/winfo_fifo plumbing, resp_mux arbitration).
    //
    // We add an info-keyed request tracker.  Every fired upstream_req gets
    // recorded under its info field; every fired upstream_rsp looks up the
    // matching entry and:
    //   - STRAY: no entry found  → cache produced an unexpected response
    //   - DIR_MISMATCH: rsp.write != stored req.write
    //   - For reads: compare upresp_data against the SB's tracked line
    //     (sb_data at the request's addr's depth, way found by sb_find_hit)
    //   - NO_LINE: a read rsp arrived but SB has no valid line for the addr
    //     (= cache delivered data without going through any tracked commit;
    //     usually points to a forwarding-buffer or refill-pass-through bug)
    // ---------------------------------------------------------------------
    typedef struct {
        logic                          valid;
        logic [ReqAddrWidth-1:0]       addr;
        logic                          write;
        time                           t_issued;
    } sb_req_track_t;

    // Associative array keyed by upreq_info (sparse — only used info-ids
    // get entries).  Each in-flight request occupies one entry until the
    // matching response consumes it.
    sb_req_track_t sb_req_track [logic [InfoWidth-1:0]];

    // -- Snoop accepted upstream requests --
    always @(posedge clk_i) begin
        if (rst_ni && upreq_valid && upreq_ready) begin
            sb_req_track_t entry;
            entry.valid    = 1'b1;
            entry.addr     = upreq_addr;
            entry.write    = upreq_write;
            entry.t_issued = $time;
            sb_req_track[upreq_info] = entry;
            n_req_fires = n_req_fires + 1;
        end
    end

    // -- Snoop accepted upstream responses + cross-check --
    always @(posedge clk_i) begin
        if (rst_ni && upresp_valid && upresp_ready) begin
            sb_req_track_t entry;
            n_resp_total = n_resp_total + 1;
            if (upresp_write) n_resp_write = n_resp_write + 1;
            else              n_resp_read  = n_resp_read  + 1;

            if (!sb_req_track.exists(upresp_info)) begin
                n_resp_stray = n_resp_stray + 1;
                $error("[SB %m] RESP STRAY  t=%0t  info=0x%0h  %s  data=0x%0h\n        (no outstanding request with this info)",
                       $time, upresp_info,
                       upresp_write ? "WRITE" : "READ ",
                       upresp_data);
            end else begin
                entry = sb_req_track[upresp_info];

                // Direction check
                if (entry.write !== upresp_write) begin
                    n_resp_dir_mismatch = n_resp_dir_mismatch + 1;
                    $error("[SB %m] RESP DIR MISMATCH  t=%0t  info=0x%0h  req.write=%0b  rsp.write=%0b  addr=0x%0h",
                           $time, upresp_info, entry.write, upresp_write, entry.addr);
                end

                // Data check (READ responses only)
                if (!upresp_write) begin
                    logic                       sb_hit;
                    logic [WayBits-1:0]         sb_hw;
                    logic [CacheLineWidth-1:0]  sb_line;
                    logic [UpstreamDataWidth-1:0] sb_expected;
                    int unsigned                ofst_bytes;
                    int unsigned                ofst_bits;

                    sb_hit = sb_find_hit(entry.addr, sb_hw);
                    if (!sb_hit) begin
                        n_resp_no_line = n_resp_no_line + 1;
                        $error("[SB %m] RESP NO_LINE  t=%0t  info=0x%0h  addr=0x%0h  depth=0x%0h  tag=0x%0h  rsp_data=0x%0h\n        (cache returned read data but SB has no valid line for this tag -- likely fwd-buffer or refill-pass-through that bypassed the tracked commit path)",
                               $time, upresp_info, entry.addr,
                               addr_depth(entry.addr), addr_tag(entry.addr),
                               upresp_data);
                    end else begin
                        sb_line = sb_data[addr_depth(entry.addr)][sb_hw];
                        // The upstream response is a slice of the cache line
                        // starting at the request's byte offset.  When the
                        // upstream data width == cache line width (typical
                        // for the wrapper's downstream port), the slice is
                        // the whole line.
                        if (UpstreamDataWidth == CacheLineWidth) begin
                            sb_expected = sb_line;
                        end else begin
                            // Use the request's lower byte-offset bits to pick
                            // the chunk from the line.
                            ofst_bytes = entry.addr[ByteOfstBits-1:0] &
                                         ~((UpstreamDataWidth/8) - 1);
                            ofst_bits  = ofst_bytes * 8;
                            sb_expected = sb_line[ofst_bits +: UpstreamDataWidth];
                        end
                        if (upresp_data !== sb_expected) begin
                            n_resp_data_mismatch = n_resp_data_mismatch + 1;
                            $error("[SB %m] RESP DATA MISMATCH  t=%0t  info=0x%0h  addr=0x%0h  depth=0x%0h  sb_way=%0d\n        cache rsp = 0x%0h\n        sb expect = 0x%0h\n        line full = 0x%0h",
                                   $time, upresp_info, entry.addr,
                                   addr_depth(entry.addr), sb_hw,
                                   upresp_data, sb_expected, sb_line);
                        end
                    end
                end

                // Consume the entry
                sb_req_track.delete(upresp_info);
            end
        end
    end

    // ---------------------------------------------------------------------
    // Final summary
    // ---------------------------------------------------------------------
    final begin
        // -- Coverage stats (per-line summary) --
        automatic int unsigned n_slots_touched = 0;
        automatic int unsigned n_slots_total   = CacheBankDepth * SetAssociativity;
        automatic int unsigned max_commits     = 0;
        automatic int unsigned max_reads       = 0;
        int unsigned per_way_touched[SetAssociativity];
        for (int w = 0; w < SetAssociativity; w++) per_way_touched[w] = 0;
        for (int d = 0; d < CacheBankDepth; d++)
            for (int w = 0; w < SetAssociativity; w++) begin
                automatic int c = cov_commits[d][w];
                automatic int r = cov_reads[d][w];
                if (c > 0 || r > 0) begin
                    n_slots_touched++;
                    per_way_touched[w]++;
                end
                if (c > max_commits) max_commits = c;
                if (r > max_reads)   max_reads   = r;
            end

        // Composite STATUS up front so we can decide whether to print the
        // verbose section.  Count orphans here so the figure is available
        // for the brief PASS/FAIL line too.
        begin
            automatic int unsigned n_orphaned = 0;
            automatic longint unsigned total_viol;
            foreach (sb_req_track[k]) begin
                if (sb_req_track[k].valid) begin
                    n_orphaned = n_orphaned + 1;
                end
            end
            total_viol = n_phantom_hits + n_data_mismatches + n_way_mismatches
                       + n_resp_stray + n_resp_dir_mismatch
                       + n_resp_data_mismatch + n_resp_no_line
                       + n_orphaned;

            // -- Verbose summary: only on FAIL or +sb_verbose --
            if (total_viol != 0 || sb_verbose) begin
                $display("[SB %m] ============================== Scoreboard Summary ==============================");
                $display("[SB %m]   Reads checked     : %0d", n_checked_reads);
                $display("[SB %m]   Phantom hits      : %0d", n_phantom_hits);
                $display("[SB %m]   Data mismatches   : %0d", n_data_mismatches);
                $display("[SB %m]   Way mismatches    : %0d", n_way_mismatches);
                $display("[SB %m]   Proc commits      : %0d", n_proc_commits);
                $display("[SB %m]   Flush commits     : %0d", n_flush_commits);
                $display("[SB %m]   Coverage          : slots touched %0d / %0d (%0.1f%%)",
                         n_slots_touched, n_slots_total,
                         (n_slots_total > 0) ? 100.0 * real'(n_slots_touched) / real'(n_slots_total) : 0.0);
                $display("[SB %m]   Per-way slot count: w0=%0d w1=%0d w2=%0d w3=%0d (out of %0d each)",
                         per_way_touched[0], per_way_touched[1],
                         per_way_touched[2], per_way_touched[3],
                         CacheBankDepth);
                $display("[SB %m]   Hottest slot      : max_commits=%0d  max_reads=%0d",
                         max_commits, max_reads);
                $display("[SB %m]   --- Wrapper-output (end-to-end) checks ---");
                $display("[SB %m]   Req fires         : %0d", n_req_fires);
                $display("[SB %m]   Resp total        : %0d (read=%0d  write=%0d)",
                         n_resp_total, n_resp_read, n_resp_write);
                $display("[SB %m]   Resp STRAY        : %0d", n_resp_stray);
                $display("[SB %m]   Resp DIR_MISMATCH : %0d", n_resp_dir_mismatch);
                $display("[SB %m]   Resp DATA_MISMATCH: %0d", n_resp_data_mismatch);
                $display("[SB %m]   Resp NO_LINE      : %0d", n_resp_no_line);
                foreach (sb_req_track[k]) begin
                    if (sb_req_track[k].valid) begin
                        $display("[SB %m]   ORPHAN REQ (no rsp): info=0x%0h  addr=0x%0h  %s  issued@%0t",
                                 k, sb_req_track[k].addr,
                                 sb_req_track[k].write ? "WRITE" : "READ ",
                                 sb_req_track[k].t_issued);
                    end
                end
                $display("[SB %m]   Orphaned requests : %0d (issued but never got a response)", n_orphaned);
                $display("[SB %m] ================================================================================");
            end
            // -- Always print a brief one-line STATUS --
            if (total_viol == 0)
                $display("[SB %m] STATUS: PASS");
            else
                $display("[SB %m] STATUS: FAIL (%0d violations)", total_viol);
        end
    end

endmodule
`endif
