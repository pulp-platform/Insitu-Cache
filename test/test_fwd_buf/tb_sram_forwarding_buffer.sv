// Unit-level testbench for `sram_forwarding_buffer`.
//
// Scope:
//   Verify the 1-entry forwarding buffer in isolation against the
//   contract documented in `test_fwd_buf/README.md`.
//
// Phase 1 of the test plan:
//   - SRAM model (`sram_model`) + clock/reset
//   - Directed sequencer + simple expect-comparator
//   - Tests T1 (full-line refill then per-part reads) and T2 (the known
//     bug: partial-coverage absorption letting a same-line different-part
//     read fall through to stale SRAM).

`timescale 1ns/1ps

module tb_sram_forwarding_buffer;

    //--------------------------------------------------------------------
    // Parameters (kept small so directed sequences stay readable)
    //--------------------------------------------------------------------
    localparam int unsigned Depth           = 64;
    localparam int unsigned NumWordsPerLine = 16;          // 16 words per line
    localparam int unsigned WordWidth       = 32;
    localparam int unsigned ByteWidth       = 8;
    localparam int unsigned PartSplit       = 4;           // 4 parts of 4 words

    localparam int unsigned DataWidth    = WordWidth * NumWordsPerLine; // 512b
    localparam int unsigned MaskBits     = DataWidth / ByteWidth;       // 64
    localparam int unsigned PartIdxWidth = $clog2(PartSplit);
    localparam int unsigned PartMaskBits = MaskBits / PartSplit;        // 16
    localparam int unsigned AddrW        = $clog2(Depth);

    typedef logic [DataWidth-1:0] data_t;
    typedef logic [MaskBits-1:0]  mask_t;
    typedef logic [AddrW-1:0]     addr_t;

    //--------------------------------------------------------------------
    // Clock / reset
    //--------------------------------------------------------------------
    logic clk_i  = 0;
    logic rst_ni = 0;
    always #5 clk_i = ~clk_i;     // 100 MHz

    //--------------------------------------------------------------------
    // DUT signals
    //--------------------------------------------------------------------
    // upstream
    addr_t                    rd_addr_i;
    logic                     rd_valid_i;
    logic                     rd_ready_i;
    logic [PartIdxWidth-1:0]  rd_part_idx_i;
    logic                     rd_all_parts_i;

    addr_t                    wr_addr_i;
    data_t                    wr_data_i;
    mask_t                    wr_mask_i;
    logic                     wr_req_i;

    // SRAM tracking (combined w/ sram_model)
    logic                     sram_rd_issued_i;
    data_t                    sram_rdata_i;
    logic                     sram_wr_req_i;
    addr_t                    sram_wr_addr_i;

    // outputs
    logic                     rd_hit_comb_o;
    logic                     wr_hit_comb_o;
    logic                     wr_full_coverage_o;
    logic                     wb_needed_o;
    addr_t                    wb_addr_o;
    data_t                    wb_data_o;
    mask_t                    wb_mask_o;
    logic                     wb_done_i;
    data_t                    fwd_rdata_o;
    logic                     fwd_hit_o;
    logic [31:0]              stat_rd_hit_o;
    logic [31:0]              stat_rd_miss_o;
    logic [31:0]              stat_wr_merge_o;
    logic [31:0]              stat_wr_inval_o;
    logic [31:0]              stat_rd_total_o;
    logic [31:0]              stat_wr_total_o;
    logic [31:0]              stat_sram_rd_o;
    logic [31:0]              stat_wb_o;

    //--------------------------------------------------------------------
    // DUT
    //--------------------------------------------------------------------
    sram_forwarding_buffer #(
        .Depth          (Depth),
        .NumWordsPerLine(NumWordsPerLine),
        .WordWidth      (WordWidth),
        .ByteWidth      (ByteWidth),
        .Enable         (1'b1),
        .PartSplit      (PartSplit)
    ) dut (
        .clk_i,
        .rst_ni,
        .rd_addr_i,
        .rd_valid_i,
        .rd_ready_i,
        .rd_part_idx_i,
        .rd_all_parts_i,
        .wr_addr_i,
        .wr_data_i,
        .wr_mask_i,
        .wr_req_i,
        .sram_rd_issued_i,
        .sram_rdata_i,
        .sram_wr_req_i,
        .sram_wr_addr_i,
        .rd_hit_comb_o,
        .wr_hit_comb_o,
        .wr_full_coverage_o,
        .wb_needed_o,
        .wb_addr_o,
        .wb_data_o,
        .wb_mask_o,
        .wb_done_i,
        .fwd_rdata_o,
        .fwd_hit_o,
        .stat_rd_hit_o,
        .stat_rd_miss_o,
        .stat_wr_merge_o,
        .stat_wr_inval_o,
        .stat_rd_total_o,
        .stat_wr_total_o,
        .stat_sram_rd_o,
        .stat_wb_o
    );

    //--------------------------------------------------------------------
    // SRAM model.  The TB drives sram_rd_issued/sram_wr_req/wb_done to
    // both the buffer's tracking inputs AND the SRAM model.  This lets us
    // exercise the buffer's interaction with a real-ish memory while
    // staying fully deterministic.
    //--------------------------------------------------------------------
    logic   sram_rd_req;
    data_t  sram_rd_data;
    logic   sram_rd_valid;
    addr_t  sram_rd_addr;
    logic   sram_wr_req;
    addr_t  sram_wr_addr;
    data_t  sram_wr_data;
    mask_t  sram_wr_mask;

    sram_model #(
        .Depth          (Depth),
        .NumWordsPerLine(NumWordsPerLine),
        .WordWidth      (WordWidth),
        .ByteWidth      (ByteWidth)
    ) i_sram (
        .clk_i,
        .rst_ni,
        .rd_req_i  (sram_rd_req),
        .rd_addr_i (sram_rd_addr),
        .rd_data_o (sram_rd_data),
        .rd_valid_o(sram_rd_valid),
        .wr_req_i  (sram_wr_req),
        .wr_addr_i (sram_wr_addr),
        .wr_data_i (sram_wr_data),
        .wr_mask_i (sram_wr_mask)
    );

    // Wire SRAM model's output to the buffer's tracking inputs.
    assign sram_rdata_i     = sram_rd_data;

    //--------------------------------------------------------------------
    // Test bookkeeping
    //--------------------------------------------------------------------
    int errors        = 0;
    int tests_run     = 0;
    int tests_passed  = 0;
    string current_test = "";

    function automatic void start_test(input string name);
        current_test = name;
        $display("");
        $display("=== TEST: %s ===", name);
        tests_run += 1;
    endfunction

    function automatic void end_test();
        if (errors == 0) begin
            $display("    PASS: %s", current_test);
            tests_passed += 1;
        end else begin
            $display("    FAIL: %s (%0d errors so far)", current_test, errors);
        end
        // reset per-test errors (we report each test independently)
        errors = 0;
    endfunction

    function automatic void check_eq32(input string label,
                                       input logic [31:0] got,
                                       input logic [31:0] exp);
        if (got !== exp) begin
            $error("%s mismatch: got=0x%08h exp=0x%08h (%s)",
                   label, got, exp, current_test);
            errors += 1;
        end
    endfunction

    function automatic void check_eq_data(input string label,
                                          input data_t got,
                                          input data_t exp);
        if (got !== exp) begin
            $error("%s mismatch:\n  got=0x%h\n  exp=0x%h (%s)",
                   label, got, exp, current_test);
            errors += 1;
        end
    endfunction

    function automatic void check_eq_bool(input string label,
                                          input logic got,
                                          input logic exp);
        if (got !== exp) begin
            $error("%s mismatch: got=%0b exp=%0b (%s)",
                   label, got, exp, current_test);
            errors += 1;
        end
    endfunction

    //--------------------------------------------------------------------
    // Drivers
    //--------------------------------------------------------------------
    // Initialise all inputs (called once at the very start).
    task automatic init_signals();
        rd_addr_i        = '0;
        rd_valid_i       = 1'b0;
        rd_ready_i       = 1'b1;
        rd_part_idx_i    = '0;
        rd_all_parts_i   = 1'b0;
        wr_addr_i        = '0;
        wr_data_i        = '0;
        wr_mask_i        = '0;
        wr_req_i         = 1'b0;
        sram_rd_issued_i = 1'b0;
        sram_wr_req_i    = 1'b0;
        sram_wr_addr_i   = '0;
        wb_done_i        = 1'b0;
        sram_rd_req      = 1'b0;
        sram_rd_addr     = '0;
        sram_wr_req      = 1'b0;
        sram_wr_addr     = '0;
        sram_wr_data     = '0;
        sram_wr_mask     = '0;
    endtask

    // ------------------------------------------------------------
    // Driver convention to avoid races with the DUT's always_ff:
    //   * stimulus is driven on the NEGEDGE (mid-cycle, when signals
    //     are stable and the DUT's NBA region for the previous edge
    //     has already committed),
    //   * the next POSEDGE is when the DUT samples the stimulus,
    //   * post-edge signals (e.g. fwd_rdata_o, buf_rd_hit_q) are read
    //     at the FOLLOWING NEGEDGE so we observe latched values.
    // ------------------------------------------------------------

    // Wait until just after the next posedge has taken effect.
    task automatic step();
        @(negedge clk_i);
        // de-assert single-cycle pulses
        rd_valid_i       = 1'b0;
        wr_req_i         = 1'b0;
        sram_rd_issued_i = 1'b0;
        sram_wr_req_i    = 1'b0;
        wb_done_i        = 1'b0;
        sram_rd_req      = 1'b0;
        sram_wr_req      = 1'b0;
    endtask

    // Issue a write to the buffer with the given byte mask. Returns
    // wr_hit_comb_o and wr_full_coverage_o sampled mid-cycle (combinational).
    task automatic do_write(input addr_t  a,
                            input data_t  d,
                            input mask_t  m,
                            output logic  hit,
                            output logic  full_cov);
        @(negedge clk_i);
        wr_addr_i = a;
        wr_data_i = d;
        wr_mask_i = m;
        wr_req_i  = 1'b1;
        // let combinational always_comb chains propagate before sample
        #1;
        hit      = wr_hit_comb_o;
        full_cov = wr_full_coverage_o;
        @(posedge clk_i);    // DUT samples our stimulus here
        @(negedge clk_i);    // wait until past NBA region
        wr_req_i  = 1'b0;
    endtask

    // Drive a "user" read attempt: assert rd_valid for one cycle and
    // sample rd_hit_comb_o.  Does NOT auto-issue a SRAM read on miss --
    // that's the surrounding FSM's job and tests should exercise it
    // explicitly via `do_sram_read_to_buffer`.  The data-side check
    // (`fwd_rdata_o`) for a buffer hit is captured ONE cycle after the
    // sampling edge (when buf_rd_hit_q has latched).
    //
    // hit_comb     -- rd_hit_comb_o sampled mid-cycle while rd_valid=1
    // hit_data_nxt -- fwd_rdata_o sampled on the next mid-cycle, after
    //                 buf_rd_hit_q has registered the hit
    task automatic do_read(input addr_t  a,
                           input logic [PartIdxWidth-1:0] part,
                           input logic   all_parts,
                           output logic  hit_comb,
                           output data_t hit_data_nxt);
        @(negedge clk_i);
        rd_addr_i      = a;
        rd_part_idx_i  = part;
        rd_all_parts_i = all_parts;
        rd_valid_i     = 1'b1;
        rd_ready_i     = 1'b1;
        // let combinational always_comb chains propagate
        #1;
        hit_comb = rd_hit_comb_o;
        @(posedge clk_i);    // DUT latches buf_rd_hit_q at this edge
        @(negedge clk_i);    // observation point: NBA committed
        rd_valid_i = 1'b0;
        hit_data_nxt = fwd_rdata_o;
    endtask

    // Drive a SRAM read whose result populates the buffer via the
    // sram_rd_issued/sram_rd_pend tracking.  Useful for tests that need
    // to bring the buffer into a "populated by SRAM (partial)" state.
    task automatic do_sram_read_to_buffer(input addr_t a,
                                          input logic [PartIdxWidth-1:0] part);
        @(negedge clk_i);
        rd_addr_i        = a;
        rd_part_idx_i    = part;
        rd_all_parts_i   = 1'b0;
        rd_valid_i       = 1'b1;
        rd_ready_i       = 1'b1;
        sram_rd_issued_i = 1'b1;
        sram_rd_req      = 1'b1;
        sram_rd_addr     = a;
        @(posedge clk_i);
        @(negedge clk_i);
        rd_valid_i       = 1'b0;
        sram_rd_issued_i = 1'b0;
        sram_rd_req      = 1'b0;
        // SRAM responds 1 cycle later -> buffer populates at that edge.
        @(posedge clk_i);
        @(negedge clk_i);
    endtask

    // Idle for N cycles (each call advances exactly one rising edge).
    task automatic idle(input int n);
        for (int i = 0; i < n; i++) step();
    endtask

    // Drain the buffer to "empty, clean, invalid" between tests so each
    // test starts from a known state.  Steps:
    //   1. If buf_dirty_q=1, pulse wb_done_i to clear dirty.
    //   2. If buf_valid_q=1, drive sram_wr_req_i for buf_addr_q to
    //      trigger the buffer's same-addr invalidation logic.
    task automatic drain_to_empty();
        @(negedge clk_i);
        // Clear dirty via wb_done if needed.
        if (dut.buf_dirty_q) begin
            wb_done_i = 1'b1;
            @(posedge clk_i);
            @(negedge clk_i);
            wb_done_i = 1'b0;
        end
        // Invalidate via same-addr sram_wr_req if still valid.
        if (dut.buf_valid_q) begin
            sram_wr_req_i  = 1'b1;
            sram_wr_addr_i = dut.buf_addr_q;
            @(posedge clk_i);
            @(negedge clk_i);
            sram_wr_req_i  = 1'b0;
        end
        // sanity
        if (dut.buf_valid_q || dut.buf_dirty_q) begin
            $error("drain_to_empty did not clear buffer (valid=%0b dirty=%0b)",
                   dut.buf_valid_q, dut.buf_dirty_q);
        end
    endtask

    // Convenience: build a per-part mask for the given part index.
    function automatic mask_t mk_part_mask(input int p);
        mask_t m = '0;
        for (int b = 0; b < PartMaskBits; b++)
            m[p*PartMaskBits + b] = 1'b1;
        return m;
    endfunction

    // Convenience: build a full-line mask.
    function automatic mask_t mk_full_mask();
        mask_t m = '1;
        return m;
    endfunction

    // Convenience: build a single-word mask within a part.
    function automatic mask_t mk_word_mask(input int word_in_line);
        // word_in_line in [0, NumWordsPerLine).
        // Each word is WordWidth/ByteWidth bytes.
        mask_t m = '0;
        int    bytes_per_word = WordWidth / ByteWidth;
        for (int b = 0; b < bytes_per_word; b++)
            m[word_in_line*bytes_per_word + b] = 1'b1;
        return m;
    endfunction

    // Convenience: build "data with sentinel pattern P at every word".
    function automatic data_t mk_data_pattern(input logic [31:0] p);
        data_t d = '0;
        for (int w = 0; w < NumWordsPerLine; w++)
            d[w*WordWidth +: WordWidth] = WordWidth'(p + w);
        return d;
    endfunction

    // SRAM model's reset-time content for row N: every word = N+1.
    function automatic data_t sram_init_data(input int row);
        data_t d = '0;
        for (int w = 0; w < NumWordsPerLine; w++)
            d[w*WordWidth +: WordWidth] = WordWidth'(row + 1);
        return d;
    endfunction

    //--------------------------------------------------------------------
    // T1: refill (full-line write) absorbed; reads of every part HIT.
    //--------------------------------------------------------------------
    //
    // Stimulus
    //   cycle N    : wr_full_line(addr=A, data=PATTERN_A)
    //   cycle N+1+ : rd(addr=A, part=p)  for p in 0..3
    //
    // Expected
    //   - wr_hit_comb_o=1 in cycle N (full-line absorption into empty buffer).
    //   - wr_full_coverage_o=1 in cycle N (buf will hold whole line).
    //   - For each subsequent rd, rd_hit_comb_o=1 and the next cycle's
    //     fwd_rdata_o == PATTERN_A.
    task automatic test_T1();
        addr_t  A;
        data_t  patternA;
        mask_t  m;
        logic   wr_hit, wr_full;
        logic   rd_hit;
        data_t  got;

        start_test("T1: refill (full-line write) -> rd part 0..3 all HIT");

        A         = 5;
        patternA  = mk_data_pattern(32'hAA00_0000);

        do_write(A, patternA, mk_full_mask(), wr_hit, wr_full);
        check_eq_bool("T1: wr_hit_comb_o on full-line absorption",
                      wr_hit, 1'b1);
        check_eq_bool("T1: wr_full_coverage_o on full-line absorption",
                      wr_full, 1'b1);
        idle(1);

        // Read each part; expect HIT (because buf_all_parts_q=1 after
        // the absorption).  do_read returns hit data sampled the cycle
        // after the read (when buf_rd_hit_q has just latched).
        for (int p = 0; p < PartSplit; p++) begin
            do_read(A, p[PartIdxWidth-1:0], 1'b0, rd_hit, got);
            check_eq_bool($sformatf("T1: rd_hit_comb_o for part %0d", p),
                          rd_hit, 1'b1);
            check_eq_data($sformatf("T1: fwd_rdata_o for part %0d", p),
                          got, patternA);
        end

        end_test();
    endtask

    //--------------------------------------------------------------------
    // T2: partial-coverage absorption -> same-line different-part read
    //     gets stale SRAM data.  This is the bug.
    //--------------------------------------------------------------------
    //
    // Stimulus
    //   1) read of (A, part 0)   -- miss, SRAM read populates buffer
    //                                with part 0 data, buf_all_parts=0.
    //   2) write of (A, part 0)  -- partial; wr_buf_hit absorbs.  Buffer
    //                                stays buf_all_parts=0, buf_dirty=1.
    //   3) read of (A, part 1)   -- DIFFERENT part of same line.
    //                                buf_part doesn't match -> rd_hit_comb=0.
    //                                With the broken cache-core bypass
    //                                (or wr_hit_comb_o=1) the cache core
    //                                would ALSO bypass its hazard, but
    //                                here we observe the buffer-side
    //                                directly: fwd_rdata after SRAM read
    //                                must be the SRAM-init pattern for
    //                                part 1 (the bug is that SRAM still
    //                                has the OLD value for part 0 instead
    //                                of the merged write data, and the
    //                                bypass would let that leak through).
    //
    // Pass criterion (this TB level)
    //   * After step 2, wr_full_coverage_o must be 0 (we are NOT
    //     full-coverage).  This is the precise signal the cache core
    //     should use to *deny* the hazard bypass.
    //   * After step 3, the buffer is repopulated with part 1's SRAM data
    //     and the previous dirty merge for part 0 SURVIVES somewhere --
    //     either still in the buffer (today the buffer drops it: see
    //     `if (sram_rd_pend_q) ... buf_dirty_q <= 1'b0`) or written back
    //     via wb_done.  Today neither happens -> data loss.  The check is
    //     that wb_needed_o is asserted before the SRAM read of part 1
    //     repopulates the buffer.
    task automatic test_T2();
        addr_t  A;
        data_t  user_word;
        logic   wr_hit, wr_full;
        logic   rd_hit;
        data_t  got;

        start_test("T2: partial absorb -> wb_needed=1 + wr_full_coverage_o=0");

        A = 7;

        // (1) Bring line A part 0 into the buffer via a SRAM read.
        //     After this, buf_valid=1, buf_addr=A, buf_dirty=0,
        //     buf_all_parts_q=0, buf_part_idx_q=0.
        do_sram_read_to_buffer(A, 2'd0);

        // (2) User write into part 0, partial mask.  This should HIT the
        //     buffer (wr_buf_hit) and mark it dirty, but coverage stays
        //     partial because buf_all_parts_q was 0 at the time of merge.
        user_word = '0;
        for (int w = 0; w < NumWordsPerLine; w++)
            user_word[w*WordWidth +: WordWidth] = 32'hCAFE_0000 + w;
        do_write(A, user_word, mk_word_mask(0), wr_hit, wr_full);
        check_eq_bool("T2: wr_hit_comb_o on partial merge",
                      wr_hit, 1'b1);
        check_eq_bool("T2: wr_full_coverage_o MUST be 0 (partial merge)",
                      wr_full, 1'b0);
        @(posedge clk_i);
        #1;

        // (3) The buffer is now valid+dirty for line A, partial coverage.
        //     The cache core must NOT bypass a same-line write/read hazard
        //     here (verified at the integrated TB by the new
        //     bank_write_data_buf_full_cov_i check); at the unit level we
        //     simply verify the signals the core relies on.
        check_eq_bool("T2: wb_needed_o must be 1 (buf is dirty)",
                      wb_needed_o, 1'b1);

        // (4) Try a same-line different-part read.  rd_hit_comb_o MUST be
        //     0 because buf_part_idx=0, buf_all_parts=0, request part=1.
        //     We do NOT drive sram_rd_issued here -- that would clobber
        //     the dirty data.  The C3 contract assertion (Phase 2) covers
        //     the FSM's responsibility to issue wb before populate.
        do_read(A, 2'd1, 1'b0, rd_hit, got);
        check_eq_bool("T2: rd_hit_comb_o for diff part must be 0",
                      rd_hit, 1'b0);

        // wb_needed_o must STILL be 1 -- buffer is still dirty since we
        // didn't issue a wb_done.
        check_eq_bool("T2: wb_needed_o still 1 after the diff-part read attempt",
                      wb_needed_o, 1'b1);

        end_test();
    endtask

    //--------------------------------------------------------------------
    // T3: dirty buffer (line A) -> wr_full_line(B) at a different addr.
    //     Buffer must REFUSE to absorb (wr_hit_comb_o=0, wr_full_coverage_o=0)
    //     because the wr_full_hit guard requires `!buf_valid | !buf_dirty
    //     | (buf_addr == wr_addr)`.  The surrounding FSM is then expected
    //     to writeback A first (we drive wb_done_i to model that), after
    //     which a re-issued wr_full_line(B) is accepted.
    //--------------------------------------------------------------------
    task automatic test_T3();
        addr_t  A, B;
        data_t  patternA, patternB;
        logic   wr_hit, wr_full;

        start_test("T3: dirty(A) + wr_full_line(B!=A) -> NO absorb until wb");

        A = 11; B = 22;
        patternA = mk_data_pattern(32'h1100_0000);
        patternB = mk_data_pattern(32'h2200_0000);

        // Bring buf into "dirty for A, all_parts" state via full-line write.
        do_write(A, patternA, mk_full_mask(), wr_hit, wr_full);
        check_eq_bool("T3: prep wr_hit_comb_o on full-line A", wr_hit, 1'b1);
        check_eq_bool("T3: prep wr_full_coverage_o on full-line A", wr_full, 1'b1);

        // Now write line B full-line.  The buffer is dirty for A so
        // wr_full_hit must NOT fire (would clobber dirty A).
        do_write(B, patternB, mk_full_mask(), wr_hit, wr_full);
        check_eq_bool("T3: wr_hit_comb_o must be 0 (different-addr full into dirty)",
                      wr_hit, 1'b0);
        check_eq_bool("T3: wr_full_coverage_o must be 0", wr_full, 1'b0);
        // Buffer state unchanged: still A, still dirty.
        @(negedge clk_i);
        check_eq_bool("T3: buf still dirty for A", dut.buf_dirty_q, 1'b1);
        check_eq32  ("T3: buf addr still A", 32'(dut.buf_addr_q), 32'(A));

        // Drive wb_done to model the FSM's writeback completion.
        @(negedge clk_i);
        wb_done_i = 1'b1;
        @(posedge clk_i);
        @(negedge clk_i);
        wb_done_i = 1'b0;
        check_eq_bool("T3: buf clean after wb_done", dut.buf_dirty_q, 1'b0);

        // Now the re-issued wr_full_line(B) should be accepted.
        do_write(B, patternB, mk_full_mask(), wr_hit, wr_full);
        check_eq_bool("T3: wr_hit_comb_o=1 after wb (clean buf, new full-line)",
                      wr_hit, 1'b1);
        check_eq_bool("T3: wr_full_coverage_o=1 after wb", wr_full, 1'b1);

        end_test();
    endtask

    //--------------------------------------------------------------------
    // T4: full-line SRAM read populate -> per-part reads HIT.
    //     Drives a single read with rd_all_parts_i=1; SRAM responds; the
    //     buffer populates with buf_all_parts_q=1.  Subsequent per-part
    //     reads must HIT and return SRAM-init data.
    //--------------------------------------------------------------------
    task automatic test_T4();
        addr_t  A;
        data_t  expected;
        logic   rd_hit;
        data_t  got;

        start_test("T4: full-line SRAM populate (rd_all_parts) -> per-part reads HIT");

        A = 9;
        expected = sram_init_data(int'(A));

        // First, trigger a SRAM read with rd_all_parts_i=1.
        @(negedge clk_i);
        rd_addr_i        = A;
        rd_part_idx_i    = '0;
        rd_all_parts_i   = 1'b1;
        rd_valid_i       = 1'b1;
        rd_ready_i       = 1'b1;
        sram_rd_issued_i = 1'b1;
        sram_rd_req      = 1'b1;
        sram_rd_addr     = A;
        @(posedge clk_i);
        @(negedge clk_i);
        rd_valid_i       = 1'b0;
        sram_rd_issued_i = 1'b0;
        sram_rd_req      = 1'b0;
        // Wait one more cycle for buffer to populate.
        @(posedge clk_i);
        @(negedge clk_i);

        check_eq_bool("T4: buf_valid_q after SRAM populate", dut.buf_valid_q, 1'b1);
        check_eq_bool("T4: buf_all_parts_q after rd_all_parts populate",
                      dut.buf_all_parts_q, 1'b1);

        // Now any per-part read must HIT.
        for (int p = 0; p < PartSplit; p++) begin
            do_read(A, p[PartIdxWidth-1:0], 1'b0, rd_hit, got);
            check_eq_bool($sformatf("T4: rd_hit_comb_o for part %0d", p),
                          rd_hit, 1'b1);
            check_eq_data($sformatf("T4: fwd_rdata_o for part %0d", p),
                          got, expected);
        end

        end_test();
    endtask

    //--------------------------------------------------------------------
    // T5: SRAM read pending + concurrent same-addr write -> wr_concurrent_hit.
    //     The buffer's `wr_concurrent_hit` path merges incoming write data
    //     with the SRAM data being returned in the same cycle.
    //--------------------------------------------------------------------
    task automatic test_T5();
        addr_t  A;
        data_t  user_word, expected, got;
        logic   wr_hit, wr_full;
        logic   rd_hit;

        start_test("T5: SRAM read pending + concurrent same-addr write -> wr_concurrent_hit");

        A = 13;

        // Step 1: drive a read on cycle T; sram_rd_issued -> sram_rd_pend at T+1.
        // Use rd_all_parts so the populate covers the whole line (so
        // wr_parts_covered_concurrent is automatically OK).
        @(negedge clk_i);
        rd_addr_i        = A;
        rd_part_idx_i    = '0;
        rd_all_parts_i   = 1'b1;
        rd_valid_i       = 1'b1;
        rd_ready_i       = 1'b1;
        sram_rd_issued_i = 1'b1;
        sram_rd_req      = 1'b1;
        sram_rd_addr     = A;
        @(posedge clk_i);
        @(negedge clk_i);
        rd_valid_i       = 1'b0;
        sram_rd_issued_i = 1'b0;
        sram_rd_req      = 1'b0;
        // Now sram_rd_pend_q=1.

        // Step 2: drive a same-address write while sram_rd_pend_q=1.
        // wr_concurrent_hit fires.
        check_eq_bool("T5: sram_rd_pend_q=1 before concurrent write",
                      dut.sram_rd_pend_q, 1'b1);
        user_word = '0;
        for (int w = 0; w < NumWordsPerLine; w++)
            user_word[w*WordWidth +: WordWidth] = 32'hCC00_0000 + w;
        // Use full mask so merge covers all bytes.
        wr_addr_i = A;
        wr_data_i = user_word;
        wr_mask_i = mk_full_mask();
        wr_req_i  = 1'b1;
        #1;
        check_eq_bool("T5: wr_hit_comb_o on concurrent same-addr write",
                      wr_hit_comb_o, 1'b1);
        @(posedge clk_i);
        @(negedge clk_i);
        wr_req_i = 1'b0;

        // Step 3: read each part; expect HIT and merged data.
        // With full mask, the merged data == user_word (writes win all bytes).
        expected = user_word;
        for (int p = 0; p < PartSplit; p++) begin
            do_read(A, p[PartIdxWidth-1:0], 1'b0, rd_hit, got);
            check_eq_bool($sformatf("T5: rd_hit_comb_o for part %0d", p),
                          rd_hit, 1'b1);
            check_eq_data($sformatf("T5: fwd_rdata_o for part %0d (merged)", p),
                          got, expected);
        end

        end_test();
    endtask

    //--------------------------------------------------------------------
    // T6: dirty(A, partial) + read of different line B.
    //     The read must MISS (rd_part_match=0 anyway because B!=A).  The
    //     C3 SVA must NOT fire here (we don't drive sram_rd_issued, so
    //     no clobber occurs).  wb_needed_o must remain 1.
    //--------------------------------------------------------------------
    task automatic test_T6();
        addr_t  A, B;
        data_t  user_word, got;
        logic   wr_hit, wr_full;
        logic   rd_hit;

        start_test("T6: dirty(A, partial) + rd(B) -> miss + wb_needed=1, no clobber");

        A = 17; B = 18;

        // Bring buf into "partial dirty for A" state.
        do_sram_read_to_buffer(A, 2'd0);
        user_word = '0;
        for (int w = 0; w < NumWordsPerLine; w++)
            user_word[w*WordWidth +: WordWidth] = 32'hC600_0000 + w;
        do_write(A, user_word, mk_word_mask(0), wr_hit, wr_full);
        check_eq_bool("T6: partial wr absorbed", wr_hit, 1'b1);
        check_eq_bool("T6: wr_full_coverage_o=0 (partial)", wr_full, 1'b0);

        // Read of different line B.  Must miss.
        do_read(B, 2'd0, 1'b0, rd_hit, got);
        check_eq_bool("T6: rd_hit_comb_o=0 for different line", rd_hit, 1'b0);

        // Buffer must still be dirty for A.
        @(negedge clk_i);
        check_eq_bool("T6: buf still dirty for A", dut.buf_dirty_q, 1'b1);
        check_eq32  ("T6: buf addr still A",
                     32'(dut.buf_addr_q), 32'(A));
        check_eq_bool("T6: wb_needed_o still 1", wb_needed_o, 1'b1);

        end_test();
    endtask

    //--------------------------------------------------------------------
    // T7: full-line absorb (dirty) + same-line different-part read -> HIT.
    //     With buf_all_parts_q=1, every part is in the buffer.
    //--------------------------------------------------------------------
    task automatic test_T7();
        addr_t  A;
        data_t  patternA, got;
        logic   wr_hit, wr_full;
        logic   rd_hit;

        start_test("T7: full-line absorb + same-line diff-part read -> HIT");

        A = 21;
        patternA = mk_data_pattern(32'h7700_0000);

        do_write(A, patternA, mk_full_mask(), wr_hit, wr_full);
        check_eq_bool("T7: wr_full_coverage_o=1", wr_full, 1'b1);

        // Read part 2 (different from part_idx=0 but covered by all_parts).
        do_read(A, 2'd2, 1'b0, rd_hit, got);
        check_eq_bool("T7: rd_hit_comb_o=1 for diff part with all_parts=1",
                      rd_hit, 1'b1);
        check_eq_data("T7: fwd_rdata_o has full-line data", got, patternA);

        end_test();
    endtask

    //--------------------------------------------------------------------
    // T8: full-line absorb -> partial merge same-line -> diff-part read -> HIT.
    //     The partial merge must NOT clear buf_all_parts_q (it stays 1
    //     because we entered the merge with buf_all_parts_q=1).  So a
    //     subsequent read of any part still hits, and the merged data is
    //     visible at the merged byte position.
    //--------------------------------------------------------------------
    task automatic test_T8();
        addr_t  A;
        data_t  patternA, partial_word, expected, got;
        logic   wr_hit, wr_full;
        logic   rd_hit;

        start_test("T8: full-line + partial merge -> diff-part read HIT (all_parts stays 1)");

        A = 27;
        patternA = mk_data_pattern(32'h8800_0000);

        // (1) Full-line absorb: buf_all_parts=1, buf_dirty=1.
        do_write(A, patternA, mk_full_mask(), wr_hit, wr_full);
        check_eq_bool("T8: full absorb wr_full=1", wr_full, 1'b1);

        // (2) Partial merge of word 5 (within part 1) -- different word
        //     from part 0 which buf was previously tracking, but
        //     buf_all_parts=1 so wr_buf_hit_safe (wr_buf_hit & buf_all_parts)
        //     fires.  buf_all_parts stays 1.
        partial_word = '0;
        partial_word[5*WordWidth +: WordWidth] = 32'hDEAD_BEEF;
        do_write(A, partial_word, mk_word_mask(5), wr_hit, wr_full);
        check_eq_bool("T8: partial-merge wr_hit=1", wr_hit, 1'b1);
        // wr_full_coverage_o must remain 1 because buf was already
        // all_parts at the time of merge.
        check_eq_bool("T8: wr_full_coverage_o=1 (merge into all-parts buf)",
                      wr_full, 1'b1);

        // (3) Read part 1 -- the part containing word 5.
        do_read(A, 2'd1, 1'b0, rd_hit, got);
        check_eq_bool("T8: rd_hit_comb_o=1 for part 1", rd_hit, 1'b1);

        // Expected: original full-line pattern with word 5 overwritten.
        expected = patternA;
        expected[5*WordWidth +: WordWidth] = 32'hDEAD_BEEF;
        check_eq_data("T8: fwd_rdata_o reflects merged write", got, expected);

        end_test();
    endtask

    //--------------------------------------------------------------------
    // Top-level test sequencer
    //--------------------------------------------------------------------
    initial begin
        $timeformat(-9, 0, " ns", 12);
        init_signals();
        // reset
        rst_ni = 0;
        repeat (4) @(posedge clk_i);
        rst_ni = 1;
        @(posedge clk_i);

        $display("================================================");
        $display("  tb_sram_forwarding_buffer  (Phase 1)");
        $display("  Depth=%0d NumWordsPerLine=%0d PartSplit=%0d",
                 Depth, NumWordsPerLine, PartSplit);
        $display("================================================");

        test_T1();             drain_to_empty();
        test_T2();             drain_to_empty();
        test_T3();             drain_to_empty();
        test_T4();             drain_to_empty();
        test_T5();             drain_to_empty();
        test_T6();             drain_to_empty();
        test_T7();             drain_to_empty();
        test_T8();             drain_to_empty();

        idle(4);

        $display("");
        $display("================================================");
        $display("  Summary: %0d/%0d tests passed", tests_passed, tests_run);
        $display("================================================");
        if (tests_passed == tests_run) begin
            $display("[PASS] All tests passed.");
            $finish(0);
        end else begin
            $display("[FAIL] %0d tests failed.", tests_run - tests_passed);
            $finish(1);
        end
    end

    //--------------------------------------------------------------------
    // Watchdog
    //--------------------------------------------------------------------
    initial begin
        #(50us);
        $display("[FAIL] Watchdog: TB ran for >50us without finishing.");
        $finish(2);
    end

endmodule
