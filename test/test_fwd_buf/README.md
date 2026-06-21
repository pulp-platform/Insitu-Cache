# `sram_forwarding_buffer` unit testbench

A stand-alone testbench for the 1-entry forwarding buffer, isolated from
the cache controller.  Its purpose:

1. Reproduce buffer-related bugs deterministically in <1 ms of sim time
   (vs. 14+ µs to even reach the failure in the integrated cluster TB).
2. Pin down the buffer ↔ cache-core contract so that future RTL changes
   to the buffer don't silently regress integration tests.

## Files

| File | Purpose |
|---|---|
| `sram_model.sv` | 1-cycle byte-masked single-port SRAM, init pattern row N = `{words{N+1}}`. |
| `tb_sram_forwarding_buffer.sv` | TB top: clock/reset, DUT instance, SRAM model, directed sequencer, scoreboard. |

## Build & run

From the cachepool repo root:

```bash
# 1. Generate compile script
bender script vsim -t test_fwd_buf \
    > sim/work/compile.fwd_buf.tcl \
    --vlog-arg="-svinputport=compat" \
    --vlog-arg="-override_timescale 1ns/1ps" \
    --vlog-arg="-suppress 2583" \
    --vlog-arg="-suppress 13314"

# 2. Compile and run
questa-2023.4-zr vsim -c \
    -do "source sim/work/compile.fwd_buf.tcl; vsim -t 1ps tb_sram_forwarding_buffer; run -a; quit"
```

Exit code reflects pass/fail: `$finish(0)` on pass, `$finish(1)` on fail,
`$finish(2)` on watchdog timeout.

Or use the helper Makefile target (see `sim/sim.mk`).

## Test list (Phase 1)

| # | Test | Stimulus | Pass criterion |
|---|---|---|---|
| **T1** | full-line refill, then per-part reads | `wr_full_line(A)` → `rd(A, p)` for p in 0..3 | All 4 reads HIT, return write data. |
| **T2** | partial-coverage absorption + same-line different-part read (the bug) | `rd(A, 0)` (miss → SRAM populate part 0) → `wr(A, word_in_part_0)` partial → `rd(A, 1)` | After the partial write: `wr_full_coverage_o=0` (cache core must NOT bypass hazard). After diff-part read attempt: `wb_needed_o=1` (buffer is dirty, FSM must writeback before populate). |

Phase 2 will add T3–T8 and bind the SVAs C1, C3, C4, C6.

## The contract being verified

| ID | Statement |
|---|---|
| C1 | If `wr_hit_comb_o=1` AND `wr_full_coverage_o=1` in cycle T, every read to (`buf_addr_q`, any part) issued in cycles T..T+N (until next eviction) returns the merged write data. |
| C2 | When `wb_done_i` is asserted while `wb_needed_o=1`, the buffer commits writeback as durable (dirty cleared next cycle). |
| C3 | While `buf_valid_q & buf_dirty_q`, the buffer never overwrites `buf_data_q` for a different `buf_addr_q` until a `wb_done_i` for the original address has fired. |
| C4 | `wr_full_coverage_o=1` at cycle T ⇒ in cycle T+1, `buf_valid_q & (buf_addr_q == wr_addr_i_T) & buf_all_parts_q=1`. |
| C5 | After `wb_done_i`, the next-cycle `wb_needed_o` is 0 (until a new dirty merge). |
| C6 | Repopulating from a SRAM read into a *dirty* buffer at a *different* address is a contract violation -- the surrounding FSM must writeback first.  The buffer should `$error` on this. |
