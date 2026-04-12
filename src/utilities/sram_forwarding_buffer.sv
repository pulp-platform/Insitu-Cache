// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// 1-entry write-back forwarding buffer for single-port SRAMs.
//
// Sits transparently between an access controller and SRAM banks.
// Caches one SRAM row in registers. Matching reads return buffer
// data; writes merge into the buffer.  The buffer is invalidated
// when a write targets a different address.
//
// Control signals (ready/valid/req) pass through UNCHANGED.
// Only the read data output is intercepted on a buffer hit.
//
// When Enable==0 the buffer registers still update (so they are
// not removed by the optimizer) but the output always returns
// SRAM data -- functionally identical to no buffer.

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
    /// Enable the forwarding buffer output (1=active, 0=passthrough)
    parameter bit          Enable         = 1'b1,
    // -- Derived parameters (do not override) --
    localparam int unsigned DataWidth     = WordWidth * NumWordsPerLine,
    localparam int unsigned MaskBits      = DataWidth / ByteWidth,
    localparam type         data_t        = logic [DataWidth-1:0],
    localparam type         mask_t        = logic [MaskBits-1:0],
    localparam type         addr_t        = logic [$clog2(Depth)-1:0]
)(
    input  logic   clk_i,
    input  logic   rst_ni,

    // -- Snoop ports (active, directly from the access controller) --
    // Read channel: address + handshake
    input  addr_t  rd_addr_i,
    input  logic   rd_valid_i,     // downstream_read_valid_o from FSM
    input  logic   rd_ready_i,     // downstream_read_ready_i from SRAM

    // Write channel: address + data + mask + request
    input  addr_t  wr_addr_i,
    input  data_t  wr_data_i,
    input  mask_t  wr_mask_i,
    input  logic   wr_req_i,       // downstream_write_req_o from FSM

    // -- SRAM read data (raw, 1 cycle after rd_valid & rd_ready) --
    input  data_t  sram_rdata_i,

    // -- Forwarded read data output --
    output data_t  fwd_rdata_o,    // replaces upstream_read_data_o
    output logic   fwd_hit_o,      // 1 = data came from buffer (for debug)

    // -- Statistics outputs (always present for observability) --
    output logic [31:0] stat_rd_hit_o,
    output logic [31:0] stat_rd_miss_o,
    output logic [31:0] stat_wr_merge_o,
    output logic [31:0] stat_wr_inval_o
);

    // -- Buffer state --
    data_t  buf_data_q;
    addr_t  buf_addr_q;
    logic   buf_valid_q;

    // -- In-flight SRAM read tracking --
    logic   sram_rd_pend_q;
    addr_t  sram_rd_addr_q;

    // -- Registered hit output (1-cycle latency to match SRAM) --
    logic   buf_rd_hit_q;
    data_t  buf_rd_data_q;

    // -- Statistics counters (always present, exposed as outputs) --
    logic [31:0] stat_rd_hit;
    logic [31:0] stat_rd_miss;
    logic [31:0] stat_wr_merge;
    logic [31:0] stat_wr_inval;

    assign stat_rd_hit_o   = stat_rd_hit;
    assign stat_rd_miss_o  = stat_rd_miss;
    assign stat_wr_merge_o = stat_wr_merge;
    assign stat_wr_inval_o = stat_wr_inval;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            buf_valid_q    <= 1'b0;
            buf_data_q     <= '0;
            buf_addr_q     <= '0;
            sram_rd_pend_q <= 1'b0;
            sram_rd_addr_q <= '0;
            buf_rd_hit_q   <= 1'b0;
            buf_rd_data_q  <= '0;
            stat_rd_hit    <= '0;
            stat_rd_miss   <= '0;
            stat_wr_merge  <= '0;
            stat_wr_inval  <= '0;
        end else begin
            // - Track SRAM reads in flight -
            sram_rd_pend_q <= rd_valid_i & rd_ready_i;
            if (rd_valid_i & rd_ready_i)
                sram_rd_addr_q <= rd_addr_i;

            // - Populate buffer from SRAM response -
            if (sram_rd_pend_q) begin
                buf_valid_q <= 1'b1;
                buf_addr_q  <= sram_rd_addr_q;
                buf_data_q  <= sram_rdata_i;
            end

            // - Write merge: update buffer if same address -
            if (wr_req_i && buf_valid_q && (wr_addr_i == buf_addr_q)) begin
                for (int b = 0; b < MaskBits; b++)
                    if (wr_mask_i[b])
                        buf_data_q[b*ByteWidth +: ByteWidth] <=
                            wr_data_i[b*ByteWidth +: ByteWidth];
                stat_wr_merge <= stat_wr_merge + 1;
            end

            // - Write invalidate: different address -
            if (wr_req_i && buf_valid_q && (wr_addr_i != buf_addr_q)) begin
                buf_valid_q <= 1'b0;
                stat_wr_inval <= stat_wr_inval + 1;
            end

            // - Read hit detection (registered for 1-cycle latency) -
            if (rd_valid_i & rd_ready_i) begin
                if (buf_valid_q && (buf_addr_q == rd_addr_i)) begin
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

    // -- End-of-simulation report --
endmodule : sram_forwarding_buffer
