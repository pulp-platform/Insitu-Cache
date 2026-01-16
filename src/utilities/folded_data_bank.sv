// Copyright 2025 ETH Zurich and
// University of Bologna
//
// Solderpad Hardware License
// Version 0.51, see LICENSE for details.
//
// SPDX-License-Identifier: SHL-0.51
//
// Folded SRAM wrapper: store per-way data in a deeper bank.
// Each way is a port; address is folded as {way, addr}.

module folded_data_bank #(
    /// Number of cache ways (ports).
    parameter int unsigned NumWays      = 4,
    /// Depth per way (bank factor already applied).
    parameter int unsigned DepthPerWay  = 128,
    /// Data width in bits.
    parameter int unsigned DataWidth    = 64,
    /// Byte width in bits.
    parameter int unsigned ByteWidth    = 8,
    /// Read latency in cycles.
    parameter int unsigned Latency      = 1,
    /// Simulation init pattern ("zeros" or "none").
    parameter string       SimInit      = "zeros",
    // Dependent parameters, do not override.
    localparam int unsigned AddrWidth       = (DepthPerWay > 1) ? $clog2(DepthPerWay) : 1,
    localparam int unsigned ByteCount       = DataWidth / ByteWidth,
    localparam int unsigned FoldedDepth     = DepthPerWay * NumWays,
    localparam int unsigned FoldedAddrWidth = (FoldedDepth > 1) ? $clog2(FoldedDepth) : 1
)(
    /// Clock, positive edge triggered.
    input  logic                             clk_i,
    /// Reset, active low.
    input  logic                             rst_ni,
    /// Per-way requests.
    input  logic        [NumWays-1:0]        req_i,
    input  logic        [NumWays-1:0]        we_i,
    input  logic        [AddrWidth-1:0]      addr_i   [NumWays],
    input  logic        [DataWidth-1:0]      wdata_i  [NumWays],
    input  logic        [ByteCount-1:0]      be_i     [NumWays],
    output logic        [DataWidth-1:0]      rdata_o  [NumWays]
);

    // Folded address per way.
    logic [FoldedAddrWidth-1:0] folded_addr [NumWays];

    for (genvar way = 0; way < NumWays; way++) begin : gen_folded_addr
        localparam int unsigned WayBase = way * DepthPerWay;
        assign folded_addr[way] = WayBase[FoldedAddrWidth-1:0] + addr_i[way];
    end

    // One SRAM instance per way-port; replace with a true multi-port macro if desired.
    for (genvar way = 0; way < NumWays; way++) begin : gen_folded_banks
        tc_sram_impl #(
            .NumWords (FoldedDepth),
            .DataWidth(DataWidth),
            .ByteWidth(ByteWidth),
            .NumPorts (1),
            .Latency  (Latency),
            .SimInit  (SimInit)
        ) i_folded_bank (
            .clk_i,
            .rst_ni,
            .impl_i ('0),
            .impl_o (/* unused */),
            .req_i  (req_i[way]),
            .we_i   (we_i[way]),
            .addr_i (folded_addr[way]),
            .wdata_i(wdata_i[way]),
            .be_i   (be_i[way]),
            .rdata_o(rdata_o[way])
        );
    end

endmodule
