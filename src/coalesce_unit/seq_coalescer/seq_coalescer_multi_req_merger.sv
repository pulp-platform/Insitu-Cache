// Copyright 2023 ETH Zurich and 
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 1.Mar.2024

`include "common_cells/registers.svh"
module seq_coalescer_multi_req_merger #(
    /// Address width of both upstream narrow request and downstream wide request
    parameter int unsigned ReqAddrWidth             = 32,
    /// Information payload needed for each narrow data
    parameter type         info_t                   = logic,
    /// Data width of upstream channel
    parameter int unsigned UpstreamDataWidth        = 32,
    /// Data width of downstream channel
    parameter int unsigned DownstreamDataWidth      = 512,
    /// Watchdog Counter
    parameter int unsigned WatchDogMax              = 4,
    /// Number of single req merger
    parameter int unsigned NumMerger                = 2,
    // Dependent parameter, do not override. Depth of cache bank.
    localparam int unsigned NumWord                 = DownstreamDataWidth/UpstreamDataWidth,
    // Dependent parameter, do not override. Address type.
    localparam type addr_t                          = logic [ReqAddrWidth-1:0],
    // Dependent parameter, do not override. Narrow word type.
    localparam type upstream_data_t                 = logic [UpstreamDataWidth-1:0],
    // Dependent parameter, do not override. Wide word type.
    localparam type downstream_data_t               = logic [DownstreamDataWidth-1:0],
    // Dependent parameter, do not override. Word mask type.
    localparam type mask_t                          = logic [DownstreamDataWidth/UpstreamDataWidth-1:0],
    // Dependent parameter, do not override. tag type.
    localparam type tag_t                           = logic [ReqAddrWidth-$clog2(DownstreamDataWidth/8)-1:0],
    // Dependent parameter, do not override. byte offset type.
    localparam type offset_t                        = logic [$clog2(DownstreamDataWidth/UpstreamDataWidth)-1:0],
    // Dependent parameter, do not override. coalescer subentry.
    localparam type sub_t                           = struct packed {info_t info; offset_t ofst;},
    // Dependent parameter, do not override. Downstream request payload.
    localparam type downstream_info_t               = struct packed {offset_t num_sub; sub_t [NumWord-1:0] subs;}
    )(
    /// Clock, positive edge triggered.
    input  logic                                    clk_i,
    /// Reset, active low.
    input  logic                                    rst_ni,

    /// Upstream request
    input  logic                                    upstream_req_valid_i,
    output logic                                    upstream_req_ready_o,
    input  addr_t                                   upstream_req_addr_i,
    input  info_t                                   upstream_req_info_i,
    input  logic                                    upstream_req_write_i,
    input  upstream_data_t                          upstream_req_wdata_i,

    /// Downstream request
    output logic                                    downstream_req_valid_o,
    input  logic                                    downstream_req_ready_i,
    output addr_t                                   downstream_req_addr_o,
    output downstream_info_t                        downstream_req_info_o,
    output logic                                    downstream_req_write_o,
    output downstream_data_t                        downstream_req_wdata_o,
    output mask_t                                   downstream_req_wmask_o
 
);

    //////////////////////////////////////
    //        Types Definition          //
    //////////////////////////////////////

    typedef struct packed {
        addr_t                                      addr;
        downstream_info_t                           info;
        logic                                       write;
        downstream_data_t                           wdata;
        mask_t                                      wmask;
    } downstream_payload_t;

    //////////////////////////////////////
    //        Signal Definition         //
    //////////////////////////////////////

    logic                   [NumMerger-1:0]         up_multi_valid;
    logic                   [NumMerger-1:0]         up_multi_ready;

    downstream_payload_t                            down_out;
    downstream_payload_t    [NumMerger-1:0]         down_multi;
    logic                   [NumMerger-1:0]         down_multi_valid;
    logic                   [NumMerger-1:0]         down_multi_ready;

    /////////////////////////////////////
    //        Instance Modules         //
    /////////////////////////////////////


    stream_demux #(.N_OUP(NumMerger)) i_stream_demux (
        .inp_valid_i(upstream_req_valid_i),
        .inp_ready_o(upstream_req_ready_o),
        .oup_sel_i  (upstream_req_addr_i[$clog2(NumMerger)+$clog2(DownstreamDataWidth/8)-1: $clog2(DownstreamDataWidth/8)]),
        .oup_valid_o(up_multi_valid),
        .oup_ready_i(up_multi_ready)
    );

    for (genvar i = 0; i < NumMerger; i++) begin: gen_req_merger
        seq_coalescer_req_merger #(
            .ReqAddrWidth       (ReqAddrWidth),
            .info_t             (info_t),
            .UpstreamDataWidth  (UpstreamDataWidth),
            .DownstreamDataWidth(DownstreamDataWidth),
            .WatchDogMax        (WatchDogMax)
        ) i_seq_coalescer_req_merger (
            .clk_i,
            .rst_ni,

            .upstream_req_valid_i  (up_multi_valid[i]     ),
            .upstream_req_ready_o  (up_multi_ready[i]     ),
            .upstream_req_addr_i   (upstream_req_addr_i   ),
            .upstream_req_info_i   (upstream_req_info_i   ),
            .upstream_req_write_i  (upstream_req_write_i  ),
            .upstream_req_wdata_i  (upstream_req_wdata_i  ),

            .downstream_req_valid_o(down_multi_valid[i]   ),
            .downstream_req_ready_i(down_multi_ready[i]   ),
            .downstream_req_addr_o (down_multi[i].addr    ),
            .downstream_req_info_o (down_multi[i].info    ),
            .downstream_req_write_o(down_multi[i].write   ),
            .downstream_req_wdata_o(down_multi[i].wdata   ),
            .downstream_req_wmask_o(down_multi[i].wmask   )
        );

    end

    stream_arbiter #(.DATA_T(downstream_payload_t), .N_INP(NumMerger)) i_req_merger_arbiter (
        .clk_i,
        .rst_ni,
        .inp_data_i (down_multi),
        .inp_valid_i(down_multi_valid),
        .inp_ready_o(down_multi_ready),
        .oup_data_o (down_out),
        .oup_valid_o(downstream_req_valid_o),
        .oup_ready_i(downstream_req_ready_i)
    );

    assign downstream_req_addr_o = down_out.addr;
    assign downstream_req_info_o = down_out.info;
    assign downstream_req_write_o = down_out.write;
    assign downstream_req_wdata_o = down_out.wdata;
    assign downstream_req_wmask_o = down_out.wmask;




endmodule : seq_coalescer_multi_req_merger