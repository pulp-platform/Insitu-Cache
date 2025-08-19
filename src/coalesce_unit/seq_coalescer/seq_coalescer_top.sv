// Copyright 2025 ETH Zurich and 
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 1.Mar.2024

`include "common_cells/registers.svh"
module seq_coalescer_top #(
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

    /// Upstream response
    output logic                                    upstream_resp_valid_o,
    input  logic                                    upstream_resp_ready_i,
    output logic                                    upstream_resp_write_o,
    output upstream_data_t                          upstream_resp_data_o,
    output info_t                                   upstream_resp_info_o,

    /// Downstream request
    output logic                                    downstream_req_valid_o,
    input  logic                                    downstream_req_ready_i,
    output addr_t                                   downstream_req_addr_o,
    output downstream_info_t                        downstream_req_info_o,
    output logic                                    downstream_req_write_o,
    output downstream_data_t                        downstream_req_wdata_o,
    output mask_t                                   downstream_req_wmask_o,

    /// Downsteam response
    input  logic                                    downstream_resp_valid_i,
    output logic                                    downstream_resp_ready_o,
    input  downstream_data_t                        downstream_resp_data_i,
    input  downstream_info_t                        downstream_resp_info_i,
    input  logic                                    downstream_resp_write_i
 
);

    //////////////////////////////////////
    //        Types Definition          //
    //////////////////////////////////////

    typedef upstream_data_t [NumWord-1:0]           wdata_in_words_t;

    typedef struct packed {
        downstream_data_t                           data;
        downstream_info_t                           info;
        logic                                       write;
    } downstream_resp_t;

    //////////////////////////////////////
    //        Signal Definition         //
    //////////////////////////////////////

    downstream_resp_t                               downstream_resp_in, downstream_resp;
    logic                                           downstream_resp_valid;
    logic                                           downstream_resp_ready;

    offset_t                                        resp_cnt_q, resp_cnt_d;
    `FFARN (resp_cnt_q, resp_cnt_d,                 '0, clk_i, rst_ni)


    /////////////////////////////////////
    //        Instance Modules         //
    /////////////////////////////////////

if (NumMerger <= 1) begin
    seq_coalescer_req_merger #(
        .ReqAddrWidth       (ReqAddrWidth),
        .info_t             (info_t),
        .UpstreamDataWidth  (UpstreamDataWidth),
        .DownstreamDataWidth(DownstreamDataWidth),
        .WatchDogMax        (WatchDogMax)
    ) i_seq_coalescer_req_merger (
        .clk_i                 (clk_i                 ),
        .rst_ni                (rst_ni                ),
        .upstream_req_valid_i  (upstream_req_valid_i  ),
        .upstream_req_ready_o  (upstream_req_ready_o  ),
        .upstream_req_addr_i   (upstream_req_addr_i   ),
        .upstream_req_info_i   (upstream_req_info_i   ),
        .upstream_req_write_i  (upstream_req_write_i  ),
        .upstream_req_wdata_i  (upstream_req_wdata_i  ),
        .downstream_req_valid_o(downstream_req_valid_o),
        .downstream_req_ready_i(downstream_req_ready_i),
        .downstream_req_addr_o (downstream_req_addr_o ),
        .downstream_req_info_o (downstream_req_info_o ),
        .downstream_req_write_o(downstream_req_write_o),
        .downstream_req_wdata_o(downstream_req_wdata_o),
        .downstream_req_wmask_o(downstream_req_wmask_o)
    );
end else begin
    seq_coalescer_multi_req_merger #(
        .ReqAddrWidth       (ReqAddrWidth),
        .info_t             (info_t),
        .UpstreamDataWidth  (UpstreamDataWidth),
        .DownstreamDataWidth(DownstreamDataWidth),
        .WatchDogMax        (WatchDogMax),
        .NumMerger          (NumMerger)
    ) i_seq_coalescer_multi_req_merger (
        .clk_i                 (clk_i                 ),
        .rst_ni                (rst_ni                ),
        .upstream_req_valid_i  (upstream_req_valid_i  ),
        .upstream_req_ready_o  (upstream_req_ready_o  ),
        .upstream_req_addr_i   (upstream_req_addr_i   ),
        .upstream_req_info_i   (upstream_req_info_i   ),
        .upstream_req_write_i  (upstream_req_write_i  ),
        .upstream_req_wdata_i  (upstream_req_wdata_i  ),
        .downstream_req_valid_o(downstream_req_valid_o),
        .downstream_req_ready_i(downstream_req_ready_i),
        .downstream_req_addr_o (downstream_req_addr_o ),
        .downstream_req_info_o (downstream_req_info_o ),
        .downstream_req_write_o(downstream_req_write_o),
        .downstream_req_wdata_o(downstream_req_wdata_o),
        .downstream_req_wmask_o(downstream_req_wmask_o)
    );
end


    assign downstream_resp_in.data = downstream_resp_data_i;
    assign downstream_resp_in.info = downstream_resp_info_i;
    assign downstream_resp_in.write = downstream_resp_write_i;

    spill_register #(
        .T       ( downstream_resp_t        ),
        .Bypass  ( 0                        )
        ) i_spill_down_resp  (
        .clk_i,
        .rst_ni,
        .valid_i ( downstream_resp_valid_i  ),
        .ready_o ( downstream_resp_ready_o  ),
        .data_i  ( downstream_resp_in       ),
        .valid_o ( downstream_resp_valid    ),
        .ready_i ( downstream_resp_ready    ),
        .data_o  ( downstream_resp          )
    );

    always_comb begin
        /*****************/
        /* Defualt Value */
        /*****************/
        resp_cnt_d = resp_cnt_q;

        downstream_resp_ready = '0;

        upstream_resp_valid_o = '0; 
        upstream_resp_write_o = '0; 
        upstream_resp_data_o = '0; 
        upstream_resp_info_o = '0;

        if (downstream_resp_valid) begin
            automatic wdata_in_words_t wdata;
            automatic offset_t w_ofst;

            //prepare upstream resp
            wdata = downstream_resp.data;
            w_ofst = downstream_resp.info.subs[resp_cnt_q].ofst;
            upstream_resp_write_o = downstream_resp.write;
            upstream_resp_data_o = wdata[w_ofst];
            upstream_resp_info_o = downstream_resp.info.subs[resp_cnt_q].info;

            //send upstream resp
            upstream_resp_valid_o = 1'b1;
            if (upstream_resp_ready_i) begin
                resp_cnt_d = resp_cnt_q + 1;
                if (resp_cnt_q == downstream_resp.info.num_sub) begin
                    downstream_resp_ready = 1'b1;
                    resp_cnt_d = '0;
                end
            end
        end 

    end


endmodule : seq_coalescer_top