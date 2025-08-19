// Copyright 2025 ETH Zurich and 
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 07.June.2023

// An axi wrapped dram model

`include "axi/assign.svh"
`include "axi/typedef.svh"
`include "insitu_cache/assign.svh"

module cache_to_axi #(
    /// Word width of cache line (512b default)
    parameter int unsigned CacheLineWidth   = 512,
    // Cache interface
    parameter type         cache_req_t      = logic,
    parameter type         cache_resp_t     = logic,
    // AXI interface 
    parameter type         axi_req_t        = logic,
    parameter type         axi_resp_t       = logic
    )(
    /// Clock, positive edge triggered.
    input   logic                           clk_i,
    /// Reset, active low.
    input   logic                           rst_ni,
    /// cache
    input   logic                           cache_req_valid_i,
    output  logic                           cache_req_ready_o,
    input   cache_req_t                     cache_req_i,
    output  logic                           cache_resp_valid_o,
    input   logic                           cache_resp_ready_i,
    output  cache_resp_t                    cache_resp_o,
    /// Axi
    output  axi_req_t                       axi_req_o,
    input   axi_resp_t                      axi_resp_i
 );

    //////////////////////////////////////
    //        Signal Definition         //
    //////////////////////////////////////

    cache_req_t                             ar_fifo_in;
    logic                                   ar_fifo_full;
    logic                                   ar_fifo_push;
    cache_req_t                             ar_fifo_out;
    logic                                   ar_fifo_empty;
    logic                                   ar_fifo_pop;

    cache_req_t                             aw_fifo_in;
    logic                                   aw_fifo_full;
    logic                                   aw_fifo_push;
    cache_req_t                             aw_fifo_out;
    logic                                   aw_fifo_empty;
    logic                                   aw_fifo_pop;

    cache_req_t                             w_fifo_in;
    logic                                   w_fifo_full;
    logic                                   w_fifo_push;
    cache_req_t                             w_fifo_out;
    logic                                   w_fifo_empty;
    logic                                   w_fifo_pop;

    cache_resp_t                            r_resp_in;
    cache_resp_t                            b_resp_in;

    /////////////////////////////////////
    //        Instance Modules         //
    /////////////////////////////////////

    assign cache_req_ready_o = cache_req_i.write? ~aw_fifo_full & ~w_fifo_full : ~ar_fifo_full;

    /****************/
    /*  AR Channel  */
    /****************/

    assign ar_fifo_push = cache_req_valid_i & cache_req_ready_o & ~cache_req_i.write;
    assign ar_fifo_in = cache_req_i;

    fifo_v3 #(
        .FALL_THROUGH                       (1'b0                       ),
        .DEPTH                              (2                          ),
        .dtype                              (cache_req_t                )
    ) i_ar_fifo (
        .clk_i,
        .rst_ni,
        .flush_i                            (1'b0                       ),
        .testmode_i                         (1'b0                       ),
        .full_o                             (ar_fifo_full            ),
        .empty_o                            (ar_fifo_empty           ),
        .usage_o                            (/*open*/                   ),
        .data_i                             (ar_fifo_in              ),
        .push_i                             (ar_fifo_push            ),
        .data_o                             (ar_fifo_out             ),
        .pop_i                              (ar_fifo_pop             )
    );

    assign axi_req_o.ar_valid = ~ar_fifo_empty;
    assign ar_fifo_pop = axi_req_o.ar_valid & axi_resp_i.ar_ready;
    `AXI_AR_ASSIGN_FROM(ar_fifo_out, axi_req_o.ar, $clog2(CacheLineWidth/8))

    /****************/
    /*  AW Channel  */
    /****************/

    assign aw_fifo_push = cache_req_valid_i & cache_req_ready_o & cache_req_i.write;
    assign aw_fifo_in = cache_req_i;

    fifo_v3 #(
        .FALL_THROUGH                       (1'b0                       ),
        .DEPTH                              (2                          ),
        .dtype                              (cache_req_t                )
    ) i_aw_fifo (
        .clk_i,
        .rst_ni,
        .flush_i                            (1'b0                       ),
        .testmode_i                         (1'b0                       ),
        .full_o                             (aw_fifo_full            ),
        .empty_o                            (aw_fifo_empty           ),
        .usage_o                            (/*open*/                   ),
        .data_i                             (aw_fifo_in              ),
        .push_i                             (aw_fifo_push            ),
        .data_o                             (aw_fifo_out             ),
        .pop_i                              (aw_fifo_pop             )
    );

    assign axi_req_o.aw_valid = ~aw_fifo_empty;
    assign aw_fifo_pop = axi_req_o.aw_valid & axi_resp_i.aw_ready;
    `AXI_AW_ASSIGN_FROM(aw_fifo_out, axi_req_o.aw, $clog2(CacheLineWidth/8))


    /***************/
    /*  W Channel  */
    /***************/

    assign w_fifo_push = cache_req_valid_i & cache_req_ready_o & cache_req_i.write;
    assign w_fifo_in = cache_req_i;

    fifo_v3 #(
        .FALL_THROUGH                       (1'b0                       ),
        .DEPTH                              (2                          ),
        .dtype                              (cache_req_t                )
    ) i_w_fifo (
        .clk_i,
        .rst_ni,
        .flush_i                            (1'b0                       ),
        .testmode_i                         (1'b0                       ),
        .full_o                             (w_fifo_full            ),
        .empty_o                            (w_fifo_empty           ),
        .usage_o                            (/*open*/                   ),
        .data_i                             (w_fifo_in              ),
        .push_i                             (w_fifo_push            ),
        .data_o                             (w_fifo_out             ),
        .pop_i                              (w_fifo_pop             )
    );

    assign axi_req_o.w_valid = ~w_fifo_empty;
    assign w_fifo_pop = axi_req_o.w_valid & axi_resp_i.w_ready;
    `AXI_W_ASSIGN_FROM(w_fifo_out, axi_req_o.w, $clog2(CacheLineWidth/8))

    /*******************/
    /*  R & B Channel  */
    /*******************/

    `AXI_R_ASSIGN_TO(r_resp_in, axi_resp_i.r, $clog2(CacheLineWidth/8))
    `AXI_B_ASSIGN_TO(b_resp_in, axi_resp_i.b, $clog2(CacheLineWidth/8))

    stream_arbiter #(.DATA_T(cache_resp_t), .N_INP(2)) i_R_B_arbiter (
        .clk_i,
        .rst_ni,
        .inp_data_i ({r_resp_in         , b_resp_in}),
        .inp_valid_i({axi_resp_i.r_valid, axi_resp_i.b_valid}),
        .inp_ready_o({axi_req_o.r_ready , axi_req_o.b_ready}),
        .oup_data_o (cache_resp_o),
        .oup_valid_o(cache_resp_valid_o),
        .oup_ready_i(cache_resp_ready_i)
    );

    
 
    


endmodule : cache_to_axi 
