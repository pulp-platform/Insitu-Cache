// Copyright 2023 ETH Zurich and 
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 07.May.2024

// ID buffer for saving downstream meta data

`include "common_cells/registers.svh"

module id_buffer #(
    /// Number of Entry
    parameter int unsigned NumEntry         = 64,
    // info type
    parameter type         info_t           = logic,
    // input interface
    parameter type         slv_req_t        = logic,
    parameter type         slv_resp_t       = logic,
    // output interface 
    parameter type         mst_req_t        = logic,
    parameter type         mst_resp_t       = logic
    )(
    /// Clock, positive edge triggered.
    input   logic                           clk_i,
    /// Reset, active low.
    input   logic                           rst_ni,
    /// slv
    input   logic                           slv_req_valid_i,
    output  logic                           slv_req_ready_o,
    input   slv_req_t                       slv_req_i,

    output  logic                           slv_resp_valid_o,
    input   logic                           slv_resp_ready_i,
    output  slv_resp_t                      slv_resp_o,

    /// mst
    output  logic                           mst_req_valid_o,
    input   logic                           mst_req_ready_i,
    output  mst_req_t                       mst_req_o,

    input   logic                           mst_resp_valid_i,
    output  logic                           mst_resp_ready_o,
    input   mst_resp_t                      mst_resp_i
);

    //////////////////////////////////////
    //        Types Definition          //
    //////////////////////////////////////

    typedef logic [$clog2(NumEntry)-1:0]    id_ptr_t;

    //////////////////////////////////////
    //        Signal Definition         //
    //////////////////////////////////////

    logic  [NumEntry-1:0]                   valid_array_q, valid_array_d;
    info_t [NumEntry-1:0]                   info_array_q, info_array_d;
    logic                                   is_full, is_empty;
    id_ptr_t                                inset_ptr;
    `FFARN (valid_array_q, valid_array_d,   '0, clk_i, rst_ni)
    `FFARN (info_array_q,  info_array_d,    '0, clk_i, rst_ni)

    //////////////////////////////////////
    //        ID Buffer Logic           //
    //////////////////////////////////////

    assign mst_req_o.addr   = slv_req_i.addr;
    assign mst_req_o.info   = inset_ptr;
    assign mst_req_o.write  = slv_req_i.write;
    assign mst_req_o.wmask  = slv_req_i.wmask;
    assign mst_req_o.wdata  = slv_req_i.wdata;

    assign slv_resp_o.info  = info_array_q[mst_resp_i.info];
    assign slv_resp_o.write = mst_resp_i.write;
    assign slv_resp_o.data  = mst_resp_i.data;

    assign is_full          = &valid_array_q;
    assign is_empty         = &(~valid_array_q);

    always_comb begin
        slv_req_ready_o     = '0;
        slv_resp_valid_o    = '0;
        mst_req_valid_o     = '0;
        mst_resp_ready_o    = '0;

        valid_array_d       = valid_array_q;
        info_array_d        = info_array_q;
        inset_ptr           = '0;

        //find insert entry
        for (int i = 0; i < NumEntry; i++) begin
            if (valid_array_q[i] == '0) begin
                inset_ptr = i;
                break;
            end
        end

        //process of req
        if (slv_req_valid_i & ~is_full ) begin
            mst_req_valid_o = 1'b1;
            if (mst_req_ready_i) begin
                slv_req_ready_o = 1'b1;
                valid_array_d[inset_ptr] = 1'b1;
                info_array_d[inset_ptr] = slv_req_i.info;
            end
        end

        //process of response
        if (mst_resp_valid_i) begin
            slv_resp_valid_o = 1'b1;
            if (slv_resp_ready_i) begin
                mst_resp_ready_o = 1'b1;
                valid_array_d[mst_resp_i.info] = 1'b0;
                info_array_d[mst_resp_i.info] = '0;
            end
        end
    end

endmodule : id_buffer