// Copyright 2023 ETH Zurich and 
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 25.Mar.2023

//the rsp_spliter_v2 deal with reading streams from coalesced reads
//
`include "common_cells/registers.svh"
module rsp_spliter_v2 #(
    /// upstream data width
    parameter int unsigned UpstreamDataWidth                = 64,
    /// downstream data width.
    parameter int unsigned DownstreamDataWidth              = 512,
    /// Number of narrow request ports.
    parameter int unsigned NumPorts                         = 16,
    /// address width
    parameter int unsigned AddrWidth                        = 32,
    /// Dependent parameter, do not override. number of address offsets of one coalesce block.
    parameter int unsigned NumAddrOfst                      = DownstreamDataWidth/UpstreamDataWidth,
    /// Dependent parameter, do not override. Axi DW Align.
    parameter int unsigned UpstreamDataAlign                = $clog2(UpstreamDataWidth/8),
    /// Dependent parameter, do not override. Mem DW Align.
    parameter int unsigned DownstreamDataAlign              = $clog2(DownstreamDataWidth/8),
    /// Dependent parameter, do not override. port align type.
    localparam type addr_ofst_t                             = logic [$clog2(NumAddrOfst)-1:0],
    /// Dependent parameter, do not override. upstream data type.
    localparam type upstream_data_t                         = logic [UpstreamDataWidth-1:0],
    /// Dependent parameter, do not override. downstream data type.
    localparam type downstream_data_t                       = logic [DownstreamDataWidth-1:0],
    /// Dependent parameter, do not override. address type.
    localparam type addr_t                                  = logic [AddrWidth-1:0]
    )(
    /// Clock, positive edge triggered.
    input  logic                            clk_i,
    /// Reset, active low.
    input  logic                            rst_ni,

    /// Downstream Data
    input logic                             downstream_valid_i,
    output logic                            downstream_ready_o,
    input downstream_data_t                 downstream_data_i,

    /// Upstream side
    input logic [NumPorts-1:0]              rsp_ready_i,
    output logic [NumPorts-1:0]             rsp_valid_o,
    output upstream_data_t [NumPorts-1:0]   rsp_data_o,

    /// Valid port bitmap (strb)
    input   logic [NumPorts-1:0]            coal_strb_i,
    input   logic                           coal_strb_empty_i,
    output  logic                           coal_strb_pop_o,

    /// Addr offset for every port
    input   addr_ofst_t [NumPorts-1:0]      coal_port_addr_ofst_i,
    input   logic       [NumPorts-1:0]      coal_port_addr_ofst_empty_i,
    output  logic       [NumPorts-1:0]      coal_port_addr_ofst_pop_o
);

    //Data path
    upstream_data_t [NumAddrOfst-1:0] downstream_unpacked_data;

    assign downstream_unpacked_data = downstream_data_i;

    for (genvar i = 0; i < NumPorts; i++) begin: gen_upstream_data
        assign rsp_data_o[i] = downstream_unpacked_data[coal_port_addr_ofst_i[i]] ;
    end

    //Control signal
    logic [NumPorts-1:0] handshack_mask_q, handshack_mask_d;
    `FFARN (handshack_mask_q, handshack_mask_d, '0, clk_i, rst_ni)
    logic [NumPorts-1:0] handshack_mask_record;
    assign handshack_mask_record = handshack_mask_q | (rsp_ready_i & rsp_valid_o);


    always_comb begin : proc_control
        handshack_mask_d = handshack_mask_record;

        downstream_ready_o = '0;
        rsp_valid_o = '0;

        coal_strb_pop_o = '0;
        coal_port_addr_ofst_pop_o = '0;

        for (int i = 0; i < NumPorts; i++) begin
            rsp_valid_o[i] = downstream_valid_i & coal_strb_i[i] & ~handshack_mask_q[i];
        end

        if (downstream_valid_i) begin
            if (handshack_mask_record == coal_strb_i) begin
                handshack_mask_d = '0;
                downstream_ready_o = 1'b1;
            end
        end
    end

 
 endmodule : rsp_spliter_v2 