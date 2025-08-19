// Copyright 2025 ETH Zurich and
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 21.Dec.2023

// Way instance composite by Pseudo dual-port write first SRAM banks

`include "common_cells/registers.svh"
module pseudo_dual_port_way #(
    /// Address depth
    parameter int unsigned  DEPTH                   = 512,
    /// Number of banks
    parameter int unsigned  BANKS                   = 8,
    /// Information payload needed for each narrow data
    parameter type          meta_t                  = logic,
    /// Information payload needed for each narrow data
    parameter type          data_t                  = logic,
    /// SRAM Configuration
    parameter type          impl_in_t               = logic,
    // Dependent parameter, do not override. Address type.
    localparam type         addr_t                  = logic [$clog2(DEPTH)-1:0]
    )(
    /// Clock, positive edge triggered.
    input  logic                                    clk_i,
    /// Reset, active low.
    input  logic                                    rst_ni,
    /// SRAM Configuration
    input  impl_in_t [BANKS-1:0]                    impl_i,

    /// Common port
    input  addr_t                                   read_addr_i,
    input  logic                                    read_valid_i,
    input  addr_t                                   write_addr_i,

    /// Meta port
    output logic                                    meta_read_ready_o,
    output meta_t                                   meta_read_data_o,
    input  logic                                    meta_write_req_i,
    input  meta_t                                   meta_write_data_i,

    /// Data port
    output logic                                    data_read_ready_o,
    output data_t                                   data_read_data_o,
    input  logic                                    data_write_req_i,
    input  data_t                                   data_write_data_i
);

    pseudo_dual_port_bank #(.DEPTH(DEPTH),.BANKS(BANKS),.data_t(meta_t),.impl_in_t(impl_in_t)) i_cache_meta_bank (
        .clk_i,
        .rst_ni,
        .impl_i      (impl_i),
        .read_addr_i(read_addr_i),
        .read_valid_i(read_valid_i),
        .read_ready_o(meta_read_ready_o),
        .read_data_o(meta_read_data_o),
        .write_addr_i(write_addr_i),
        .write_req_i(meta_write_req_i),
        .write_data_i(meta_write_data_i)
    );

    pseudo_dual_port_bank #(.DEPTH(DEPTH),.BANKS(BANKS),.data_t(data_t),.impl_in_t(impl_in_t)) i_cache_data_bank (
        .clk_i,
        .rst_ni,
        .impl_i      (impl_i),
        .read_addr_i(read_addr_i),
        .read_valid_i(read_valid_i),
        .read_ready_o(data_read_ready_o),
        .read_data_o(data_read_data_o),
        .write_addr_i(write_addr_i),
        .write_req_i(data_write_req_i),
        .write_data_i(data_write_data_i)
    );

endmodule : pseudo_dual_port_way
