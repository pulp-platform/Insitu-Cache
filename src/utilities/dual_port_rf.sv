// Copyright 2025 ETH Zurich and 
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 21.Dec.2023

// Dual-port write first SRAM bank (golden model)

`include "common_cells/registers.svh"
module dual_port_rf #(
    /// Number of bank entries
    parameter int unsigned  DEPTH                   = 512,
    /// Information payload needed for each narrow data
    parameter type          data_t                  = logic,
    // Dependent parameter, do not override. Address type.
    localparam type         addr_t                  = logic [$clog2(DEPTH)-1:0]
    )(
    /// Clock, positive edge triggered.
    input  logic                                    clk_i,
    /// Reset, active low.
    input  logic                                    rst_ni,

    /// Read port
    input  addr_t                                   read_addr_i,
    input  logic                                    read_valid_i,
    output logic                                    read_ready_o,
    output data_t                                   read_data_o,

    /// Upstream response
    input  addr_t                                   write_addr_i,
    input  logic                                    write_req_i,
    input  data_t                                   write_data_i
    
);

    data_t [DEPTH-1:0] data_array_q, data_array_d;
    `FFARN (data_array_q, data_array_d, '0, clk_i, rst_ni)

    addr_t read_addr;
    `FFARN (read_addr, read_addr_i, '0, clk_i, rst_ni)

    assign read_data_o = data_array_q[read_addr];
    assign read_ready_o = 1'b1;


    ///////////////////////////////
    //        Memory FSM         //
    ///////////////////////////////

    always_comb begin
        data_array_d = data_array_q;
        if (write_req_i) begin
            data_array_d[write_addr_i] = write_data_i;
        end
    end


endmodule