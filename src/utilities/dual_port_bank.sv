// Copyright 2023 ETH Zurich and 
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 21.Dec.2023

// Dual-port write first SRAM bank (golden model)

`include "common_cells/registers.svh"
module dual_port_bank #(
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

    // data_t data_array[DEPTH] = '{default:'0};
    data_t [DEPTH-1:0] data_array;

    logic write_read_meet_q, write_read_meet_d;
    `FFARN (write_read_meet_q, write_read_meet_d, '0, clk_i, rst_ni)
    assign write_read_meet_d = write_req_i & (read_addr_i == write_addr_i);

    data_t old_data;
    `FFARN (old_data, data_array[read_addr_i], '0, clk_i, rst_ni)

    data_t new_data;
    `FFARN (new_data, write_data_i, '0, clk_i, rst_ni)

    assign read_data_o = write_read_meet_q? new_data : old_data;
    assign read_ready_o = 1'b1;


    //////////////////////////////
    //        Cache FSM         //
    //////////////////////////////

    task cycle_tt;
      #800ps;
    endtask

    task cycle_end;
      @(posedge clk_i);
    endtask

    task cycle_at;
      #10ps;
    endtask

    initial begin
        for (int i = 0; i < DEPTH; i++) begin
            data_array[i] = '0;
        end
        @(posedge rst_ni);
        @(posedge clk_i);
        forever begin
            cycle_tt();
            if (write_req_i) begin
                data_array[write_addr_i] = write_data_i;
                // $info("SRAM Bank Write at %0d, data = 0x%x, next read at %0d", write_addr_i, write_data_i, _read_addr);
                // $stop;
            end
            cycle_end();
        end
    end


endmodule