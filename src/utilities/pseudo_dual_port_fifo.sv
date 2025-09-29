// Copyright 2023 ETH Zurich and
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 21.Dec.2023

// FIFO built up by Pseudo dual-port write first SRAM bank

`include "common_cells/registers.svh"
module pseudo_dual_port_fifo #(
    parameter int unsigned DATA_WIDTH   = 32,   // default data width if the fifo is of type logic
    parameter int unsigned DEPTH        = 8,    // depth can be arbitrary from 0 to 2**32
    parameter type dtype                = logic [DATA_WIDTH-1:0],
    parameter type impl_in_t            = logic,
    // DO NOT OVERWRITE THIS PARAMETER
    parameter int unsigned ADDR_DEPTH   = (DEPTH > 1) ? $clog2(DEPTH) : 1
)(
    input  logic  clk_i,            // Clock
    input  logic  rst_ni,           // Asynchronous reset active low
    input  impl_in_t [1:0] impl_i,  // SRAM configuration
    // status flags
    output logic  full_o,           // queue is full
    output logic  empty_o,          // queue is empty
    output logic  [ADDR_DEPTH-1:0] usage_o,  // fill pointer
    // as long as the queue is not full we can push new data
    input  dtype  data_i,           // data to push into the queue
    input  logic  push_i,           // data is valid and can be pushed to the queue
    // as long as the queue is not empty we can pop new elements
    output dtype  data_o,           // output data
    input  logic  pop_i             // pop head from queue
);

    // Internal signals
    logic [ADDR_DEPTH-1:0]  write_ptr;
    logic                   write_req;
    logic [ADDR_DEPTH-1:0]  fifo_count;

    logic                   read_valid;
    logic                   read_ready;
    logic [ADDR_DEPTH-1:0]  read_ptr;
    dtype                   bank_read_data;
    logic [ADDR_DEPTH-1:0]  read_ptr_q, read_ptr_d;
    logic                   read_lock_q,read_lock_d;
    dtype                   read_lock_data_q,read_lock_data_d;
    logic                   bank_data_ready_q,bank_data_ready_d;
    `FFARN (read_ptr_q, read_ptr_d, '0, clk_i, rst_ni)
    `FFARN (read_lock_q,read_lock_d, '0, clk_i, rst_ni)
    `FFARN (read_lock_data_q,read_lock_data_d, '0, clk_i, rst_ni)
    `FFARN (bank_data_ready_q,bank_data_ready_d, '0, clk_i, rst_ni)

    // Instantiate pseudo_dual_port_bank
    pseudo_dual_port_bank #(
        .DEPTH(DEPTH),
        .BANKS(2),          // Single bank used for FIFO
        .impl_in_t(impl_in_t),
        .data_t(dtype)
    ) bank_inst (
        .clk_i(clk_i),
        .rst_ni(rst_ni),
        .impl_i(impl_i),
        .read_addr_i(read_ptr),     // Read address is the read pointer
        .read_valid_i(read_valid),  // Read valid signal
        .read_ready_o(read_ready),  // Read ready signal from bank
        .read_data_o(bank_read_data),       // Output data from the pseudo_dual_port_bank
        .write_addr_i(write_ptr),   // Write address is the write pointer
        .write_req_i(write_req),    // Write request when push is valid
        .write_data_i(data_i)       // Input data to write
    );

    // Write logic (push)
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            write_ptr <= '0;
        end else if (push_i && !full_o) begin
            write_ptr <= (write_ptr + 1) % DEPTH;
        end
    end

    // FIFO counter (usage)
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            fifo_count <= '0;
        end else begin
            case ({push_i && !full_o, pop_i && !empty_o})
                2'b01: fifo_count <= fifo_count - 1; // Pop only
                2'b10: fifo_count <= fifo_count + 1; // Push only
                default: fifo_count <= fifo_count;   // No push/pop or both
            endcase
        end
    end

    // Assign flags
    assign empty_o = (fifo_count == 0) | (~bank_data_ready_q & ~read_lock_q);
    assign full_o  = (fifo_count == (DEPTH-1));
    assign usage_o = fifo_count;

    // Handshaking signals
    assign write_req  = push_i && !full_o;

    always_comb begin : proc_read_fifo
        read_valid          = '0;
        read_ptr            = read_ptr_q;
        read_ptr_d          = read_ptr_q;
        read_lock_d         = read_lock_q;
        read_lock_data_d    = read_lock_data_q;
        bank_data_ready_d   = bank_data_ready_q;

        if ((fifo_count == 0) && write_req) begin
            read_lock_data_d = data_i;
            read_lock_d = 1'b1;
        end else
        if (pop_i && ~empty_o) begin
            if (fifo_count == 1) begin
                read_ptr_d = read_ptr_q + 1'b1;
                read_lock_d = 1'b0;
                read_lock_data_d = '0;
                bank_data_ready_d = '0;
            end else begin
                read_ptr = read_ptr_q + 1'b1;
                read_ptr_d = read_ptr_q + 1'b1;
                read_valid = 1'b1;
                read_lock_d = 1'b0;
                read_lock_data_d = '0;
                bank_data_ready_d = read_ready;
            end
        end else begin
            if (~read_lock_q) begin
                if (bank_data_ready_q) begin
                    read_lock_d = 1'b1;
                    read_lock_data_d = bank_read_data;
                    bank_data_ready_d = '0;
                end else begin
                    read_valid = 1'b1;
                    read_lock_d = 1'b0;
                    read_lock_data_d = '0;
                    bank_data_ready_d = read_ready;
                end
            end
        end
    end

    assign data_o = read_lock_q? read_lock_data_q : bank_data_ready_q? bank_read_data : '0;



endmodule
