// Copyright 2023 ETH Zurich and
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 21.Dec.2023

// Pseudo dual-port write first SRAM bank

`include "common_cells/registers.svh"
module pseudo_dual_port_bank #(
    /// Address depth
    parameter int unsigned  DEPTH                   = 512,
    /// Number of banks
    parameter int unsigned  BANKS                   = 2,
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
    input impl_in_t         [BANKS-1:0]             impl_i,

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
    //////////////////////////////////////
    //        Types Definition          //
    //////////////////////////////////////

    typedef enum logic [2:0] {
        IDLE = '0,
        W_ONLY,
        R_ONLY,
        WR_DIFF_BANK,
        WR_SAME_ADDR,
        WR_CONFLICT
    } pseudo_dual_status_t;

    typedef logic [$clog2(BANKS)-1:0]               bank_select_t;
    typedef logic [$clog2(DEPTH)-$clog2(BANKS)-1:0] bank_addr_t;

    //////////////////////////////////////
    //        Signal Definition         //
    //////////////////////////////////////

    //status
    pseudo_dual_status_t                            status;

    //write line buffer
    data_t write_line_buffer;
    `FFARN (write_line_buffer, write_data_i, '0,    clk_i, rst_ni)

    //bank signals
    logic         [BANKS-1:0]                       bank_req;
    logic         [BANKS-1:0]                       bank_we;
    bank_addr_t   [BANKS-1:0]                       bank_addr;
    data_t        [BANKS-1:0]                       bank_wdata;
    data_t        [BANKS-1:0]                       bank_rdata;

    //read & write port address info
    bank_select_t                                   read_bank_select;
    bank_addr_t                                     read_bank_addr;
    bank_select_t                                   write_bank_select;
    bank_addr_t                                     write_bank_addr;

    //read data selection
    logic                                           read_data_from_line_buffer_q, read_data_from_line_buffer_d;
    bank_select_t                                   read_data_from_bank_select_q, read_data_from_bank_select_d;
    `FFARN (read_data_from_line_buffer_q,
            read_data_from_line_buffer_d,
            '0, clk_i, rst_ni)
    `FFARN (read_data_from_bank_select_q,
            read_data_from_bank_select_d,
            '0, clk_i, rst_ni)

    //////////////////////////////////////
    //        Instance Modules          //
    //////////////////////////////////////

    for (genvar i = 0; i < BANKS; i++) begin
        tc_sram_impl #(
            .NumWords(DEPTH/BANKS),
            .DataWidth($bits(data_t)),
            .ByteWidth($bits(data_t)),
            .NumPorts(1),
            .Latency(1),
            .SimInit("zeros"),
            .impl_in_t(impl_in_t)
        ) i_sram_bank (
            .clk_i,
            .rst_ni,
            .impl_i (impl_i[i]    ),
            .impl_o (/* unused */ ),
            .req_i  (bank_req[i]  ),
            .we_i   (bank_we[i]   ),
            .addr_i (bank_addr[i] ),
            .wdata_i(bank_wdata[i]),
            .be_i   ('1           ),
            .rdata_o(bank_rdata[i])
        );
    end

    //////////////////////////////////////
    //        Pseudo Dual Logics        //
    //////////////////////////////////////

    always_comb begin : proc_pseudo_dual
        /*****************/
        /* Defualt Value */
        /*****************/
        status = IDLE;

        bank_req = '0;
        bank_we = '0;
        bank_addr = '0;
        bank_wdata = '0;

        {read_bank_addr,    read_bank_select}   = read_addr_i;
        {write_bank_addr,   write_bank_select}  = write_addr_i;

        read_ready_o = 1'b1;
        read_data_from_line_buffer_d = '0;
        read_data_from_bank_select_d = '0;

        /*******************/
        /* Determin Status */
        /*******************/
        if (read_valid_i & write_req_i) begin
            if (read_bank_select != write_bank_select) begin
                status = WR_DIFF_BANK;
            end else
            if (read_addr_i == write_addr_i) begin
                status = WR_SAME_ADDR;
            end else begin
                status = WR_CONFLICT;
            end
        end else
        if (read_valid_i) begin
            status = R_ONLY;
        end else
        if (write_req_i) begin
            status = W_ONLY;
        end

        /************/
        /* Main FSM */
        /************/
        case (status)
            W_ONLY: begin
                bank_req[write_bank_select]     = 1'b1;
                bank_we[write_bank_select]      = 1'b1;
                bank_addr[write_bank_select]    = write_bank_addr;
                bank_wdata[write_bank_select]   = write_data_i;
            end

            R_ONLY: begin
                bank_req[read_bank_select]      = 1'b1;
                bank_we[read_bank_select]       = 1'b0;
                bank_addr[read_bank_select]     = read_bank_addr;

                read_data_from_line_buffer_d    = '0;
                read_data_from_bank_select_d    = read_bank_select;
            end

            WR_DIFF_BANK: begin
                bank_req[write_bank_select]     = 1'b1;
                bank_we[write_bank_select]      = 1'b1;
                bank_addr[write_bank_select]    = write_bank_addr;
                bank_wdata[write_bank_select]   = write_data_i;

                bank_req[read_bank_select]      = 1'b1;
                bank_we[read_bank_select]       = 1'b0;
                bank_addr[read_bank_select]     = read_bank_addr;

                read_data_from_line_buffer_d    = '0;
                read_data_from_bank_select_d    = read_bank_select;
            end

            WR_SAME_ADDR: begin
                bank_req[write_bank_select]     = 1'b1;
                bank_we[write_bank_select]      = 1'b1;
                bank_addr[write_bank_select]    = write_bank_addr;
                bank_wdata[write_bank_select]   = write_data_i;

                read_data_from_line_buffer_d    = 1'b1;
                read_data_from_bank_select_d    = '0;
            end

            WR_CONFLICT: begin
                bank_req[write_bank_select]     = 1'b1;
                bank_we[write_bank_select]      = 1'b1;
                bank_addr[write_bank_select]    = write_bank_addr;
                bank_wdata[write_bank_select]   = write_data_i;

                read_data_from_line_buffer_d    = '0;
                read_data_from_bank_select_d    = '0;
            end

            default : /* default */;
        endcase

        /*********************/
        /* Read Ready Logics */
        /*********************/
        if (write_req_i & (read_addr_i != write_addr_i) & (read_bank_select == write_bank_select)) begin
            read_ready_o = 1'b0;
        end
    end

    assign read_data_o = read_data_from_line_buffer_q? write_line_buffer: bank_rdata[read_data_from_bank_select_q];

endmodule : pseudo_dual_port_bank
