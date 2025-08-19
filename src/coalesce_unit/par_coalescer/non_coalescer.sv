// Copyright 2025 ETH Zurich and 
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 22.Mar.2023

//Non-Coalescing Unit for Baseline comparision

`include "common_cells/registers.svh"
`include "axi/assign.svh"
`include "axi/typedef.svh"

module non_coalescer #(
    /// upstream data width
    parameter int unsigned UpstreamDataWidth                = 64,
    /// downstream data width.
    parameter int unsigned DownstreamDataWidth              = 512,
    /// Number of narrow request ports.
    parameter int unsigned NumPorts                         = 16,
    /// address width
    parameter int unsigned AddrWidth                        = 32,
    /// Choose stratege to determine next tag addr
    parameter int unsigned USE_ORDER_PRIOR                  = 1,
    /// Dependent parameter, do not override. number of address offsets of one coalesce block.
    parameter int unsigned NumAddrOfst                      = DownstreamDataWidth/UpstreamDataWidth,
    /// Dependent parameter, do not override. Mem DW Align.
    parameter int unsigned PortAlign                        = $clog2(NumPorts),
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

    /// Upstream side
    input  addr_t [NumPorts-1:0]            upstream_addr_i,
    input  logic [NumPorts-1:0]             upstream_valid_i,
    output logic [NumPorts-1:0]             upstream_ready_o,

    /// Downstream side
    output  logic                           coal_valid_o,
    input   logic                           coal_ready_i,
    output  addr_t                          coal_addr_o,

    /// metadata fifo of valid port bitmap (strb)
    output  logic [NumPorts-1:0]            coal_strb_o,
    input   logic                           coal_strb_full_i,
    output  logic                           coal_strb_push_o,

    /// metadata fifos for every port
    output  addr_ofst_t [NumPorts-1:0]      coal_port_addr_ofst_o,
    input   logic       [NumPorts-1:0]      coal_port_addr_ofst_full_i,
    output  logic       [NumPorts-1:0]      coal_port_addr_ofst_push_o

);

	logic rr_req_valid;

	logic all_valid;

	logic [PortAlign-1:0] idx;

	addr_ofst_t [NumPorts-1:0] addr_ofst_of_port;

	addr_t [NumPorts-1:0] aligned_addr;

	//Calculate every tag address of ports--
	for (genvar i = 0; i < NumPorts; i++) begin: gen_tag
		assign aligned_addr[i] = ((upstream_addr_i[i])>>DownstreamDataAlign)<<DownstreamDataAlign;
	end

	//Calculate every slot index of ports--
	for (genvar i = 0; i < NumPorts; i++) begin: gen_slot_idx
		assign addr_ofst_of_port[i] = upstream_addr_i[i][DownstreamDataAlign-1:UpstreamDataAlign];
	end

	/*Round-robin artribution among all valid requests*/
	rr_arb_tree #(
	  .NumIn    ( NumPorts ),
	  .DataType ( addr_t   ),
	  .AxiVldRdy( 1'b1       ),
	  .LockIn   ( 1'b1       )
	) i_next_CSHR_addr_mux (
	  .clk_i,
	  .rst_ni,
	  .flush_i( 1'b0          ),
	  .rr_i   ( '0            ),
	  .req_i  ( upstream_valid_i ),
	  .gnt_o  ( /*open*/ ),
	  .data_i ( aligned_addr   ),
	  .gnt_i  ( coal_ready_i   ),
	  .req_o  ( rr_req_valid   ),
	  .data_o ( coal_addr_o  ),
	  .idx_o  ( idx )
	);

	assign all_valid = rr_req_valid & ~coal_strb_full_i & ~(coal_port_addr_ofst_full_i[idx]);

	
	always_comb begin
		upstream_ready_o = '0;
		coal_valid_o = '0;
		coal_strb_push_o = '0;
		coal_port_addr_ofst_push_o = '0;

		coal_strb_o = '0;
		coal_port_addr_ofst_o = '0;

		if (all_valid) begin
			coal_valid_o = 1;

			if (coal_valid_o & coal_ready_i) begin
				coal_strb_push_o = 1;
				for (int i = 0; i < NumPorts; i++) begin
					if (i == idx) begin
						upstream_ready_o[i] = 1;
						coal_port_addr_ofst_push_o[i] = 1;

						coal_strb_o[i] = 1;
						coal_port_addr_ofst_o[i] = addr_ofst_of_port[i];
					end
				end
			end
				
		end
	end


endmodule