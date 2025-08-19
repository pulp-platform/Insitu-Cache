// Copyright 2025 ETH Zurich and 
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 28.Feb.2024

`ifndef _CACHE_ASSIGN_SVH_
`define _CACHE_ASSIGN_SVH_

`define AXI_AW_ASSIGN_FROM(_req, axi_aw, BUS_ALIGN) \
	assign axi_aw.id = _req.info; \
	assign axi_aw.addr = _req.addr; \
	assign axi_aw.len = '0; \
	assign axi_aw.size = BUS_ALIGN; \
	assign axi_aw.burst = axi_pkg::BURST_INCR; \
	assign axi_aw.lock = '0; \
	assign axi_aw.cache = '0; \
	assign axi_aw.prot = '0; \
	assign axi_aw.qos = '0; \
	assign axi_aw.region = '0; \
	assign axi_aw.atop = '0; \
	assign axi_aw.user = '0;

`define AXI_AR_ASSIGN_FROM(_req, axi_ar, BUS_ALIGN) \
	assign axi_ar.id = _req.info; \
	assign axi_ar.addr = _req.addr; \
	assign axi_ar.len = '0; \
	assign axi_ar.size = BUS_ALIGN; \
	assign axi_ar.burst = axi_pkg::BURST_INCR; \
	assign axi_ar.lock = '0; \
	assign axi_ar.cache = '0; \
	assign axi_ar.prot = '0; \
	assign axi_ar.qos = '0; \
	assign axi_ar.region = '0; \
	assign axi_ar.user = '0;

`define AXI_AR_ASSIGN_FROM(_req, axi_ar, BUS_ALIGN) \
	assign axi_ar.id = _req.info; \
	assign axi_ar.addr = _req.addr; \
	assign axi_ar.len = '0; \
	assign axi_ar.size = BUS_ALIGN; \
	assign axi_ar.burst = axi_pkg::BURST_INCR; \
	assign axi_ar.lock = '0; \
	assign axi_ar.cache = '0; \
	assign axi_ar.prot = '0; \
	assign axi_ar.qos = '0; \
	assign axi_ar.region = '0; \
	assign axi_ar.user = '0;

`define AXI_W_ASSIGN_FROM(_req, axi_w, BUS_ALIGN) \
	assign axi_w.data = _req.wdata; \
	assign axi_w.strb = _req.wstrb; \
	assign axi_w.last = 1'b1; \
	assign axi_w.user = '0; 

`define AXI_R_ASSIGN_FROM(_resp, axi_r, BUS_ALIGN) \
	assign axi_r.id   = _resp.info; \
	assign axi_r.data = _resp.data; \
	assign axi_r.resp = '0; \
	assign axi_r.last = 1'b1; \
	assign axi_r.user = '0; 

`define AXI_B_ASSIGN_FROM(_resp, axi_b, BUS_ALIGN) \
	assign axi_b.id   = _resp.info; \
	assign axi_b.resp = '0; \
	assign axi_b.user = '0; 

`define AXI_R_ASSIGN_TO(_resp, axi_r, BUS_ALIGN) \
	assign _resp.write = '0; \
	assign _resp.info = axi_r.id; \
	assign _resp.data = axi_r.data;


`define AXI_B_ASSIGN_TO(_resp, axi_b, BUS_ALIGN) \
	assign _resp.write = 1'b1; \
	assign _resp.data = '0; \
	assign _resp.info = axi_b.id; 

`endif