// Copyright 2023 ETH Zurich and 
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 5.Oct.2024


`include "common_cells/registers.svh"
module par_coalescer_top #(
    /// Address width of both upstream narrow request and downstream wide request
    parameter int unsigned ReqAddrWidth             = 32,
    /// Number of Upstream Ports
    parameter int unsigned NumPorts                 = 4,
    /// Extned Factor
    parameter int unsigned ExtFactor                = 1,
    /// Information payload needed for each narrow data
    parameter type         info_t                   = logic,
    /// Id field needed to add in downstream information
    parameter type         down_id_t                = logic,
    /// Data width of upstream channel
    parameter int unsigned UpstreamDataWidth        = 32,
    /// Data width of downstream channel
    parameter int unsigned DownstreamDataWidth      = 512,
    // Dependent parameter, do not override. Depth of cache bank.
    localparam int unsigned NumWord                 = DownstreamDataWidth/UpstreamDataWidth,
    // Dependent parameter, do not override. Depth of cache bank.
    localparam int unsigned ExtPorts                = NumPorts * ExtFactor,
    // Dependent parameter, do not override. Address type.
    localparam type addr_t                          = logic [ReqAddrWidth-1:0],
    // Dependent parameter, do not override. Narrow word type.
    localparam type upstream_data_t                 = logic [UpstreamDataWidth-1:0],
    // Dependent parameter, do not override. Wide word type.
    localparam type downstream_data_t               = logic [DownstreamDataWidth-1:0],
    // Dependent parameter, do not override. Word mask type.
    localparam type mask_t                          = logic [DownstreamDataWidth/UpstreamDataWidth-1:0],
    // Dependent parameter, do not override. byte offset type.
    localparam type offset_t                        = logic [$clog2(DownstreamDataWidth/UpstreamDataWidth)-1:0],
    // Dependent parameter, do not override. Downstream request payload.
    localparam type downstream_info_t               = struct packed {down_id_t id; logic [ExtPorts-1:0] hitmap; offset_t [ExtPorts-1:0] ofsts; info_t [ExtPorts-1:0] infos;}
    )(
    /// Clock, positive edge triggered.
    input  logic                                    clk_i,
    /// Reset, active low.
    input  logic                                    rst_ni,

    /// ID
    input  down_id_t                                id_i,

    /// Upstream request
    input  logic            [NumPorts-1:0]          upstream_req_valid_i,
    output logic            [NumPorts-1:0]          upstream_req_ready_o,
    input  addr_t           [NumPorts-1:0]          upstream_req_addr_i,
    input  info_t           [NumPorts-1:0]          upstream_req_info_i,
    input  logic            [NumPorts-1:0]          upstream_req_write_i,
    input  upstream_data_t  [NumPorts-1:0]          upstream_req_wdata_i,

    /// Upstream response
    output logic            [NumPorts-1:0]          upstream_resp_valid_o,
    input  logic            [NumPorts-1:0]          upstream_resp_ready_i,
    output logic            [NumPorts-1:0]          upstream_resp_write_o,
    output upstream_data_t  [NumPorts-1:0]          upstream_resp_data_o,
    output info_t           [NumPorts-1:0]          upstream_resp_info_o,

    /// Downstream request
    output logic                                    downstream_req_valid_o,
    input  logic                                    downstream_req_ready_i,
    output addr_t                                   downstream_req_addr_o,
    output downstream_info_t                        downstream_req_info_o,
    output logic                                    downstream_req_write_o,
    output downstream_data_t                        downstream_req_wdata_o,
    output mask_t                                   downstream_req_wmask_o,

    /// Downsteam response
    input  logic                                    downstream_resp_valid_i,
    output logic                                    downstream_resp_ready_o,
    input  downstream_data_t                        downstream_resp_data_i,
    input  downstream_info_t                        downstream_resp_info_i,
    input  logic                                    downstream_resp_write_i
 
);

if (ExtFactor > 1) begin : gen_extend_window
    par_coalescer_extend_window #(
        .ReqAddrWidth       (ReqAddrWidth),
        .NumPorts           (NumPorts),
        .ExtFactor          (ExtFactor),
        .info_t             (info_t),
        .down_id_t          (down_id_t),
        .UpstreamDataWidth  (UpstreamDataWidth),
        .DownstreamDataWidth(DownstreamDataWidth)
    ) i_par_coalescer_extend_window (
        .clk_i,
        .rst_ni,
        .id_i,
        .upstream_req_valid_i,
        .upstream_req_ready_o,
        .upstream_req_addr_i,
        .upstream_req_info_i,
        .upstream_req_write_i,
        .upstream_req_wdata_i,
        .upstream_resp_valid_o,
        .upstream_resp_ready_i,
        .upstream_resp_write_o,
        .upstream_resp_data_o,
        .upstream_resp_info_o,
        .downstream_req_valid_o,
        .downstream_req_ready_i,
        .downstream_req_addr_o,
        .downstream_req_info_o,
        .downstream_req_write_o,
        .downstream_req_wdata_o,
        .downstream_req_wmask_o,
        .downstream_resp_valid_i,
        .downstream_resp_ready_o,
        .downstream_resp_data_i,
        .downstream_resp_info_i,
        .downstream_resp_write_i
    );
end else begin : gen_equal_window
    par_coalescer_equal_window #(
        .ReqAddrWidth       (ReqAddrWidth),
        .NumPorts           (NumPorts),
        .info_t             (info_t),
        .down_id_t          (down_id_t),
        .UpstreamDataWidth  (UpstreamDataWidth),
        .DownstreamDataWidth(DownstreamDataWidth)
    ) i_par_coalescer_equal_window (
        .clk_i,
        .rst_ni,
        .id_i,
        .upstream_req_valid_i,
        .upstream_req_ready_o,
        .upstream_req_addr_i,
        .upstream_req_info_i,
        .upstream_req_write_i,
        .upstream_req_wdata_i,
        .upstream_resp_valid_o,
        .upstream_resp_ready_i,
        .upstream_resp_write_o,
        .upstream_resp_data_o,
        .upstream_resp_info_o,
        .downstream_req_valid_o,
        .downstream_req_ready_i,
        .downstream_req_addr_o,
        .downstream_req_info_o,
        .downstream_req_write_o,
        .downstream_req_wdata_o,
        .downstream_req_wmask_o,
        .downstream_resp_valid_i,
        .downstream_resp_ready_o,
        .downstream_resp_data_i,
        .downstream_resp_info_i,
        .downstream_resp_write_i
    );
end

endmodule : par_coalescer_top