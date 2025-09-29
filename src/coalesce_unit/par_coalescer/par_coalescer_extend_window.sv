// Copyright 2023 ETH Zurich and 
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 5.Oct.2024


`include "common_cells/registers.svh"
module par_coalescer_extend_window #(
    /// Address width of both upstream narrow request and downstream wide request
    parameter int unsigned ReqAddrWidth             = 32,
    /// Number of Upstream Ports
    parameter int unsigned NumPorts                 = 4,
    /// Extned Factor
    parameter int unsigned ExtFactor                = 2,
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

     //////////////////////////////////////
    //        Types Definition          //
    //////////////////////////////////////

    typedef logic [$clog2(ExtFactor)-1:0]           port_select_t;
    

    //////////////////////////////////////
    //        Signal Definition         //
    //////////////////////////////////////

    logic            [NumPorts-1:0][ExtFactor-1:0]  extend_req_valid;
    logic            [NumPorts-1:0][ExtFactor-1:0]  extend_req_ready;
    addr_t           [NumPorts-1:0][ExtFactor-1:0]  extend_req_addr;
    info_t           [NumPorts-1:0][ExtFactor-1:0]  extend_req_info;
    logic            [NumPorts-1:0][ExtFactor-1:0]  extend_req_write;
    upstream_data_t  [NumPorts-1:0][ExtFactor-1:0]  extend_req_wdata;

    /// Upstream response
    logic            [NumPorts-1:0][ExtFactor-1:0]  extend_resp_valid;
    logic            [NumPorts-1:0][ExtFactor-1:0]  extend_resp_ready;
    logic            [NumPorts-1:0][ExtFactor-1:0]  extend_resp_write;
    upstream_data_t  [NumPorts-1:0][ExtFactor-1:0]  extend_resp_data;
    info_t           [NumPorts-1:0][ExtFactor-1:0]  extend_resp_info;


    port_select_t    [NumPorts-1:0]                 req_port_select_q, req_port_select_d;
    port_select_t    [NumPorts-1:0]                 resp_port_select;
    `FFARN (req_port_select_q, req_port_select_d,   '0, clk_i, rst_ni)


    //////////////////////////////
    //        Port arbitor      //
    //////////////////////////////

    always_comb begin : proc_req_arbiter
        //Defualt value
        extend_req_valid    = '0;
        upstream_req_ready_o= '0;
        extend_req_addr     = '0;
        extend_req_info     = '0;
        extend_req_write    = '0;
        extend_req_wdata    = '0;
        req_port_select_d   = req_port_select_q;

        for (int i = 0; i < NumPorts; i++) begin
            //Control path
            extend_req_valid[i][req_port_select_q[i]] = upstream_req_valid_i[i];
            upstream_req_ready_o[i] = extend_req_ready[i][req_port_select_q[i]];
            if (upstream_req_valid_i[i] & upstream_req_ready_o[i]) begin
                req_port_select_d[i] = req_port_select_q[i] + 1'b1;
            end

            //Data path
            extend_req_addr[i][req_port_select_q[i]]  = upstream_req_addr_i[i];
            extend_req_info[i][req_port_select_q[i]]  = upstream_req_info_i[i];
            extend_req_write[i][req_port_select_q[i]] = upstream_req_write_i[i];
            extend_req_wdata[i][req_port_select_q[i]] = upstream_req_wdata_i[i];
        end
    end


    for (genvar i = 0; i < NumPorts; i++) begin : gen_resp_arbiters
        rr_arb_tree #(
          .NumIn      (ExtFactor),
          .DataType   (logic),
          .ExtPrio    (1'b0),
          .AxiVldRdy  (1'b1),
          .LockIn     (1'b1)
        ) i_resp_arbiter (
          .clk_i,
          .rst_ni,
          .flush_i('0),
          .rr_i   ('0),
          .req_i  (extend_resp_valid[i]),
          .gnt_o  (extend_resp_ready[i]),
          .data_i ('0),
          .req_o  (upstream_resp_valid_o[i]),
          .gnt_i  (upstream_resp_ready_i[i]),
          .data_o (/*open*/),
          .idx_o  (resp_port_select[i])
        );

        always_comb begin : forward_resp
            //Data path
            upstream_resp_write_o[i] = extend_resp_write[i][resp_port_select[i]];
            upstream_resp_data_o[i] = extend_resp_data[i][resp_port_select[i]];
            upstream_resp_info_o[i] = extend_resp_info[i][resp_port_select[i]];
        end
    end

    

    //////////////////////////////////
    //        Instance Modules      //
    //////////////////////////////////

    par_coalescer_equal_window #(
        .ReqAddrWidth       (ReqAddrWidth),
        .NumPorts           (ExtPorts),
        .info_t             (info_t),
        .down_id_t          (down_id_t),
        .UpstreamDataWidth  (UpstreamDataWidth),
        .DownstreamDataWidth(DownstreamDataWidth),
        .SpliterSpillReg    (0)
    ) i_par_coalescer (
        .clk_i,
        .rst_ni,
        .id_i,

        .upstream_req_valid_i   (extend_req_valid     ),
        .upstream_req_ready_o   (extend_req_ready     ),
        .upstream_req_addr_i    (extend_req_addr      ),
        .upstream_req_info_i    (extend_req_info      ),
        .upstream_req_write_i   (extend_req_write     ),
        .upstream_req_wdata_i   (extend_req_wdata     ),

        .upstream_resp_valid_o  (extend_resp_valid    ),
        .upstream_resp_ready_i  (extend_resp_ready    ),
        .upstream_resp_write_o  (extend_resp_write    ),
        .upstream_resp_data_o   (extend_resp_data     ),
        .upstream_resp_info_o   (extend_resp_info     ),

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

endmodule : par_coalescer_extend_window