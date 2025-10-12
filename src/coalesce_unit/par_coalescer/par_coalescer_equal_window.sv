// Copyright 2023 ETH Zurich and 
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 1.Mar.2024

`include "common_cells/registers.svh"
module par_coalescer_equal_window #(
    /// Address width of both upstream narrow request and downstream wide request
    parameter int unsigned ReqAddrWidth             = 32,
     /// Number of Upstream Ports
    parameter int unsigned NumPorts                 = 4,
    /// Information payload needed for each narrow data
    parameter type         info_t                   = logic,
    /// Id field needed to add in downstream information
    parameter type         down_id_t                = logic,
    /// Data width of upstream channel
    parameter int unsigned UpstreamDataWidth        = 32,
    /// Data width of downstream channel
    parameter int unsigned DownstreamDataWidth      = 512,
    /// Number of narrow request ports.
    parameter bit          SpliterSpillReg          = 0,
    // Dependent parameter, do not override. Depth of cache bank.
    localparam int unsigned NumWord                 = DownstreamDataWidth/UpstreamDataWidth,
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
    localparam type downstream_info_t               = struct packed {down_id_t id; logic [NumPorts-1:0] hitmap; offset_t [NumPorts-1:0] ofsts; info_t [NumPorts-1:0] infos; logic bypass_coalescer;}
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

    typedef logic [ReqAddrWidth:0]                  write_mixed_addr_t;
    typedef upstream_data_t [NumWord-1:0]           wdata_in_words_t;

    typedef struct packed {
        write_mixed_addr_t                          write_mixed_addr;
        logic       [NumPorts-1:0]                  hitmap;
        offset_t    [NumPorts-1:0]                  ofsts;
    } coal_req_t;

    typedef struct packed {
        logic                                       write;
        upstream_data_t                             data;
        info_t                                      info;
    } upstream_resp_t;

    //////////////////////////////////////
    //        Signal Definition         //
    //////////////////////////////////////

    write_mixed_addr_t [NumPorts-1:0]               write_mixed_addr;

    logic                                           coal_req_valid;
    logic                                           coal_req_ready;
    coal_req_t                                      coal_req;
    coal_req_t                                      coal_req_cut;

    info_t             [NumPorts-1:0]               buffer_req_info;
    upstream_data_t    [NumPorts-1:0]               buffer_req_wdata;


    logic              [NumPorts-1:0]               upstream_resp_valid;
    logic              [NumPorts-1:0]               upstream_resp_ready;
    logic              [NumPorts-1:0]               upstream_resp_write;
    upstream_data_t    [NumPorts-1:0]               upstream_resp_data;
    info_t             [NumPorts-1:0]               upstream_resp_info;

    upstream_resp_t    [NumPorts-1:0]               upstream_resp_spillin;
    upstream_resp_t    [NumPorts-1:0]               upstream_resp_spillout;

    //////////////////////////////////
    //        Instance Modules      //
    //////////////////////////////////

    /***************/
    /*  Req Phase  */
    /***************/

    for (genvar i = 0; i < NumPorts; i++) begin
        assign write_mixed_addr[i] = {upstream_req_write_i[i], upstream_req_addr_i[i]};
    end

    req_coalescer_v2 #(
        .UpstreamDataWidth         (UpstreamDataWidth),
        .DownstreamDataWidth       (DownstreamDataWidth),
        .NumPorts                  (NumPorts),
        .AddrWidth                 (ReqAddrWidth+1),
        .USE_ORDER_PRIOR           (0)
    ) i_req_coalescer (
        .clk_i,
        .rst_ni,

        .upstream_addr_i           (write_mixed_addr           ), 
        .upstream_valid_i          (upstream_req_valid_i       ), 
        .upstream_ready_o          (upstream_req_ready_o       ),

        .coal_valid_o              (coal_req_valid             ),
        .coal_ready_i              (coal_req_ready             ),
        .coal_addr_o               (coal_req.write_mixed_addr  ),

        .coal_strb_o               (coal_req.hitmap            ),
        .coal_strb_full_i          ('0                         ),
        .coal_strb_push_o          (/*open*/),

        .coal_port_addr_ofst_o     (coal_req.ofsts             ),
        .coal_port_addr_ofst_full_i('0                         ),
        .coal_port_addr_ofst_push_o(/*open*/)
    );

    spill_register #(
        .T                         (coal_req_t                 ),
        .Bypass                    ( 0                         )
        ) i_spill_down_resp  (
        .clk_i,
        .rst_ni,
        .valid_i                   (coal_req_valid             ),
        .ready_o                   (coal_req_ready             ),
        .data_i                    (coal_req                   ),
        .valid_o                   (downstream_req_valid_o     ),
        .ready_i                   (downstream_req_ready_i     ),
        .data_o                    (coal_req_cut               )
    );


    for (genvar i = 0; i < NumPorts; i++) begin

        fifo_v3 #(
            .FALL_THROUGH          (1'b0                       ),
            .DEPTH                 (4                          ),
            .dtype                 (info_t                     )
        ) i_req_info_fifo (
            .clk_i,
            .rst_ni,
            .flush_i               (1'b0                       ),
            .testmode_i            (1'b0                       ),
            .full_o                (/*open*/),
            .empty_o               (/*open*/),
            .usage_o               (/*open*/                   ),
            .data_i                (upstream_req_info_i[i]     ),
            .push_i                (upstream_req_valid_i[i] & upstream_req_ready_o[i]),
            .data_o                (buffer_req_info[i]         ),
            .pop_i                 (downstream_req_valid_o & downstream_req_ready_i & downstream_req_info_o.hitmap[i] )
        );

        fifo_v3 #(
            .FALL_THROUGH          (1'b0                       ),
            .DEPTH                 (4                          ),
            .dtype                 (upstream_data_t            )
        ) i_req_wdata_fifo (
            .clk_i,
            .rst_ni,
            .flush_i               (1'b0                       ),
            .testmode_i            (1'b0                       ),
            .full_o                (/*open*/),
            .empty_o               (/*open*/),
            .usage_o               (/*open*/                   ),
            .data_i                (upstream_req_wdata_i[i]    ),
            .push_i                (upstream_req_valid_i[i] & upstream_req_ready_o[i]),
            .data_o                (buffer_req_wdata[i]        ),
            .pop_i                 (downstream_req_valid_o & downstream_req_ready_i & downstream_req_info_o.hitmap[i] )
        );

    end

    always_comb begin : gen_down_req_data
        automatic wdata_in_words_t wdata = '0;
        downstream_req_wmask_o = '0;

        {downstream_req_write_o, downstream_req_addr_o} = coal_req_cut.write_mixed_addr;
        downstream_req_info_o.id = id_i;
        downstream_req_info_o.hitmap = coal_req_cut.hitmap;
        downstream_req_info_o.ofsts = coal_req_cut.ofsts;
        downstream_req_info_o.bypass_coalescer = 1'b0;

        for (int i = 0; i < NumPorts; i++) begin
            downstream_req_info_o.infos[i] = coal_req_cut.hitmap[i]? buffer_req_info[i] : '0;
            if (coal_req_cut.hitmap[i]) begin
                wdata[coal_req_cut.ofsts[i]] = buffer_req_wdata[i];
                downstream_req_wmask_o[coal_req_cut.ofsts[i]] = 1'b1;
            end
        end

        downstream_req_wdata_o = wdata;
    end





    /****************/
    /*  Resp Phase  */
    /****************/

    rsp_spliter_v2 #(
        .UpstreamDataWidth          (UpstreamDataWidth),
        .DownstreamDataWidth        (DownstreamDataWidth),
        .NumPorts                   (NumPorts)
    ) i_rsp_spliter (
        .clk_i,
        .rst_ni,
        .downstream_valid_i         (downstream_resp_valid_i         ),
        .downstream_ready_o         (downstream_resp_ready_o         ),
        .downstream_data_i          (downstream_resp_data_i          ),

        .rsp_ready_i                (upstream_resp_ready           ), 
        .rsp_valid_o                (upstream_resp_valid           ), 
        .rsp_data_o                 (upstream_resp_data            ), 

        .coal_strb_i                (downstream_resp_info_i.hitmap   ),
        .coal_strb_empty_i          ('0                              ),
        .coal_strb_pop_o            (/*open*/),

        .coal_port_addr_ofst_i      (downstream_resp_info_i.ofsts    ),
        .coal_port_addr_ofst_empty_i('0                              ),
        .coal_port_addr_ofst_pop_o  (/*open*/)
    );

    always_comb begin : gen_up_resp_data
        upstream_resp_info = downstream_resp_info_i.infos;
        upstream_resp_write = '0;
        for (int i = 0; i<NumPorts ; i++ ) begin
            upstream_resp_write[i] = downstream_resp_info_i.hitmap[i]? downstream_resp_write_i: '0;
        end
    end


    /*************************/
    /*  Resp Spill Register  */
    /*************************/

    for (genvar i = 0; i < NumPorts; i++) begin
        always_comb begin
            upstream_resp_spillin[i].write = upstream_resp_write[i];
            upstream_resp_spillin[i].data  = upstream_resp_data[i];
            upstream_resp_spillin[i].info  = upstream_resp_info[i];

            upstream_resp_write_o[i] = upstream_resp_spillout[i].write;
            upstream_resp_data_o[i]  = upstream_resp_spillout[i].data;
            upstream_resp_info_o[i]  = upstream_resp_spillout[i].info;
        end
    end


    //Spill register
    for (genvar i = 0; i < NumPorts; i++) begin
        logic fifo_full, fifo_empty;
        assign upstream_resp_valid_o[i] = ~fifo_empty;
        assign upstream_resp_ready[i] = ~fifo_full;

        fifo_v3 #(
            .FALL_THROUGH          (1'b0                       ),
            .DEPTH                 (4                          ),
            .dtype                 (upstream_resp_t            )
        ) i_resp_fifo (
            .clk_i,
            .rst_ni,
            .flush_i               (1'b0                       ),
            .testmode_i            (1'b0                       ),
            .full_o                (fifo_full),
            .empty_o               (fifo_empty),
            .usage_o               (/*open*/                   ),
            .data_i                (upstream_resp_spillin[i]   ),
            .push_i                (upstream_resp_valid[i] & upstream_resp_ready[i]),
            .data_o                (upstream_resp_spillout[i]  ),
            .pop_i                 (upstream_resp_valid_o[i] & upstream_resp_ready_i[i])
        );
    end

       

endmodule : par_coalescer_equal_window