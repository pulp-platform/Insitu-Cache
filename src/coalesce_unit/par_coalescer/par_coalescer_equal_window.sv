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
    /// Width of strb (byte enable) for each word
    parameter int unsigned ByteWidth                = 8,
    /// Number of narrow request ports.
    parameter bit          SpliterSpillReg          = 0,
    // Dependent parameter, do not override. Depth of cache bank.
    localparam int unsigned NumWord                 = DownstreamDataWidth/UpstreamDataWidth,
    // Dependent parameter, do not override. Number of bytes per upstream word.
    localparam int unsigned WordBytes               = UpstreamDataWidth/ByteWidth,
    // Dependent parameter, do not override. Address type.
    localparam type addr_t                          = logic [ReqAddrWidth-1:0],
    // Dependent parameter, do not override. Narrow word type.
    localparam type upstream_data_t                 = logic [UpstreamDataWidth-1:0],
    // Dependent parameter, do not override. byte strobe type.
    localparam type upstream_strb_t                 = logic [UpstreamDataWidth/ByteWidth-1:0],
    // Dependent parameter, do not override. Wide word type.
    localparam type downstream_data_t               = logic [DownstreamDataWidth-1:0],
    // Dependent parameter, do not override. Byte mask type.
    localparam type mask_t                          = logic [DownstreamDataWidth/ByteWidth-1:0],
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
    input  upstream_strb_t  [NumPorts-1:0]          upstream_req_wstrb_i,

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

    typedef struct packed {
        downstream_data_t                           data;
        downstream_info_t                           info;
        logic                                       write;
    } downstream_resp_t;

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
    upstream_strb_t    [NumPorts-1:0]               buffer_req_wstrb;

    logic              [NumPorts-1:0]               req_info_full;
    logic              [NumPorts-1:0]               req_wdata_full;
    logic              [NumPorts-1:0]               req_wstrb_full;
    logic              [NumPorts-1:0]               req_fifo_full;
    logic              [NumPorts-1:0]               upstream_req_valid_gated;
    logic              [NumPorts-1:0]               upstream_req_ready_raw;


    logic              [NumPorts-1:0]               upstream_resp_valid;
    logic              [NumPorts-1:0]               upstream_resp_ready;
    logic              [NumPorts-1:0]               upstream_resp_write;
    upstream_data_t    [NumPorts-1:0]               upstream_resp_data;
    info_t             [NumPorts-1:0]               upstream_resp_info;

    upstream_resp_t    [NumPorts-1:0]               upstream_resp_spillin;
    upstream_resp_t    [NumPorts-1:0]               upstream_resp_spillout;

    downstream_resp_t                               downstream_resp_bundle_in;
    downstream_resp_t                               downstream_resp_bundle;
    logic                                           downstream_resp_valid;
    logic                                           downstream_resp_ready;

    //////////////////////////////////
    //        Instance Modules      //
    //////////////////////////////////

    /***************/
    /*  Req Phase  */
    /***************/

    for (genvar i = 0; i < NumPorts; i++) begin
        assign write_mixed_addr[i] = {upstream_req_write_i[i], upstream_req_addr_i[i]};
    end

    assign req_fifo_full = req_info_full | req_wdata_full | req_wstrb_full;
    assign upstream_req_valid_gated = upstream_req_valid_i & ~req_fifo_full;
    assign upstream_req_ready_o = upstream_req_ready_raw & ~req_fifo_full;

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
        .upstream_valid_i          (upstream_req_valid_gated   ),
        .upstream_ready_o          (upstream_req_ready_raw     ),

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
            .full_o                (req_info_full[i]           ),
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
            .full_o                (req_wdata_full[i]          ),
            .empty_o               (/*open*/),
            .usage_o               (/*open*/                   ),
            .data_i                (upstream_req_wdata_i[i]    ),
            .push_i                (upstream_req_valid_i[i] & upstream_req_ready_o[i]),
            .data_o                (buffer_req_wdata[i]        ),
            .pop_i                 (downstream_req_valid_o & downstream_req_ready_i & downstream_req_info_o.hitmap[i] )
        );

        fifo_v3 #(
            .FALL_THROUGH          (1'b0                       ),
            .DEPTH                 (4                          ),
            .dtype                 (upstream_strb_t            )
        ) i_req_wstrb_fifo (
            .clk_i,
            .rst_ni,
            .flush_i               (1'b0                       ),
            .testmode_i            (1'b0                       ),
            .full_o                (req_wstrb_full[i]          ),
            .empty_o               (/*open*/),
            .usage_o               (/*open*/                   ),
            .data_i                (upstream_req_wstrb_i[i]    ),
            .push_i                (upstream_req_valid_i[i] & upstream_req_ready_o[i]),
            .data_o                (buffer_req_wstrb[i]        ),
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

        // Higher index ports override earlier bytes on overlap.
        for (int i = 0; i < NumPorts; i++) begin
            downstream_req_info_o.infos[i] = coal_req_cut.hitmap[i]? buffer_req_info[i] : '0;
            if (coal_req_cut.hitmap[i]) begin
                automatic int unsigned word_index;
                automatic upstream_data_t wdata_word;
                automatic upstream_data_t req_word;
                word_index = coal_req_cut.ofsts[i];
                if (downstream_req_write_o) begin
                    wdata_word = wdata[word_index];
                    req_word = buffer_req_wdata[i];
                    for (int b = 0; b < WordBytes; b++) begin
                        if (buffer_req_wstrb[i][b]) begin
                            wdata_word[b * ByteWidth +: ByteWidth] = req_word[b * ByteWidth +: ByteWidth];
                            downstream_req_wmask_o[word_index * WordBytes + b] = 1'b1;
                        end
                    end
                    wdata[word_index] = wdata_word;
                end else begin
                    wdata[word_index] = buffer_req_wdata[i];
                end
            end
        end

        downstream_req_wdata_o = wdata;
    end





    /****************/
    /*  Resp Phase  */
    /****************/

    always_comb begin
        downstream_resp_bundle_in.data = downstream_resp_data_i;
        downstream_resp_bundle_in.info = downstream_resp_info_i;
        downstream_resp_bundle_in.write = downstream_resp_write_i;
    end

    spill_register #(
        .T                         (downstream_resp_t          ),
        .Bypass                    (~SpliterSpillReg          )
    ) i_spill_downstream_resp (
        .clk_i,
        .rst_ni,
        .valid_i                   (downstream_resp_valid_i    ),
        .ready_o                   (downstream_resp_ready_o    ),
        .data_i                    (downstream_resp_bundle_in  ),
        .valid_o                   (downstream_resp_valid      ),
        .ready_i                   (downstream_resp_ready      ),
        .data_o                    (downstream_resp_bundle     )
    );

    rsp_spliter_v2 #(
        .UpstreamDataWidth          (UpstreamDataWidth),
        .DownstreamDataWidth        (DownstreamDataWidth),
        .NumPorts                   (NumPorts)
    ) i_rsp_spliter (
        .clk_i,
        .rst_ni,
        .downstream_valid_i         (downstream_resp_valid           ),
        .downstream_ready_o         (downstream_resp_ready           ),
        .downstream_data_i          (downstream_resp_bundle.data     ),

        .rsp_ready_i                (upstream_resp_ready           ), 
        .rsp_valid_o                (upstream_resp_valid           ), 
        .rsp_data_o                 (upstream_resp_data            ), 

        .coal_strb_i                (downstream_resp_bundle.info.hitmap),
        .coal_strb_empty_i          ('0                              ),
        .coal_strb_pop_o            (/*open*/),

        .coal_port_addr_ofst_i      (downstream_resp_bundle.info.ofsts),
        .coal_port_addr_ofst_empty_i('0                              ),
        .coal_port_addr_ofst_pop_o  (/*open*/)
    );

    always_comb begin : gen_up_resp_data
        upstream_resp_info = downstream_resp_bundle.info.infos;
        upstream_resp_write = '0;
        for (int i = 0; i<NumPorts ; i++ ) begin
            upstream_resp_write[i] = downstream_resp_bundle.info.hitmap[i] ? downstream_resp_bundle.write : '0;
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


    // ====================================================================
    // Coalescer / splitter scoreboard  (passive observer)
    // ====================================================================
    // Verifies the end-to-end per-port data path of the rsp_spliter +
    // per-port spill FIFO.  For every cycle the splitter writes to a
    // per-port spill FIFO (= upstream_resp_valid[p] & upstream_resp_ready[p]
    // internal handshake), we enqueue the expected data slice in a model
    // queue.  When the per-port output to upstream fires
    // (= upstream_resp_valid_o[p] & upstream_resp_ready_i[p]), we pop the
    // queue head and compare against the actual data delivered.
    //
    // Catches: splitter mis-routing (wrong ofst), per-port FIFO data
    // corruption, write/read direction mismatch on the per-port output.
    // ====================================================================
`ifndef TARGET_SYNTHESIS
    // Per-port queue of expected response data (one entry per push to the
    // per-port spill FIFO).  Bounded by the FIFO's depth (4) at steady state.
    upstream_data_t              sb_exp_data [NumPorts][$];
    logic                        sb_exp_write[NumPorts][$];

    longint unsigned n_pp_pushes   [NumPorts];
    longint unsigned n_pp_pops     [NumPorts];
    longint unsigned n_pp_data_mm  [NumPorts];
    longint unsigned n_pp_write_mm [NumPorts];
    longint unsigned n_pp_underflow[NumPorts];

    initial begin
        for (int p = 0; p < NumPorts; p++) begin
            n_pp_pushes[p]    = 0;
            n_pp_pops[p]      = 0;
            n_pp_data_mm[p]   = 0;
            n_pp_write_mm[p]  = 0;
            n_pp_underflow[p] = 0;
        end
    end

    for (genvar p = 0; p < NumPorts; p++) begin : gen_coal_sb_per_port
        always @(posedge clk_i) begin
            if (!rst_ni) begin
                while (sb_exp_data[p].size() > 0) begin
                    void'(sb_exp_data[p].pop_front());
                    void'(sb_exp_write[p].pop_front());
                end
            end else begin
                // -- PUSH: splitter wrote to per-port spill FIFO --
                if (upstream_resp_valid[p] && upstream_resp_ready[p]) begin
                    automatic int unsigned ofst;
                    automatic upstream_data_t expected;
                    ofst = downstream_resp_bundle.info.ofsts[p];
                    expected = downstream_resp_bundle.data[ofst*UpstreamDataWidth +: UpstreamDataWidth];
                    sb_exp_data[p].push_back(expected);
                    sb_exp_write[p].push_back(downstream_resp_bundle.write);
                    n_pp_pushes[p] = n_pp_pushes[p] + 1;
                end
                // -- POP + CHECK: per-port external rsp fired --
                if (upstream_resp_valid_o[p] && upstream_resp_ready_i[p]) begin
                    n_pp_pops[p] = n_pp_pops[p] + 1;
                    if (sb_exp_data[p].size() == 0) begin
                        n_pp_underflow[p] = n_pp_underflow[p] + 1;
                        $error("[COAL-SB %m] port[%0d] UNDERFLOW t=%0t  external rsp fired (write=%0b data=0x%0h) but no expected entry pending in SB queue",
                               $time, p, upstream_resp_write_o[p], upstream_resp_data_o[p]);
                    end else begin
                        automatic upstream_data_t expected = sb_exp_data[p].pop_front();
                        automatic logic           expected_write = sb_exp_write[p].pop_front();
                        // Write/read direction sanity
                        if (upstream_resp_write_o[p] !== expected_write) begin
                            n_pp_write_mm[p] = n_pp_write_mm[p] + 1;
                            $error("[COAL-SB %m] port[%0d] WRITE/READ MISMATCH t=%0t  expected_write=%0b  actual_write=%0b",
                                   $time, p, expected_write, upstream_resp_write_o[p]);
                        end
                        // Data check (READ only)
                        if (!expected_write && (upstream_resp_data_o[p] !== expected)) begin
                            n_pp_data_mm[p] = n_pp_data_mm[p] + 1;
                            $error("[COAL-SB %m] port[%0d] SPLIT/FIFO DATA MISMATCH t=%0t  expected=0x%0h  actual=0x%0h",
                                   $time, p, expected, upstream_resp_data_o[p]);
                        end
                    end
                end
            end
        end
    end

    final begin
        automatic longint unsigned total_data_mm   = 0;
        automatic longint unsigned total_write_mm  = 0;
        automatic longint unsigned total_underflow = 0;
        automatic longint unsigned total_viol;
        automatic bit              coal_sb_verbose = $test$plusargs("sb_verbose");
        for (int p = 0; p < NumPorts; p++) begin
            total_data_mm   += n_pp_data_mm[p];
            total_write_mm  += n_pp_write_mm[p];
            total_underflow += n_pp_underflow[p];
        end
        total_viol = total_data_mm + total_write_mm + total_underflow;
        // -- Verbose summary: only on FAIL or +sb_verbose --
        if (total_viol != 0 || coal_sb_verbose) begin
            $display("[COAL-SB %m] ============================= Coalescer Scoreboard =============================");
            for (int p = 0; p < NumPorts; p++) begin
                $display("[COAL-SB %m]   port[%0d]: pushes=%0d  pops=%0d  data_mismatch=%0d  write_mismatch=%0d  underflow=%0d  pending_in_sb_q=%0d",
                         p, n_pp_pushes[p], n_pp_pops[p],
                         n_pp_data_mm[p], n_pp_write_mm[p], n_pp_underflow[p],
                         sb_exp_data[p].size());
            end
            $display("[COAL-SB %m] ================================================================================");
        end
        // -- Always print a brief one-line STATUS --
        if (total_viol == 0)
            $display("[COAL-SB %m] STATUS: PASS");
        else
            $display("[COAL-SB %m] STATUS: FAIL (data_mm=%0d  write_mm=%0d  underflow=%0d)",
                     total_data_mm, total_write_mm, total_underflow);
    end
`endif

endmodule : par_coalescer_equal_window
