// Copyright 2023 ETH Zurich and
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 21.Dec.2023

// Insitu-Cache top-level
// Limitation: 1. Upstream Datawidth = Downstream Datawidth = Cache line data width

`include "common_cells/registers.svh"
module insitu_cache_top #(
    /// Address width of both upstream narrow request and downstream wide request
    parameter int unsigned ReqAddrWidth             = 32,
    /// Information payload needed for each narrow data
    parameter type         info_t                   = logic,
    /// Word width of cache line (512b default)
    parameter int unsigned CacheLineWidth           = 512,
    /// Number of Cache entries
    parameter int unsigned NumCacheEntry            = 512,
    /// Number of Associatity
    parameter int unsigned SetAssociativity         = 16,
    /// Number of parts per cache line for data banks (1 = unfolded).
    parameter int unsigned DataPartSplit            = 1,
    /// If Use Dual-Port RF, Default No!
    parameter bit          UseDualPortRF            = 0,
    /// If Use Pseudo-Dual Banks, Default yes!
    parameter bit          UsePseudoDualBanks       = 1,
    /// Number of Pseudo-Dual Banks
    parameter int unsigned NumPseudoDualBanks       = 8,
    /// Width of word (granularity of non-blocking write)
    parameter int unsigned WordWidth                = 64,
    /// Width of byte (granularity of byte mask)
    parameter int unsigned ByteWidth                = 8,
    /// Log Debug information for questa-sim.
    parameter int unsigned LogDebug                 = 1,
    /// Counter cache line life cycle information for questa-sim.
    parameter int unsigned LogLifeCycle             = 0,
    /// Depth of Write Through Fifo.
    parameter int unsigned WriteThroughFifoDepth    = 4,
    /// Depth of Write Info Fifo.
    parameter int unsigned WRespFifoDepth           = 4,
    /// Depth of Retrieve Fifo.
    parameter int unsigned RetrFifoDepth            = 4,
    /// Depth of Response Fifo.
    parameter int unsigned RespFifoDepth            = 4,
    /// Depth of Miss Fifo.
    parameter int unsigned MissFifoDepth            = 4,
    /// Depth of Eviction Fifo.
    parameter int unsigned EvicFifoDepth            = 4,
    /// Address Hashing Field Length.
    parameter int unsigned AddrHashLength           = 0,
    /// Whether the cache is in Write-Through mode
    /// Otherwise the cache is defualtly in Write-Back mode
    parameter bit          WriteThroughMode         = 0,
`ifndef TARGET_SYNTHESIS
    /// Name the cache
    parameter string       ModeleName               = "none",
`endif
    /// Word width of narrow data to upstream
    parameter int unsigned UpstreamWidth            = CacheLineWidth,
    /// Word width of wide data from downsteam
    parameter int unsigned DownstreamWidth          = CacheLineWidth,
    /// SRAM Configuration
    parameter type impl_in_t                        = logic,
    // Dependent parameter, do not override. Part index width.
    localparam int unsigned PartIdxWidth            = (DataPartSplit > 1) ? $clog2(DataPartSplit) : 1,
    // Dependent parameter, do not override. Depth of cache bank.
    localparam int unsigned CacheBankDepth          = NumCacheEntry/SetAssociativity,
    // Dependent parameter, do not override. set ptr type.
    localparam type way_ptr_t                       = logic [$clog2(SetAssociativity)-1:0],
    // Dependent parameter, do not override. Address type.
    localparam type addr_t                          = logic [ReqAddrWidth-1:0],
    // Dependent parameter, do not override. Narrow word type.
    localparam type upstream_data_t                 = logic [UpstreamWidth-1:0],
    // Dependent parameter, do not override. Wide word type.
    localparam type downstream_data_t               = logic [DownstreamWidth-1:0],
    // Dependent parameter, do not override. Wide word type.
    localparam type cache_data_t                    = logic [DownstreamWidth-1:0],
    // Dependent parameter, do not override. Byte mask type.
    localparam type cache_mask_t                    = logic [DownstreamWidth/ByteWidth-1:0],
    // Dependent parameter, do not override. tag type.
    localparam type cache_tag_t                     = logic [ReqAddrWidth-$clog2(DownstreamWidth/8)-$clog2(CacheBankDepth)-1:0],
    // Dependent parameter, do not override. bank depth ptr type.
    localparam type cache_bank_depth_ptr_t          = logic [$clog2(CacheBankDepth)-1:0],
    // Dependent parameter, do not override. Downstream request payload.
    localparam type downstream_info_t               = struct packed {logic for_write_pend; cache_bank_depth_ptr_t depth; way_ptr_t way;},
    // Dependent parameter, do not override. Downstream request payload.
    localparam type miss_meta_t                     = struct packed {logic is_full; logic is_prime; logic link_enable; way_ptr_t link_ptr;},
    // Dependent parameter, do not override. Cache line status.
    localparam type cache_status_t                  = enum logic[1:0] { INVALID = '0, VALID, READ_PEND, WRITE_PEND }
    )(
    /// Clock, positive edge triggered.
    input  logic                                    clk_i,
    /// Reset, active low.
    input  logic                                    rst_ni,
    /// SRAM Configuration
    input  impl_in_t [NumPseudoDualBanks-1:0]       impl_i,

    /// Upstream request
    input  logic                                    upstream_req_valid_i,
    output logic                                    upstream_req_ready_o,
    input  addr_t                                   upstream_req_addr_i,
    input  info_t                                   upstream_req_info_i,
    input  logic                                    upstream_req_write_i,
    input  upstream_data_t                          upstream_req_wdata_i,
    input  cache_mask_t                             upstream_req_wmask_i,

    /// Upstream response
    output logic                                    upstream_resp_valid_o,
    input  logic                                    upstream_resp_ready_i,
    output logic                                    upstream_resp_write_o,
    output upstream_data_t                          upstream_resp_data_o,
    output info_t                                   upstream_resp_info_o,

    /// Downstream request
    output logic                                    downstream_req_valid_o,
    input  logic                                    downstream_req_ready_i,
    output addr_t                                   downstream_req_addr_o,
    output downstream_info_t                        downstream_req_info_o,
    output logic                                    downstream_req_write_o,
    output downstream_data_t                        downstream_req_wdata_o,
    output cache_mask_t                             downstream_req_wmask_o,

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

    typedef logic [$bits(cache_status_t) + 1 + $bits(miss_meta_t) + $bits(cache_mask_t) + $bits(cache_tag_t) + $bits(way_ptr_t) - 1 : 0] cache_meta_t;

    typedef struct packed {
        cache_data_t                                        data;
        info_t                                              info;
        logic                                               write;
    } cache_resp_t;

    typedef struct packed {
        addr_t                                              addr;
        info_t                                              info;
        logic                                               write;
        upstream_data_t                                     wdata;
        cache_mask_t                                        wmask;
    } up_req_t;

    typedef struct packed {
        addr_t                                              addr;
        downstream_info_t                                   info;
        logic                                               write;
        downstream_data_t                                   wdata;
        cache_mask_t                                        wmask;
    } down_req_t;

    //////////////////////////////////////
    //        Signal Definition         //
    //////////////////////////////////////

    /*********************************/
    /*  WriteThrough Related Signals */
    /*********************************/

    logic                                                   upstream_req_to_cache_valid;
    logic                                                   upstream_req_to_cache_ready;
    up_req_t                                                upstream_req_to_cache_payload;

    logic                                                   write_through_valid;
    logic                                                   write_through_ready;
    down_req_t                                              write_through_req_payload;

    /**************************/
    /*  Cache Write Response  */
    /**************************/

    info_t                                                  winfo_fifo_in;
    logic                                                   winfo_fifo_full;
    logic                                                   winfo_fifo_push;
    info_t                                                  winfo_fifo_out;
    logic                                                   winfo_fifo_empty;
    logic                                                   winfo_fifo_pop;
    logic                                                   wresp_valid;
    logic                                                   wresp_ready;
    cache_resp_t                                            wresp_in;


    /*************************/
    /*  Cache Read Response  */
    /*************************/

    logic                                                   core_resp_valid;
    logic                                                   core_resp_ready;
    cache_data_t                                            core_resp_data;
    info_t                                                  core_resp_info;
    cache_resp_t                                            rresp_in;
    cache_resp_t                                            resp_out;

    /********************/
    /*  Cache Miss Req  */
    /********************/

    logic                                                   core_miss_valid;
    logic                                                   core_miss_ready;
    addr_t                                                  core_miss_addr;
    downstream_info_t                                       core_miss_info;
    down_req_t                                              miss_req_payload;

    /********************/
    /*  Cache Evic Req  */
    /********************/

    logic                                                   core_evic_valid;
    logic                                                   core_evic_ready;
    addr_t                                                  core_evic_addr;
    downstream_data_t                                       core_evic_data;
    cache_mask_t                                            core_evic_mask;
    down_req_t                                              evic_req_payload;
    down_req_t                                              down_req_payload;

    /***********************/
    /*  Cache refill Resp  */
    /***********************/

    logic                                                   core_refill_valid;
    logic                                                   core_refill_ready;
    downstream_data_t                                       core_refill_data;
    downstream_info_t                                       core_refill_info;


    /*****************/
    /*  Cache Banks  */
    /*****************/


    cache_meta_t            [SetAssociativity - 1 : 0]      cache_meta_read_data;
    cache_meta_t            [SetAssociativity - 1 : 0]      cache_meta_write_data;


    cache_bank_depth_ptr_t                                  bank_read_cache_addr;
    logic [PartIdxWidth-1:0]                                bank_read_part_idx;
    logic                                                   bank_read_all_parts;
    logic                                                   bank_read_cache_valid;
    logic                                                   bank_read_cache_ready;
    logic                   [SetAssociativity - 1 : 0]     bank_read_way_mask;
    logic                   [SetAssociativity - 1 : 0]      bank_read_cache_ready_per_way;
    cache_status_t          [SetAssociativity - 1 : 0]      bank_read_cache_status;
    logic                   [SetAssociativity - 1 : 0]      bank_read_cache_dirty;
    miss_meta_t             [SetAssociativity - 1 : 0]      bank_read_cache_miss_meta;
    cache_mask_t            [SetAssociativity - 1 : 0]      bank_read_cache_mask;
    cache_tag_t             [SetAssociativity - 1 : 0]      bank_read_cache_tag;
    cache_data_t            [SetAssociativity - 1 : 0]      bank_read_cache_data;
    way_ptr_t               [SetAssociativity - 1 : 0]      bank_read_cache_LRU;


    cache_bank_depth_ptr_t                                  bank_write_cache_addr;
    logic                                                   bank_write_cache_req;
    cache_status_t          [SetAssociativity - 1 : 0]      bank_write_cache_status;
    logic                   [SetAssociativity - 1 : 0]      bank_write_cache_dirty;
    miss_meta_t             [SetAssociativity - 1 : 0]      bank_write_cache_miss_meta;
    cache_mask_t            [SetAssociativity - 1 : 0]      bank_write_cache_mask;
    cache_tag_t             [SetAssociativity - 1 : 0]      bank_write_cache_tag;
    cache_data_t            [SetAssociativity - 1 : 0]      bank_write_cache_data;
    cache_mask_t            [SetAssociativity - 1 : 0]      bank_write_data_mask;
    logic                                                   bank_write_LRU_req;
    way_ptr_t               [SetAssociativity - 1 : 0]      bank_write_cache_LRU;

    /////////////////////////////////////
    //        Instance Modules         //
    /////////////////////////////////////

    /***************************************/
    /*  Write-through / Write-back Logics  */
    /***************************************/

    if (WriteThroughMode) begin

        up_req_t                                req_fifo_wt_in;
        logic                                   req_fifo_wt_full;
        logic                                   req_fifo_wt_push;
        up_req_t                                req_fifo_wt_out;
        logic                                   req_fifo_wt_empty;
        logic                                   req_fifo_wt_pop;

        logic                                   write_through_merger_valid;
        logic                                   write_through_merger_ready;

        logic                                   write_through_valid_cut0;
        logic                                   write_through_ready_cut0;
        down_req_t                              write_through_req_payload_cut0;

        fifo_v3 #(
            .FALL_THROUGH                       (1'b0                       ),
            .DEPTH                              (WriteThroughFifoDepth      ),
            .dtype                              (up_req_t                   )
        ) i_cache_req_fifo_wt (
            .clk_i,
            .rst_ni,
            .flush_i                            (1'b0                       ),
            .testmode_i                         (1'b0                       ),
            .full_o                             (req_fifo_wt_full           ),
            .empty_o                            (req_fifo_wt_empty          ),
            .usage_o                            (/*open*/                   ),
            .data_i                             (req_fifo_wt_in             ),
            .push_i                             (req_fifo_wt_push           ),
            .data_o                             (req_fifo_wt_out            ),
            .pop_i                              (req_fifo_wt_pop            )
        );

        write_through_merger #(
            .ReqAddrWidth                       (ReqAddrWidth),
            .info_t                             (info_t),
            .downstream_info_t                  (downstream_info_t),
            .CacheLineWidth                     (CacheLineWidth),
            .WordWidth                          (WordWidth),
            .ByteWidth                          (ByteWidth)
        ) i_write_through_merger (
            .clk_i,
            .rst_ni,
            .upstream_req_valid_i  (write_through_merger_valid  ),
            .upstream_req_ready_o  (write_through_merger_ready  ),
            .upstream_req_addr_i   (cache_addr_hashing(
                                        upstream_req_addr_i,
                                        $clog2(DownstreamWidth/8),
                                        $clog2(CacheBankDepth) + $clog2(DownstreamWidth/8),
                                        AddrHashLength
                                    )),
            .upstream_req_wdata_i,
            .upstream_req_wmask_i,

            .mon_read_handshaked_i (upstream_req_valid_i & upstream_req_ready_o & ~upstream_req_write_i),
            .mon_read_addr_i       (cache_addr_hashing(
                                        upstream_req_addr_i,
                                        $clog2(DownstreamWidth/8),
                                        $clog2(CacheBankDepth) + $clog2(DownstreamWidth/8),
                                        AddrHashLength
                                    )),

            .downstream_req_valid_o(write_through_valid_cut0),
            .downstream_req_ready_i(write_through_ready_cut0),
            .downstream_req_addr_o (write_through_req_payload_cut0.addr ),
            .downstream_req_info_o (write_through_req_payload_cut0.info ),
            .downstream_req_write_o(write_through_req_payload_cut0.write),
            .downstream_req_wdata_o(write_through_req_payload_cut0.wdata),
            .downstream_req_wmask_o(write_through_req_payload_cut0.wmask)
        );


        spill_register #(.T(down_req_t), .Bypass('0)) i_write_through_spill_register (
            .clk_i,
            .rst_ni,
            .valid_i(write_through_valid_cut0       ),
            .ready_o(write_through_ready_cut0       ),
            .data_i (write_through_req_payload_cut0 ),
            .valid_o(write_through_valid            ),
            .ready_i(write_through_ready            ),
            .data_o (write_through_req_payload      )
        );


        //datapath
        assign upstream_req_to_cache_payload = req_fifo_wt_out;

        assign req_fifo_wt_in = '{
            addr:  cache_addr_hashing(
                        upstream_req_addr_i,
                        $clog2(DownstreamWidth/8),
                        $clog2(CacheBankDepth) + $clog2(DownstreamWidth/8),
                        AddrHashLength
                    ),
            info:  upstream_req_info_i,
            write: upstream_req_write_i,
            wdata: upstream_req_wdata_i,
            wmask: upstream_req_wmask_i
        };

        assign winfo_fifo_in = upstream_req_info_i;

        //control logic
        assign upstream_req_to_cache_valid = ~req_fifo_wt_empty;
        assign req_fifo_wt_pop = upstream_req_to_cache_valid & upstream_req_to_cache_ready;

        assign write_through_merger_valid = upstream_req_write_i & upstream_req_valid_i & ~req_fifo_wt_full & ~winfo_fifo_full;

        assign upstream_req_ready_o =   upstream_req_write_i?
                                        write_through_merger_valid & write_through_merger_ready:
                                        upstream_req_valid_i & ~req_fifo_wt_full;

        assign req_fifo_wt_push =       upstream_req_valid_i & upstream_req_ready_o;

        assign winfo_fifo_push =        ~winfo_fifo_full & upstream_req_valid_i & upstream_req_ready_o & upstream_req_write_i;

    end else begin
        assign upstream_req_to_cache_valid =    upstream_req_write_i?
                                                ~winfo_fifo_full & upstream_req_valid_i:
                                                upstream_req_valid_i;
        assign upstream_req_ready_o        =    upstream_req_to_cache_valid & upstream_req_to_cache_ready;

        assign winfo_fifo_push             =    ~winfo_fifo_full & upstream_req_valid_i & upstream_req_ready_o & upstream_req_write_i;
        assign winfo_fifo_in               =    upstream_req_info_i;

        assign write_through_req_payload   =    '0;
        assign write_through_valid         =    '0;

        assign upstream_req_to_cache_payload.addr = cache_addr_hashing(
                                                        upstream_req_addr_i,
                                                        $clog2(DownstreamWidth/8),
                                                        $clog2(CacheBankDepth) + $clog2(DownstreamWidth/8),
                                                        AddrHashLength
                                                    );
        assign upstream_req_to_cache_payload.info = upstream_req_info_i;
        assign upstream_req_to_cache_payload.write = upstream_req_write_i;
        assign upstream_req_to_cache_payload.wdata = upstream_req_wdata_i;
        assign upstream_req_to_cache_payload.wmask = upstream_req_wmask_i;
    end

    /****************/
    /*  Cache Core  */
    /****************/

    insitu_cache_core #(
        .ReqAddrWidth    (ReqAddrWidth),
        .CacheLineWidth  (CacheLineWidth),
        .info_t          (info_t),
        .NumCacheEntry   (NumCacheEntry),
        .SetAssociativity(SetAssociativity),
        .DataPartSplit   (DataPartSplit),
        .WordWidth       (WordWidth),
        .ByteWidth       (ByteWidth),
        .LogDebug        (LogDebug),
        .LogLifeCycle    (LogLifeCycle),
        .RespFifoDepth   (RespFifoDepth),
        .RetrFifoDepth   (RetrFifoDepth),
        .MissFifoDepth   (MissFifoDepth),
        .EvicFifoDepth   (EvicFifoDepth),
        .WriteThroughMode(WriteThroughMode),
`ifndef TARGET_SYNTHESIS
        .ModeleName      (ModeleName),
`endif
        .ShowDebug       (0)
    ) i_insitu_cache_core (
        .clk_i,
        .rst_ni,
        .has_pend_line_o                (/*open*/),

        .upstream_req_valid_i           (upstream_req_to_cache_valid),
        .upstream_req_ready_o           (upstream_req_to_cache_ready),
        .upstream_req_addr_i            (upstream_req_to_cache_payload.addr),
        .upstream_req_info_i            (upstream_req_to_cache_payload.info),
        .upstream_req_write_i           (upstream_req_to_cache_payload.write),
        .upstream_req_wdata_i           (upstream_req_to_cache_payload.wdata),
        .upstream_req_wmask_i           (upstream_req_to_cache_payload.wmask),

        .upstream_resp_valid_o          (core_resp_valid    ),
        .upstream_resp_ready_i          (core_resp_ready    ),
        .upstream_resp_data_o           (core_resp_data     ),
        .upstream_resp_info_o           (core_resp_info     ),

        .downstream_req_evic_valid_o    (core_evic_valid    ),
        .downstream_req_evic_ready_i    (core_evic_ready | WriteThroughMode   ),
        .downstream_req_evic_addr_o     (core_evic_addr     ),
        .downstream_req_evic_data_o     (core_evic_data     ),
        .downstream_req_evic_mask_o     (core_evic_mask     ),


        .downstream_req_miss_valid_o    (core_miss_valid    ),
        .downstream_req_miss_ready_i    (core_miss_ready    ),
        .downstream_req_miss_addr_o     (core_miss_addr     ),
        .downstream_req_miss_info_o     (core_miss_info     ),

        .downstream_resp_refill_valid_i (core_refill_valid  ),
        .downstream_resp_refill_ready_o (core_refill_ready  ),
        .downstream_resp_refill_info_i  (core_refill_info   ),
        .downstream_resp_refill_data_i  (core_refill_data   ),

        //Bank
        .bank_read_addr_o               (bank_read_cache_addr),
        .bank_read_part_idx_o           (bank_read_part_idx),
        .bank_read_all_parts_o          (bank_read_all_parts),
        .bank_read_valid_o              (bank_read_cache_valid),
        .bank_read_ready_i              (bank_read_cache_ready),
        .bank_read_way_mask_o           (bank_read_way_mask),
        .bank_read_cache_status_i       (bank_read_cache_status),
        .bank_read_cache_dirty_i        (bank_read_cache_dirty),
        .bank_read_cache_miss_meta_i    (bank_read_cache_miss_meta),
        .bank_read_cache_mask_i         (bank_read_cache_mask),
        .bank_read_cache_tag_i          (bank_read_cache_tag),
        .bank_read_cache_data_i         (bank_read_cache_data),
        .bank_read_cache_LRU_i          (bank_read_cache_LRU),

        .bank_write_req_o               (bank_write_cache_req),
        .bank_write_addr_o              (bank_write_cache_addr),
        .bank_write_way_o               (/*open*/),
        .bank_write_cache_status_o      (bank_write_cache_status),
        .bank_write_cache_dirty_o       (bank_write_cache_dirty),
        .bank_write_cache_miss_meta_o   (bank_write_cache_miss_meta),
        .bank_write_cache_mask_o        (bank_write_cache_mask),
        .bank_write_cache_tag_o         (bank_write_cache_tag),
        .bank_write_cache_data_o        (bank_write_cache_data),
        .bank_write_data_mask_o         (bank_write_data_mask),
        .bank_write_LRU_req_o           (bank_write_LRU_req),
        .bank_write_cache_LRU_o         (bank_write_cache_LRU)
    );

    /***************************/
    /*  Cache Response Logics  */
    /***************************/

    //Cache response write info Fifo
    fifo_v3 #(
        .FALL_THROUGH                       (1'b0                       ),
        .DEPTH                              (WRespFifoDepth             ),
        .dtype                              (info_t                     )
    ) i_cache_winfo_fifo (
        .clk_i,
        .rst_ni,
        .flush_i                            (1'b0                       ),
        .testmode_i                         (1'b0                       ),
        .full_o                             (winfo_fifo_full            ),
        .empty_o                            (winfo_fifo_empty           ),
        .usage_o                            (/*open*/                   ),
        .data_i                             (winfo_fifo_in              ),
        .push_i                             (winfo_fifo_push            ),
        .data_o                             (winfo_fifo_out             ),
        .pop_i                              (winfo_fifo_pop             )
    );

    assign wresp_valid = ~winfo_fifo_empty;
    assign winfo_fifo_pop = wresp_valid & wresp_ready;
    assign wresp_in = '{
        data: '0,
        info: winfo_fifo_out,
        write: 1'b1
    };

    assign rresp_in = '{
        data: core_resp_data,
        info: core_resp_info,
        write: 1'b0
    };

    stream_arbiter #(.DATA_T(cache_resp_t), .N_INP(2)) i_cache_resp_arbiter (
        .clk_i,
        .rst_ni,
        .inp_data_i ({wresp_in,    rresp_in}),
        .inp_valid_i({wresp_valid, core_resp_valid}),
        .inp_ready_o({wresp_ready, core_resp_ready}),
        .oup_data_o (resp_out),
        .oup_valid_o(upstream_resp_valid_o),
        .oup_ready_i(upstream_resp_ready_i)
    );

    assign {upstream_resp_data_o, upstream_resp_info_o, upstream_resp_write_o} = resp_out;


    /*************************/
    /*  Miss & Evic Logics  */
    /************************/

    assign miss_req_payload = '{
        addr:  core_miss_addr,
        info:  core_miss_info,
        write: 1'b0,
        wdata: '0,
        wmask: '0
    };

    assign evic_req_payload = '{
        addr:  core_evic_addr,
        info:  '0,
        write: 1'b1,
        wdata: core_evic_data,
        wmask: core_evic_mask
    };

    stream_arbiter #(.DATA_T(down_req_t), .N_INP(3)) i_cache_miss_evic_arbiter (
        .clk_i,
        .rst_ni,
        .inp_data_i ({miss_req_payload,     evic_req_payload                    ,   write_through_req_payload}),
        .inp_valid_i({core_miss_valid,      core_evic_valid & ~ WriteThroughMode,   write_through_valid}),
        .inp_ready_o({core_miss_ready,      core_evic_ready                     ,   write_through_ready}),
        .oup_data_o (down_req_payload),
        .oup_valid_o(downstream_req_valid_o),
        .oup_ready_i(downstream_req_ready_i)
    );

    assign downstream_req_addr_o = cache_addr_hashing(
                                        down_req_payload.addr,
                                        $clog2(DownstreamWidth/8),
                                        $clog2(CacheBankDepth) + $clog2(DownstreamWidth/8),
                                        AddrHashLength
                                    );
    assign downstream_req_info_o = down_req_payload.info;
    assign downstream_req_write_o = down_req_payload.write;
    assign downstream_req_wdata_o = down_req_payload.wdata;
    assign downstream_req_wmask_o = down_req_payload.wmask;

    /*******************/
    /*  Refill logics  */
    /*******************/

    assign core_refill_data = downstream_resp_data_i;
    assign core_refill_info = downstream_resp_info_i;
    assign core_refill_valid = downstream_resp_valid_i & ~downstream_resp_write_i;
    assign downstream_resp_ready_o = downstream_resp_write_i? 1'b1: core_refill_ready;


    /*****************/
    /*  Cache Banks  */
    /*****************/

    for (genvar i = 0; i < SetAssociativity; i++) begin: gen_cache_banks

        logic __data_bank_read_ready;
        logic __meta_bank_read_ready;

        always_comb begin
            {bank_read_cache_status[i],
            bank_read_cache_dirty[i],
            bank_read_cache_miss_meta[i],
            bank_read_cache_mask[i],
            bank_read_cache_tag[i],
            bank_read_cache_LRU[i]} = cache_meta_read_data[i];

            cache_meta_write_data[i] = {
                bank_write_cache_status[i],
                bank_write_cache_dirty[i],
                bank_write_cache_miss_meta[i],
                bank_write_cache_mask[i],
                bank_write_cache_tag[i],
                bank_write_cache_LRU[i]
            };
        end

        /******************************/
        /* Use Pesudo Dual-Port Banks */
        /******************************/
        pseudo_dual_port_way #(.DEPTH(CacheBankDepth),.BANKS(NumPseudoDualBanks),.meta_t(cache_meta_t),.data_t(cache_data_t),.impl_in_t(impl_in_t)) i_cache_way (
            .clk_i,
            .rst_ni,
            .impl_i           (impl_i),

            .read_addr_i(bank_read_cache_addr),
            .read_valid_i(bank_read_cache_valid & bank_read_way_mask[i]),
            .write_addr_i(bank_write_cache_addr),

            .meta_read_ready_o(__meta_bank_read_ready),
            .meta_read_data_o(cache_meta_read_data[i]),
            .meta_write_req_i(bank_write_cache_req | bank_write_LRU_req),
            .meta_write_data_i(cache_meta_write_data[i]),

            .data_read_ready_o(__data_bank_read_ready),
            .data_read_data_o(bank_read_cache_data[i]),
            .data_write_req_i(bank_write_cache_req),
            .data_write_data_i(bank_write_cache_data[i])
        );


        assign bank_read_cache_ready_per_way[i] = bank_read_way_mask[i] ?
                                                   (__data_bank_read_ready & __meta_bank_read_ready) :
                                                   1'b1;
    end

    assign bank_read_cache_ready = &bank_read_cache_ready_per_way;

endmodule
