// Copyright 2023 ETH Zurich and 
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 21.Dec.2023

// Insitu-Cache tcdm wrapper
//  TCDM organization (for data banks):
//                    |<-   SetAssociativity   ->|
//         |<- NumPseudoDualBanks ->|<- NumPseudoDualBanks ->|
//      ---       Bank     Bank            Bank     Bank
//       ^        Bank     Bank            Bank     Bank
//       |        Bank     Bank            Bank     Bank
//    Word Per    Bank     Bank            Bank     Bank
//   Cache Line   Bank     Bank            Bank     Bank
//       |        Bank     Bank            Bank     Bank
//       v        Bank     Bank            Bank     Bank
//      ---       Bank     Bank            Bank     Bank
//
//  TCDM organization (for meta banks):
//                    |<-   SetAssociativity   ->|
//         |<- NumPseudoDualBanks ->|<- NumPseudoDualBanks ->|
//      ---       Bank     Bank            Bank     Bank


`include "common_cells/registers.svh"
module insitu_cache_tcdm_wrapper
  import insitu_cache_pkg::*;
  #(
    /// Address width of both upstream narrow request and downstream wide request
    parameter int unsigned ReqAddrWidth             = 32,
    /// Information payload needed for each narrow data
    parameter type         info_t                   = logic,
    /// Word width of cache line (512b default)
    parameter int unsigned CacheLineWidth           = 512,
    /// Number of Cache entries
    parameter int unsigned NumCacheEntry            = 512,
    /// Number of Associatity
    parameter int unsigned SetAssociativity         = 2,
    /// Number of Pseudo-Dual Banks
    parameter int unsigned NumPseudoDualBanks       = 1,
    /// Number of parts per cache line for data banks (1 = unfolded).
    parameter int unsigned DataPartSplit            = 1,
    /// Use hash-based way selection (1 way per lookup, no LRU).
    parameter bit          UseHashWaySelect         = 1'b0,
    /// Width of word (granularity of non-blocking write)
    parameter int unsigned WordWidth                = 32,
    /// Width of byte (granularity of byte mask)
    parameter int unsigned ByteWidth                = 8,
    /// Tag Width
    parameter int unsigned TagWidth                 = 64,
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
    /// Forwarding-buffer RAW-forward.  When 1, a same-cycle read on the
    /// data-side fwd-buffer that coincides with a buffer-absorbing write
    /// to the same line returns the post-write merged data instead of
    /// the pre-write value.  Adds a wide byte-mask mux from wr_data into
    /// the buffer's response register.  Default 0 keeps the original
    /// read-before-write semantics.  Applies to data-side buffer only;
    /// meta-side has PartSplit=1 and rarely sees same-cycle R+W to the
    /// same line, so we leave it tied off.
    parameter bit          DataFwdBufEnableRawForwarding = 1'b1,
    parameter bit          MetaFwdBufEnableRawForwarding = 1'b1,
    /// Enable the SRAM forwarding buffer on the data/meta access controllers.
    /// Default 1 (production).  Set 0 for the unfolded "conventional" cache
    /// (LRU way-select, UseHashWaySelect=0), which is incompatible with the
    /// buffer (see the elaboration guard below).
    parameter bit          UseForwardingBuffer           = 1'b1,
`ifndef TARGET_SYNTHESIS
    /// Name the cache
    parameter              ModeleName               = "none",
`endif
    /// Word width of narrow data to upstream
    parameter int unsigned UpstreamWidth            = CacheLineWidth,
    /// Word width of wide data from downsteam
    parameter int unsigned DownstreamWidth          = CacheLineWidth,
    // Dependent parameter, do not override. Depth of cache bank.
    localparam int unsigned CacheBankDepth          = NumCacheEntry/SetAssociativity,
    // Dependent parameter, do not override. Number of data bank per way.
    localparam int unsigned NumDataBankPerWay       = NumPseudoDualBanks * (CacheLineWidth/WordWidth),
    // Dependent parameter, do not override. Number of parts in a cache line.
    localparam int unsigned PartSplit               = (DataPartSplit == 0) ? 1 : DataPartSplit,
    // Dependent parameter, do not override. Number of words per part.
    localparam int unsigned PartWords               = (CacheLineWidth/WordWidth) / PartSplit,
    // Dependent parameter, do not override. Part index width.
    localparam int unsigned PartIdxWidth            = (PartSplit > 1) ? $clog2(PartSplit) : 1,
    // Dependent parameter, do not override. Number of meta bank per way.
    localparam int unsigned NumMetaBankPerWay       = NumPseudoDualBanks,
    // Dependent parameter, do not override. Address type.
    localparam type tcdm_bank_addr_t                = logic [$clog2(CacheBankDepth)-$clog2(NumPseudoDualBanks)-1:0],
    /// Dependent parameter, do not override. word type
    localparam type word_t                          = logic [WordWidth-1:0],
    /// Dependent parameter, do not override. word type
    localparam type tcdm_meta_data_t                = logic [TagWidth-1:0],
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
    // Dependent parameter, do not override. Byte offset type.
    localparam type byte_offset_t                   = logic [$clog2(CacheLineWidth/8)-1:0],
    // Dependent parameter, do not override. Downstream request payload.
    localparam type downstream_info_t               = struct packed {logic for_write_pend; cache_bank_depth_ptr_t depth; way_ptr_t way;},
    // Dependent parameter, do not override. Downstream request payload.
    localparam type miss_meta_t                     = struct packed {logic is_full; logic is_prime; logic link_enable; way_ptr_t link_ptr;}
    )(
    /// Clock, positive edge triggered.
    input  logic                                    clk_i,
    /// Reset, active low.
    input  logic                                    rst_ni,

    /// Sync Control Signals
    input  logic                                    cache_sync_valid_i,
    output logic                                    cache_sync_ready_o,
    /*  0-> flush+invalidation
        1-> flush only
        2-> invalidation only*/
    input  logic [1:0]                              cache_sync_insn_i,

    /// Partition Base for Cache
    input cache_bank_depth_ptr_t                    cache_part_base_i,

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
    input  logic                                    downstream_resp_write_i,

    /// Meta Banks
    output logic             [SetAssociativity-1:0][NumMetaBankPerWay-1:0]   tcdm_meta_bank_req_o,
    output logic             [SetAssociativity-1:0][NumMetaBankPerWay-1:0]   tcdm_meta_bank_we_o,
    output tcdm_bank_addr_t  [SetAssociativity-1:0][NumMetaBankPerWay-1:0]   tcdm_meta_bank_addr_o,
    output tcdm_meta_data_t  [SetAssociativity-1:0][NumMetaBankPerWay-1:0]   tcdm_meta_bank_wdata_o,
    output logic             [SetAssociativity-1:0][NumMetaBankPerWay-1:0]   tcdm_meta_bank_be_o,
    input  tcdm_meta_data_t  [SetAssociativity-1:0][NumMetaBankPerWay-1:0]   tcdm_meta_bank_rdata_i,

    /// Data Banks
    output logic             [SetAssociativity-1:0][NumDataBankPerWay-1:0]   tcdm_data_bank_req_o,
    output logic             [SetAssociativity-1:0][NumDataBankPerWay-1:0]   tcdm_data_bank_we_o,
    output tcdm_bank_addr_t  [SetAssociativity-1:0][NumDataBankPerWay-1:0]   tcdm_data_bank_addr_o,
    output word_t            [SetAssociativity-1:0][NumDataBankPerWay-1:0]   tcdm_data_bank_wdata_o,
    output logic             [SetAssociativity-1:0][NumDataBankPerWay-1:0][WordWidth/ByteWidth-1:0] tcdm_data_bank_be_o,
    input  word_t            [SetAssociativity-1:0][NumDataBankPerWay-1:0]   tcdm_data_bank_rdata_i,

    /// Data Bank Request GNT for Cache
    input  logic             [SetAssociativity-1:0][NumDataBankPerWay-1:0]   tcdm_data_bank_gnt_i

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

    // Guard against truncating cache metadata in tag/meta banks.
`ifndef SYNTHESIS
    initial begin
        if (TagWidth < $bits(cache_meta_t)) begin
            $error("TagWidth (%0d) is smaller than cache_meta_t (%0d); update L1D_TAG_DATA_WIDTH for byte masks.",
                   TagWidth, $bits(cache_meta_t));
        end
    end
    initial begin
        if (((CacheLineWidth/WordWidth) % PartSplit) != 0) begin
            $fatal(1, "PartSplit (%0d) must divide NumWordsPerLine (%0d).",
                   PartSplit, (CacheLineWidth/WordWidth));
        end
    end
    // When the forwarding buffer is enabled, UseHashWaySelect MUST be 1.
    // With UseHashWaySelect=0 the cache_core sets bank_read_way_mask to
    // all-ones (insitu_cache_core.sv "{SetAssociativity{1'b1}}" fallback),
    // every way drives the SRAM port simultaneously, and the tile-level
    // skewed-fold arbiter (cachepool_tile.sv: gen_folded_data_banks)
    // collapses the requests onto a single partition slot per column.
    // The buffer masks this initially by serving reads from buf_data_q,
    // but at the first eviction the SRAM read returns the wrong row's
    // data and the dirty install is lost.  See
    // reports/MULTI_TILE_BUG_ROOT_CAUSE.md for the full trace.
    // Two independent reasons UseHashWaySelect=0 is unsafe:
    //   (a) Forwarding buffer on: at the first eviction the all-ways-active
    //       SRAM read returns the wrong row -> dirty install lost.
    //   (b) Skewed-fold banks (PartSplit > 1): the tile arbiter collapses the
    //       all-ways read onto one partition slot per column and cannot
    //       disambiguate ways -- corrupt even with the buffer off.
    // So UseHashWaySelect=0 is only legal for the UNFOLDED (PartSplit==1) cache
    // with the forwarding buffer disabled (the conventional LRU config).
    initial begin
        if ((PartSplit > 1) && !UseHashWaySelect) begin
            $fatal(1, "[insitu_cache_tcdm_wrapper %m] Skewed-fold (DataPartSplit=%0d) requires UseHashWaySelect=1: with all-ways-active reads the tile fold arbiter cannot disambiguate ways.  Set UseHashWaySelect=1, or use the unfolded (DataPartSplit=1) cache.", DataPartSplit);
        end
        if (UseForwardingBuffer && !UseHashWaySelect) begin
            $fatal(1, "[insitu_cache_tcdm_wrapper %m] Forwarding buffer is enabled but UseHashWaySelect=0.  This combination silently corrupts dirty buffer data on eviction in multi-way + skewed-fold builds.  Set UseHashWaySelect=1 (the cachepool_cluster.sv default -- multi-tile configs need it propagated through cachepool_group.sv) or disable the forwarding buffer (UseForwardingBuffer=0).");
        end
    end
`endif

    typedef enum logic[2:0] {
        SYNC_CTRL_IDLE = '0,
        SYNC_CTRL_READ_BANK,
        SYNC_CTRL_INIT,
        SYNC_CTRL_CHECK_PEND,
        SYNC_CTRL_FLUSH,
        SYNC_CTRL_INVALID,
        SYNC_CTRL_FINISH
    } cache_sync_ctrl_status_t;

    //////////////////////////////////////
    //        Signal Definition         //
    //////////////////////////////////////

    /*********************************/
    /*  WriteThrough Related Signals */
    /*********************************/

    logic                                                   upstream_req_to_cache_valid;
    logic                                                   upstream_req_to_cache_ready;
    up_req_t                                                upstream_req_to_cache_payload;

    up_req_t                                                req_fifo_wt_in;
    logic                                                   req_fifo_wt_full;
    logic                                                   req_fifo_wt_push;
    up_req_t                                                req_fifo_wt_out;
    logic                                                   req_fifo_wt_empty;
    logic                                                   req_fifo_wt_pop;

    logic                                                   write_through_merger_valid;
    logic                                                   write_through_merger_ready;

    logic                                                   write_through_valid_cut0;
    logic                                                   write_through_ready_cut0;
    down_req_t                                              write_through_req_payload_cut0;

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
    logic                                                   resp_mux_lock_q;
    logic                                                   resp_mux_lock_d;
    logic                                                   resp_mux_sel_q;
    logic                                                   resp_mux_sel_d;
    logic                                                   resp_mux_use_core;
    logic                                                   resp_mux_use_wresp;
    logic                                                   resp_mux_valid;

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
    logic          [PartIdxWidth-1:0]                       bank_read_part_idx;
    logic                                                   bank_read_all_parts;
    logic                                                   bank_read_cache_valid;
    logic                                                   bank_read_cache_ready;
    logic                   [SetAssociativity - 1 : 0]     bank_read_way_mask;
    logic                   [SetAssociativity - 1 : 0]     bank_read_way_mask_sel;
    logic          [PartIdxWidth-1:0]                       bank_read_part_idx_sel;
    logic                                                   bank_read_all_parts_sel;
    logic                   [SetAssociativity - 1 : 0]      bank_read_cache_ready_per_way;
    cache_status_t          [SetAssociativity - 1 : 0]      bank_read_cache_status;
    logic                   [SetAssociativity - 1 : 0]      bank_read_cache_dirty;
    miss_meta_t             [SetAssociativity - 1 : 0]      bank_read_cache_miss_meta;
    cache_mask_t            [SetAssociativity - 1 : 0]      bank_read_cache_mask;
    cache_tag_t             [SetAssociativity - 1 : 0]      bank_read_cache_tag;
    cache_data_t            [SetAssociativity - 1 : 0]      bank_read_cache_data;
    way_ptr_t               [SetAssociativity - 1 : 0]      bank_read_cache_LRU;

    logic                   [SetAssociativity - 1 : 0]      data_bank_read_ready;
    logic                   [SetAssociativity - 1 : 0]      meta_bank_read_ready;
    // Per-way upstream_write_ready_o (Phase 1 handshake; always-1 in Phase 1).
    logic                   [SetAssociativity - 1 : 0]      data_bank_write_ready;
    logic                   [SetAssociativity - 1 : 0]      meta_bank_write_ready;
    logic                   [SetAssociativity - 1 : 0]      data_bank_write_hit;
    logic                   [SetAssociativity - 1 : 0]      data_bank_write_full_cov;

    cache_meta_t            [SetAssociativity - 1 : 0][NumMetaBankPerWay-1:0] tcdm_meta_rdata_int;
    cache_meta_t            [SetAssociativity - 1 : 0][NumMetaBankPerWay-1:0] tcdm_meta_wdata_int;

    cache_bank_depth_ptr_t  [SetAssociativity - 1 : 0]      gnt_data_bank_read_addr;
    logic                   [SetAssociativity - 1 : 0]      gnt_data_bank_read_valid;
    logic                   [SetAssociativity - 1 : 0]      gnt_data_bank_read_ready;
    cache_data_t            [SetAssociativity - 1 : 0]      gnt_data_bank_read_data;

    cache_bank_depth_ptr_t  [SetAssociativity - 1 : 0]      gnt_data_bank_write_addr;
    logic                   [SetAssociativity - 1 : 0]      gnt_data_bank_write_req;
    cache_data_t            [SetAssociativity - 1 : 0]      gnt_data_bank_write_data;
    cache_mask_t            [SetAssociativity - 1 : 0]      gnt_data_bank_write_mask;

    cache_bank_depth_ptr_t  [SetAssociativity - 1 : 0]      gnt_meta_bank_read_addr;
    logic                   [SetAssociativity - 1 : 0]      gnt_meta_bank_read_valid;
    logic                   [SetAssociativity - 1 : 0]      gnt_meta_bank_read_ready;
    cache_meta_t            [SetAssociativity - 1 : 0]      gnt_meta_bank_read_data;

    cache_bank_depth_ptr_t  [SetAssociativity - 1 : 0]      gnt_meta_bank_write_addr;
    logic                   [SetAssociativity - 1 : 0]      gnt_meta_bank_write_req;
    cache_meta_t            [SetAssociativity - 1 : 0]      gnt_meta_bank_write_data;
    logic                   [SetAssociativity - 1 : 0]      gnt_meta_bank_write_mask;


    cache_bank_depth_ptr_t                                  bank_write_cache_addr;
    logic                                                   bank_write_cache_req;
    cache_status_t          [SetAssociativity - 1 : 0]      bank_write_cache_status;
    logic                   [SetAssociativity - 1 : 0]      bank_write_cache_dirty;
    miss_meta_t             [SetAssociativity - 1 : 0]      bank_write_cache_miss_meta;
    cache_mask_t            [SetAssociativity - 1 : 0]      bank_write_cache_mask;
    cache_tag_t             [SetAssociativity - 1 : 0]      bank_write_cache_tag;
    cache_data_t            [SetAssociativity - 1 : 0]      bank_write_cache_data;
    cache_mask_t            [SetAssociativity - 1 : 0]      bank_write_data_mask;
    cache_mask_t            [SetAssociativity - 1 : 0]      bank_write_data_mask_sel;
    logic                                                   bank_write_LRU_req;
    way_ptr_t               [SetAssociativity - 1 : 0]      bank_write_cache_LRU;
    logic                                                   bank_write_meta_skip;
    logic                                                   bank_read_data_skip;

    /****************/
    /*  Cache Proc  */
    /****************/
    cache_bank_depth_ptr_t                                  proc_read_cache_addr;
    logic                                                   proc_read_cache_valid;
    logic                                                   proc_read_cache_ready;
    cache_status_t          [SetAssociativity - 1 : 0]      proc_read_cache_status;
    logic                   [SetAssociativity - 1 : 0]      proc_read_cache_dirty;
    miss_meta_t             [SetAssociativity - 1 : 0]      proc_read_cache_miss_meta;
    cache_mask_t            [SetAssociativity - 1 : 0]      proc_read_cache_mask;
    cache_tag_t             [SetAssociativity - 1 : 0]      proc_read_cache_tag;
    cache_data_t            [SetAssociativity - 1 : 0]      proc_read_cache_data;
    way_ptr_t               [SetAssociativity - 1 : 0]      proc_read_cache_LRU;
    

    cache_bank_depth_ptr_t                                  proc_write_cache_addr;
    logic                                                   proc_write_cache_req;
    way_ptr_t                                               proc_write_cache_way;
    cache_status_t          [SetAssociativity - 1 : 0]      proc_write_cache_status;
    logic                   [SetAssociativity - 1 : 0]      proc_write_cache_dirty;
    miss_meta_t             [SetAssociativity - 1 : 0]      proc_write_cache_miss_meta;
    cache_mask_t            [SetAssociativity - 1 : 0]      proc_write_cache_mask;
    cache_tag_t             [SetAssociativity - 1 : 0]      proc_write_cache_tag;
    cache_data_t            [SetAssociativity - 1 : 0]      proc_write_cache_data;
    logic                                                   proc_write_LRU_req;
    way_ptr_t               [SetAssociativity - 1 : 0]      proc_write_cache_LRU;
    logic                                                   proc_write_meta_skip;
    logic                                                   proc_read_data_skip;

    logic                                                   proc_write_select;

    // Phase 1 handshake: aggregate per-way write_ready into bank_write_cache_ready,
    // and route up to cache_core as bank_write_ready_i.  Phase 1 is always-1
    // (no behavioral change); Phase 2 lowers it for transient buffer states.
    logic                                                   bank_write_cache_ready;
    logic                                                   proc_write_cache_ready;

    // Phase 3: advisory bit from cache_core indicating the line being
    // written is currently in VALID state.  Distributed to the data-side
    // access ctrls so the buffer's ACCUMULATE-CONCURRENT-MERGE branch can
    // gate its parts-preserving behavior on it.
    logic                                                   proc_write_target_valid;

    /*****************/
    /*  Cache Flush  */
    /*****************/
    cache_bank_depth_ptr_t                                  flush_read_cache_addr;
    logic                                                   flush_read_cache_valid;
    logic                                                   flush_read_cache_ready;
    cache_status_t          [SetAssociativity - 1 : 0]      flush_read_cache_status;
    logic                   [SetAssociativity - 1 : 0]      flush_read_cache_dirty;
    miss_meta_t             [SetAssociativity - 1 : 0]      flush_read_cache_miss_meta;
    cache_mask_t            [SetAssociativity - 1 : 0]      flush_read_cache_mask;
    cache_tag_t             [SetAssociativity - 1 : 0]      flush_read_cache_tag;
    cache_data_t            [SetAssociativity - 1 : 0]      flush_read_cache_data;
    way_ptr_t               [SetAssociativity - 1 : 0]      flush_read_cache_LRU;
    

    cache_bank_depth_ptr_t                                  flush_write_cache_addr;
    logic                                                   flush_write_cache_req_valid;
    logic                                                   flush_write_cache_req_ready;
    cache_status_t          [SetAssociativity - 1 : 0]      flush_write_cache_status;
    logic                   [SetAssociativity - 1 : 0]      flush_write_cache_dirty;
    miss_meta_t             [SetAssociativity - 1 : 0]      flush_write_cache_miss_meta;
    cache_mask_t            [SetAssociativity - 1 : 0]      flush_write_cache_mask;
    cache_tag_t             [SetAssociativity - 1 : 0]      flush_write_cache_tag;
    cache_data_t            [SetAssociativity - 1 : 0]      flush_write_cache_data;
    way_ptr_t               [SetAssociativity - 1 : 0]      flush_write_cache_LRU;

    logic                                                   flush_read_select_q, flush_read_select_d;
    cache_bank_depth_ptr_t                                  bank_read_cache_addr_q;
    // Per-set dirty register file -- declared early so the
    // gen_sync_ctrl_fsm always_comb (which iterates the bank and reads
    // dirty_rf directly for the FRESH dirty mask) can reference it.
    logic [CacheBankDepth-1:0][SetAssociativity-1:0]        dirty_rf;
    logic                   [SetAssociativity - 1 : 0]      bank_read_way_mask_q, bank_read_way_mask_d;
    logic                                                   bank_read_sel_flush;
    `FFARN (flush_read_select_q, flush_read_select_d,       '0, clk_i, rst_ni)
    `FFARN (bank_read_way_mask_q, bank_read_way_mask_d,     '0, clk_i, rst_ni)
    logic          [PartIdxWidth-1:0]                       flush_read_part_idx;
    logic                                                   flush_read_all_parts;
    logic                   [SetAssociativity - 1 : 0]      flush_read_way_mask;

    logic                                                   sync_ctrl_still_pending;
    logic                                                   sync_ctrl_has_dirty_line;
    way_ptr_t                                               sync_ctrl_dirty_line;
    cache_sync_ctrl_status_t                                sync_ctrl_status_q, sync_ctrl_status_d;
    logic [1:0]                                             sync_ctrl_insn_q, sync_ctrl_insn_d;
    cache_bank_depth_ptr_t                                  sync_ctrl_ptr_q,sync_ctrl_ptr_d;
    down_req_t                                              sync_ctrl_payload_q,sync_ctrl_payload_d;
    logic                                                   clear_pend_cnt;
    localparam int unsigned OutstandingRefillCntWidth =
        ((CacheBankDepth * SetAssociativity) > 1) ? $clog2((CacheBankDepth * SetAssociativity) + 1) : 1;
    logic [OutstandingRefillCntWidth-1:0]                   outstanding_refill_cnt_q, outstanding_refill_cnt_d;
    logic                                                   consumed_refill_resp;
    logic                                                   issued_refill_req;
    // CHECK_PEND additional drain delay: only advance to FLUSH after the
    // existing drain conditions have been STABLE for N consecutive cycles,
    // giving any in-flight install pipeline stage (preread -> bank-read ->
    // encoder -> bank-write) time to commit before sync starts writing
    // the meta SRAM.  The cache_core latches refill data internally
    // before issuing the bank-write, so without this delay an install
    // can fire DURING flush meta-writes and corrupt the tag.
    localparam int unsigned CheckPendDrainCycles = 20;
    logic [4:0] check_pend_drain_cnt_q, check_pend_drain_cnt_d;
    `FFARN (check_pend_drain_cnt_q, check_pend_drain_cnt_d, '0, clk_i, rst_ni)
`ifndef TARGET_SYNTHESIS
    // Verbose sync-FSM and meta-bank tracing — off by default; enable
    // with `+insitu_trace`.  Used to debug sync<->install races; assertions
    // and scoreboard errors fire independently.
    bit insitu_trace_en = 1'b0;
    initial insitu_trace_en = $test$plusargs("insitu_trace");
`endif
    `FFARN (sync_ctrl_status_q, sync_ctrl_status_d,         SYNC_CTRL_IDLE, clk_i, rst_ni)
    `FFARN (sync_ctrl_insn_q, sync_ctrl_insn_d,             '0, clk_i, rst_ni)
    `FFARN (sync_ctrl_ptr_q,sync_ctrl_ptr_d,                '0, clk_i, rst_ni)
    `FFARN (sync_ctrl_payload_q,sync_ctrl_payload_d,        '0, clk_i, rst_ni)
    `FFARN (outstanding_refill_cnt_q, outstanding_refill_cnt_d, '0, clk_i, rst_ni)
    cache_data_t                                            flush_full_data_q, flush_full_data_d;
    cache_mask_t                                            flush_full_mask_q, flush_full_mask_d;
    cache_tag_t                                             flush_full_tag_q, flush_full_tag_d;
    cache_bank_depth_ptr_t                                  flush_full_addr_q, flush_full_addr_d;
    way_ptr_t                                               flush_full_way_q, flush_full_way_d;
    logic                                                   flush_full_data_valid_q, flush_full_data_valid_d;
    logic                                                   flush_full_wait_q, flush_full_wait_d;
    `FFARN (flush_full_data_q, flush_full_data_d,            '0, clk_i, rst_ni)
    `FFARN (flush_full_mask_q, flush_full_mask_d,            '0, clk_i, rst_ni)
    `FFARN (flush_full_tag_q, flush_full_tag_d,              '0, clk_i, rst_ni)
    `FFARN (flush_full_addr_q, flush_full_addr_d,            '0, clk_i, rst_ni)
    `FFARN (flush_full_way_q, flush_full_way_d,              '0, clk_i, rst_ni)
    `FFARN (flush_full_data_valid_q, flush_full_data_valid_d,'0, clk_i, rst_ni)
    `FFARN (flush_full_wait_q, flush_full_wait_d,            '0, clk_i, rst_ni)
    logic                                                   flush_full_read_active;
    byte_offset_t                                           sync_ctrl_ofst;

    /////////////////////////////////////
    //        Instance Modules         //
    /////////////////////////////////////

    /***************************************/
    /*  Write-through / Write-back Logics  */
    /***************************************/

    // Drain-state taps from the cache core for the sync-flush FSM.  Declared
    // at module scope (before the WriteThroughMode generate) so both the FSM
    // inside it and the i_insitu_cache_core instance below can see them.
    logic core_preread_task_valid;
    logic core_retr_fifo_empty;

    if (WriteThroughMode) begin

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
        // Block new upstream requests from entering the cache_core pipeline
        // while the sync FSM is anywhere other than IDLE / FINISH.  Without
        // this gate, a request accepted during sync can issue its PEND
        // meta-write during the FLUSH phase; that write races with the
        // sync invalidate, the line ends up INVALID, and the subsequent
        // refill response trips proc_assert_read_refill_reread
        // (insitu_cache_core.sv:1071, 1080) -- losing the proc response
        // and hanging the sim before EOC.  Bypass-cached peripheral
        // accesses go through a separate path and are unaffected.
        logic sync_block_upstream;
        assign sync_block_upstream = (sync_ctrl_status_q != SYNC_CTRL_IDLE
                                    && sync_ctrl_status_q != SYNC_CTRL_FINISH);
        assign upstream_req_to_cache_valid = ~sync_block_upstream &&
                                            (upstream_req_write_i?
                                                ~winfo_fifo_full & upstream_req_valid_i:
                                                upstream_req_valid_i);
        assign upstream_req_ready_o        =    upstream_req_to_cache_valid & upstream_req_to_cache_ready;

        assign winfo_fifo_push             =    ~winfo_fifo_full & upstream_req_valid_i & upstream_req_ready_o & upstream_req_write_i;
        assign winfo_fifo_in               =    upstream_req_info_i;

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

        /***************************************/
        /*  Cache Flush + Invalidation Process */
        /***************************************/

        //SynC CTRL FSM
        always_comb begin : gen_sync_ctrl_fsm
            //Default value
            sync_ctrl_has_dirty_line    = 1'b0;
            sync_ctrl_dirty_line        = '0;
            sync_ctrl_status_d          = sync_ctrl_status_q;
            sync_ctrl_insn_d            = sync_ctrl_insn_q;
            sync_ctrl_ptr_d             = sync_ctrl_ptr_q;
            sync_ctrl_payload_d         = sync_ctrl_payload_q;
            outstanding_refill_cnt_d    = outstanding_refill_cnt_q;
            check_pend_drain_cnt_d      = '0;  // default: reset drain counter outside CHECK_PEND
            flush_full_data_d           = flush_full_data_q;
            flush_full_mask_d           = flush_full_mask_q;
            flush_full_tag_d            = flush_full_tag_q;
            flush_full_addr_d           = flush_full_addr_q;
            flush_full_way_d            = flush_full_way_q;
            flush_full_data_valid_d     = flush_full_data_valid_q;
            flush_full_wait_d           = flush_full_wait_q;
            flush_full_read_active      = 1'b0;
            sync_ctrl_ofst              = '0;

            flush_read_cache_valid      = '0;
            flush_read_cache_addr       = '0;
            flush_write_cache_req_valid = '0;
            flush_write_cache_addr      = '0;
            flush_read_part_idx         = '0;
            flush_read_all_parts        = 1'b0;
            flush_read_way_mask         = {SetAssociativity{1'b1}};

            flush_write_cache_status    = flush_read_cache_status;
            flush_write_cache_dirty     = flush_read_cache_dirty;
            flush_write_cache_miss_meta = flush_read_cache_miss_meta;
            flush_write_cache_mask      = flush_read_cache_mask;
            flush_write_cache_tag       = flush_read_cache_tag;
            flush_write_cache_data      = flush_read_cache_data;
            flush_write_cache_LRU       = flush_read_cache_LRU;

            write_through_req_payload   = '0;
            write_through_valid         = '0;

            cache_sync_ready_o          = '0;
            clear_pend_cnt              = 1'b0;

            //FSM
            case (sync_ctrl_status_q)

                SYNC_CTRL_IDLE : begin
                    if (cache_sync_valid_i) begin
                        sync_ctrl_insn_d = cache_sync_insn_i;
                        sync_ctrl_ptr_d = cache_part_base_i;
                        if (cache_sync_insn_i == 2'b11) begin
                            sync_ctrl_ptr_d = '0;
                        end
                        sync_ctrl_status_d = SYNC_CTRL_READ_BANK;
                    end
                end

                SYNC_CTRL_READ_BANK : begin
                    flush_read_cache_addr = sync_ctrl_ptr_q;
                    flush_read_cache_valid = 1'b1;
                    if (flush_read_cache_ready) begin
                        // Always route through CHECK_PEND so the install
                        // pipeline is drained before sync starts its own
                        // bank writes -- both the flush path (-> FLUSH)
                        // and the init-all path (-> INIT) must wait for
                        // any pre-existing refill install to commit,
                        // otherwise an in-flight install's bank-read can
                        // sample meta DURING the sync writes and an
                        // install bank-write can commit DURING those
                        // sync writes, corrupting tag/status of the
                        // line and tripping
                        // proc_assert_read_refill_reread on the next
                        // refill response.
                        sync_ctrl_status_d = SYNC_CTRL_CHECK_PEND;
                    end
                end

                SYNC_CTRL_INIT : begin
                    clear_pend_cnt              = 1'b1;
                    flush_write_cache_addr      = sync_ctrl_ptr_q;
                    flush_write_cache_status    = '0;
                    flush_write_cache_dirty     = '0;
                    flush_write_cache_miss_meta = '0;
                    flush_write_cache_mask      = '0;
                    flush_write_cache_tag       = '0;
                    flush_write_cache_LRU       = '0;

                    flush_write_cache_req_valid = 1'b1;
                    if (flush_write_cache_req_ready) begin
                        sync_ctrl_ptr_d = sync_ctrl_ptr_q + 1'b1;
                        if (sync_ctrl_ptr_q == (CacheBankDepth - 1)) begin
                            sync_ctrl_status_d = SYNC_CTRL_FINISH;
                        end
                    end
                end

                SYNC_CTRL_CHECK_PEND : begin
                    // The four conditions below are derived from FIFO/handshake
                    // counters and are reliable. We deliberately do NOT gate
                    // on `sync_ctrl_still_pending` (= pendline_cnt_q != 0):
                    // the encoder-maintained pendline counter can leak when
                    // the meta SRAM read returns stale data on a PEND->VALID
                    // refill write (a known timing race between the 1-cycle
                    // synchronous meta SRAM and the encoder's transition
                    // detection at insitu_cache_encoder.sv:124-156).  A
                    // leaked counter would otherwise wedge a pure flush in
                    // CHECK_PEND forever even though no real work is pending.
                    // We also assert `clear_pend_cnt` on the transition into
                    // FLUSH so subsequent post-flush operations start from a
                    // clean counter.
                    // Also wait for the cache_core's preread pipeline to be
                    // empty.  A request can be accepted upstream, latch
                    // into preread_task_q, complete its bank-read, then
                    // issue a PEND meta-write and (later) an install
                    // VALID -- all AFTER the four signals below have
                    // already gone low.  If we advance to FLUSH/INVAL
                    // with a preread task still in flight, the install
                    // VALID will fire during FLUSH and race with the
                    // sync invalidate, leaving the line VALID with a
                    // tag wiped to 0 -- which then trips
                    // proc_assert_read_refill_reread when its refill
                    // response arrives.  preread_task_q is accessed via
                    // hierarchical reference (matches the scoreboard
                    // binding's existing pattern).
                    // Also wait for the cache_core's refill retrieval pipeline
                    // to be empty.  retr_fifo holds subarray refill commits
                    // that haven't been installed yet; retr_entry tracks the
                    // currently-installing entry.  Even after preread_task_q
                    // goes idle, an in-flight refill that completed at the
                    // downstream interface may still be queued in retr_fifo
                    // waiting for the bank-write commit.  If we advance to
                    // FLUSH/INVAL with that data still queued, the install
                    // fires DURING the flush meta-write window and trips
                    // proc_assert_read_refill_reread (tag wiped to 0 by
                    // concurrent flush meta-writes).
                    // Wait for the existing drain conditions to be stable for
                    // CheckPendDrainCycles consecutive cycles before advancing
                    // to FLUSH.  The cache_core's refill-install pipeline can
                    // be 2-3 cycles deep from preread_task_q to the bank-write
                    // commit; even after preread.valid goes low, an install
                    // can still fire a meta-write this cycle or the next.
                    // The extra delay lets that drain.
                    begin
                      automatic logic drain_now;
                      drain_now = (outstanding_refill_cnt_q == '0) &&
                                  ~core_miss_valid &&
                                  ~core_evic_valid &&
                                  ~write_through_valid &&
                                  ~core_preread_task_valid &&
                                  core_retr_fifo_empty &&
                                  ~proc_write_cache_req;
                      if (drain_now) begin
                          if (check_pend_drain_cnt_q < CheckPendDrainCycles[4:0]) begin
                              check_pend_drain_cnt_d = check_pend_drain_cnt_q + 1'b1;
                          end else begin
                              clear_pend_cnt = 1'b1;
                              check_pend_drain_cnt_d = '0;
                              if (sync_ctrl_insn_q == 2'b11) begin
                                  // init-all: ptr was set to 0 in IDLE,
                                  // jump straight into the INIT writer.
                                  sync_ctrl_status_d = SYNC_CTRL_INIT;
                              end else begin
                                  sync_ctrl_status_d = SYNC_CTRL_FLUSH;
                                  sync_ctrl_ptr_d = cache_part_base_i;
                                  flush_read_cache_addr = sync_ctrl_ptr_d;
                                  flush_read_cache_valid = 1'b1;
                              end
`ifndef TARGET_SYNTHESIS
                              if (insitu_trace_en) begin
                                  $display("[CHECK_PEND->%s %m] t=%0t  drained, advancing",
                                           (sync_ctrl_insn_q == 2'b11) ? "INIT" : "FLUSH", $time);
                              end
`endif
                          end
                      end else begin
                          check_pend_drain_cnt_d = '0;
                      end
                    end
                end

                SYNC_CTRL_FLUSH : begin
                    //Check Dirty Line
                    //
                    // Source-of-truth: dirty_rf is a per-set flop array
                    // updated atomically on every bank_write_cache_req
                    // (including flush's own cleanup writes).  Reading it
                    // directly with sync_ctrl_ptr_q always yields the
                    // FRESH dirty state for the current pointer.
                    //
                    // We deliberately avoid `flush_read_cache_status[i] ==
                    // VALID` because:
                    //   (a) on multi-way dirty sets, the initial flush_read
                    //       used way_mask=all-1 and read all ways' status.
                    //       But the per-way writeback's flush_read uses a
                    //       one-hot way_mask, so subsequent cycles return
                    //       0 for non-selected ways' status -- has_dirty
                    //       then drops to 0 and the FSM silently advances
                    //       ptr, leaving multi-way dirty data behind.
                    //   (b) a write to a line implies the line is VALID
                    //       (proc-side writes always go to a refilled
                    //       VALID line), so dirty=1 already implies status
                    //       != INVALID; the status check is redundant.
                    //
                    // The flush_full_read for tag/mask still issues a
                    // fresh one-hot read against the meta SRAM and
                    // captures correct tag/mask for the writeback.
                    // Unrolled to avoid any for-loop / break evaluation
                    // ambiguity in always_comb iteration.
                    if (dirty_rf[sync_ctrl_ptr_q][0]) begin
                        sync_ctrl_has_dirty_line = 1'b1;
                        sync_ctrl_dirty_line = 2'd0;
                    end else if (dirty_rf[sync_ctrl_ptr_q][1]) begin
                        sync_ctrl_has_dirty_line = 1'b1;
                        sync_ctrl_dirty_line = 2'd1;
                    end else if (dirty_rf[sync_ctrl_ptr_q][2]) begin
                        sync_ctrl_has_dirty_line = 1'b1;
                        sync_ctrl_dirty_line = 2'd2;
                    end else if (dirty_rf[sync_ctrl_ptr_q][3]) begin
                        sync_ctrl_has_dirty_line = 1'b1;
                        sync_ctrl_dirty_line = 2'd3;
                    end
                    // STICKY DIRTY GATE: once we've committed to a dirty-line
                    // eviction (flush_full_wait_q or flush_full_data_valid_q
                    // is set), we MUST stay in this branch until wb_done.
                    // `sync_ctrl_has_dirty_line` is driven by dirty_rf
                    // directly (a flop array indexed by sync_ctrl_ptr_q),
                    // so it's always FRESH for the current ptr -- no meta
                    // SRAM read alignment needed for the dirty check.  The
                    // meta SRAM is only needed for fetching tag/mask for
                    // the writeback, which is gated by the `~wait_q &&
                    // ~dvalid_q` sub-state inside the dirty branch (which
                    // also gates on read alignment via flush_full_*_q).
                    if (sync_ctrl_has_dirty_line || flush_full_wait_q || flush_full_data_valid_q) begin
                        if (PartSplit > 1) begin
                            flush_full_read_active = 1'b1;
                            if (~flush_full_wait_q && ~flush_full_data_valid_q) begin
                                // READ-COMPLETION GUARD: we need the meta
                                // SRAM read for sync_ctrl_ptr_q to be valid
                                // before we can latch the tag/mask for the
                                // writeback.  If the read isn't aligned
                                // (proc-side contention or first cycle),
                                // re-issue and wait.
                                if (~(flush_read_select_q
                                      && (bank_read_cache_addr_q == sync_ctrl_ptr_q))) begin
                                    flush_read_cache_addr = sync_ctrl_ptr_q;
                                    flush_read_cache_valid = 1'b1;
                                    // Don't advance into wait state yet.
                                end else begin
                                    flush_full_way_d = sync_ctrl_dirty_line;
                                    flush_full_tag_d = flush_read_cache_tag[sync_ctrl_dirty_line];
                                    flush_full_mask_d = flush_read_cache_mask[sync_ctrl_dirty_line];
                                    flush_full_addr_d = sync_ctrl_ptr_q;
                                    flush_read_cache_addr = sync_ctrl_ptr_q;
                                    flush_read_cache_valid = 1'b1;
                                    flush_read_all_parts = 1'b1;
                                    flush_read_way_mask = '0;
                                    flush_read_way_mask[sync_ctrl_dirty_line] = 1'b1;
                                    if (flush_read_cache_ready) begin
                                        flush_full_wait_d = 1'b1;
                                    end
                                end
                            end else if (flush_full_wait_q) begin
                                flush_full_data_d = bank_read_cache_data[flush_full_way_q];
                                flush_full_data_valid_d = 1'b1;
                                flush_full_wait_d = 1'b0;
                            end

                            if (flush_full_data_valid_q) begin
                                //Try to evict dirty line
                                write_through_req_payload.addr  = {flush_full_tag_q, flush_full_addr_q, sync_ctrl_ofst};
                                write_through_req_payload.info  = '0;
                                write_through_req_payload.write = 1'b1;
                                write_through_req_payload.wdata = flush_full_data_q;
                                write_through_req_payload.wmask = flush_full_mask_q;
                                write_through_valid = 1'b1;
                                if (write_through_ready) begin
                                    //clean up the dirty line
                                    flush_write_cache_addr                          = flush_full_addr_q;
                                    flush_write_cache_status[flush_full_way_q]      = INVALID;
                                    flush_write_cache_dirty[flush_full_way_q]       = 1'b0;
                                    flush_write_cache_req_valid                     = 1'b1;
                                    flush_full_data_valid_d                         = 1'b0;
                                    flush_full_wait_d                               = 1'b0;
                                end
                            end
                        end else begin
                            //Try to evict dirty line
                            write_through_req_payload.addr  = {flush_read_cache_tag[sync_ctrl_dirty_line], sync_ctrl_ptr_q, sync_ctrl_ofst};
                            write_through_req_payload.info  = '0;
                            write_through_req_payload.write = 1'b1;
                            write_through_req_payload.wdata = flush_read_cache_data[sync_ctrl_dirty_line];
                            write_through_req_payload.wmask = flush_read_cache_mask[sync_ctrl_dirty_line];
                            write_through_valid = 1'b1;
                            if (write_through_ready) begin
                                //clean up the dirty line
                                flush_write_cache_addr                          = sync_ctrl_ptr_q;
                                flush_write_cache_status[sync_ctrl_dirty_line]  = INVALID;
                                flush_write_cache_dirty[sync_ctrl_dirty_line]   = 1'b0;
                                flush_write_cache_req_valid                     = 1'b1;
                            end
                        end
                    end else begin
                        //clean up whole way
                        flush_write_cache_addr      = sync_ctrl_ptr_q;
                        flush_write_cache_status    = '0;
                        flush_write_cache_dirty     = '0;
                        flush_write_cache_miss_meta = '0;
                        flush_write_cache_mask      = '0;
                        flush_write_cache_tag       = '0;
                        flush_write_cache_LRU       = '0;

                        flush_write_cache_req_valid = 1'b1;
                        sync_ctrl_ptr_d = sync_ctrl_ptr_q + 1'b1;
                        flush_full_data_valid_d = 1'b0;
                        flush_full_wait_d = 1'b0;
                        if (sync_ctrl_ptr_q == (CacheBankDepth - 1)) begin
                            sync_ctrl_status_d = SYNC_CTRL_FINISH;
                        end
                    end

                    //Read Bank at Background
                    if (~flush_full_read_active) begin
                        flush_read_cache_addr = sync_ctrl_ptr_d;
                        flush_read_cache_valid = 1'b1;
                    end

                end

                SYNC_CTRL_INVALID : begin
                end

                SYNC_CTRL_FINISH : begin
                    cache_sync_ready_o = 1'b1;
                    sync_ctrl_status_d = SYNC_CTRL_IDLE;
                end

                default : begin
                    sync_ctrl_status_d = SYNC_CTRL_IDLE;
                end
            endcase

            if (issued_refill_req && ~consumed_refill_resp) begin
                outstanding_refill_cnt_d = outstanding_refill_cnt_q + 1'b1;
            end else if (~issued_refill_req && consumed_refill_resp) begin
                if (outstanding_refill_cnt_q != '0) begin
                    outstanding_refill_cnt_d = outstanding_refill_cnt_q - 1'b1;
                end
            end
        end
    end

    /****************/
    /*  Cache Core  */
    /****************/

    // Forwarding buffer write hit for the written way's data bank.
    // When high, the data write was absorbed by the buffer and the
    // write-read hazard in the core can be relaxed.
    logic bank_write_data_buf_hit;
    assign bank_write_data_buf_hit = data_bank_write_hit[proc_write_cache_way];
    // Full-line coverage hint: 1 iff after this absorption the buffer
    // for the written way holds the WHOLE line.  Cache core uses this
    // to bypass the bank-write/upstream-read hazard SAFELY -- partial
    // coverage absorptions leave OTHER parts in SRAM, so a same-line
    // read for a different part would otherwise pull stale data.
    logic bank_write_data_buf_full_cov;
    assign bank_write_data_buf_full_cov = data_bank_write_full_cov[proc_write_cache_way];

    insitu_cache_core #(
        .ReqAddrWidth    (ReqAddrWidth),
        .CacheLineWidth  (CacheLineWidth),
        .info_t          (info_t),
        .NumCacheEntry   (NumCacheEntry),
        .SetAssociativity(SetAssociativity),
        .DataPartSplit   (PartSplit),
        .UseHashWaySelect(UseHashWaySelect),
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
        .has_pend_line_o                (sync_ctrl_still_pending),
        .clear_pend_cnt_i               (clear_pend_cnt),

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
        .bank_read_addr_o               (proc_read_cache_addr),
        .bank_read_part_idx_o           (bank_read_part_idx),
        .bank_read_all_parts_o          (bank_read_all_parts),
        .bank_read_valid_o              (proc_read_cache_valid),
        .bank_read_ready_i              (proc_read_cache_ready),
        .bank_read_way_mask_o           (bank_read_way_mask),
        .bank_read_cache_status_i       (proc_read_cache_status),
        .bank_read_cache_dirty_i        (proc_read_cache_dirty),
        .bank_read_cache_miss_meta_i    (proc_read_cache_miss_meta),
        .bank_read_cache_mask_i         (proc_read_cache_mask),
        .bank_read_cache_tag_i          (proc_read_cache_tag),
        .bank_read_cache_data_i         (proc_read_cache_data),
        .bank_read_cache_LRU_i          (proc_read_cache_LRU),

        .bank_write_req_o               (proc_write_cache_req),
        .bank_write_ready_i             (proc_write_cache_ready),
        .bank_write_target_valid_o      (proc_write_target_valid),
        .bank_write_addr_o              (proc_write_cache_addr),
        .bank_write_way_o               (proc_write_cache_way),
        .bank_write_cache_status_o      (proc_write_cache_status),
        .bank_write_cache_dirty_o       (proc_write_cache_dirty),
        .bank_write_cache_miss_meta_o   (proc_write_cache_miss_meta),
        .bank_write_cache_mask_o        (proc_write_cache_mask),
        .bank_write_cache_tag_o         (proc_write_cache_tag),
        .bank_write_cache_data_o        (proc_write_cache_data),
        .bank_write_data_mask_o         (bank_write_data_mask),
        .bank_write_LRU_req_o           (proc_write_LRU_req),
        .bank_write_cache_LRU_o         (proc_write_cache_LRU),
        .bank_write_meta_skip_o         (proc_write_meta_skip),
        .bank_read_data_skip_o          (proc_read_data_skip),
        .bank_write_data_buf_hit_i      (bank_write_data_buf_hit),
        .bank_write_data_buf_full_cov_i (bank_write_data_buf_full_cov),
        .preread_task_valid_o           (core_preread_task_valid),
        .retr_fifo_empty_o              (core_retr_fifo_empty)
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

    // Prioritize read/refill responses over write responses. Keep the selected
    // source stable under backpressure to satisfy valid/ready stability rules.
    always_comb begin : proc_cache_resp_sel
        resp_mux_lock_d    = resp_mux_lock_q;
        resp_mux_sel_d     = resp_mux_sel_q;
        resp_mux_use_core  = 1'b0;
        resp_mux_use_wresp = 1'b0;
        resp_mux_valid     = 1'b0;

        if (resp_mux_lock_q) begin
            resp_mux_use_core  = ~resp_mux_sel_q;
            resp_mux_use_wresp =  resp_mux_sel_q;
        end else begin
            resp_mux_use_core  = core_resp_valid;
            resp_mux_use_wresp = ~core_resp_valid & wresp_valid;
        end

        resp_mux_valid = (resp_mux_use_core & core_resp_valid) |
                         (resp_mux_use_wresp & wresp_valid);

        if (!resp_mux_lock_q) begin
            if (resp_mux_valid && !upstream_resp_ready_i) begin
                resp_mux_lock_d = 1'b1;
                resp_mux_sel_d  = resp_mux_use_wresp;
            end
        end else if (resp_mux_valid && upstream_resp_ready_i) begin
            resp_mux_lock_d = 1'b0;
        end
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin : proc_cache_resp_sel_ff
        if (!rst_ni) begin
            resp_mux_lock_q <= 1'b0;
            resp_mux_sel_q  <= 1'b0;
        end else begin
            resp_mux_lock_q <= resp_mux_lock_d;
            resp_mux_sel_q  <= resp_mux_sel_d;
        end
    end

    always_comb begin : proc_cache_resp_mux
        resp_out             = '0;
        upstream_resp_valid_o = resp_mux_valid;
        wresp_ready          = 1'b0;
        core_resp_ready      = 1'b0;
        if (resp_mux_use_core) begin
            resp_out              = rresp_in;
            core_resp_ready       = upstream_resp_ready_i;
        end else if (resp_mux_use_wresp) begin
            resp_out              = wresp_in;
            wresp_ready           = upstream_resp_ready_i;
        end
    end

    assign winfo_fifo_pop = wresp_valid & wresp_ready;

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

`ifndef TARGET_SYNTHESIS
    // WB probe — off by default; enable with `+wb_trace`.
    bit wb_trace_en_wrap = 1'b0;
    initial wb_trace_en_wrap = $test$plusargs("wb_trace");
    // WRITEBACK-PROBE Stage 3: writeback (write=1) leaves the wrapper's
    // arbiter to head out toward AXI / cachepool_cache_ctrl.
    always @(posedge clk_i) begin
        if (wb_trace_en_wrap && rst_ni && downstream_req_valid_o
            && downstream_req_ready_i && downstream_req_write_o) begin
            $display("[WB-S3-WRAP %m] t=%0t DOWNSTREAM_WRITE addr=0x%0h wmask=0x%0h wdata[31:0]=0x%0h",
                     $time, downstream_req_addr_o, downstream_req_wmask_o,
                     downstream_req_wdata_o[31:0]);
        end
    end
`endif

    /*******************/
    /*  Refill logics  */
    /*******************/

    assign core_refill_data = downstream_resp_data_i;
    assign core_refill_info = downstream_resp_info_i;
    // Defer refill-install during the sync FSM's bank-writing phases
    // (INIT, FLUSH, INVALID).  The cache_core's install path drives
    // the data and meta SRAMs; if it fires concurrent with the sync
    // FSM's own bank writes the install's data lands in a partially-
    // cleared line, which the scoreboard catches as DATA MISMATCH on
    // the next proc read.  INIT must be included because the init-all
    // path (insn=2'b11) skips CHECK_PEND -- it transitions
    // READ_BANK -> INIT directly, so a refill in flight when sync
    // starts can install during INIT writes.  We intentionally do NOT
    // block during CHECK_PEND: the CHECK_PEND drain (see
    // SYNC_CTRL_CHECK_PEND above) already waits for the install
    // pipeline to empty before advancing, so there is nothing to
    // block there and gating CHECK_PEND would stall arriving refills
    // for the full drain window.
    // CHECK_PEND is intentionally NOT blocked here.  The CHECK_PEND
    // drain logic (see SYNC_CTRL_CHECK_PEND above) waits for
    // outstanding_refill_cnt and preread_task_q to drain BEFORE
    // advancing to FLUSH; if we also gate refills during CHECK_PEND
    // the drain can never complete (the very refill we are waiting
    // for is held at the wrapper boundary), the sync FSM wedges in
    // CHECK_PEND, cache_sync_ready_o is never asserted, the tile's
    // l1d_insn_ready_o never pulses, and the peripheral's
    // l1d_lock_q[t] is stuck at 1 forever.
    logic sync_block_install;
    assign sync_block_install = (sync_ctrl_status_q == SYNC_CTRL_INIT
                              || sync_ctrl_status_q == SYNC_CTRL_FLUSH
                              || sync_ctrl_status_q == SYNC_CTRL_INVALID);
    assign core_refill_valid = downstream_resp_valid_i & ~downstream_resp_write_i & ~sync_block_install;
    assign downstream_resp_ready_o = downstream_resp_write_i ? 1'b1
                                  : (~sync_block_install & core_refill_ready);
    assign consumed_refill_resp = downstream_resp_valid_i & ~downstream_resp_write_i & downstream_resp_ready_o;
    assign issued_refill_req = core_miss_valid & core_miss_ready;

    /***********************/
    /*  Flush Proc Arbiter */
    /***********************/

    // Flush gets bank-read priority whenever the sync-ctrl FSM is in a
    // state that actively iterates the bank: SYNC_CTRL_READ_BANK,
    // SYNC_CTRL_INIT, and SYNC_CTRL_FLUSH all rely on flush_read_cache_*
    // to retire.  Without this, an in-flight proc read (e.g., a refill
    // straggler whose post-fill load is still draining the cache_core's
    // pipeline) keeps proc_read_cache_valid high and starves the
    // flush_read handshake.  bank_read_cache_addr_q then never updates,
    // dirty_rf[addr_q] is read for an unrelated set, and the FSM clears
    // each set without writeback.  Software gates new proc activity
    // through cache_sync_ready_o, so blocking residual proc reads
    // during flush is safe -- the proc has already issued the sync and
    // is waiting for it to complete.
    // Bank-read priority:
    //   - During the sync FSM's WRITING phases (INIT/FLUSH/INVAL) the flush
    //     side gets the bank-read port unconditionally.  These are the
    //     states in which sync writes the meta SRAM; if we let an install's
    //     bank-read fire here it samples the wiped meta (status=0/tag=0)
    //     and the install bank-write later commits with that stale data,
    //     tripping proc_assert_read_refill_reread.
    //   - During CHECK_PEND we intentionally DO NOT block proc reads.  The
    //     CHECK_PEND drain (see SYNC_CTRL_CHECK_PEND above) is waiting for
    //     the install pipeline to empty -- including any in-flight
    //     bank-read.  Blocking proc reads here would deadlock the drain
    //     (preread_task_q.valid would stay 1 forever, CHECK_PEND would
    //     never advance, cache_sync_ready_o never asserts, peripheral
    //     l1d_lock_q[t] stuck at 1).
    //   - During IDLE/FINISH/READ_BANK, fall back to the original
    //     priority: flush only when proc has nothing pending.
    // Timing (T1.2): the inner ternary arms were identical (~proc_read_cache_valid),
    // and the outer (INIT||FLUSH||INVALID) condition is exactly sync_block_install
    // (:1394).  Reuse it (CSE) and drop the dead SYNC_CTRL_READ_BANK comparator +
    // one mux level.  Bit-identical incl. 4-state X:  (a ? 1'b1 : b) === (a | b).
    assign bank_read_sel_flush         = sync_block_install | ~proc_read_cache_valid;
    assign bank_read_cache_valid       = bank_read_sel_flush? flush_read_cache_valid : proc_read_cache_valid;
    // Fix: proc must only see ready when the arbiter is actually serving
    // proc.  Previously this was unconditionally bank_read_cache_ready,
    // which spuriously completed a proc handshake whenever the bank
    // accepted a flush read -- cache_core then advanced its pipeline as
    // if its bank-read had been accepted, but the bank had read the
    // flush address (not the proc's), corrupting the install's view of
    // meta and tripping proc_assert_read_refill_reread.
    assign proc_read_cache_ready       = ~bank_read_sel_flush & bank_read_cache_ready;
    assign flush_read_cache_ready      = bank_read_sel_flush? bank_read_cache_ready : '0;
    assign bank_read_way_mask_sel      = bank_read_sel_flush? flush_read_way_mask : bank_read_way_mask;

    // Phase 1 handshake: write-side ready aggregation.  Only the targeted
    // way's ready matters; other ways are independent.  In Phase 1 the
    // access-ctrls always assert ready=1, so this is always 1.
    assign bank_write_cache_ready  = data_bank_write_ready[proc_write_cache_way]
                                   & meta_bank_write_ready[proc_write_cache_way];
    assign proc_write_cache_ready  = bank_write_cache_ready;
    assign bank_read_part_idx_sel      = bank_read_sel_flush? flush_read_part_idx : bank_read_part_idx;
    assign bank_read_all_parts_sel     = bank_read_sel_flush? flush_read_all_parts : bank_read_all_parts;

    assign bank_read_cache_addr        =  bank_read_sel_flush? flush_read_cache_addr: proc_read_cache_addr;

    always_comb begin : proc_read_source_sel
        flush_read_select_d = flush_read_select_q;
        bank_read_way_mask_d = bank_read_way_mask_q;
        if (bank_read_cache_valid && bank_read_cache_ready) begin
            flush_read_select_d = bank_read_sel_flush;
            bank_read_way_mask_d = bank_read_way_mask_sel;
        end
    end

    assign proc_read_cache_status      =  flush_read_select_q? '0: bank_read_cache_status;
    assign proc_read_cache_dirty       =  flush_read_select_q? '0: bank_read_cache_dirty;
    assign proc_read_cache_miss_meta   =  flush_read_select_q? '0: bank_read_cache_miss_meta;
    assign proc_read_cache_mask        =  flush_read_select_q? '0: bank_read_cache_mask;
    assign proc_read_cache_tag         =  flush_read_select_q? '0: bank_read_cache_tag;
    assign proc_read_cache_data        =  flush_read_select_q? '0: bank_read_cache_data;
    assign proc_read_cache_LRU         =  flush_read_select_q? '0: bank_read_cache_LRU;

    assign flush_read_cache_status     =  ~flush_read_select_q? '0: bank_read_cache_status;
    assign flush_read_cache_dirty      =  ~flush_read_select_q? '0: bank_read_cache_dirty;
    assign flush_read_cache_miss_meta  =  ~flush_read_select_q? '0: bank_read_cache_miss_meta;
    assign flush_read_cache_mask       =  ~flush_read_select_q? '0: bank_read_cache_mask;
    assign flush_read_cache_tag        =  ~flush_read_select_q? '0: bank_read_cache_tag;
    assign flush_read_cache_data       =  ~flush_read_select_q? '0: bank_read_cache_data;
    assign flush_read_cache_LRU        =  ~flush_read_select_q? '0: bank_read_cache_LRU;

    assign proc_write_select           = proc_write_cache_req | proc_write_LRU_req;
    assign bank_write_cache_req        = proc_write_select? proc_write_cache_req : flush_write_cache_req_valid;
    assign bank_write_LRU_req          = proc_write_select? proc_write_LRU_req : flush_write_cache_req_valid;
    assign flush_write_cache_req_ready = ~proc_write_select;

    assign bank_write_cache_addr       = proc_write_select? proc_write_cache_addr : flush_write_cache_addr;
    assign bank_write_cache_status     = proc_write_select? proc_write_cache_status : flush_write_cache_status;
    assign bank_write_cache_dirty      = proc_write_select? proc_write_cache_dirty : flush_write_cache_dirty;
    assign bank_write_cache_miss_meta  = proc_write_select? proc_write_cache_miss_meta : flush_write_cache_miss_meta;
    assign bank_write_cache_mask       = proc_write_select? proc_write_cache_mask : flush_write_cache_mask;
    assign bank_write_cache_tag        = proc_write_select? proc_write_cache_tag : flush_write_cache_tag;
    assign bank_write_cache_data       = proc_write_select? proc_write_cache_data : flush_write_cache_data;
    assign bank_write_cache_LRU        = proc_write_select? proc_write_cache_LRU : flush_write_cache_LRU;
    assign bank_write_data_mask_sel    = proc_write_select? bank_write_data_mask : '{default: '1};
    assign bank_write_meta_skip        = proc_write_select? proc_write_meta_skip : 1'b0;
    assign bank_read_data_skip         = bank_read_sel_flush? 1'b0 : proc_read_data_skip;

`ifndef TARGET_SYNTHESIS
    // META-TRACE: log every committed meta-bank write that targets the
    // watched depth.  Used to reconstruct the lifetime of a specific
    // cache line (status transitions, who wrote it, sync state at the
    // time) so we can pin down refill<->flush races that wipe the
    // READ_PEND marker before its refill response arrives.
    //
    // Change META_TRACE_DEPTH to target a specific depth from the
    // proc_assert_read_refill_reread error message.
    localparam int unsigned META_TRACE_DEPTH = 217;
    always_ff @(posedge clk_i) begin
      if (insitu_trace_en && rst_ni && bank_write_cache_req && bank_write_cache_ready) begin
        if (int'(bank_write_cache_addr) == META_TRACE_DEPTH) begin
          $display("[META-TRACE %m] t=%0t  depth=%0d  src=%s  meta_skip=%0b  sync=%0d  way_focus=%0d  status[0]=%0d  status[1]=%0d  status[2]=%0d  status[3]=%0d  tag[0]=0x%0h  tag[1]=0x%0h  tag[2]=0x%0h  tag[3]=0x%0h",
                   $time, bank_write_cache_addr,
                   proc_write_select ? "PROC" : "FLSH",
                   bank_write_meta_skip,
                   sync_ctrl_status_q,
                   proc_write_select ? proc_write_cache_way : 4'hF,
                   bank_write_cache_status[0],
                   bank_write_cache_status[1],
                   bank_write_cache_status[2],
                   bank_write_cache_status[3],
                   bank_write_cache_tag[0],
                   bank_write_cache_tag[1],
                   bank_write_cache_tag[2],
                   bank_write_cache_tag[3]);
        end
      end
    end
`endif

    /*****************/
    /*  Cache Banks  */
    /*****************/

    // (declaration of bank_read_cache_addr_q hoisted above the
    //  gen_sync_ctrl_fsm always_comb so the FSM's read-completion guard
    //  can reference it -- vlog requires module-scope declarations before
    //  use in functional always_comb expressions, even though $display
    //  tolerates forward references.)
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            bank_read_cache_addr_q <= '0;
        end else if (bank_read_cache_valid && bank_read_cache_ready) begin
            bank_read_cache_addr_q <= bank_read_cache_addr;
        end
    end

    way_ptr_t [SetAssociativity-1:0] lru_read_data;
    way_ptr_t [SetAssociativity-1:0] lru_meta_unused;

    if (!UseHashWaySelect) begin : gen_lru_rf
        // LRU mode: full register file eliminates meta SRAM write on
        // read hits (the LRU update goes to the RF, not the SRAM).
        way_ptr_t [CacheBankDepth-1:0][SetAssociativity-1:0] lru_rf;

        always_comb begin
            lru_read_data = lru_rf[bank_read_cache_addr_q];
        end

        always_ff @(posedge clk_i or negedge rst_ni) begin
            if (!rst_ni) begin
                lru_rf <= '0;
            end else if (bank_write_LRU_req || bank_write_cache_req) begin
                lru_rf[bank_write_cache_addr] <= bank_write_cache_LRU;
            end
        end
    end else begin : gen_no_lru_rf
        // Hash mode: no LRU RF -- way selection uses hash, not LRU.
        // Read LRU from meta SRAM so the encoder's LRU_array_update
        // still gets input for pendline_cnt tracking.
        assign lru_read_data = lru_meta_unused;
    end

    // -- Dirty register file: true dual-port (1R + 1W per cycle) --
    // Separating dirty from meta SRAM so that on write hits where
    // cache_mask is already all-1s, the meta SRAM write can be
    // skipped entirely (dirty and LRU handled by register files).
    // (declaration of dirty_rf hoisted above the gen_sync_ctrl_fsm
    //  always_comb so the FSM can index it directly for the fresh
    //  dirty mask without going through the meta SRAM read pipeline.)
    logic [SetAssociativity-1:0] dirty_read_data;
    logic [SetAssociativity-1:0] dirty_meta_unused; // discarded dirty from meta SRAM

    // Read port: registered address to match SRAM 1-cycle read latency
    always_comb begin
        dirty_read_data = dirty_rf[bank_read_cache_addr_q];
    end

    // Write port: edge-triggered.
    // With hash way select, only update the TARGET way's dirty bit
    // to prevent the encoder's passthrough from corrupting non-target
    // ways (which were not read from SRAM and may carry stale data).
    // Flush writes still update all ways (proc_write_select = 0).
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            dirty_rf <= '0;
        end else if (bank_write_cache_req) begin
            if (UseHashWaySelect && proc_write_select) begin
                dirty_rf[bank_write_cache_addr][proc_write_cache_way] <=
                    bank_write_cache_dirty[proc_write_cache_way];
            end else begin
                dirty_rf[bank_write_cache_addr] <= bank_write_cache_dirty;
            end
        end
    end

    logic mc_suppress_meta_write;
    assign mc_suppress_meta_write = 1'b0;
    logic mc_hit;
    assign mc_hit = 1'b0;

    // Data forwarding buffer placeholder (for future implementation).
    logic [SetAssociativity-1:0] dbuf_hit;
    assign dbuf_hit = '0;

    for (genvar i = 0; i < SetAssociativity; i++) begin: gen_cache_banks
        always_comb begin
            {bank_read_cache_status[i],
            dirty_meta_unused[i],
            bank_read_cache_miss_meta[i],
            bank_read_cache_mask[i],
            bank_read_cache_tag[i],
            lru_meta_unused[i]} = cache_meta_read_data[i];
            bank_read_cache_LRU[i]   = lru_read_data[i];
            bank_read_cache_dirty[i] = dirty_read_data[i];

            cache_meta_write_data[i] = {
                bank_write_cache_status[i],
                bank_write_cache_dirty[i],
                bank_write_cache_miss_meta[i],
                bank_write_cache_mask[i],
                bank_write_cache_tag[i],
                bank_write_cache_LRU[i]
            };
        end

        // T2.6: the per-way data write request is inlined into
        // upstream_write_req_i below as a flat (proc-arm | flush-broadcast),
        // removing the nested proc_write_select mux + the data_proc_write_req
        // intermediate. Only the targeted way sees the proc write pulse; flush
        // writes still broadcast to every way.

        insitu_cache_bank_access_controller #(
            .DEPTH              (CacheBankDepth),
            .NumWordsPerLine    (CacheLineWidth/WordWidth),
            .WordWidth          (WordWidth),
            .ByteWidth          (ByteWidth),
            // PERF mode: data forwarding buffer + spec-WB + AllowReadDuringWrite.
            // Vector load: ~1 elem/cycle on warm hits (full bandwidth).
            // Vector store: partial-coverage stores absorbed into buffer.
            // KNOWN bug: RLC kernel hits a meta-side MSHR-subarray race
            // (read refill did not reread / Response without outstanding)
            // around 44.85 us; root-cause traced to meta spec-WB and is
            // pending a real fix.  Use the rollback (UseForwardingBuffer=0)
            // for RLC correctness; this config is for vector-rw perf runs.
            .AllowReadDuringWrite(1'b1),
            .UseForwardingBuffer(UseForwardingBuffer),
            .FwdBufEntries      (1),
            .PartSplit          (PartSplit),
            .UseSpecWbIdle      (1'b1),
            .UseSpecWbAddrTrans (1'b1),
            .EnableRawForwarding(DataFwdBufEnableRawForwarding),
            // (b3) Inflight-populate + concurrent same-addr write merge:
            // when a read targets the in-flight SRAM read addr AND a
            // same-addr write is concurrent, serve the read off the
            // populated sram_rdata_i with wr_data overlaid on wr_mask
            // bytes -- saves the redundant SRAM read, returns post-write
            // semantics matching buf_data_q at posedge T+1.
            .EnableInflightWriteMerge (1'b1)
        ) i_access_ctrl_for_data (
            .clk_i,
            .rst_ni,

            .upstream_read_addr_i        (bank_read_cache_addr),
            .upstream_read_valid_i       (bank_read_cache_valid & bank_read_way_mask_sel[i]),
            .upstream_read_ready_o       (data_bank_read_ready[i]),
            .upstream_read_data_o        (bank_read_cache_data[i]),
            .upstream_read_part_idx_i    (bank_read_part_idx_sel),
            .upstream_read_all_parts_i   (bank_read_all_parts_sel),

            .upstream_write_addr_i       (bank_write_cache_addr),
            // T2.6: flat (proc-arm | flush-broadcast). proc_write_cache_req is 0
            // whenever proc_write_select=0, so the proc term vanishes and flush
            // broadcasts to every way -- equivalent to the prior nested mux.
            .upstream_write_req_i        ((proc_write_cache_req & (proc_write_cache_way == way_ptr_t'(i)))
                                          | (~proc_write_select & flush_write_cache_req_valid)),
            .upstream_write_data_i       (bank_write_cache_data[i]),
            .upstream_write_mask_i       (bank_write_data_mask_sel[i]),
            .upstream_write_ready_o      (data_bank_write_ready[i]),
            .upstream_write_target_valid_i (proc_write_target_valid),

            .downstream_read_addr_o      (gnt_data_bank_read_addr[i]),
            .downstream_read_valid_o     (gnt_data_bank_read_valid[i]),
            .downstream_read_ready_i     (gnt_data_bank_read_ready[i]),
            .downstream_read_data_i      (gnt_data_bank_read_data[i]),

            .downstream_write_addr_o     (gnt_data_bank_write_addr[i]),
            .downstream_write_req_o      (gnt_data_bank_write_req[i]),
            .downstream_write_data_o     (gnt_data_bank_write_data[i]),
            .downstream_write_mask_o     (gnt_data_bank_write_mask[i]),

            .bank_gnt_i                  (&(tcdm_data_bank_gnt_i[i])),
            .fwd_wr_hit_o                (data_bank_write_hit[i]),
            .fwd_wr_full_coverage_o      (data_bank_write_full_cov[i])

        );

        pseudo_dual_port_tcdm_wrapper #(
            .DEPTH              (CacheBankDepth),
            .NumPseudoDualBanks (NumPseudoDualBanks),
            .NumWordsPerLine    (CacheLineWidth/WordWidth),
            .WordWidth          (WordWidth),
            .ByteWidth          (ByteWidth),
            .PartSplit          (PartSplit)
        ) i_cache_data_bank (
            .clk_i,
            .rst_ni,

            .read_addr_i        (gnt_data_bank_read_addr[i]),
            .read_valid_i       (gnt_data_bank_read_valid[i]),
            .read_part_idx_i    (bank_read_part_idx_sel),
            .read_all_parts_i   (bank_read_all_parts_sel),
            .read_ready_o       (gnt_data_bank_read_ready[i]),
            .read_data_o        (gnt_data_bank_read_data[i]),

            .write_addr_i       (gnt_data_bank_write_addr[i]),
            .write_req_i        (gnt_data_bank_write_req[i]),
            .write_data_i       (gnt_data_bank_write_data[i]),
            .write_mask_i       (gnt_data_bank_write_mask[i]),

            .tcdm_bank_req_o    (tcdm_data_bank_req_o[i]),
            .tcdm_bank_we_o     (tcdm_data_bank_we_o[i]),
            .tcdm_bank_addr_o   (tcdm_data_bank_addr_o[i]),
            .tcdm_bank_wdata_o  (tcdm_data_bank_wdata_o[i]),
            .tcdm_bank_be_o     (tcdm_data_bank_be_o[i]),
            .tcdm_bank_rdata_i  (tcdm_data_bank_rdata_i[i])
        );

        always_comb begin
            for (int j = 0; j < NumMetaBankPerWay; j++) begin
                tcdm_meta_bank_wdata_o[i][j] = tcdm_meta_wdata_int[i][j];
                tcdm_meta_rdata_int[i][j] = tcdm_meta_bank_rdata_i[i][j];
            end
        end

        // T2.6: the per-way meta write request is inlined into
        // upstream_write_req_i below as a flat (proc-arm | flush-broadcast).
        // LRU-only updates → LRU register file (no meta SRAM write).
        // Write hits on VALID → dirty RF + LRU RF only (meta_skip=1).

        insitu_cache_bank_access_controller #(
            .DEPTH              (CacheBankDepth),
            .NumWordsPerLine    (1),
            .WordWidth          ($bits(cache_meta_t)),
            .ByteWidth          ($bits(cache_meta_t)),
            // Meta path: aligned with the data-side perf knobs --
            // AllowReadDuringWrite=1 and EnableInflightWriteMerge=1 -- so
            // the same-cycle R+W and inflight-write-merge paths cover the
            // secondary-miss MSHR-mask RAW hazard.  (Was AllowReadDuringWrite=0
            // and no inflight merge previously; see the "KNOWN bug" comment
            // in the i_access_ctrl_for_data instantiation for the original
            // meta spec-WB race.)
            .AllowReadDuringWrite (1'b1),
            .UseForwardingBuffer(UseForwardingBuffer),
            .PartSplit          (1),
            .UseSpecWbIdle      (1'b1),
            .UseSpecWbAddrTrans (1'b1),
            .EnableRawForwarding(MetaFwdBufEnableRawForwarding),
            .EnableInflightWriteMerge (1'b1)
        ) i_access_ctrl_for_meta (
            .clk_i,
            .rst_ni,

            .upstream_read_addr_i        (bank_read_cache_addr),
            .upstream_read_valid_i       (bank_read_cache_valid & bank_read_way_mask_sel[i]),
            .upstream_read_ready_o       (meta_bank_read_ready[i]),
            .upstream_read_data_o        (cache_meta_read_data[i]),
            .upstream_read_part_idx_i    ('0),
            .upstream_read_all_parts_i   (1'b1),

            .upstream_write_addr_i       (bank_write_cache_addr),
            // T2.6: flat (proc-arm | flush-broadcast). meta_skip / mc_suppress
            // kept verbatim; proc_write_cache_req=0 when proc_write_select=0.
            .upstream_write_req_i        ((proc_write_cache_req & ~bank_write_meta_skip & ~mc_suppress_meta_write
                                           & (proc_write_cache_way == way_ptr_t'(i)))
                                          | (~proc_write_select & flush_write_cache_req_valid)),
            .upstream_write_data_i       (cache_meta_write_data[i]),
            .upstream_write_mask_i       ('1    ),
            .upstream_write_ready_o      (meta_bank_write_ready[i]),
            // Meta side does not have (D) gating today; tie to 1 (no-op
            // for the existing buffer code paths).
            .upstream_write_target_valid_i (1'b1),

            .downstream_read_addr_o      (gnt_meta_bank_read_addr[i]),
            .downstream_read_valid_o     (gnt_meta_bank_read_valid[i]),
            .downstream_read_ready_i     (gnt_meta_bank_read_ready[i]),
            .downstream_read_data_i      (gnt_meta_bank_read_data[i]),

            .downstream_write_addr_o     (gnt_meta_bank_write_addr[i]),
            .downstream_write_req_o      (gnt_meta_bank_write_req[i]),
            .downstream_write_data_o     (gnt_meta_bank_write_data[i]),
            .downstream_write_mask_o     (gnt_meta_bank_write_mask[i]),

            .bank_gnt_i                  (&(tcdm_data_bank_gnt_i[i])),
            .fwd_wr_hit_o                (),
            .fwd_wr_full_coverage_o      ()

        );

        pseudo_dual_port_tcdm_wrapper #(
            .DEPTH              (CacheBankDepth),
            .NumPseudoDualBanks (NumPseudoDualBanks),
            .NumWordsPerLine    (1),
            .WordWidth          ($bits(cache_meta_t)),
            .ByteWidth          ($bits(cache_meta_t)),
            .PartSplit          (1)
        ) i_cache_meta_bank (
            .clk_i,
            .rst_ni,

            .read_addr_i        (gnt_meta_bank_read_addr[i]),
            .read_valid_i       (gnt_meta_bank_read_valid[i]),
            .read_part_idx_i    ('0),
            .read_all_parts_i   (1'b1),
            .read_ready_o       (gnt_meta_bank_read_ready[i]),
            .read_data_o        (gnt_meta_bank_read_data[i]),

            .write_addr_i       (gnt_meta_bank_write_addr[i]),
            .write_req_i        (gnt_meta_bank_write_req[i]),
            .write_data_i       (gnt_meta_bank_write_data[i]),
            .write_mask_i       (gnt_meta_bank_write_mask[i]),

            .tcdm_bank_req_o    (tcdm_meta_bank_req_o[i]),
            .tcdm_bank_we_o     (tcdm_meta_bank_we_o[i]),
            .tcdm_bank_addr_o   (tcdm_meta_bank_addr_o[i]),
            .tcdm_bank_wdata_o  (tcdm_meta_wdata_int[i]),
            .tcdm_bank_be_o     (tcdm_meta_bank_be_o[i]),
            .tcdm_bank_rdata_i  (tcdm_meta_rdata_int[i])
        );

        assign bank_read_cache_ready_per_way[i] = bank_read_way_mask_sel[i] ?
                                                   (bank_read_data_skip ?
                                                     meta_bank_read_ready[i] :
                                                     (data_bank_read_ready[i] & meta_bank_read_ready[i])) :
                                                   1'b1;
    end

    assign bank_read_cache_ready = &bank_read_cache_ready_per_way;

`ifndef TARGET_SYNTHESIS
    // ---------------------------------------------------------------------
    // Verification IP: per-controller scoreboard.
    // Passive observer — drives no RTL signal.  Reports phantom hits,
    // wrong-data-on-hit, and way mismatches via $error.  See
    //   working_dir/insitu-cache/src/verif/insitu_cache_scoreboard.sv
    // ---------------------------------------------------------------------
    // MSHR sub-entry count = low SubarrayCntWidth bits of the meta mask.
    // Recompute the cache_core's localparams locally so the SB doesn't have
    // to re-derive them.  Keep these in sync with insitu_cache_core.sv.
    localparam int unsigned SBInfoWidth         = $bits(info_t);
    localparam int unsigned SBInfoStoreWidth    = ((SBInfoWidth + ByteWidth - 1) / ByteWidth) * ByteWidth;
    localparam int unsigned SBSubCntCounterW    = CacheLineWidth/WordWidth;
    localparam int unsigned SBMaxNumSubarrayRaw = CacheLineWidth/SBInfoStoreWidth;
    localparam int unsigned SBMaxNumSubarray    =
        (SBMaxNumSubarrayRaw > 0 && (CacheLineWidth % SBInfoStoreWidth) == 0)
            ? (SBMaxNumSubarrayRaw - 1) : SBMaxNumSubarrayRaw;
    localparam int unsigned SBNumSubarray       =
        SBMaxNumSubarray > (2**SBSubCntCounterW)-2 ? (2**SBSubCntCounterW)-2 : SBMaxNumSubarray;
    localparam int unsigned SBSubarrayCntWidth  =
        (SBNumSubarray > 0) ? $clog2(SBNumSubarray + 1) : 1;

    insitu_cache_scoreboard #(
        .CacheBankDepth      (CacheBankDepth    ),
        .SetAssociativity    (SetAssociativity  ),
        .CacheLineWidth      (CacheLineWidth    ),
        .MaskWidth           (CacheLineWidth/8  ),
        .ReqAddrWidth        ($bits(addr_t)     ),
        .CacheTagWidth       ($bits(cache_tag_t)),
        .UpstreamDataWidth   ($bits(upstream_data_t)),
        .UpstreamMaskWidth   ($bits(cache_mask_t)),
        .InfoWidth           ($bits(info_t)     ),
        .DownstreamDataWidth ($bits(downstream_data_t)),
        .DownstreamInfoWidth ($bits(downstream_info_t)),
        .MetaMaskWidth       ($bits(cache_mask_t)),
        .SubarrayCntWidth    (SBSubarrayCntWidth),
        .CtrlName            (ModeleName        )
    ) i_scoreboard (
        .clk_i               (clk_i ),
        .rst_ni              (rst_ni),

        .flush_commit_valid  (flush_write_cache_req_valid &&  bank_write_cache_ready
                                                       && !proc_write_select),
        .flush_commit_addr   (flush_write_cache_addr   ),
        .flush_commit_status (flush_write_cache_status ),
        .flush_commit_dirty  (flush_write_cache_dirty  ),
        .flush_commit_tag    (flush_write_cache_tag    ),

        .proc_commit_valid   (bank_write_cache_req && bank_write_cache_ready
                                                  && proc_write_select),
        .proc_commit_addr    (bank_write_cache_addr                          ),
        .proc_commit_way     (proc_write_cache_way                           ),
        .proc_commit_status  (bank_write_cache_status[proc_write_cache_way]  ),
        .proc_commit_dirty   (bank_write_cache_dirty [proc_write_cache_way]  ),
        .proc_commit_tag     (bank_write_cache_tag   [proc_write_cache_way]  ),
        .proc_commit_data    (bank_write_cache_data  [proc_write_cache_way]  ),
        // Use the DATA-side mask (not the meta mask).  For refills the cache
        // core sets data_mask = all-1s (writing the whole line); for write
        // hits it sets the per-byte mask.
        .proc_commit_mask    (bank_write_data_mask   [proc_write_cache_way]  ),

        .dec_valid           ( i_insitu_cache_core.preread_task_q.valid
                            && !i_insitu_cache_core.preread_task_q.is_refill
                            && !i_insitu_cache_core.preread_task_q.task_pay.request.write ),
        .dec_addr            ( i_insitu_cache_core.preread_task_q.task_pay.request.addr  ),
        .dec_is_hit          ( i_insitu_cache_core.dec_is_hit                            ),
        .dec_way             ( i_insitu_cache_core.dec_way                               ),
        .dec_data            ( i_insitu_cache_core.dec_cache_data                        ),
        .dec_info            ( i_insitu_cache_core.preread_task_q.task_pay.request.info  ),

        // MSHR snoop: dec_is_hit_pend + raw mask read, plus the meta mask
        // being committed (mirrored by the SB to predict what dec_cache_mask
        // should read on the NEXT secondary-miss decode).
        .dec_is_hit_pend     ( i_insitu_cache_core.dec_is_hit_pend                       ),
        .dec_cache_mask      ( i_insitu_cache_core.dec_cache_mask                        ),
        .proc_commit_meta_mask( bank_write_cache_mask[proc_write_cache_way]              ),

        // Upstream request snoop -- the addr fed to the cache core (already
        // hashed by cache_addr_hashing()) and its accept handshake.  Captures
        // each scalar/vector request as it enters the cache.
        .upreq_valid         (upstream_req_to_cache_valid                                ),
        .upreq_ready         (upstream_req_to_cache_ready                                ),
        .upreq_addr          (upstream_req_to_cache_payload.addr                         ),
        .upreq_write         (upstream_req_to_cache_payload.write                        ),
        .upreq_wdata         (upstream_req_to_cache_payload.wdata                        ),
        .upreq_wmask         (upstream_req_to_cache_payload.wmask                        ),
        .upreq_info          (upstream_req_to_cache_payload.info                         ),

        // Upstream response snoop -- the wrapper's actual output back to
        // the cache_ctrl.  Lets the SB perform an end-to-end RAW check:
        // each fired response is matched (by info) to a pending request and
        // its data is cross-checked against the SB-tracked line.
        .upresp_valid        (upstream_resp_valid_o                                       ),
        .upresp_ready        (upstream_resp_ready_i                                       ),
        .upresp_write        (upstream_resp_write_o                                       ),
        .upresp_data         (upstream_resp_data_o                                        ),
        .upresp_info         (upstream_resp_info_o                                        ),

        // Downstream refill snoop (verif-only).  Match req fires to resp
        // fires by info-id to recover the addr of each refill, then
        // populate the SB shadow with the refill's line data the moment
        // the response lands at the wrapper boundary.
        .dwn_req_valid       (downstream_req_valid_o                                      ),
        .dwn_req_ready       (downstream_req_ready_i                                      ),
        .dwn_req_addr        (downstream_req_addr_o                                       ),
        .dwn_req_info        (downstream_req_info_o                                       ),
        .dwn_req_write       (downstream_req_write_o                                      ),
        .dwn_resp_valid      (downstream_resp_valid_i                                     ),
        .dwn_resp_ready      (downstream_resp_ready_o                                     ),
        .dwn_resp_data       (downstream_resp_data_i                                      ),
        .dwn_resp_info       (downstream_resp_info_i                                      ),
        .dwn_resp_write      (downstream_resp_write_i                                     )
    );
`endif

endmodule





module pseudo_dual_port_tcdm_wrapper #(
    /// Address depth
    parameter int unsigned  DEPTH                   = 512,
    /// Number of banks
    parameter int unsigned  NumPseudoDualBanks      = 2,
    /// Number of words
    parameter int unsigned  NumWordsPerLine         = 2,
    /// Number of parts per line (1 = no part gating).
    parameter int unsigned  PartSplit               = 1,
    /// Width of word
    parameter int unsigned  WordWidth               = 32,
    /// Width of byte (granularity of byte mask)
    parameter int unsigned  ByteWidth               = 8,
    /// Dependent parameter, do not override. data type
    localparam type         data_t                  = logic [WordWidth*NumWordsPerLine-1:0],
    /// Dependent parameter, do not override. word type
    localparam type         word_t                  = logic [WordWidth-1:0],
    // Dependent parameter, do not override. Byte mask type.
    localparam type         mask_t                  = logic [NumWordsPerLine*WordWidth/ByteWidth-1:0],
    // Dependent parameter, do not override. Address type.
    localparam type         addr_t                  = logic [$clog2(DEPTH)-1:0],
    // Dependent parameter, do not override. number of banks
    localparam int unsigned NumBanks                = NumPseudoDualBanks * NumWordsPerLine,
    // Dependent parameter, do not override. words per part.
    localparam int unsigned PartWords               = (PartSplit == 0) ? NumWordsPerLine : (NumWordsPerLine / PartSplit),
    // Dependent parameter, do not override. part index width.
    localparam int unsigned PartIdxWidth            = (PartSplit > 1) ? $clog2(PartSplit) : 1,
    // Dependent parameter, do not override. Address type.
    localparam int unsigned SELECT_DEPTH            = (NumPseudoDualBanks > 1) ? $clog2(NumPseudoDualBanks) : 1,
    // Dependent parameter, do not override. Select type.
    localparam type         bank_select_t           = logic [SELECT_DEPTH-1:0],
    // Dependent parameter, do not override. Address type.
    localparam type         bank_addr_t             = logic [$clog2(DEPTH)-$clog2(NumPseudoDualBanks)-1:0]
    )(
    /// Clock, positive edge triggered.
    input  logic                                    clk_i,
    /// Reset, active low.
    input  logic                                    rst_ni,

    /// Read port
    input  addr_t                                   read_addr_i,
    input  logic                                    read_valid_i,
    input  logic        [PartIdxWidth-1:0]          read_part_idx_i,
    input  logic                                    read_all_parts_i,
    output logic                                    read_ready_o,
    output data_t                                   read_data_o,

    /// write port
    input  addr_t                                   write_addr_i,
    input  logic                                    write_req_i,
    input  data_t                                   write_data_i,
    input  mask_t                                   write_mask_i,

    /// bank ports
    output logic         [NumBanks-1:0]             tcdm_bank_req_o,
    output logic         [NumBanks-1:0]             tcdm_bank_we_o,
    output bank_addr_t   [NumBanks-1:0]             tcdm_bank_addr_o,
    output word_t        [NumBanks-1:0]             tcdm_bank_wdata_o,
    output logic         [NumBanks-1:0][WordWidth/ByteWidth-1:0] tcdm_bank_be_o,
    input  word_t        [NumBanks-1:0]             tcdm_bank_rdata_i

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

    //////////////////////////////////////
    //        Signal Definition         //
    //////////////////////////////////////

    localparam int unsigned                       WordBytes = WordWidth/ByteWidth;

    //status
    pseudo_dual_status_t                            status;

    //write line buffer
    data_t write_line_buffer;
    `FFARN (write_line_buffer, write_data_i, '0,    clk_i, rst_ni)

    //bank signals
    logic         [NumPseudoDualBanks-1:0]          bank_req_read;
    logic                                           read_issue; // T2.5: flat read-enable term
    logic         [NumPseudoDualBanks-1:0]          bank_req_write;
    bank_addr_t   [NumPseudoDualBanks-1:0]          bank_addr;
    data_t        [NumPseudoDualBanks-1:0]          bank_wdata;
    mask_t        [NumPseudoDualBanks-1:0]          bank_wmask;
    data_t        [NumPseudoDualBanks-1:0]          bank_rdata;
    logic                                           write_has_data;
    word_t        [NumPseudoDualBanks-1:0][NumWordsPerLine-1:0] bank_wdata_words;
    word_t        [NumPseudoDualBanks-1:0][NumWordsPerLine-1:0] bank_rdata_words;
    logic         [NumPseudoDualBanks-1:0][NumWordsPerLine-1:0] word_in_part;
    logic         [NumPseudoDualBanks-1:0][NumWordsPerLine-1:0] word_read_en;
    logic         [NumPseudoDualBanks-1:0][NumWordsPerLine-1:0] word_read_en_q;
    logic         [NumPseudoDualBanks-1:0][NumWordsPerLine-1:0] word_write_en;
    // T2.4: per-word "has write data" reduce, precomputed once from write_mask_i
    // (no pseudo-bank dependence) -- see use at word_write_en below.
    logic         [NumWordsPerLine-1:0]                         word_has_wmask;

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
    logic         [NumPseudoDualBanks-1:0][NumWordsPerLine-1:0] word_write_en_q;
    word_t        [NumWordsPerLine-1:0]  write_line_buffer_words;
    assign write_line_buffer_words = write_line_buffer;

    `FFARN (word_read_en_q,
            word_read_en,
            '0, clk_i, rst_ni)
    `FFARN (word_write_en_q,
            word_write_en,
            '0, clk_i, rst_ni)

    // synopsys translate_off
    initial begin
        if ((PartSplit > 1) && ((NumWordsPerLine % PartSplit) != 0)) begin
            $fatal(1, "PartSplit (%0d) must divide NumWordsPerLine (%0d).",
                   PartSplit, NumWordsPerLine);
        end
    end
    // synopsys translate_on

    //////////////////////////////////////
    //        Instance Modules          //
    //////////////////////////////////////

    // T2.4: precompute the per-word byte-mask OR once, directly from the module
    // input write_mask_i, so the WordBytes-wide reduce resolves in parallel with
    // the bank-select demux instead of being serialized AFTER it (it currently
    // OR-reduces the demuxed bank_wmask[i]).  See word_write_en below.
    for (genvar j = 0; j < NumWordsPerLine; j++) begin : gen_word_has_wmask
        assign word_has_wmask[j] = |write_mask_i[j*WordBytes +: WordBytes];
    end

    for (genvar i = 0; i < NumPseudoDualBanks; i++) begin
        assign bank_wdata_words[i] = bank_wdata[i];
        for (genvar j = 0; j < NumWordsPerLine; j++) begin
            localparam int unsigned WordPart = (PartSplit > 1) ? (j / PartWords) : 0;
            assign word_in_part[i][j] = read_all_parts_i ? 1'b1 :
              ((PartSplit > 1) ? (read_part_idx_i == WordPart[PartIdxWidth-1:0]) : 1'b1);
            // T2.3: bank_req_read[i] is asserted only in the read_valid_i-gated
            // status arms (R_ONLY / WR_DIFF_BANK / WR_SAME_ADDR), so it already
            // implies read_valid_i -- drop the redundant term.  Read-side mirror
            // of T2.2: removes one series AND on the latest net (read_valid_i)
            // right at the SRAM clock-gate enable and halves its endpoint load.
            // INVARIANT: never assert bank_req_read[i] outside a read_valid_i
            // -gated arm, or this redundancy becomes a real (missing) term.
            assign word_read_en[i][j] = bank_req_read[i] & word_in_part[i][j];
            // T2.2: bank_req_write[i] is set only inside `if (write_has_data)`,
            // so it already implies write_has_data -- drop the redundant term
            // (smaller endpoint AND fan-in, lower write_has_data load).
            // T2.4: use the precomputed word_has_wmask[j] instead of OR-reducing
            // the demuxed bank_wmask[i].  Identical: bank_wmask[i] is '0 except
            // at write_bank_select (== write_mask_i there) and bank_req_write[i]
            // =1 only there, so bank_req_write[i] & |bank_wmask[i][word]
            // === bank_req_write[i] & |write_mask_i[word].
            assign word_write_en[i][j] = bank_req_write[i] & word_has_wmask[j];
            assign tcdm_bank_req_o[i*NumWordsPerLine + j]    = word_read_en[i][j] | word_write_en[i][j];
            assign tcdm_bank_we_o[i*NumWordsPerLine + j]     = word_write_en[i][j];
            assign tcdm_bank_addr_o[i*NumWordsPerLine + j]   = bank_addr[i];
            assign tcdm_bank_wdata_o[i*NumWordsPerLine + j]  = bank_wdata_words[i][j];
            assign tcdm_bank_be_o[i*NumWordsPerLine + j]     = bank_wmask[i][j*WordBytes +: WordBytes];
            // Per-word forwarding: when a TCDM bank was both read and written
            // (WR_SAME_ADDR overlap), the SRAM port was used for the write, so
            // forward from write_line_buffer instead of unreliable SRAM rdata.
            assign bank_rdata_words[i][j] =
                (word_read_en_q[i][j] & word_write_en_q[i][j]) ? write_line_buffer_words[j] :
                 word_read_en_q[i][j]                           ? tcdm_bank_rdata_i[i*NumWordsPerLine + j] :
                                                                  '0;
        end

        assign bank_rdata[i] = bank_rdata_words[i];
    end

    //////////////////////////////////////
    //        Pseudo Dual Logics        //
    //////////////////////////////////////

    always_comb begin : proc_pseudo_dual
        /*****************/
        /* Defualt Value */
        /*****************/
        status = IDLE;

        bank_req_read = '0;
        bank_req_write = '0;
        bank_addr = '0;
        bank_wdata = '0;
        bank_wmask = '0;

        if (NumPseudoDualBanks <= 1) begin
            read_bank_select = '0;
            write_bank_select = '0;
            read_bank_addr = read_addr_i;
            write_bank_addr = write_addr_i;
        end else begin
            {read_bank_addr,    read_bank_select}   = read_addr_i;
            {write_bank_addr,   write_bank_select}  = write_addr_i;
        end

        read_ready_o = 1'b1;
        read_data_from_line_buffer_d = '0;
        read_data_from_bank_select_d = '0;

        write_has_data = write_req_i & (|write_mask_i);

        /*******************/
        /* Determin Status */
        /*******************/
        if (read_valid_i & write_has_data) begin
            if (read_bank_select != write_bank_select) begin
                status = WR_DIFF_BANK;
            end else if (read_bank_addr == write_bank_addr) begin
                // Same pseudo-bank, same address: issue both read and write.
                // Non-overlapping TCDM words proceed independently (different
                // parts of the line).  For overlapping words (same part),
                // per-word forwarding from write_line_buffer provides correct
                // data even when the SRAM port is used for the write.
                status = WR_SAME_ADDR;
            end else begin
                status = WR_CONFLICT;
            end
        end else 
        if (read_valid_i) begin
            status = R_ONLY;
        end else
        if (write_has_data) begin
            status = W_ONLY;
        end

        /******************************************************************/
        /* WRITE side -- write-priority, INDEPENDENT of read arbitration. */
        /******************************************************************/
        // Hoisted OUT of case(status) so bank_req_write / tcdm_bank_we_o do NOT
        // combinationally depend on read_valid_i.  This breaks the (false)
        // grant-feedback combinational loop reported by lint:
        //   read_valid_i -> status -> bank_req_write -> tcdm_bank_we_o
        //     -> (tile) part_we -> any_other_write_in_col -> l1_data_bank_gnt
        //     -> bank_gnt_i -> read_valid_i
        // Behaviour is bit-identical: every write-bearing status (W_ONLY /
        // WR_DIFF_BANK / WR_SAME_ADDR / WR_CONFLICT) already asserted exactly
        // these signals with the same write_bank_addr/data/mask -- a write is
        // always issued when it has data; the read never alters the write that
        // is granted.
        if (write_has_data) begin
            bank_req_write[write_bank_select] = 1'b1;
            bank_addr[write_bank_select]      = write_bank_addr;
            bank_wdata[write_bank_select]     = write_data_i;
            bank_wmask[write_bank_select]     = write_mask_i;
        end

        /************/
        /* Read FSM */
        /************/
        // Read issue depends on the shared-bank arbitration status.  Reads do
        // NOT feed the tile grant (gnt uses part_we only), so this side is
        // loop-free.
        case (status)
            R_ONLY, WR_DIFF_BANK: begin
                // Read targets a bank distinct from any concurrent write (or
                // there is no write): drive the read address directly.  In
                // WR_DIFF_BANK read_bank_select != write_bank_select, so this
                // does not collide with the write's bank_addr above.
                // T2.5: bank_req_read driven by the flat one-hot after endcase.
                bank_addr[read_bank_select]     = read_bank_addr;
                read_data_from_line_buffer_d    = '0;
                read_data_from_bank_select_d    = read_bank_select;
            end

            WR_SAME_ADDR: begin
                // Same pseudo-bank AND same address as the write: issue the
                // read sharing the write's bank_addr (read_bank_addr ==
                // write_bank_addr here).  Overlapping words forward from
                // write_line_buffer via the per-word bypass.
                // T2.5: bank_req_read driven by the flat one-hot after endcase.
                read_data_from_line_buffer_d    = '0;
                read_data_from_bank_select_d    = read_bank_select;
            end

            WR_CONFLICT: begin
                // Same bank, different address: write wins, read retries next
                // cycle.
                read_ready_o                 = 1'b0;
                read_data_from_bank_select_d = '0;
            end

            default : /* W_ONLY / IDLE: no read issued */;
        endcase

        // T2.5: drive bank_req_read as a flat one-hot instead of the status-enum
        // encode -> case-decode round-trip, so the late read_valid_i is the FINAL
        // AND on the path to the SRAM clock-gate enable.
        //   read issued === status in {R_ONLY, WR_DIFF_BANK, WR_SAME_ADDR}
        //              === read_valid_i & ~(write_has_data & same-bank & diff-addr)
        // (absorption: ~wd | wd&~C === ~(wd&C)).  The wide read/write bank-addr
        // compare resolves from early write operands; read_valid_i ANDs in last.
        // bank_req_read defaults to '0 above; only read_bank_select is set here.
        read_issue = read_valid_i &
            ~(write_has_data & (read_bank_select == write_bank_select)
                             & (read_bank_addr   != write_bank_addr));
        bank_req_read[read_bank_select] = read_issue;
    end

    assign read_data_o = read_data_from_line_buffer_q? write_line_buffer: bank_rdata[read_data_from_bank_select_q];

endmodule : pseudo_dual_port_tcdm_wrapper


module insitu_cache_bank_access_controller #(
    /// Address depth
    parameter int unsigned  DEPTH                   = 512,
    /// Number of words
    parameter int unsigned  NumWordsPerLine         = 2,
    /// Width of word
    parameter int unsigned  WordWidth               = 32,
    /// Width of byte (granularity of byte mask)
    parameter int unsigned  ByteWidth               = 8,
    /// Allow reads to proceed during writes (safe when the downstream
    /// pseudo_dual_port has WR_SAME_ADDR bypass and there is no folded
    /// banking that could silently drop the read).
    parameter bit           AllowReadDuringWrite    = 1'b0,
    /// Enable write-back forwarding buffer.  Buffer caches N SRAM rows;
    /// matching reads return buffer data, writes merge into buffer.
    /// Dirty data written back on eviction.
    parameter bit           UseForwardingBuffer     = 1'b0,
    /// Number of entries in the forwarding buffer (>=1).  1 uses the
    /// legacy single-entry module; >1 uses the multi-entry module with
    /// SpecWb gated on buffer fullness for real multi-entry utilisation.
    parameter int unsigned  FwdBufEntries           = 1,
    /// Number of parts per cache line (1 = no part gating).
    /// Forwarding buffer tracks which part is cached and only reports
    /// hits for that part, avoiding stale data with PartSplit > 1.
    parameter int unsigned  PartSplit               = 1,
    /// Speculative writeback: issue dirty-buffer writeback alongside
    /// normal reads when the SRAM port is available.  The pseudo_dual_port
    /// resolves R/W conflicts (WR_DIFF_BANK proceeds in 1 cycle).
    /// Option A: writeback when buffer dirty and no upstream write.
    parameter bit           UseSpecWbIdle           = 1'b0,
    /// Option C: writeback when incoming read address differs from buffer
    /// address (predict eviction, overlap writeback with miss fetch).
    parameter bit           UseSpecWbAddrTrans      = 1'b0,
    /// Read-after-write forwarding within the forwarding buffer.
    /// 0: same-cycle read on a same-line write hit returns pre-write data.
    /// 1: same-cycle read returns post-write merged data (adds a wide
    ///    byte-mask mux from wr_data into buf_rd_data_q).
    parameter bit           EnableRawForwarding     = 1'b0,
    /// (b3) Inflight-populate + concurrent same-addr write merge.
    /// When a read targets the in-flight SRAM read addr AND a same-addr
    /// write is concurrent, serve the read off the populated
    /// sram_rdata_i with wr_data overlaid on wr_mask bytes -- saves the
    /// redundant SRAM read, returns post-write semantics matching
    /// buf_data_q at posedge T+1.
    parameter bit           EnableInflightWriteMerge = 1'b0,
    /// Dependent parameter, do not override. data type
    localparam type         data_t                  = logic [WordWidth*NumWordsPerLine-1:0],
    /// Dependent parameter, do not override. Byte mask type.
    localparam type         mask_t                  = logic [NumWordsPerLine*WordWidth/ByteWidth-1:0],
    /// Dependent parameter, do not override. word type
    localparam type         word_t                  = logic [WordWidth-1:0],
    // Dependent parameter, do not override. Address type.
    localparam type         addr_t                  = logic [$clog2(DEPTH)-1:0],
    // Dependent parameter, do not override. Part index width.
    localparam int unsigned PartIdxWidth             = (PartSplit > 1) ? $clog2(PartSplit) : 1
    )(
    /// Clock, positive edge triggered.
    input  logic                                    clk_i,
    /// Reset, active low.
    input  logic                                    rst_ni,

    /// Upstream Read port
    input  addr_t                                   upstream_read_addr_i,
    input  logic                                    upstream_read_valid_i,
    output logic                                    upstream_read_ready_o,
    output data_t                                   upstream_read_data_o,
    input  logic [PartIdxWidth-1:0]                 upstream_read_part_idx_i,
    input  logic                                    upstream_read_all_parts_i,

    /// Upstream write port
    input  addr_t                                   upstream_write_addr_i,
    input  logic                                    upstream_write_req_i,
    input  data_t                                   upstream_write_data_i,
    input  mask_t                                   upstream_write_mask_i,
    /// Backpressure: when low, the upstream write must NOT be consumed this
    /// cycle (i.e., the cache controller must hold the request and replay
    /// it next cycle).  Phase 1: always 1 (always-ready, behavior identical
    /// to pre-handshake).  Phase 2 will lower it for transient buffer states
    /// (wb_active, populate-with-conflict, etc.) so that ACCUMULATE-CONCURRENT-
    /// MERGE-class optimizations can be added safely in later phases.
    output logic                                    upstream_write_ready_o,
    /// Phase 3 advisory: the line being written is currently in VALID state
    /// (already cached).  When 1, the buffer's ACCUMULATE-CONCURRENT-MERGE
    /// branch may safely fire (preserving old parts).  When 0 (line is
    /// INVALID/PEND), the buffer falls back to REPLACE-merge to avoid
    /// confusing the cache controller's status-array protocol.
    input  logic                                    upstream_write_target_valid_i,

    /// Downstream Read port
    output addr_t                                   downstream_read_addr_o,
    output logic                                    downstream_read_valid_o,
    input  logic                                    downstream_read_ready_i,
    input  data_t                                   downstream_read_data_i,

    /// Downstream write port
    output addr_t                                   downstream_write_addr_o,
    output logic                                    downstream_write_req_o,
    output data_t                                   downstream_write_data_o,
    output mask_t                                   downstream_write_mask_o,

    // Bank Access Grant
    input  logic                                    bank_gnt_i,
    // Forwarding buffer write hit (combinational, for hazard relaxation)
    output logic                                    fwd_wr_hit_o,
    // Forwarding buffer FULL-coverage hit: 1 iff this absorption leaves the
    // buffer holding the WHOLE line.  Cache core must use this -- not
    // fwd_wr_hit_o -- to bypass the bank-write/upstream-read same-line
    // hazard, because partial-coverage absorptions leave OTHER parts in
    // SRAM where a stale-data read would otherwise sneak through.
    output logic                                    fwd_wr_full_coverage_o
);
    //////////////////////////////////////
    //        Types Definition          //
    //////////////////////////////////////

    typedef enum logic {
        ACCESS_THROUGH = '0,
        ACCESS_STALL
    } access_status_t;

    //////////////////////////////////////
    //        Signal Definition         //
    //////////////////////////////////////

    access_status_t access_status_q, access_status_d;
    data_t          access_stall_data_q, access_stall_data_d;
    addr_t          access_stall_addr_q, access_stall_addr_d;
    mask_t          access_stall_mask_q, access_stall_mask_d;
    `FFARN (access_status_q, access_status_d, ACCESS_THROUGH, clk_i, rst_ni)
    `FFARN (access_stall_data_q, access_stall_data_d, '0, clk_i, rst_ni)
    `FFARN (access_stall_addr_q, access_stall_addr_d, '0, clk_i, rst_ni)
    `FFARN (access_stall_mask_q, access_stall_mask_d, '0, clk_i, rst_ni)

    // -- Writeback state (forwarding buffer active mode) --
    // wb_active: writeback write in progress (overrides main FSM outputs).
    // wb_has_stall: after writeback, go to ACCESS_STALL to resend the
    //              original write that triggered the dirty eviction.
    logic wb_active_q, wb_active_d;
    logic wb_has_stall_q, wb_has_stall_d;
    `FFARN (wb_active_q, wb_active_d, 1'b0, clk_i, rst_ni)
    `FFARN (wb_has_stall_q, wb_has_stall_d, 1'b0, clk_i, rst_ni)

    //////////////////////////////////////
    //     Forwarding Buffer Instance   //
    //////////////////////////////////////

    data_t fwd_rdata;
    logic  fwd_hit;
    logic  fwd_rd_hit;        // combinational: buffer can serve this read
    logic  fwd_wr_hit;        // combinational: buffer can absorb this write
    logic  fwd_wr_full_cov;   // combinational: absorption gives full-line coverage
    logic  fwd_wb_needed;     // buffer dirty
    addr_t fwd_wb_addr;       // writeback address
    data_t fwd_wb_data;       // writeback data
    mask_t fwd_wb_mask;       // writeback byte mask (only cached parts)

    // Multi-entry-specific signals (only driven when FwdBufEntries>1).
    // buf_has_free_clean: at least one entry is invalid or clean, so the
    //   next allocation won't force a writeback.  SpecWb uses this to
    //   skip draining until the buffer is actually pressured.
    logic  buf_has_free_clean;

    // -- Speculative writeback --
    // Fire-and-forget: issue writeback alongside normal read when the
    // SRAM port is available.  No stall on denial.
    logic spec_wb_fire;

    // Writeback done: fires on explicit writeback OR speculative writeback.
    // bank_gnt_i is `&(tcdm_data_bank_gnt_i[i])` (all parts of way i).  In
    // wb_active we only write (no read), and the tile arbiter unconditionally
    // grants our write (we=1 forces gnt=1).  bank_gnt_i can drop to 0 when
    // OTHER ways are writing our idle parts' columns -- but those parts
    // are not being touched by us.  Gating wb_done on bank_gnt_i would
    // spuriously stall writeback completion (and lose the writeback) on
    // such unrelated denials.  Drop the gate.
    logic  fwd_wb_done;
    assign fwd_wb_done = wb_active_q | spec_wb_fire;

    // Expose write hit for upstream hazard relaxation.
    assign fwd_wr_hit_o = fwd_wr_hit;
    assign fwd_wr_full_coverage_o = fwd_wr_full_cov;

    // Phase 2 of the handshake: deassert ready when the access ctrl is in
    // a transient state where it cannot safely accept a new upstream write
    // without losing data.  Three deassertion conditions:
    //   (i)   wb_active_q=1: the writeback overlay is firing this cycle;
    //         the FSM logic doesn't run, so a new write would be lost.
    //   (ii)  ACCESS_STALL: the saved write from a prior Path A is being
    //         replayed; same FSM-overlay reasoning.
    //   (iii) Path A is about to fire combinationally this cycle: the
    //         current write IS captured (into stall regs), but a SECOND
    //         concurrent write would be lost.  In practice the cache
    //         controller drives at most one write per cycle, so this
    //         condition currently never matters; included for safety.
    // Phase 3 will add a (D)-related condition gated on cache-status info.
    assign upstream_write_ready_o = !wb_active_q
                                  & (access_status_q != ACCESS_STALL);

    generate
    if (FwdBufEntries <= 1) begin : gen_fwd_buf_single
        // Legacy single-entry module (preserves existing behaviour).
        // buf_has_free_clean is tied to 0 so SpecWb fires whenever dirty
        // -- same as the original gating.
        assign buf_has_free_clean = 1'b0;

        sram_forwarding_buffer #(
            .Depth          (DEPTH),
            .NumWordsPerLine(NumWordsPerLine),
            .WordWidth      (WordWidth),
            .ByteWidth      (ByteWidth),
            .Enable         (UseForwardingBuffer),
            .PartSplit      (PartSplit),
            .EnableRawForwarding(EnableRawForwarding),
            .EnableInflightWriteMerge(EnableInflightWriteMerge)
        ) i_fwd_buf (
            .clk_i,
            .rst_ni,
            .rd_addr_i      (upstream_read_addr_i),
            .rd_valid_i     (upstream_read_valid_i),
            .rd_ready_i     (upstream_read_ready_o),
            .rd_part_idx_i  (upstream_read_part_idx_i),
            .rd_all_parts_i (upstream_read_all_parts_i),
            .wr_addr_i      (upstream_write_addr_i),
            .wr_data_i      (upstream_write_data_i),
            .wr_mask_i      (upstream_write_mask_i),
            .wr_req_i       (upstream_write_req_i),
            .sram_rd_issued_i(downstream_read_valid_o & downstream_read_ready_i),
            .sram_rdata_i    (downstream_read_data_i),
            .sram_wr_req_i   (downstream_write_req_o),
            .sram_wr_addr_i  (downstream_write_addr_o),
            .rd_hit_comb_o  (fwd_rd_hit),
            .wr_hit_comb_o  (fwd_wr_hit),
            .wr_full_coverage_o(fwd_wr_full_cov),
            .wb_needed_o    (fwd_wb_needed),
            .wb_addr_o      (fwd_wb_addr),
            .wb_data_o      (fwd_wb_data),
            .wb_mask_o      (fwd_wb_mask),
            .wb_done_i      (fwd_wb_done),
            .wr_target_valid_i (upstream_write_target_valid_i),
            .fwd_rdata_o    (fwd_rdata),
            .fwd_hit_o      (fwd_hit),
            .stat_rd_hit_o  (),
            .stat_rd_miss_o (),
            .stat_wr_merge_o(),
            .stat_wr_inval_o(),
            .stat_rd_total_o(),
            .stat_wr_total_o(),
            .stat_sram_rd_o (),
            .stat_wb_o      ()
        );
    end else begin : gen_fwd_buf_multi
        // Multi-entry module -- drives buf_has_free_clean so SpecWb only
        // fires when the buffer is actually pressured (all entries dirty).
        logic buf_near_full_unused;

        sram_forwarding_buffer_multi #(
            .Depth          (DEPTH),
            .NumWordsPerLine(NumWordsPerLine),
            .WordWidth      (WordWidth),
            .ByteWidth      (ByteWidth),
            .Enable         (UseForwardingBuffer),
            .PartSplit      (PartSplit),
            .NumEntries     (FwdBufEntries),
            .EnableRawForwarding(EnableRawForwarding)
        ) i_fwd_buf (
            .clk_i,
            .rst_ni,
            .rd_addr_i      (upstream_read_addr_i),
            .rd_valid_i     (upstream_read_valid_i),
            .rd_ready_i     (upstream_read_ready_o),
            .rd_part_idx_i  (upstream_read_part_idx_i),
            .rd_all_parts_i (upstream_read_all_parts_i),
            .wr_addr_i      (upstream_write_addr_i),
            .wr_data_i      (upstream_write_data_i),
            .wr_mask_i      (upstream_write_mask_i),
            .wr_req_i       (upstream_write_req_i),
            .sram_rd_issued_i(downstream_read_valid_o & downstream_read_ready_i),
            .sram_rdata_i    (downstream_read_data_i),
            .sram_wr_req_i   (downstream_write_req_o),
            .sram_wr_addr_i  (downstream_write_addr_o),
            .rd_hit_comb_o  (fwd_rd_hit),
            .wr_hit_comb_o  (fwd_wr_hit),
            .wr_full_coverage_o(fwd_wr_full_cov),
            .wb_needed_o    (fwd_wb_needed),
            .wb_addr_o      (fwd_wb_addr),
            .wb_data_o      (fwd_wb_data),
            .wb_mask_o      (fwd_wb_mask),
            .wb_done_i      (fwd_wb_done),
            .wr_target_valid_i (upstream_write_target_valid_i),
            .fwd_rdata_o    (fwd_rdata),
            .fwd_hit_o      (fwd_hit),
            .buf_has_free_clean_o (buf_has_free_clean),
            .buf_near_full_o      (buf_near_full_unused),
            .stat_rd_hit_o  (),
            .stat_rd_miss_o (),
            .stat_wr_merge_o(),
            .stat_wr_inval_o(),
            .stat_rd_total_o(),
            .stat_wr_total_o(),
            .stat_sram_rd_o (),
            .stat_wb_o      ()
        );
    end
    endgenerate

    //////////////////////////////////////
    //       Access CTRL Logics         //
    //////////////////////////////////////

    // "Effective write": write that needs SRAM (not absorbed by buffer).
    logic effective_write;
    assign effective_write = upstream_write_req_i & !fwd_wr_hit;

    // Speculative writeback trigger: buffer dirty, SRAM available, no
    // conflicting write, and the trigger condition is met.
    //
    // NOTE: We deliberately do NOT gate on `bank_gnt_i`.  At the tile,
    // grant for a write is `gnt = we | !any_other_write_in_col`, so any
    // write we issue is granted unconditionally by the arbiter.  Adding
    // `& bank_gnt_i` here would be redundant AND would create a
    // combinational loop:
    //   spec_wb_fire -> downstream_write_req_o -> l1_data_bank_we
    //                -> part_we -> any_other_write_in_col[other_way]
    //                -> bank_gnt[other_way] -> spec_wb_fire[other_way]
    //                -> ... -> back to our spec_wb_fire.
    // The loop converges but tools flag it and synthesis breaks.
    //
    // The previous gate `& !(upstream_write_req_i & fwd_wr_hit)` blocked
    // spec_wb during a concurrent absorb.  That was overly conservative:
    // when the absorb is to the SAME line as fwd_wb_addr (true whenever
    // buf_dirty_q AND fwd_wr_hit, by buffer's C3 + the wr_full_hit
    // clean-buffer gate), the buffer's wb_data_o now combinationally
    // merges the absorb's bytes/parts into the wb so SRAM stays
    // consistent after wb_done clears dirty.  This recovers ~1 cycle per
    // line transition in vector-store workloads (preread for next line +
    // store for current line in the same pipeline cycle).
    assign spec_wb_fire = fwd_wb_needed & !wb_active_q
        & (access_status_q == ACCESS_THROUGH)
        & !effective_write
        & ( (UseSpecWbIdle      & !upstream_write_req_i)
          | (UseSpecWbAddrTrans & upstream_read_valid_i
             & !fwd_rd_hit
             & (upstream_read_addr_i != fwd_wb_addr)) );

    always_comb begin
        access_status_d         = access_status_q;
        access_stall_data_d     = access_stall_data_q;
        access_stall_addr_d     = access_stall_addr_q;
        access_stall_mask_d     = access_stall_mask_q;
        wb_active_d             = wb_active_q;
        wb_has_stall_d          = wb_has_stall_q;

        // Defaults: buffer-aware gating.
        // Read hit  -> serve from buffer (no SRAM read, upstream ready).
        // Write hit -> absorb into buffer (no SRAM write).
        upstream_read_ready_o   = fwd_rd_hit ? 1'b1 : downstream_read_ready_i;
        downstream_read_valid_o = fwd_rd_hit ? 1'b0 : upstream_read_valid_i;
        downstream_write_req_o  = fwd_wr_hit ? 1'b0 : upstream_write_req_i;

        upstream_read_data_o    = fwd_rdata;

        downstream_read_addr_o  = upstream_read_addr_i;
        downstream_write_addr_o = upstream_write_addr_i;
        downstream_write_data_o = upstream_write_data_i;
        downstream_write_mask_o = upstream_write_mask_i;

        // -- Writeback overlay: takes priority over the main FSM --
        // Writes proceed unconditionally -- with the tile-level grant
        // propagation (cachepool_tile.sv:any_other_write_in_col),
        // bank_gnt can go to 0 for our idle words in unrelated columns
        // even though our write succeeds at the tile arbiter.  Gating
        // the write on bank_gnt here would (a) create a combinational
        // loop through downstream_write_req_o -> part_we -> gnt ->
        // bank_gnt, and (b) spuriously stall writes that actually get
        // served by the arbiter's write-priority.
        if (wb_active_q) begin
            upstream_read_ready_o   = '0;
            downstream_read_valid_o = '0;
            downstream_write_req_o  = 1'b1;
            downstream_write_addr_o = fwd_wb_addr;
            downstream_write_data_o = fwd_wb_data;
            downstream_write_mask_o = fwd_wb_mask;
            wb_active_d = 1'b0;
            if (wb_has_stall_q) begin
                access_status_d = ACCESS_STALL;
                wb_has_stall_d = 1'b0;
            end
        end else begin
            // -- Main FSM --
            case (access_status_q)
                ACCESS_THROUGH: begin
                    if (effective_write) begin
                        // Write needs SRAM. Block reads (unless AllowReadDuringWrite).
                        if (fwd_wb_needed) begin
                            // Dirty eviction before SRAM write: save the original
                            // write in stall regs and start writeback.
                            wb_active_d = 1'b1;
                            wb_has_stall_d = 1'b1;
                            access_stall_data_d = upstream_write_data_i;
                            access_stall_addr_d = upstream_write_addr_i;
                            access_stall_mask_d = upstream_write_mask_i;
                            upstream_read_ready_o   = '0;
                            downstream_read_valid_o = '0;
                            downstream_write_req_o  = '0;
                        end else begin
                            // Writes proceed unconditionally (see wb_active above).
                            if (AllowReadDuringWrite) begin
                                upstream_read_ready_o = downstream_read_ready_i;
                            end else begin
                                upstream_read_ready_o   = '0;
                                downstream_read_valid_o = '0;
                            end
                        end
                    end else if (upstream_write_req_i && fwd_wr_hit) begin
                        // Write absorbed by buffer -- SRAM port is free.
                        // Three sub-cases for a concurrent read:
                        //   (a) spec_wb_fire (read miss to a DIFFERENT
                        //       line than fwd_wb_addr): drive a concurrent
                        //       writeback so the buffer's dirty data is
                        //       committed to SRAM before populate, AND
                        //       let the read proceed -- pseudo_dual_port
                        //       handles the same-cycle R+W on the SRAM
                        //       bank.  buf_data_for_wb in the buffer
                        //       merges the absorb's bytes into the wb.
                        //   (b) read miss to the SAME line as fwd_wb_addr
                        //       (or buf clean): block the read so it
                        //       doesn't clobber the just-dirtied buffer
                        //       on populate.  Next cycle buf_dirty_q=1
                        //       triggers a normal eviction.
                        //   (c) no concurrent read: nothing to do.
                        if (spec_wb_fire) begin
                            // (a) Concurrent spec_wb during absorb.
                            // Read defaults pass through unchanged so
                            // upstream_read_ready/downstream_read_valid
                            // continue to handshake the read alongside
                            // this writeback.
                            downstream_write_req_o  = 1'b1;
                            downstream_write_addr_o = fwd_wb_addr;
                            downstream_write_data_o = fwd_wb_data;
                            downstream_write_mask_o = fwd_wb_mask;
                        end else if (upstream_read_valid_i && !fwd_rd_hit) begin
                            // (b) Block read to avoid clobbering dirty buffer.
                            upstream_read_ready_o   = '0;
                            downstream_read_valid_o = '0;
                        end
                    end else if (spec_wb_fire) begin
                        // Speculative writeback: issue downstream write
                        // alongside the read.  pseudo_dual_port resolves
                        // the R/W conflict (WR_DIFF_BANK = hidden,
                        // WR_CONFLICT = read retries next cycle).
                        // No ACCESS_STALL: fire-and-forget.
                        downstream_write_req_o  = 1'b1;
                        downstream_write_addr_o = fwd_wb_addr;
                        downstream_write_data_o = fwd_wb_data;
                        downstream_write_mask_o = fwd_wb_mask;
                    end else if (fwd_wb_needed && upstream_read_valid_i
                                 && !fwd_rd_hit
                                 && (upstream_read_addr_i != fwd_wb_addr)
                                 && !buf_has_free_clean) begin
                        // Read miss with dirty buffer at a DIFFERENT line:
                        // writeback first to preserve dirty data before the
                        // populate replaces the buffer entry.
                        //
                        // Same-line read miss (addr == fwd_wb_addr) skips
                        // this writeback: the populate ACCUMULATEs into the
                        // existing buffer entry, merging the missing parts
                        // with the dirty parts already present.  Forcing a
                        // writeback in that case is wasted bandwidth -- the
                        // dirty parts would be re-dirtied by the very next
                        // store hit.
                        //
                        // For multi-entry with a free-clean slot, the read
                        // can populate into the free slot without evicting
                        // dirty data -- skip the blocking writeback.
                        wb_active_d = 1'b1;
                        wb_has_stall_d = 1'b0;
                        upstream_read_ready_o   = '0;
                        downstream_read_valid_o = '0;
                    end else if (bank_gnt_i == '0 && !fwd_rd_hit) begin
                        // Read lost arbitration to another way's write in the
                        // shared skew column: squash only the READ.  The write
                        // is unconditionally granted by the tile arbiter
                        // (gnt = we | ~any_other_write_in_col), so it is never
                        // gated here; squashing downstream_write_req_o on
                        // bank_gnt_i is both a no-op (this branch is reached
                        // only when upstream_write_req_i==0, so the default is
                        // already 0) and would re-create the grant-feedback
                        // combinational loop.  Leaving the write ungated breaks
                        // the loop arm through downstream_write_req_o.
                        upstream_read_ready_o   = '0;
                        downstream_read_valid_o = '0;
                    end
                end

                ACCESS_STALL: begin
                    upstream_read_ready_o   = '0;
                    downstream_read_valid_o = '0;
                    downstream_write_req_o  = 1'b1;
                    downstream_write_addr_o = access_stall_addr_q;
                    downstream_write_data_o = access_stall_data_q;
                    downstream_write_mask_o = access_stall_mask_q;
                    access_status_d = ACCESS_THROUGH;
                end

                default : access_status_d = ACCESS_THROUGH;
            endcase
        end
    end

`ifndef TARGET_SYNTHESIS
    // ---------------------------------------------------------------
    // Access-controller contract assertions.  Each `$error` fires at
    // the cycle of divergence, so a contract violation here is
    // pinpointed instead of surfacing 80us later as an illegal-addr at
    // the cluster xbar.  Guarded by `TARGET_SYNTHESIS so synth is
    // unaffected.
    //
    // The contract codifies what the cache core relies on:
    //   - while writing back, the controller actually drives the wb
    //     onto downstream and blocks the upstream read,
    //   - a dirty-buffer eviction trigger transitions to wb_active,
    //   - no SRAM read fires while the buffer is dirty for a DIFFERENT
    //     line (this would clobber the dirty data on populate -- the
    //     buffer's own C3 catches that case, AC-4 stops it earlier),
    //   - ACCESS_STALL actually drives the saved write.
    // ---------------------------------------------------------------

    // AC-1: while writing back, downstream_write_req is asserted with
    //       the buffer's writeback address/data/mask.
    property p_AC1_wb_active_drives_writeback;
        @(posedge clk_i) disable iff (!rst_ni)
        wb_active_q |->
            (downstream_write_req_o &&
             downstream_write_addr_o == fwd_wb_addr);
    endproperty
    a_AC1_wb_active_drives_writeback: assert property (p_AC1_wb_active_drives_writeback)
        else $error("[AC-1 %m] wb_active_q=1 but downstream_write_req=%0b addr=0x%0h (expected addr=0x%0h)",
                    downstream_write_req_o, downstream_write_addr_o, fwd_wb_addr);

    // AC-2: while writing back, the upstream read must not progress.
    property p_AC2_wb_active_blocks_read;
        @(posedge clk_i) disable iff (!rst_ni)
        wb_active_q |->
            (!upstream_read_ready_o && !downstream_read_valid_o);
    endproperty
    a_AC2_wb_active_blocks_read: assert property (p_AC2_wb_active_blocks_read)
        else $error("[AC-2 %m] wb_active_q=1 but read still proceeding (upstream_read_ready=%0b downstream_read_valid=%0b)",
                    upstream_read_ready_o, downstream_read_valid_o);

    // AC-3: a dirty-buffer eviction trigger must transition to wb_active.
    //   (effective_write && fwd_wb_needed in ACCESS_THROUGH not already
    //   in wb_active) implies wb_active_q on the next cycle.
    property p_AC3_dirty_eviction_triggers_wb;
        @(posedge clk_i) disable iff (!rst_ni)
        ((access_status_q == ACCESS_THROUGH) &&
          effective_write && fwd_wb_needed && !wb_active_q)
        |=> wb_active_q;
    endproperty
    a_AC3_dirty_eviction_triggers_wb: assert property (p_AC3_dirty_eviction_triggers_wb)
        else $error("[AC-3 %m] dirty-eviction trigger present but wb_active didn't activate");

    // AC-4: NO SRAM read may fire while the buffer is dirty for a
    //       DIFFERENT address WITHOUT a concurrent spec_wb committing
    //       the dirty data.  When spec_wb_fire is asserted in the same
    //       cycle, the buffer's dirty data is being written to SRAM
    //       alongside the read -- the dirty data is preserved and the
    //       pseudo_dual_port handles the concurrent R/W.  When
    //       spec_wb_fire is not asserted, the SRAM read's populate
    //       will clobber the dirty data next cycle (== buffer C3 vio).
    //
    //   Note: when UseForwardingBuffer=0, fwd_wb_needed is always 0,
    //   so this property is vacuously true.
    property p_AC4_no_sram_read_clobber_dirty;
        @(posedge clk_i) disable iff (!rst_ni)
        UseForwardingBuffer ->
            !((downstream_read_valid_o && downstream_read_ready_i) &&
              fwd_wb_needed &&
              (upstream_read_addr_i != fwd_wb_addr) &&
              !spec_wb_fire);
    endproperty
    a_AC4_no_sram_read_clobber_dirty: assert property (p_AC4_no_sram_read_clobber_dirty)
        else $error("[AC-4 %m] SRAM read for addr=0x%0h fired while buffer dirty for addr=0x%0h (no concurrent spec_wb)",
                    upstream_read_addr_i, fwd_wb_addr);

    // AC-5: spec_wb_fire ⇒ same cycle, downstream_write_req drives the
    //       buffer's writeback addr/data/mask.
    property p_AC5_spec_wb_drives_write;
        @(posedge clk_i) disable iff (!rst_ni)
        spec_wb_fire |->
            (downstream_write_req_o &&
             downstream_write_addr_o == fwd_wb_addr);
    endproperty
    a_AC5_spec_wb_drives_write: assert property (p_AC5_spec_wb_drives_write)
        else $error("[AC-5 %m] spec_wb_fire=1 but downstream_write didn't follow");

    // AC-6: in ACCESS_STALL (and not overlaid by wb_active), drive the
    //       previously saved write.
    property p_AC6_stall_drives_saved_write;
        @(posedge clk_i) disable iff (!rst_ni)
        ((access_status_q == ACCESS_STALL) && !wb_active_q) |->
            (downstream_write_req_o &&
             downstream_write_addr_o == access_stall_addr_q);
    endproperty
    a_AC6_stall_drives_saved_write: assert property (p_AC6_stall_drives_saved_write)
        else $error("[AC-6 %m] ACCESS_STALL but downstream_write didn't get the saved addr (got 0x%0h, expected 0x%0h)",
                    downstream_write_addr_o, access_stall_addr_q);
`endif // !TARGET_SYNTHESIS

endmodule : insitu_cache_bank_access_controller
