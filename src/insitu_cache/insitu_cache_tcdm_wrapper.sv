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
`ifndef TARGET_SYNTHESIS
    /// Name the cache
    parameter string       ModeleName               = "none",
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
    logic                   [SetAssociativity - 1 : 0]      data_bank_write_hit;

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
        assign upstream_req_to_cache_valid =    upstream_req_write_i?
                                                ~winfo_fifo_full & upstream_req_valid_i:
                                                upstream_req_valid_i;
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
                        if (sync_ctrl_insn_q == 2'b11) begin
                            sync_ctrl_status_d = SYNC_CTRL_INIT;
                        end else begin : proc_sync_ctrl_init
                            sync_ctrl_status_d = SYNC_CTRL_CHECK_PEND;
                        end
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
                    if ((outstanding_refill_cnt_q == '0) &&
                        ~sync_ctrl_still_pending &&
                        ~core_miss_valid &&
                        ~core_evic_valid &&
                        ~write_through_valid) begin
                        sync_ctrl_status_d = SYNC_CTRL_FLUSH;
                        sync_ctrl_ptr_d = cache_part_base_i;
                        flush_read_cache_addr = sync_ctrl_ptr_d;
                        flush_read_cache_valid = 1'b1;
                    end
                end

                SYNC_CTRL_FLUSH : begin
                    //Check Dirty Line
                    for (int i = 0; i < SetAssociativity; i++) begin
                        if ((flush_read_cache_status[i] == VALID) && (flush_read_cache_dirty[i] == 1'b1)) begin
                            sync_ctrl_has_dirty_line = 1'b1;
                            sync_ctrl_dirty_line = i;
                            break;
                        end
                    end

                    if (sync_ctrl_has_dirty_line) begin
                        if (PartSplit > 1) begin
                            flush_full_read_active = 1'b1;
                            if (~flush_full_wait_q && ~flush_full_data_valid_q) begin
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
        .bank_write_data_buf_hit_i      (bank_write_data_buf_hit)
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

    /*******************/
    /*  Refill logics  */
    /*******************/

    assign core_refill_data = downstream_resp_data_i;
    assign core_refill_info = downstream_resp_info_i;
    assign core_refill_valid = downstream_resp_valid_i & ~downstream_resp_write_i;
    assign downstream_resp_ready_o = downstream_resp_write_i ? 1'b1 : core_refill_ready;
    assign consumed_refill_resp = downstream_resp_valid_i & ~downstream_resp_write_i & downstream_resp_ready_o;
    assign issued_refill_req = core_miss_valid & core_miss_ready;

    /***********************/
    /*  Flush Proc Arbiter */
    /***********************/

    assign bank_read_sel_flush         = ~proc_read_cache_valid;
    assign bank_read_cache_valid       = bank_read_sel_flush? flush_read_cache_valid : proc_read_cache_valid;
    assign proc_read_cache_ready       = bank_read_cache_ready;
    assign flush_read_cache_ready      = bank_read_sel_flush? bank_read_cache_ready : '0;
    assign bank_read_way_mask_sel      = bank_read_sel_flush? flush_read_way_mask : bank_read_way_mask;
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


    /*****************/
    /*  Cache Banks  */
    /*****************/

    cache_bank_depth_ptr_t bank_read_cache_addr_q;
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
    logic [CacheBankDepth-1:0][SetAssociativity-1:0] dirty_rf;
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

        insitu_cache_bank_access_controller #(
            .DEPTH              (CacheBankDepth),
            .NumWordsPerLine    (CacheLineWidth/WordWidth),
            .WordWidth          (WordWidth),
            .ByteWidth          (ByteWidth),
            .UseForwardingBuffer(1'b0),
            .PartSplit          (PartSplit),
            .UseSpecWbIdle      (1'b0),
            .UseSpecWbAddrTrans (1'b0)
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
            .upstream_write_req_i        (bank_write_cache_req),
            .upstream_write_data_i       (bank_write_cache_data[i]),
            .upstream_write_mask_i       (bank_write_data_mask_sel[i]),

            .downstream_read_addr_o      (gnt_data_bank_read_addr[i]),
            .downstream_read_valid_o     (gnt_data_bank_read_valid[i]),
            .downstream_read_ready_i     (gnt_data_bank_read_ready[i]),
            .downstream_read_data_i      (gnt_data_bank_read_data[i]),

            .downstream_write_addr_o     (gnt_data_bank_write_addr[i]),
            .downstream_write_req_o      (gnt_data_bank_write_req[i]),
            .downstream_write_data_o     (gnt_data_bank_write_data[i]),
            .downstream_write_mask_o     (gnt_data_bank_write_mask[i]),

            .bank_gnt_i                  (&(tcdm_data_bank_gnt_i[i])),
            .fwd_wr_hit_o                (data_bank_write_hit[i])

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

        logic meta_proc_write_req;
        always_comb begin
            meta_proc_write_req = 1'b0;
            if (bank_write_cache_req && !bank_write_meta_skip &&
                !mc_suppress_meta_write &&
                (proc_write_cache_way == way_ptr_t'(i))) begin
                meta_proc_write_req = 1'b1;
            end
            // LRU-only updates → LRU register file (no meta SRAM write).
            // Write hits on VALID → dirty RF + LRU RF only (meta_skip=1).
        end

        insitu_cache_bank_access_controller #(
            .DEPTH              (CacheBankDepth),
            .NumWordsPerLine    (1),
            .WordWidth          ($bits(cache_meta_t)),
            .ByteWidth          ($bits(cache_meta_t)),
            .AllowReadDuringWrite (1'b0),
            .UseForwardingBuffer(1'b1),
            .PartSplit          (1),
            .UseSpecWbIdle      (1'b1),
            .UseSpecWbAddrTrans (1'b1)
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
            .upstream_write_req_i        (proc_write_select ? meta_proc_write_req :
                                          flush_write_cache_req_valid),
            .upstream_write_data_i       (cache_meta_write_data[i]),
            .upstream_write_mask_i       ('1    ),

            .downstream_read_addr_o      (gnt_meta_bank_read_addr[i]),
            .downstream_read_valid_o     (gnt_meta_bank_read_valid[i]),
            .downstream_read_ready_i     (gnt_meta_bank_read_ready[i]),
            .downstream_read_data_i      (gnt_meta_bank_read_data[i]),

            .downstream_write_addr_o     (gnt_meta_bank_write_addr[i]),
            .downstream_write_req_o      (gnt_meta_bank_write_req[i]),
            .downstream_write_data_o     (gnt_meta_bank_write_data[i]),
            .downstream_write_mask_o     (gnt_meta_bank_write_mask[i]),

            .bank_gnt_i                  (&(tcdm_data_bank_gnt_i[i])),
            .fwd_wr_hit_o                ()

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

    for (genvar i = 0; i < NumPseudoDualBanks; i++) begin
        assign bank_wdata_words[i] = bank_wdata[i];
        for (genvar j = 0; j < NumWordsPerLine; j++) begin
            localparam int unsigned WordPart = (PartSplit > 1) ? (j / PartWords) : 0;
            assign word_in_part[i][j] = read_all_parts_i ? 1'b1 :
              ((PartSplit > 1) ? (read_part_idx_i == WordPart[PartIdxWidth-1:0]) : 1'b1);
            assign word_read_en[i][j] = bank_req_read[i] & read_valid_i & word_in_part[i][j];
            assign word_write_en[i][j] = bank_req_write[i] & write_has_data &
              (|bank_wmask[i][j*WordBytes +: WordBytes]);
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

        /************/
        /* Main FSM */
        /************/
        case (status)
            W_ONLY: begin
                bank_req_write[write_bank_select] = 1'b1;
                bank_addr[write_bank_select]    = write_bank_addr;
                bank_wdata[write_bank_select]   = write_data_i;
                bank_wmask[write_bank_select]   = write_mask_i;
            end

            R_ONLY: begin
                bank_req_read[read_bank_select] = 1'b1;
                bank_addr[read_bank_select]     = read_bank_addr;

                read_data_from_line_buffer_d    = '0;
                read_data_from_bank_select_d    = read_bank_select;
            end

            WR_DIFF_BANK: begin
                bank_req_write[write_bank_select] = 1'b1;
                bank_addr[write_bank_select]    = write_bank_addr;
                bank_wdata[write_bank_select]   = write_data_i;
                bank_wmask[write_bank_select]   = write_mask_i;

                bank_req_read[read_bank_select] = 1'b1;
                bank_addr[read_bank_select]     = read_bank_addr;

                read_data_from_line_buffer_d    = '0;
                read_data_from_bank_select_d    = read_bank_select;
            end

            WR_SAME_ADDR: begin
                // Issue BOTH read and write to the same pseudo-bank.
                // Each TCDM word-bank independently reads or writes:
                //   - Read-only words  → SRAM read (correct data)
                //   - Write-only words → SRAM write
                //   - Overlapping words → SRAM write, forward from write buffer
                bank_req_write[write_bank_select] = 1'b1;
                bank_req_read[read_bank_select]   = 1'b1;
                bank_addr[write_bank_select]    = write_bank_addr;
                bank_wdata[write_bank_select]   = write_data_i;
                bank_wmask[write_bank_select]   = write_mask_i;

                read_data_from_line_buffer_d    = '0;
                read_data_from_bank_select_d    = read_bank_select;
            end

            WR_CONFLICT: begin
                bank_req_write[write_bank_select] = 1'b1;
                bank_addr[write_bank_select]    = write_bank_addr;
                bank_wdata[write_bank_select]   = write_data_i;
                bank_wmask[write_bank_select]   = write_mask_i;

                read_data_from_line_buffer_d    = '0;
                read_data_from_bank_select_d    = '0;
            end

            default : /* default */;
        endcase

        /*********************/
        /* Read Ready Logics */
        /*********************/
        // Block read only on true conflict (same bank, different address).
        // WR_SAME_ADDR uses the write-line-buffer bypass, so read can proceed.
        if (status == WR_CONFLICT) begin
            read_ready_o = 1'b0;
        end
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
    /// Enable 1-entry write-back forwarding buffer.  Buffer caches 1
    /// SRAM row; matching reads return buffer data, writes merge into
    /// buffer.  Dirty data written back on address change.
    parameter bit           UseForwardingBuffer     = 1'b0,
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
    output logic                                    fwd_wr_hit_o
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
    logic  fwd_rd_hit;     // combinational: buffer can serve this read
    logic  fwd_wr_hit;     // combinational: buffer can absorb this write
    logic  fwd_wb_needed;  // buffer dirty
    addr_t fwd_wb_addr;    // writeback address
    data_t fwd_wb_data;    // writeback data
    mask_t fwd_wb_mask;    // writeback byte mask (only cached parts)

    // -- Speculative writeback --
    // Fire-and-forget: issue writeback alongside normal read when the
    // SRAM port is available.  No stall on denial.
    logic spec_wb_fire;

    // Writeback done: fires on explicit writeback OR speculative writeback.
    logic  fwd_wb_done;
    assign fwd_wb_done = (wb_active_q & bank_gnt_i) | spec_wb_fire;

    // Expose write hit for upstream hazard relaxation.
    assign fwd_wr_hit_o = fwd_wr_hit;

    sram_forwarding_buffer #(
        .Depth          (DEPTH),
        .NumWordsPerLine(NumWordsPerLine),
        .WordWidth      (WordWidth),
        .ByteWidth      (ByteWidth),
        .Enable         (UseForwardingBuffer),
        .PartSplit      (PartSplit)
    ) i_fwd_buf (
        .clk_i,
        .rst_ni,
        // Upstream signals (for hit checking and write merge)
        .rd_addr_i      (upstream_read_addr_i),
        .rd_valid_i     (upstream_read_valid_i),
        .rd_ready_i     (upstream_read_ready_o),
        .rd_part_idx_i  (upstream_read_part_idx_i),
        .rd_all_parts_i (upstream_read_all_parts_i),
        .wr_addr_i      (upstream_write_addr_i),
        .wr_data_i      (upstream_write_data_i),
        .wr_mask_i      (upstream_write_mask_i),
        .wr_req_i       (upstream_write_req_i),
        // SRAM tracking (gated downstream signals)
        .sram_rd_issued_i(downstream_read_valid_o & downstream_read_ready_i),
        .sram_rdata_i    (downstream_read_data_i),
        .sram_wr_req_i   (downstream_write_req_o),
        .sram_wr_addr_i  (downstream_write_addr_o),
        // Combinational outputs
        .rd_hit_comb_o  (fwd_rd_hit),
        .wr_hit_comb_o  (fwd_wr_hit),
        // Writeback
        .wb_needed_o    (fwd_wb_needed),
        .wb_addr_o      (fwd_wb_addr),
        .wb_data_o      (fwd_wb_data),
        .wb_mask_o      (fwd_wb_mask),
        .wb_done_i      (fwd_wb_done),
        // Read data
        .fwd_rdata_o    (fwd_rdata),
        .fwd_hit_o      (fwd_hit),
        // Statistics
        .stat_rd_hit_o  (),
        .stat_rd_miss_o (),
        .stat_wr_merge_o(),
        .stat_wr_inval_o(),
        .stat_rd_total_o(),
        .stat_wr_total_o(),
        .stat_sram_rd_o (),
        .stat_wb_o      ()
    );

    //////////////////////////////////////
    //       Access CTRL Logics         //
    //////////////////////////////////////

    // "Effective write": write that needs SRAM (not absorbed by buffer).
    logic effective_write;
    assign effective_write = upstream_write_req_i & !fwd_wr_hit;

    // Speculative writeback trigger: buffer dirty, SRAM available, no
    // conflicting write, and the trigger condition is met.
    assign spec_wb_fire = fwd_wb_needed & !wb_active_q
        & (access_status_q == ACCESS_THROUGH)
        & !effective_write
        & !(upstream_write_req_i & fwd_wr_hit)
        & bank_gnt_i
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
        if (wb_active_q) begin
            upstream_read_ready_o   = '0;
            downstream_read_valid_o = '0;
            downstream_write_req_o  = '0;
            downstream_write_addr_o = fwd_wb_addr;
            downstream_write_data_o = fwd_wb_data;
            downstream_write_mask_o = fwd_wb_mask;
            if (bank_gnt_i) begin
                downstream_write_req_o = 1'b1;
                wb_active_d = 1'b0;
                if (wb_has_stall_q) begin
                    // Original write saved in stall regs -- resend it.
                    access_status_d = ACCESS_STALL;
                    wb_has_stall_d = 1'b0;
                end
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
                            if (AllowReadDuringWrite && bank_gnt_i) begin
                                upstream_read_ready_o = downstream_read_ready_i;
                            end else begin
                                upstream_read_ready_o   = '0;
                                downstream_read_valid_o = '0;
                            end
                            if (bank_gnt_i == '0) begin
                                downstream_write_req_o = '0;
                                access_status_d = ACCESS_STALL;
                                access_stall_data_d = upstream_write_data_i;
                                access_stall_addr_d = upstream_write_addr_i;
                                access_stall_mask_d = upstream_write_mask_i;
                            end
                        end
                    end else if (upstream_write_req_i && fwd_wr_hit) begin
                        // Write absorbed by buffer -- SRAM port is free.
                        // BUT: block reads to a different address that
                        // would trigger an SRAM read and evict the
                        // just-dirtied buffer without writeback.
                        // Next cycle buf_dirty_q=1 triggers writeback.
                        if (upstream_read_valid_i && !fwd_rd_hit) begin
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
                                 && !fwd_rd_hit) begin
                        // Read miss with dirty buffer: writeback first.
                        // The read will retry after writeback completes.
                        wb_active_d = 1'b1;
                        wb_has_stall_d = 1'b0;
                        upstream_read_ready_o   = '0;
                        downstream_read_valid_o = '0;
                    end else if (bank_gnt_i == '0 && !fwd_rd_hit) begin
                        upstream_read_ready_o   = '0;
                        downstream_read_valid_o = '0;
                        downstream_write_req_o  = '0;
                    end
                end

                ACCESS_STALL: begin
                    upstream_read_ready_o   = '0;
                    downstream_read_valid_o = '0;
                    downstream_write_req_o  = '0;

                    downstream_write_addr_o = access_stall_addr_q;
                    downstream_write_data_o = access_stall_data_q;
                    downstream_write_mask_o = access_stall_mask_q;
                    if (bank_gnt_i) begin
                        downstream_write_req_o = 1'b1;
                        access_status_d = ACCESS_THROUGH;
                    end
                end

                default : access_status_d = ACCESS_THROUGH;
            endcase
        end
    end

endmodule : insitu_cache_bank_access_controller
