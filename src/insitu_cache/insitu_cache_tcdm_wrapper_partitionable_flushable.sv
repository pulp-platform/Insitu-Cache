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
`include "insitu_cache/hash.svh"
module insitu_cache_tcdm_wrapper_partitionable_flushable
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
    /// Width of word (granularity of non-blocking write)
    parameter int unsigned WordWidth                = 32,
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
    // Dependent parameter, do not override. Number of meta bank per way.
    localparam int unsigned NumMetaBankPerWay       = NumPseudoDualBanks,
    // Dependent parameter, do not override. Address type.
    localparam type tcdm_bank_addr_t                = logic [$clog2(CacheBankDepth)-$clog2(NumPseudoDualBanks)-1:0],
    /// Dependent parameter, do not override. word type
    localparam type word_t                          = logic [WordWidth-1:0],
    /// Dependent parameter, do not override. word type
    localparam type tcdm_meta_data_t                = logic [63:0],
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
    localparam type cache_mask_t                    = logic [DownstreamWidth/WordWidth-1:0],
    // Dependent parameter, do not override. bank depth ptr type.
    localparam type cache_bank_depth_ptr_t          = logic [$clog2(CacheBankDepth)-1:0],
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
    input  logic [1:0]                              cache_sync_insn_i, //0-> flush+invalidation | 1-> flush only | 2-> invalidation only | 3-> bank initialization

    /// Cache Partitioning Signals
    input  tcdm_bank_addr_t                         bank_depth_for_SPM_i,

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
    output logic             [SetAssociativity-1:0][NumDataBankPerWay-1:0]   tcdm_data_bank_be_o,
    input  word_t            [SetAssociativity-1:0][NumDataBankPerWay-1:0]   tcdm_data_bank_rdata_i,

    /// Data Bank Request GNT for Cache
    input  logic             [SetAssociativity-1:0][NumDataBankPerWay-1:0]   tcdm_data_bank_gnt_i
    
);

    localparam int unsigned CacheAddrWidth  = ReqAddrWidth + $clog2(CacheBankDepth);
    localparam type         cache_addr_t    = logic [CacheAddrWidth-1:0];
    localparam type         cache_tag_t     = logic [CacheAddrWidth-$clog2(CacheLineWidth/8)-$clog2(CacheBankDepth)-1:0];
    localparam type         cache_set_t     = logic [$clog2(CacheBankDepth)-1:0];
    localparam type         cache_byt_t     = logic [$clog2(CacheLineWidth/8)-1:0];
    localparam type         partition_t     = logic [$clog2(CacheBankDepth):0];

    ////////////////////////////////////////
    //        Address Translation         //
    ////////////////////////////////////////

    cache_set_t cache_base_for_SPM;
    partition_t cache_partition_set_for_SPM;
    partition_t cache_partition_set_for_cache;
    assign cache_partition_set_for_SPM      = NumPseudoDualBanks * bank_depth_for_SPM_i;
    assign cache_base_for_SPM               = cache_partition_set_for_SPM;
    assign cache_partition_set_for_cache    = CacheBankDepth - cache_partition_set_for_SPM;

    //upstream
    cache_addr_t upstream_req_cache_addr;
    cache_tag_t  upstream_tag;
    cache_set_t  upstream_set;
    cache_byt_t  upstream_byt;
    always_comb begin
        upstream_tag = (upstream_req_addr_i >> $clog2(CacheLineWidth/8))/cache_partition_set_for_cache;
        upstream_set = (upstream_req_addr_i >> $clog2(CacheLineWidth/8))%cache_partition_set_for_cache + cache_partition_set_for_SPM;
        upstream_byt = '0;
        upstream_req_cache_addr = {upstream_tag, upstream_set, upstream_byt};
    end


    //downstream
    cache_addr_t downstream_req_cache_addr;
    cache_tag_t  downstream_tag;
    cache_set_t  downstream_set;
    cache_byt_t  downstream_byt;
    addr_t       downstream_restored_addr;
    always_comb begin
        {downstream_tag, downstream_set, downstream_byt} = downstream_req_cache_addr;
        downstream_restored_addr = (downstream_tag * cache_partition_set_for_cache) + (downstream_set - cache_partition_set_for_SPM);
        downstream_req_addr_o = downstream_restored_addr << $clog2(CacheLineWidth/8);
    end


    /////////////////////////////////////
    //        Instance Modules         //
    /////////////////////////////////////

    insitu_cache_tcdm_wrapper #(
        .ReqAddrWidth         (CacheAddrWidth),
        .info_t               (info_t),
        .CacheLineWidth       (CacheLineWidth),
        .NumCacheEntry        (NumCacheEntry),
        .SetAssociativity     (SetAssociativity),
        .NumPseudoDualBanks   (NumPseudoDualBanks),
        .WordWidth            (WordWidth),
        .LogDebug             (LogDebug),
        .LogLifeCycle         (LogLifeCycle),
        .WriteThroughFifoDepth(WriteThroughFifoDepth),
        .WRespFifoDepth       (WRespFifoDepth),
        .RetrFifoDepth        (RetrFifoDepth),
        .RespFifoDepth        (RespFifoDepth),
        .MissFifoDepth        (MissFifoDepth),
        .EvicFifoDepth        (EvicFifoDepth),
        .AddrHashLength       (0),
        .WriteThroughMode     (WriteThroughMode)
    ) i_insitu_cache_tcdm_wrapper (
        .clk_i,
        .rst_ni,

        .cache_sync_valid_i,
        .cache_sync_ready_o,
        .cache_sync_insn_i,

        .cache_part_base_i      (cache_base_for_SPM),

        .upstream_req_valid_i   (upstream_req_valid_i   ),
        .upstream_req_ready_o   (upstream_req_ready_o   ),
        .upstream_req_addr_i    (upstream_req_cache_addr    ),
        .upstream_req_info_i    (upstream_req_info_i    ),
        .upstream_req_write_i   (upstream_req_write_i   ),
        .upstream_req_wdata_i   (upstream_req_wdata_i   ),
        .upstream_req_wmask_i   (upstream_req_wmask_i   ),
        .upstream_resp_valid_o  (upstream_resp_valid_o  ),
        .upstream_resp_ready_i  (upstream_resp_ready_i  ),
        .upstream_resp_write_o  (upstream_resp_write_o  ),
        .upstream_resp_data_o   (upstream_resp_data_o   ),
        .upstream_resp_info_o   (upstream_resp_info_o   ),
        .downstream_req_valid_o (downstream_req_valid_o ),
        .downstream_req_ready_i (downstream_req_ready_i ),
        .downstream_req_addr_o  (downstream_req_cache_addr  ),
        .downstream_req_info_o  (downstream_req_info_o  ),
        .downstream_req_write_o (downstream_req_write_o ),
        .downstream_req_wdata_o (downstream_req_wdata_o ),
        .downstream_req_wmask_o (downstream_req_wmask_o ),
        .downstream_resp_valid_i(downstream_resp_valid_i),
        .downstream_resp_ready_o(downstream_resp_ready_o),
        .downstream_resp_data_i (downstream_resp_data_i ),
        .downstream_resp_info_i (downstream_resp_info_i ),
        .downstream_resp_write_i(downstream_resp_write_i),
        .tcdm_meta_bank_req_o   (tcdm_meta_bank_req_o   ),
        .tcdm_meta_bank_we_o    (tcdm_meta_bank_we_o    ),
        .tcdm_meta_bank_addr_o  (tcdm_meta_bank_addr_o  ),
        .tcdm_meta_bank_wdata_o (tcdm_meta_bank_wdata_o ),
        .tcdm_meta_bank_be_o    (tcdm_meta_bank_be_o    ),
        .tcdm_meta_bank_rdata_i (tcdm_meta_bank_rdata_i ),
        .tcdm_data_bank_req_o   (tcdm_data_bank_req_o   ),
        .tcdm_data_bank_we_o    (tcdm_data_bank_we_o    ),
        .tcdm_data_bank_addr_o  (tcdm_data_bank_addr_o  ),
        .tcdm_data_bank_wdata_o (tcdm_data_bank_wdata_o ),
        .tcdm_data_bank_be_o    (tcdm_data_bank_be_o    ),
        .tcdm_data_bank_rdata_i (tcdm_data_bank_rdata_i ),
        .tcdm_data_bank_gnt_i   (tcdm_data_bank_gnt_i   )
    );


endmodule