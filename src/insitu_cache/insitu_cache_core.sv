// Copyright 2023 ETH Zurich and 
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 12.Feb.2024

// RTL model of insitu cache core logics
// Deposition of Address:   [Tag][Cache Bank Depth Ptr][Byte Pad]
// Deposition of CacheLine: [Status][Is Dirty][Dirty Byte Mask][Tag][Cache Payload][LRU]
//                                            [Num of Subarray]
//                          [Status]        =   0. invalid
//                                              1. valid
//                                              2. read pend
//                                              3. write pend
//                          [Cache Payload] =   [Cache Data] -- for Status 0,1,3
//                                              [Subarrays]  -- for Status 2



`include "common_cells/registers.svh"
module insitu_cache_core
  import insitu_cache_pkg::*;
  #(
    /// Address width of both upstream narrow request and downstream wide request
    parameter int unsigned ReqAddrWidth                     = 32,
    /// Word width of cache line (512b default)
    parameter int unsigned CacheLineWidth                   = 512,
    /// Information payload
    parameter type         info_t                           = logic[7:0],
    /// Number of Cache entries
    parameter int unsigned NumCacheEntry                    = 512,
    /// Number of Associatity
    parameter int unsigned SetAssociativity                 = 16,
    /// Number of parts per cache line for data banks (1 = unfolded).
    parameter int unsigned DataPartSplit                    = 1,
    /// Width of word (granularity of non-blocking write)
    parameter int unsigned WordWidth                        = 64,
    /// Width of byte (granularity of byte mask)
    parameter int unsigned ByteWidth                        = 8,
    /// Show Debug information on screen.
    parameter int unsigned ShowDebug                        = 0,
    /// Log Debug information for questa-sim.
    parameter int unsigned LogDebug                         = 1,
    /// Counter cache line life cycle information for questa-sim.
    parameter int unsigned LogLifeCycle                     = 0,
    /// Depth of Retrieve Fifo.
    parameter int unsigned RetrFifoDepth                    = 16,
    /// Depth of Response Fifo.
    parameter int unsigned RespFifoDepth                    = 16,
    /// Depth of Miss Fifo.
    parameter int unsigned MissFifoDepth                    = 16,
    /// Depth of Eviction Fifo.
    parameter int unsigned EvicFifoDepth                    = 16,
    /// Depth of Pesudo Refill Fifo.
    parameter int unsigned PesudoRefillFifoDepth            = 8,
    /// Whether the cache is in Write-Through mode
    /// Otherwise the cache is defualtly in Write-Back mode
    parameter bit          WriteThroughMode                 = 0,
`ifndef TARGET_SYNTHESIS
    /// Name the cache
    parameter string       ModeleName                       = "none",
`endif
    // Dependent parameter, do not override. Depth of cache bank.
    localparam int unsigned CacheBankDepth                  = NumCacheEntry/SetAssociativity,
    // Dependent parameter, do not override. way ptr type.
    localparam type way_ptr_t                               = logic [$clog2(SetAssociativity)-1:0],
    // Dependent parameter, do not override. Address type.
    localparam type addr_t                                  = logic [ReqAddrWidth-1:0],
    // Dependent parameter, do not override. Narrow word type.
    localparam type cache_data_t                            = logic [CacheLineWidth-1:0],
    // Dependent parameter, do not override. Byte mask type.
    localparam type cache_mask_t                            = logic [CacheLineWidth/ByteWidth-1:0],
    // Dependent parameter, do not override. tag type.
    localparam type cache_tag_t                             = logic [ReqAddrWidth-$clog2(CacheLineWidth/8)-$clog2(CacheBankDepth)-1:0],
    // Dependent parameter, do not override. bank depth ptr type.
    localparam type cache_bank_depth_ptr_t                  = logic [$clog2(CacheBankDepth)-1:0],
    // Dependent parameter, do not override. Byte offset type.
    localparam type byte_offset_t                           = logic [$clog2(CacheLineWidth/8)-1:0],
    // Dependent parameter, do not override. Number of parts.
    localparam int unsigned PartSplit                       = (DataPartSplit == 0) ? 1 : DataPartSplit,
    // Dependent parameter, do not override. Part index width.
    localparam int unsigned PartIdxWidth                    = (PartSplit > 1) ? $clog2(PartSplit) : 1,
    // Dependent parameter, do not override. Line bytes.
    localparam int unsigned LineBytes                       = CacheLineWidth/8,
    // Dependent parameter, do not override. Part bytes.
    localparam int unsigned PartBytes                       = LineBytes / PartSplit,
    // Dependent parameter, do not override. Downstream request payload.
    localparam type downstream_info_t                       = struct packed {logic for_write_pend; cache_bank_depth_ptr_t depth; way_ptr_t way;},
    // Dependent parameter, do not override. Downstream request payload.
    localparam type miss_meta_t                             = struct packed {logic is_full; logic is_prime; logic link_enable; way_ptr_t link_ptr;}
    // Dependent parameter, do not override. Cache line status.
    // VCS will complain about mismatched enum, passed it in from upper level
    // parameter type cache_status_t                          = enum logic[1:0] { INVALID = '0, VALID, READ_PEND, WRITE_PEND }
    )(
    /// Clock, positive edge triggered.
    input  logic                                            clk_i,
    /// Reset, active low.
    input  logic                                            rst_ni,
    /// Indicate Pend Lines
    output logic                                            has_pend_line_o,
    /// Clear pending-line counter (used by flush/invalidate)
    input  logic                                            clear_pend_cnt_i,

    /// Upstream request -- cache requests channel
    input  logic                                            upstream_req_valid_i,
    output logic                                            upstream_req_ready_o,
    input  addr_t                                           upstream_req_addr_i,
    input  info_t                                           upstream_req_info_i,
    input  logic                                            upstream_req_write_i,
    input  cache_data_t                                     upstream_req_wdata_i,
    input  cache_mask_t                                     upstream_req_wmask_i,

    /// Upstream response -- cache read output
    output logic                                            upstream_resp_valid_o,
    input  logic                                            upstream_resp_ready_i,
    output cache_data_t                                     upstream_resp_data_o,
    output info_t                                           upstream_resp_info_o,

    /// Downstream request -- evict request to DRAM
    output logic                                            downstream_req_evic_valid_o,
    input  logic                                            downstream_req_evic_ready_i,
    output addr_t                                           downstream_req_evic_addr_o,
    output cache_data_t                                     downstream_req_evic_data_o,
    output cache_mask_t                                     downstream_req_evic_mask_o,

    /// Downstream request -- miss request to DRAM
    output logic                                            downstream_req_miss_valid_o,
    input  logic                                            downstream_req_miss_ready_i,
    output addr_t                                           downstream_req_miss_addr_o,
    output downstream_info_t                                downstream_req_miss_info_o,

    /// Downsteam response -- cache refilling channel
    input  logic                                            downstream_resp_refill_valid_i,
    output logic                                            downstream_resp_refill_ready_o,
    input  cache_data_t                                     downstream_resp_refill_data_i,
    input  downstream_info_t                                downstream_resp_refill_info_i,

    /// Cache Bank Req/Resp
    output cache_bank_depth_ptr_t                           bank_read_addr_o,
    output logic [PartIdxWidth-1:0]                         bank_read_part_idx_o,
    output logic                                            bank_read_all_parts_o,
    output logic                                            bank_read_valid_o,
    input  logic                                            bank_read_ready_i,
    output logic                    [SetAssociativity-1:0]  bank_read_way_mask_o,
    input  cache_status_t           [SetAssociativity-1:0]  bank_read_cache_status_i,
    input  logic                    [SetAssociativity-1:0]  bank_read_cache_dirty_i,
    input  miss_meta_t              [SetAssociativity-1:0]  bank_read_cache_miss_meta_i,
    input  cache_mask_t             [SetAssociativity-1:0]  bank_read_cache_mask_i,
    input  cache_tag_t              [SetAssociativity-1:0]  bank_read_cache_tag_i,
    input  cache_data_t             [SetAssociativity-1:0]  bank_read_cache_data_i,
    input  way_ptr_t                [SetAssociativity-1:0]  bank_read_cache_LRU_i,

    output logic                                            bank_write_req_o,
    output cache_bank_depth_ptr_t                           bank_write_addr_o,
    output way_ptr_t                                        bank_write_way_o,
    output cache_status_t           [SetAssociativity-1:0]  bank_write_cache_status_o,
    output logic                    [SetAssociativity-1:0]  bank_write_cache_dirty_o,
    output miss_meta_t              [SetAssociativity-1:0]  bank_write_cache_miss_meta_o,
    output cache_mask_t             [SetAssociativity-1:0]  bank_write_cache_mask_o,
    output cache_tag_t              [SetAssociativity-1:0]  bank_write_cache_tag_o,
    output cache_data_t             [SetAssociativity-1:0]  bank_write_cache_data_o,
    output cache_mask_t             [SetAssociativity-1:0]  bank_write_data_mask_o,
    output logic                                            bank_write_LRU_req_o,
    output way_ptr_t                [SetAssociativity-1:0]  bank_write_cache_LRU_o
    
);

    initial begin
        if ((LineBytes % PartSplit) != 0) begin
            $fatal(1, "PartSplit (%0d) must divide line bytes (%0d).", PartSplit, LineBytes);
        end
    end

    //////////////////////////////////////
    //        Local Parameters          //
    //////////////////////////////////////

    localparam int unsigned InfoWidth                       = $bits(info_t);
    localparam int unsigned InfoStoreWidth                  = ((InfoWidth + ByteWidth - 1) / ByteWidth) * ByteWidth;
    localparam int unsigned SubarrayCounterWidth            = CacheLineWidth/WordWidth;
    localparam int unsigned MaxNumSubarray                  = CacheLineWidth/InfoStoreWidth;
    localparam int unsigned NumSubarray                     = MaxNumSubarray > (2**SubarrayCounterWidth)-2? (2**SubarrayCounterWidth)-2 : MaxNumSubarray;
    localparam int unsigned SubarrayCntWidth                = (NumSubarray > 0) ? $clog2(NumSubarray + 1) : 1;
    localparam int unsigned MSHRPadWidth                    = CacheLineWidth - MaxNumSubarray*InfoStoreWidth;
    localparam int unsigned TaskPayloadPad                  = ReqAddrWidth + CacheLineWidth/ByteWidth + InfoWidth - $bits(downstream_info_t);
    localparam type subarray_cnt_t                          = logic [SubarrayCntWidth-1:0];
`ifdef INSITU_CACHE_CORE_USE_MSHR_PADING
    localparam int unsigned MshrPadBits                     = MSHRPadWidth;
`else
    localparam int unsigned MshrPadBits                     = 0;
`endif



    //////////////////////////////////////
    //        Types Definition          //
    //////////////////////////////////////

    /**********/
    /*  MSHR  */
    /**********/

    //MSHR types
    typedef logic [MaxNumSubarray-1:0][InfoStoreWidth-1:0] mshr_subarrays;

    //MSHR payload
`ifdef INSITU_CACHE_CORE_USE_MSHR_PADING
    typedef struct packed {
        mshr_subarrays                                      subarrays;
        logic [MSHRPadWidth-1:0]                            mshr_pad;
    } mshr_payload_t;
`else
    typedef struct packed {
        mshr_subarrays                                      subarrays;
    } mshr_payload_t;
`endif




    /****************/
    /*  Cache Line  */
    /****************/

    //Byte packed cache data
    typedef logic [WordWidth-1:0]                           cache_word_t;
    typedef cache_word_t [CacheLineWidth/WordWidth-1:0]     cache_data_in_words_t;
    typedef logic [ByteWidth-1:0]                           cache_byte_t;
    typedef cache_byte_t [CacheLineWidth/ByteWidth-1:0]     cache_data_in_bytes_t;

    //Cache payload
    typedef union packed {
        mshr_payload_t                                      mshr;
        cache_data_t                                        data;
    } cache_payload_union_t;

    //Cache line type
    typedef struct packed {
        cache_status_t                                      status;
        logic                                               dirty;
        cache_mask_t                                        mask;
        cache_tag_t                                         tag;
        cache_payload_union_t                               payload;
        way_ptr_t                                           LRU;
    } cache_line_t;



    /****************/
    /*  Cache Task  */
    /****************/

    //Payload of requests to cache
    typedef struct packed {
        logic                                               write;
        addr_t                                              addr;
        cache_data_t                                        wdata;
        cache_mask_t                                        wmask;
        info_t                                              info;
    } cache_request_payload_t;

    function automatic cache_mask_t mshr_subarray_mask(input subarray_cnt_t idx);
        automatic cache_mask_t mask;
        automatic int unsigned bit_base;
        automatic int unsigned byte_base;
        automatic int unsigned byte_end;
        mask = '0;
        // Use byte-aligned subarray slots to avoid overlap between adjacent infos.
        bit_base = MshrPadBits + (idx * InfoStoreWidth);
        byte_base = bit_base / ByteWidth;
        byte_end = (bit_base + InfoStoreWidth + ByteWidth - 1) / ByteWidth;
        for (int bt = byte_base; bt < byte_end; bt++) begin
            mask[bt] = 1'b1;
        end
        return mask;
    endfunction

    function automatic cache_data_t mshr_subarray_data(input subarray_cnt_t idx, input info_t info);
        automatic cache_payload_union_t payload;
        payload = '0;
        payload.mshr.subarrays = '0;
        payload.mshr.subarrays[idx][InfoWidth-1:0] = info;
        return payload.data;
    endfunction

    //Payload of cache refill
    typedef struct packed {
        downstream_info_t                                   info;
        cache_data_t                                        data;
        logic [TaskPayloadPad:0]                            pad;
    } cache_refill_payload_t;

    //Union for request + refill
    typedef union packed {
        cache_request_payload_t                             request;
        cache_refill_payload_t                              refill;
    } cache_task_union_t;

    //Payload for cache tasks (request & refill)
    typedef struct packed {
        logic                                               valid;
        logic                                               is_refill;
        cache_task_union_t                                  task_pay;
    } task_payload_t;



    /********************/
    /*  Cache Response  */
    /********************/
    typedef struct packed {
        cache_data_t                                        data;
        info_t                                              info;
    } cache_resp_t;



    /********************/
    /*  MSHR Retrieval  */
    /********************/
    typedef struct packed {
        cache_data_t                                        data;
        cache_mask_t                                        num_subarray;
        mshr_subarrays                                      subarrays;
        logic                                               one_more;
        info_t                                              extra_subarray;
    } MSHR_retr_t;



    /****************/
    /*  Cache Miss  */
    /****************/
    typedef struct packed {
        addr_t                                              addr;
        downstream_info_t                                   info;
    } cache_miss_t;



    /*****************/
    /*  Cache Evict  */
    /*****************/
    typedef struct packed {
        addr_t                                              addr;
        cache_data_t                                        wdata;
        cache_mask_t                                        wmask;
    } cache_evic_t;



    /***************/
    /*  Cache FSM  */
    /***************/
    typedef enum logic[3:0] {
        REQ_PROC = '0,
        RESP_STALL,
        MISS_STALL,
        EVIC_STALL,
        ALL_PEND_STALL,
        MSHR_FULL_STALL,
        WR_CONFLICT_STALL
    } cache_fsm_status_t;

    typedef struct packed {
        logic                                               evic_stall;
        cache_evic_t                                        evic;
        way_ptr_t                                           evic_way;
        cache_bank_depth_ptr_t                              evic_depth;
        cache_miss_t                                        miss;
    } miss_stall_t;

`ifndef TARGET_SYNTHESIS

    /**********************/
    /*  Cache Life Cycle  */
    /**********************/

    typedef enum logic[1:0] {
        LIFECYCLE_INVALID = '0,
        LIFECYCLE_WRITE_PENDING,
        LIFECYCLE_READ_PENDING,
        LIFECYCLE_USELESS
    } life_cycle_state_t;

    typedef struct packed {
        logic [63:0]                                        invalid_cnt;
        logic [63:0]                                        write_pending_cnt;
        logic [63:0]                                        read_pending_cnt;
        logic [63:0]                                        usefull_cnt;
        logic [63:0]                                        useless_cnt;
        logic [63:0]                                        last_record_time;
        life_cycle_state_t                                  current_state;
    } life_cycle_scoreboard_t;

`endif

    //////////////////////////////////////
    //        Signal Definition         //
    //////////////////////////////////////

    //Response Fifo
    cache_resp_t                                            resp_fifo_in;
    logic                                                   resp_fifo_full;
    logic                                                   resp_fifo_push;
    cache_resp_t                                            resp_fifo_out;
    logic                                                   resp_fifo_empty;
    logic                                                   resp_fifo_pop;

    //MSHR retrieve Fifo
    MSHR_retr_t                                             retr_fifo_in;
    logic                                                   retr_fifo_full;
    logic                                                   retr_fifo_push;
    MSHR_retr_t                                             retr_fifo_out;
    logic                                                   retr_fifo_empty;
    logic                                                   retr_fifo_pop;
    logic [$clog2(RetrFifoDepth)-1:0]                       retr_fifo_usage;

    //Data response after MSHR retrieving
    logic                                                   retr_resp_valid;
    logic                                                   retr_resp_ready;
    cache_resp_t                                            retr_resp_payload;
    //Counter for response retrieving
    logic [$clog2(NumSubarray) :0]                          resp_cnt_q, resp_cnt_d;
    `FFARN (resp_cnt_q,             resp_cnt_d,             '0, clk_i, rst_ni)

    //Miss Fifo
    cache_miss_t                                            miss_fifo_in;
    logic                                                   miss_fifo_full;
    logic                                                   miss_fifo_push;
    cache_miss_t                                            miss_fifo_out;
    logic                                                   miss_fifo_empty;
    logic                                                   miss_fifo_pop;

    //Evict Fifo
    cache_evic_t                                            evic_fifo_in;
    logic                                                   evic_fifo_full;
    logic                                                   evic_fifo_push;
    cache_evic_t                                            evic_fifo_out;
    logic                                                   evic_fifo_empty;
    logic                                                   evic_fifo_pop;

`ifdef ENABLE_MULTI_READ_PEND
    //Pesudo Refill Fifo
    cache_refill_payload_t                                  pesudo_refill_fifo_in;
    logic                                                   pesudo_refill_fifo_full;
    logic                                                   pesudo_refill_fifo_push;
    cache_refill_payload_t                                  pesudo_refill_fifo_out;
    logic                                                   pesudo_refill_fifo_empty;
    logic                                                   pesudo_refill_fifo_pop;
    logic [$clog2(PesudoRefillFifoDepth):0]                 pesudo_refill_cnt_q, pesudo_refill_cnt_d;
    `FFARN (pesudo_refill_cnt_q, pesudo_refill_cnt_d,       '0, clk_i, rst_ni)

    //multi-read-pend counter
    logic [63:0]                                            MRP_duplicat_cnt_q,MRP_duplicat_cnt_d;
    logic [63:0]                                            MRP_duplicat_max_q,MRP_duplicat_max_d;
    `FFARN (MRP_duplicat_cnt_q,MRP_duplicat_cnt_d,          '0, clk_i, rst_ni)
    `FFARN (MRP_duplicat_max_q,MRP_duplicat_max_d,          '0, clk_i, rst_ni)
`endif

    //Bank Pre-reader
    logic                                                   preread_allowed;
    logic                                                   preread_request_valid;
    logic                                                   preread_request_ready;
    logic                                                   preread_refill_valid;
    logic                                                   preread_refill_ready;
    logic                                                   preread_refill_allowed;
    task_payload_t                                          preread_request_payload;
    task_payload_t                                          preread_refill_payload;
    task_payload_t                                          preread_arbiter_payload;
    task_payload_t                                          preread_task_q, preread_task_d;
    `FFARN (preread_task_q,         preread_task_d,         '0, clk_i, rst_ni)
    logic           [PartIdxWidth-1:0]                      preread_part_idx;
    logic                                                   bank_read_valid_arb;
    cache_bank_depth_ptr_t                                  bank_read_addr_arb;
    logic                                                   evict_full_block;
    logic                                                   refill_full_read_req;
    logic                    [SetAssociativity-1:0]        refill_read_way_mask;

    way_ptr_t                                               evict_stall_way_q,   evict_stall_way_d;
    cache_bank_depth_ptr_t                                  evict_stall_depth_q, evict_stall_depth_d;
    cache_data_t                                            evict_full_data_q,   evict_full_data_d;
    logic                                                   evict_full_data_valid_q, evict_full_data_valid_d;
    logic                                                   evict_full_wait_q,   evict_full_wait_d;
    logic                                                   evict_full_read_req;
    cache_bank_depth_ptr_t                                  evict_full_read_addr;
    logic                    [SetAssociativity-1:0]        evict_read_way_mask;
`ifdef ENABLE_MULTI_READ_PEND
    logic                                                   multi_read_pend_break;
`endif

    //Cache Bank Decoder
    way_ptr_t                                               dec_way;
    logic                                                   dec_is_write_req;
    logic                                                   dec_is_hit;
    logic                                                   dec_is_hit_pend;
    logic                                                   dec_is_hit_conflit;
    logic                                                   dec_is_all_pend;
    cache_status_t                                          dec_cache_status;
    logic                                                   dec_cache_dirty;
    miss_meta_t                                             dec_cache_miss_meta;
    cache_mask_t                                            dec_cache_mask;
    cache_tag_t                                             dec_cache_tag;
    cache_data_t                                            dec_cache_data;
`ifdef ENABLE_MULTI_READ_PEND
    logic                                                   dec_is_hit_pend_new_entry;
    way_ptr_t                                               dec_read_hit_pend_prime_way;
    way_ptr_t                                               dec_read_hit_pend_linkable_way;
`endif

    //Cache Bank Encoder
    way_ptr_t                                               enc_way;
    cache_status_t                                          enc_cache_status;
    logic                                                   enc_cache_dirty;
    miss_meta_t                                             enc_cache_miss_meta;
    cache_mask_t                                            enc_cache_mask;
    cache_tag_t                                             enc_cache_tag;
    cache_data_t                                            enc_cache_data;
    logic                                                   enc_mod_data_with_mask;
    cache_mask_t                                            enc_mod_mask;
    cache_data_t                                            enc_mod_write_data;
`ifdef ENABLE_MULTI_READ_PEND
    logic                                                   enc_link_exec;
    way_ptr_t                                               enc_link_src_way;
    way_ptr_t                                               enc_link_dst_way;
`endif

    //Cache FSM
    cache_fsm_status_t                                      cache_status_q,         cache_status_d;
    cache_resp_t                                            fsm_resp_stall_q,       fsm_resp_stall_d;
    miss_stall_t                                            fsm_miss_stall_q,       fsm_miss_stall_d;
    cache_evic_t                                            fsm_evic_stall_q,       fsm_evic_stall_d;
    cache_request_payload_t                                 fsm_refill_stall_q,     fsm_refill_stall_d;
    way_ptr_t                                               fsm_refill_way_q,       fsm_refill_way_d;

    `FFARN (cache_status_q,         cache_status_d,         REQ_PROC, clk_i, rst_ni)
    `FFARN (fsm_resp_stall_q,       fsm_resp_stall_d,       '0, clk_i, rst_ni)
    `FFARN (fsm_miss_stall_q,       fsm_miss_stall_d,       '0, clk_i, rst_ni)
    `FFARN (fsm_evic_stall_q,       fsm_evic_stall_d,       '0, clk_i, rst_ni)
    `FFARN (evict_stall_way_q,      evict_stall_way_d,      '0, clk_i, rst_ni)
    `FFARN (evict_stall_depth_q,    evict_stall_depth_d,    '0, clk_i, rst_ni)
    `FFARN (evict_full_data_q,      evict_full_data_d,      '0, clk_i, rst_ni)
    `FFARN (evict_full_data_valid_q, evict_full_data_valid_d, '0, clk_i, rst_ni)
    `FFARN (evict_full_wait_q,      evict_full_wait_d,      '0, clk_i, rst_ni)
    `FFARN (fsm_refill_stall_q,     fsm_refill_stall_d,     '0, clk_i, rst_ni)
    `FFARN (fsm_refill_way_q,       fsm_refill_way_d,       '0, clk_i, rst_ni)

    //Waveform-visible temporary signals used in Cache_FSM
    cache_tag_t                                             req_tag_tmp;
    cache_bank_depth_ptr_t                                  req_depth_tmp;
    byte_offset_t                                           req_ofst_tmp;
    way_ptr_t                                               req_way_tmp;
    logic                                                   req_is_write_tmp;
    logic                                                   req_is_hit_tmp;
    logic                                                   req_is_hit_pend_tmp;
    logic                                                   req_is_hit_conflict_tmp;
    logic                                                   req_is_all_pend_tmp;
    logic                                                   req_is_write_through_ignore_tmp;
    cache_payload_union_t                                   req_hit_pend_cache_payload_tmp;
    subarray_cnt_t                                          req_hit_pend_subarray_cnt_tmp;
    logic                                                   miss_is_full_masked_write_tmp;
    cache_payload_union_t                                   miss_cache_payload_tmp;
    logic                                                   refill_is_write_tmp;
    cache_tag_t                                             refill_req_tag_tmp;
    cache_bank_depth_ptr_t                                  refill_req_depth_tmp;
    byte_offset_t                                           refill_req_ofst_tmp;
    cache_payload_union_t                                   refill_cache_payload_tmp;
    cache_data_in_words_t                                   refill_cache_data_in_words_tmp;
    cache_data_in_words_t                                   refill_evic_data_in_words_tmp;
    cache_data_in_words_t                                   refill_write_data_in_words_tmp;
    cache_data_in_bytes_t                                   refill_cache_data_in_bytes_tmp;
    cache_data_in_bytes_t                                   refill_write_data_in_bytes_tmp;
    cache_mask_t                                            refill_write_storb_tmp;
    way_ptr_t                                               refill_way_tmp;
    logic                                                   refill_is_stalled_req_write_tmp;
    subarray_cnt_t                                          refill_retr_subarray_cnt_tmp;
    logic                                                   refill_all_pend_is_full_masked_write_tmp;
    cache_payload_union_t                                   refill_all_pend_cache_payload_tmp;

    logic [16-1:0] here_debug;

`ifndef TARGET_SYNTHESIS
    //Debugging
    cache_tag_t                                             debug_req_tag;
    cache_bank_depth_ptr_t                                  debug_req_depth;
    byte_offset_t                                           debug_req_ofst;
    cache_data_in_words_t                                   debug_modway_read_data;
    cache_data_in_words_t                                   debug_modway_write_data;
    cache_tag_t                                             debug_stalled_req_tag;
    cache_bank_depth_ptr_t                                  debug_stalled_req_depth;
    way_ptr_t                                               debug_stalled_way;
    way_ptr_t                                               debug_way;
    logic                                                   debug_is_write_req;
    logic                                                   debug_is_hit;
    logic                                                   debug_is_hit_pend;
    logic                                                   debug_is_hit_conflit;
    logic                                                   debug_is_all_pend;
    logic [63:0]                                            debug_num_hit_q, debug_num_hit_d;
    logic [63:0]                                            debug_num_miss_q, debug_num_miss_d;
    logic [63:0]                                            debug_num_resp_q, debug_num_resp_d;
    logic [63:0]                                            debug_num_cycle_q, debug_num_cycle_d;
    `FFARN (debug_num_hit_q,    debug_num_hit_d,            '0, clk_i, rst_ni)
    `FFARN (debug_num_miss_q,   debug_num_miss_d,           '0, clk_i, rst_ni)
    `FFARN (debug_num_resp_q,   debug_num_resp_d,           '0, clk_i, rst_ni)
    `FFARN (debug_num_cycle_q,  debug_num_cycle_d,          '0, clk_i, rst_ni)

    logic [63:0]                                            fsmcnt_REQ_PROC_q,          fsmcnt_REQ_PROC_d;
    logic [63:0]                                            fsmcnt_REQ_NOP_q,           fsmcnt_REQ_NOP_d;
    logic [63:0]                                            fsmcnt_RESP_STALL_q,        fsmcnt_RESP_STALL_d;
    logic [63:0]                                            fsmcnt_MISS_STALL_q,        fsmcnt_MISS_STALL_d;
    logic [63:0]                                            fsmcnt_EVIC_STALL_q,        fsmcnt_EVIC_STALL_d;
    logic [63:0]                                            fsmcnt_ALL_PEND_STALL_q,    fsmcnt_ALL_PEND_STALL_d;
    logic [63:0]                                            fsmcnt_MSHR_FULL_STALL_q,   fsmcnt_MSHR_FULL_STALL_d;
    logic [63:0]                                            fsmcnt_WR_CONFLICT_STALL_q, fsmcnt_WR_CONFLICT_STALL_d;
    `FFARN (fsmcnt_REQ_PROC_q,          fsmcnt_REQ_PROC_d,          '0, clk_i, rst_ni)
    `FFARN (fsmcnt_REQ_NOP_q,           fsmcnt_REQ_NOP_d,           '0, clk_i, rst_ni)
    `FFARN (fsmcnt_RESP_STALL_q,        fsmcnt_RESP_STALL_d,        '0, clk_i, rst_ni)
    `FFARN (fsmcnt_MISS_STALL_q,        fsmcnt_MISS_STALL_d,        '0, clk_i, rst_ni)
    `FFARN (fsmcnt_EVIC_STALL_q,        fsmcnt_EVIC_STALL_d,        '0, clk_i, rst_ni)
    `FFARN (fsmcnt_ALL_PEND_STALL_q,    fsmcnt_ALL_PEND_STALL_d,    '0, clk_i, rst_ni)
    `FFARN (fsmcnt_MSHR_FULL_STALL_q,   fsmcnt_MSHR_FULL_STALL_d,   '0, clk_i, rst_ni)
    `FFARN (fsmcnt_WR_CONFLICT_STALL_q, fsmcnt_WR_CONFLICT_STALL_d, '0, clk_i, rst_ni)

    //Cache line life cycle counter
    life_cycle_scoreboard_t [NumCacheEntry-1:0]             life_cycle_scoreboard_q, life_cycle_scoreboard_d;
    `FFARN (life_cycle_scoreboard_q,life_cycle_scoreboard_d,'0, clk_i, rst_ni)
`endif


    ///////////////////////////////////////////
    //        Bank Pre-Reader Logics         //
    ///////////////////////////////////////////

`ifdef ENABLE_MULTI_READ_PEND
    cache_refill_payload_t      all_refill_payload;
    logic                       all_refill_valid;
    logic                       all_refill_ready;

    cache_refill_payload_t      pesudo_refill_payload;
    logic                       pesudo_refill_valid;
    logic                       pesudo_refill_ready;

    cache_refill_payload_t      downstream_refill_payload;
    logic                       downstream_refill_valid;
    logic                       downstream_refill_ready;
    logic                       downstream_refill_lockup_q, downstream_refill_lockup_d;
    `FFARN (downstream_refill_lockup_q, downstream_refill_lockup_d,'0, clk_i, rst_ni)

    stream_arbiter #(.DATA_T(cache_refill_payload_t), .N_INP(2)) i_refill_arbiter (
        .clk_i,
        .rst_ni,
        .inp_data_i ({pesudo_refill_payload,    downstream_refill_payload}),
        .inp_valid_i({pesudo_refill_valid,      downstream_refill_valid}),
        .inp_ready_o({pesudo_refill_ready,      downstream_refill_ready}),
        .oup_data_o (all_refill_payload),
        .oup_valid_o(all_refill_valid),
        .oup_ready_i(all_refill_ready)
    );

    //pesudo refill
    assign pesudo_refill_payload                    = pesudo_refill_fifo_out;
    assign pesudo_refill_valid                      = ~pesudo_refill_fifo_empty;
    assign pesudo_refill_fifo_pop                   = pesudo_refill_ready & ~pesudo_refill_fifo_empty;

    //downstream refill
    assign downstream_refill_lockup_d = downstream_refill_valid & ~downstream_refill_ready;
    assign downstream_refill_payload = '{
        info: downstream_resp_refill_info_i,
        data: downstream_resp_refill_data_i,
        pad: '0
    };
    assign downstream_refill_valid                  =
        (downstream_refill_lockup_q | (pesudo_refill_cnt_q < (PesudoRefillFifoDepth-2))) &
        downstream_resp_refill_valid_i;
    assign downstream_resp_refill_ready_o           =
        (downstream_refill_lockup_q | (pesudo_refill_cnt_q < (PesudoRefillFifoDepth-2))) &
        downstream_refill_ready & ~evict_full_block;

    //all refill
    assign preread_refill_valid                     = all_refill_valid & ~evict_full_block;
    assign preread_refill_payload.task_pay.refill   = all_refill_payload;
    assign all_refill_ready                         = preread_refill_ready;

    //break request process if needed
    assign preread_request_valid = upstream_req_valid_i & preread_allowed & ~multi_read_pend_break &
                                   (pesudo_refill_cnt_q == '0) & ~evict_full_block;
`else
    assign preread_request_valid = upstream_req_valid_i & preread_allowed & ~evict_full_block;
    assign preread_refill_valid = downstream_resp_refill_valid_i & ~evict_full_block;
`endif
    assign evict_full_block = (cache_status_q == EVIC_STALL) && (PartSplit > 1);
    assign preread_refill_allowed = ~preread_arbiter_payload.is_refill | downstream_resp_refill_info_i.for_write_pend |  ((retr_fifo_usage < RetrFifoDepth - 2));

    assign preread_request_payload.valid = 1;
    assign preread_request_payload.is_refill = 0;
    assign preread_request_payload.task_pay.request = '{
        write: upstream_req_write_i,
        addr: upstream_req_addr_i,
        wdata: upstream_req_wdata_i,
        wmask: upstream_req_wmask_i,
        info: upstream_req_info_i
    };

    assign preread_refill_payload.valid = 1;
    assign preread_refill_payload.is_refill = 1;
`ifndef ENABLE_MULTI_READ_PEND
    assign preread_refill_payload.task_pay.refill = '{
        info: downstream_resp_refill_info_i,
        data: downstream_resp_refill_data_i,
        pad: '0
    };
`endif

    stream_arbiter #(.DATA_T(task_payload_t), .N_INP(2)) i_pre_reader_arbiter (
        .clk_i,
        .rst_ni,
        .inp_data_i ({preread_request_payload , preread_refill_payload}),
        .inp_valid_i({preread_request_valid, preread_refill_valid}),
        .inp_ready_o({preread_request_ready, preread_refill_ready}),
        .oup_data_o (preread_arbiter_payload),
        .oup_valid_o(bank_read_valid_arb),
        .oup_ready_i(bank_read_ready_i & preread_refill_allowed)
    );

    assign preread_task_d.valid = bank_read_valid_arb & bank_read_ready_i & preread_refill_allowed;
    assign preread_task_d.is_refill = preread_arbiter_payload.is_refill;
    assign preread_task_d.task_pay = preread_arbiter_payload.task_pay;
    assign upstream_req_ready_o = bank_read_valid_arb & bank_read_ready_i & preread_request_ready;
`ifndef ENABLE_MULTI_READ_PEND
    assign downstream_resp_refill_ready_o = preread_refill_ready & ~evict_full_block;
`endif

    assign bank_read_addr_arb = preread_task_d.is_refill?
                                preread_arbiter_payload.task_pay.refill.info.depth :
                                preread_arbiter_payload.task_pay.request.addr[ $clog2(CacheBankDepth) + $clog2(CacheLineWidth/8)-1 : $clog2(CacheLineWidth/8)];

    always_comb begin
        preread_part_idx = '0;
        if ((PartSplit > 1) && bank_read_valid_arb && ~preread_arbiter_payload.is_refill) begin
            preread_part_idx = preread_arbiter_payload.task_pay.request.addr[$clog2(LineBytes)-1 : $clog2(PartBytes)];
        end
    end
    assign refill_full_read_req = (PartSplit > 1) && bank_read_valid_arb && preread_arbiter_payload.is_refill;
    generate
        if (SetAssociativity == 1) begin : gen_evict_way_mask_single
            assign evict_read_way_mask = 1'b1;
        end else begin : gen_evict_way_mask
            always_comb begin
                evict_read_way_mask = '0;
                evict_read_way_mask[evict_stall_way_q] = 1'b1;
            end
        end
    endgenerate
    generate
        if (SetAssociativity == 1) begin : gen_refill_way_mask_single
            assign refill_read_way_mask = 1'b1;
        end else begin : gen_refill_way_mask
            always_comb begin
                refill_read_way_mask = '0;
                refill_read_way_mask[preread_arbiter_payload.task_pay.refill.info.way] = 1'b1;
            end
        end
    endgenerate

    assign bank_read_part_idx_o = (evict_full_read_req || refill_full_read_req) ? '0 : preread_part_idx;
    assign bank_read_addr_o = evict_full_read_req ? evict_full_read_addr : bank_read_addr_arb;
    assign bank_read_valid_o = evict_full_read_req ? 1'b1 : bank_read_valid_arb;
    assign bank_read_all_parts_o = evict_full_read_req | refill_full_read_req;
    assign bank_read_way_mask_o = evict_full_read_req ? evict_read_way_mask :
                                  refill_full_read_req ? refill_read_way_mask :
                                  {SetAssociativity{1'b1}};



    //////////////////////////////////////////
    //        Fifos and Output Path         //
    //////////////////////////////////////////

    /*****************************************/
    /*  Resp + Retrieve -> upsteam response  */
    /*****************************************/
    //Resp Fifo
    fifo_v3 #(
        .FALL_THROUGH                       (1'b0                       ),
        .DEPTH                              (RespFifoDepth              ),
        .dtype                              (cache_resp_t               )
    ) i_resp_fifo (
        .clk_i,
        .rst_ni,
        .flush_i                            (1'b0                       ),
        .testmode_i                         (1'b0                       ),
        .full_o                             (resp_fifo_full             ),
        .empty_o                            (resp_fifo_empty            ),
        .usage_o                            (/*open*/                   ),
        .data_i                             (resp_fifo_in               ),
        .push_i                             (resp_fifo_push             ),
        .data_o                             (resp_fifo_out              ),
        .pop_i                              (resp_fifo_pop              )
    );


    //Retrieve Fifo
    fifo_v3 #(
        .FALL_THROUGH                       (1'b0                       ),
        .DEPTH                              (RetrFifoDepth              ),
        .dtype                              (MSHR_retr_t                )
    ) i_retrieval_fifo (
        .clk_i,
        .rst_ni,
        .flush_i                            (1'b0                       ),
        .testmode_i                         (1'b0                       ),
        .full_o                             (retr_fifo_full             ),
        .empty_o                            (retr_fifo_empty            ),
        .usage_o                            (retr_fifo_usage            ),
        .data_i                             (retr_fifo_in               ),
        .push_i                             (retr_fifo_push             ),
        .data_o                             (retr_fifo_out              ),
        .pop_i                              (retr_fifo_pop              )
    );

    //Control signals
    subarray_cnt_t                                          retr_num_subarray;
    logic                                                   retr_entry_empty;
    logic                                                   retr_last_resp;

    assign retr_num_subarray = retr_fifo_out.num_subarray[SubarrayCntWidth-1:0];
    assign retr_entry_empty  = (retr_num_subarray == '0);
    assign retr_last_resp    = (resp_cnt_q == (retr_num_subarray - 1'b1));

    // If a malformed/empty retrieval entry sneaks in, drop it without emitting
    // response beats to prevent out-of-bound subarray indexing and X propagation.
    assign retr_resp_valid = ~retr_fifo_empty & ~retr_entry_empty;
    assign retr_fifo_pop = ~retr_fifo_empty &
                           (retr_entry_empty | (retr_last_resp & retr_resp_ready));
    assign resp_cnt_d = retr_fifo_pop ? '0 :
                        (retr_resp_valid & retr_resp_ready) ? (resp_cnt_q + 1'b1) :
                        resp_cnt_q;

    //Datapath
    always_comb begin
        retr_resp_payload.info = '0;
        if (~retr_entry_empty) begin
            if (retr_last_resp & retr_fifo_out.one_more) begin
                retr_resp_payload.info = retr_fifo_out.extra_subarray;
            end else begin
                retr_resp_payload.info = retr_fifo_out.subarrays[resp_cnt_q][InfoWidth-1:0];
            end
        end
        retr_resp_payload.data = retr_fifo_out.data;
    end

    //Merge responses
    logic resp_fifo_ready;
    assign resp_fifo_pop = ~resp_fifo_empty & resp_fifo_ready;
    cache_resp_t cache_resp_payload;
    stream_arbiter #(.DATA_T(cache_resp_t), .N_INP(2)) i_cache_resp_arbiter (
        .clk_i,
        .rst_ni,
        .inp_data_i ({resp_fifo_out     , retr_resp_payload}),
        .inp_valid_i({~resp_fifo_empty  , retr_resp_valid}),
        .inp_ready_o({resp_fifo_ready   , retr_resp_ready}),
        .oup_data_o (cache_resp_payload),
        .oup_valid_o(upstream_resp_valid_o),
        .oup_ready_i(upstream_resp_ready_i)
    );

    assign {upstream_resp_data_o, upstream_resp_info_o} = cache_resp_payload;

    /**************************************/
    /*  miss fifo -> downstream requests  */
    /**************************************/

    //Miss Fifo
    fifo_v3 #(
        .FALL_THROUGH                       (1'b0                       ),
        .DEPTH                              (MissFifoDepth              ),
        .dtype                              (cache_miss_t               )
    ) i_miss_fifo (
        .clk_i,
        .rst_ni,
        .flush_i                            (1'b0                       ),
        .testmode_i                         (1'b0                       ),
        .full_o                             (miss_fifo_full             ),
        .empty_o                            (miss_fifo_empty            ),
        .usage_o                            (/*open*/                   ),
        .data_i                             (miss_fifo_in               ),
        .push_i                             (miss_fifo_push             ),
        .data_o                             (miss_fifo_out              ),
        .pop_i                              (miss_fifo_pop              )
    );

    assign downstream_req_miss_valid_o = ~miss_fifo_empty;
    assign miss_fifo_pop = downstream_req_miss_valid_o & downstream_req_miss_ready_i;
    assign downstream_req_miss_addr_o = miss_fifo_out.addr;
    assign downstream_req_miss_info_o = miss_fifo_out.info;


    /**************************************/
    /*  evic fifo -> downstream requests  */
    /**************************************/

    //evic Fifo
    fifo_v3 #(
        .FALL_THROUGH                       (1'b0                       ),
        .DEPTH                              (EvicFifoDepth              ),
        .dtype                              (cache_evic_t               )
    ) i_evic_fifo (
        .clk_i,
        .rst_ni,
        .flush_i                            (1'b0                       ),
        .testmode_i                         (1'b0                       ),
        .full_o                             (evic_fifo_full             ),
        .empty_o                            (evic_fifo_empty            ),
        .usage_o                            (/*open*/                   ),
        .data_i                             (evic_fifo_in               ),
        .push_i                             (evic_fifo_push             ),
        .data_o                             (evic_fifo_out              ),
        .pop_i                              (evic_fifo_pop              )
    );

    assign downstream_req_evic_valid_o = ~evic_fifo_empty;
    assign evic_fifo_pop = downstream_req_evic_valid_o & downstream_req_evic_ready_i;
    assign downstream_req_evic_addr_o = evic_fifo_out.addr;
    assign downstream_req_evic_data_o = evic_fifo_out.wdata;
    assign downstream_req_evic_mask_o = evic_fifo_out.wmask;

`ifdef ENABLE_MULTI_READ_PEND
    /**************************************/
    /*         pesudo refill fifo         */
    /**************************************/
    fifo_v3 #(
        .FALL_THROUGH                       (1'b0                       ),
        .DEPTH                              (PesudoRefillFifoDepth      ),
        .dtype                              (cache_refill_payload_t     )
    ) i_pesudo_refill_fifo (
        .clk_i,
        .rst_ni,
        .flush_i                            (1'b0                       ),
        .testmode_i                         (1'b0                       ),
        .full_o                             (pesudo_refill_fifo_full    ),
        .empty_o                            (pesudo_refill_fifo_empty   ),
        .usage_o                            (/*open*/                   ),
        .data_i                             (pesudo_refill_fifo_in      ),
        .push_i                             (pesudo_refill_fifo_push    ),
        .data_o                             (pesudo_refill_fifo_out     ),
        .pop_i                              (pesudo_refill_fifo_pop     )
    );
`endif

    ///////////////////////////////////////
    //        Cache Bank Decoder         //
    ///////////////////////////////////////

    insitu_cache_decoder #(
        .ReqAddrWidth    (ReqAddrWidth),
        .CacheLineWidth  (CacheLineWidth),
        .task_payload_t  (task_payload_t),
        .NumCacheEntry   (NumCacheEntry),
        .SetAssociativity(SetAssociativity),
        .WordWidth       (WordWidth),
        .ByteWidth       (ByteWidth)
    ) i_insitu_cache_decoder (
        .bank_read_cache_status_i,
        .bank_read_cache_dirty_i,
        .bank_read_cache_miss_meta_i,
        .bank_read_cache_mask_i,
        .bank_read_cache_tag_i,
        .bank_read_cache_data_i,
        .bank_read_cache_LRU_i,
`ifdef ENABLE_MULTI_READ_PEND
        .dec_is_hit_pend_new_entry_o     (dec_is_hit_pend_new_entry),
        .dec_read_hit_pend_prime_way_o   (dec_read_hit_pend_prime_way),
        .dec_read_hit_pend_linkable_way_o(dec_read_hit_pend_linkable_way),
`endif
        .cache_task_i            (preread_task_q        ),
        .dec_way_o               (dec_way               ),
        .dec_is_write_req_o      (dec_is_write_req      ),
        .dec_is_hit_o            (dec_is_hit            ),
        .dec_is_hit_pend_o       (dec_is_hit_pend       ),
        .dec_is_hit_conflit_o    (dec_is_hit_conflit    ),
        .dec_is_all_pend_o       (dec_is_all_pend       ),
        .dec_cache_status_o      (dec_cache_status      ),
        .dec_cache_dirty_o       (dec_cache_dirty       ),
        .dec_cache_miss_meta_o   (dec_cache_miss_meta   ),
        .dec_cache_mask_o        (dec_cache_mask        ),
        .dec_cache_tag_o         (dec_cache_tag         ),
        .dec_cache_data_o        (dec_cache_data        )
    );


    ///////////////////////////////////////
    //        Cache Bank Encoder         //
    ///////////////////////////////////////



    insitu_cache_encoder #(
        .ReqAddrWidth    (ReqAddrWidth),
        .CacheLineWidth  (CacheLineWidth),
        .task_payload_t  (task_payload_t),
        .NumCacheEntry   (NumCacheEntry),
        .SetAssociativity(SetAssociativity),
        .WordWidth       (WordWidth),
        .ByteWidth       (ByteWidth)
    ) i_insitu_cache_encoder (
        .clk_i,
        .rst_ni,
        .has_pend_line_o,
        .enc_LRU_update           (bank_write_LRU_req_o   ),
        .enc_way_i                (enc_way                ),
        .enc_cache_status_i       (enc_cache_status       ),
        .enc_cache_dirty_i        (enc_cache_dirty        ),
        .enc_cache_miss_meta_i    (enc_cache_miss_meta    ),
        .enc_cache_mask_i         (enc_cache_mask         ),
        .enc_cache_tag_i          (enc_cache_tag          ),
        .enc_cache_data_i         (enc_cache_data         ),
        .enc_mod_data_with_mask_i (enc_mod_data_with_mask ),
        .enc_mod_mask_i           (enc_mod_mask           ),
        .enc_mod_write_data_i     (enc_mod_write_data     ),
        .clear_pend_cnt_i         (clear_pend_cnt_i       ),
`ifdef ENABLE_MULTI_READ_PEND
        .enc_link_exec_i          (enc_link_exec          ),
        .enc_link_src_way_i       (enc_link_src_way       ),
        .enc_link_dst_way_i       (enc_link_dst_way       ),
`endif

        .bank_read_cache_status_i,
        .bank_read_cache_dirty_i,
        .bank_read_cache_miss_meta_i,
        .bank_read_cache_mask_i,
        .bank_read_cache_tag_i,
        .bank_read_cache_data_i,
        .bank_read_cache_LRU_i,

        .bank_write_cache_status_o,
        .bank_write_cache_dirty_o,
        .bank_write_cache_miss_meta_o,
        .bank_write_cache_mask_o,
        .bank_write_cache_tag_o,
        .bank_write_cache_data_o,
        .bank_write_data_mask_o,
        .bank_write_cache_LRU_o
    );



    //////////////////////////////
    //        Cache FSM         //
    //////////////////////////////

    always_comb begin : Cache_FSM
        /*****************/
        /* Defualt Value */
        /*****************/

        //Response Fifo
        resp_fifo_in = '0;
        resp_fifo_push = 0;
        //MSHR retrieve Fifo
        retr_fifo_in = '0;
        retr_fifo_push = 0;
        //Miss Fifo
        miss_fifo_in = '0;
        miss_fifo_push = 0;
        //Evict Fifo
        evic_fifo_in = '0;
        evic_fifo_push = 0;
        //Bank Pre-reader
        preread_allowed = 1;
        //FSM
        cache_status_d = cache_status_q;
        fsm_resp_stall_d = fsm_resp_stall_q;
        fsm_miss_stall_d = fsm_miss_stall_q;
        fsm_evic_stall_d = fsm_evic_stall_q;
        evict_stall_way_d = evict_stall_way_q;
        evict_stall_depth_d = evict_stall_depth_q;
        evict_full_data_d = evict_full_data_q;
        evict_full_data_valid_d = evict_full_data_valid_q;
        evict_full_wait_d = evict_full_wait_q;
        evict_full_read_req = 1'b0;
        evict_full_read_addr = evict_stall_depth_q;
        fsm_refill_stall_d = fsm_refill_stall_q;
        fsm_refill_way_d = fsm_refill_way_q;

        //Cache Bank Encoder
        enc_way = dec_way;
        enc_cache_status = dec_cache_status;
        enc_cache_dirty = dec_cache_dirty;
        enc_cache_miss_meta = dec_cache_miss_meta;
        enc_cache_mask = dec_cache_mask;
        enc_cache_tag = dec_cache_tag;
        enc_cache_data = dec_cache_data;
        enc_mod_data_with_mask = '0;
        enc_mod_mask = '0;
        enc_mod_write_data = '0;
`ifdef ENABLE_MULTI_READ_PEND
        enc_link_exec = '0;
        enc_link_src_way = dec_read_hit_pend_linkable_way;
        enc_link_dst_way = dec_way;
        //pesudo refill
        pesudo_refill_fifo_in = '0;
        pesudo_refill_fifo_push = '0;
        pesudo_refill_cnt_d = pesudo_refill_cnt_q;
        multi_read_pend_break = '0;
        //multi-read-pend counter
        MRP_duplicat_cnt_d = MRP_duplicat_cnt_q;
        MRP_duplicat_max_d = (MRP_duplicat_cnt_q > MRP_duplicat_max_q) ? MRP_duplicat_cnt_q : MRP_duplicat_max_q;
`endif

        //Cache Bank
        bank_write_addr_o = '0;
        bank_write_req_o = '0;
        bank_write_way_o = '0;
        bank_write_LRU_req_o = '0;

        //Waveform-visible temporary signals
        req_tag_tmp = '0;
        req_depth_tmp = '0;
        req_ofst_tmp = '0;
        req_way_tmp = dec_way;
        req_is_write_tmp = dec_is_write_req;
        req_is_hit_tmp = dec_is_hit;
        req_is_hit_pend_tmp = dec_is_hit_pend;
        req_is_hit_conflict_tmp = dec_is_hit_conflit;
        req_is_all_pend_tmp = dec_is_all_pend;
        req_is_write_through_ignore_tmp = 1'b0;
        req_hit_pend_cache_payload_tmp = '0;
        req_hit_pend_subarray_cnt_tmp = '0;
        miss_is_full_masked_write_tmp = 1'b0;
        miss_cache_payload_tmp = '0;
        refill_is_write_tmp = preread_task_q.task_pay.refill.info.for_write_pend;
        refill_req_tag_tmp = '0;
        refill_req_depth_tmp = '0;
        refill_req_ofst_tmp = '0;
        refill_cache_payload_tmp = '0;
        refill_cache_data_in_words_tmp = '0;
        refill_evic_data_in_words_tmp = '0;
        refill_write_data_in_words_tmp = '0;
        refill_cache_data_in_bytes_tmp = '0;
        refill_write_data_in_bytes_tmp = '0;
        refill_write_storb_tmp = '0;
        refill_way_tmp = dec_way;
        refill_is_stalled_req_write_tmp = 1'b0;
        refill_retr_subarray_cnt_tmp = '0;
        refill_all_pend_is_full_masked_write_tmp = 1'b0;
        refill_all_pend_cache_payload_tmp = '0;

        here_debug = '0;
`ifndef TARGET_SYNTHESIS
        //debug information
        if (LogDebug) begin
            debug_num_hit_d = debug_num_hit_q;
            debug_num_miss_d = debug_num_miss_q;
            debug_num_resp_d = debug_num_resp_q;
            if (upstream_resp_valid_o & upstream_resp_ready_i) begin
                debug_num_resp_d += 1;
            end
            if (upstream_req_valid_i & upstream_req_ready_o & upstream_req_write_i) begin
                debug_num_resp_d += 1;
            end
            debug_num_cycle_d = debug_num_cycle_q + 1;

            //FSM Statues Counter
            fsmcnt_REQ_PROC_d           = fsmcnt_REQ_PROC_q;
            fsmcnt_REQ_NOP_d            = fsmcnt_REQ_NOP_q;
            fsmcnt_RESP_STALL_d         = fsmcnt_RESP_STALL_q;
            fsmcnt_MISS_STALL_d         = fsmcnt_MISS_STALL_q;
            fsmcnt_EVIC_STALL_d         = fsmcnt_EVIC_STALL_q;
            fsmcnt_ALL_PEND_STALL_d     = fsmcnt_ALL_PEND_STALL_q;
            fsmcnt_MSHR_FULL_STALL_d    = fsmcnt_MSHR_FULL_STALL_q;
            fsmcnt_WR_CONFLICT_STALL_d  = fsmcnt_WR_CONFLICT_STALL_q;
        end

        //life-cycle scoreboard
        if (LogLifeCycle) begin
            life_cycle_scoreboard_d = life_cycle_scoreboard_q;
        end
`endif


        /*************************************/
        /* Main FSM -- Cache Request Process */
        /*************************************/
        case (cache_status_q)

            /*Request processing*/
            REQ_PROC: begin

                //1. Process when prereader got request task
                if (preread_task_q.valid & ~preread_task_q.is_refill) begin : prec_req_process
                    {req_tag_tmp, req_depth_tmp, req_ofst_tmp} = preread_task_q.task_pay.request.addr;
                    req_is_write_through_ignore_tmp = WriteThroughMode & req_is_write_tmp;

                    bank_write_way_o = req_way_tmp;
                    bank_write_addr_o = req_depth_tmp;

                    //4. Process for different request type
                    if (req_is_hit_tmp) begin

                        /*The request hit cache line*/
                        if (~req_is_write_tmp) begin

                            //5. Process of read hit
                            if (~resp_fifo_full) begin

                                //5.1 Prepare resp data to fifo
                                resp_fifo_in = '{
                                    data: dec_cache_data,
                                    info: preread_task_q.task_pay.request.info
                                };

                                //5.2 Push to resp fifo
                                resp_fifo_push = 1;

                            end else begin

                                //5.3 Prepare resp data to reg
                                fsm_resp_stall_d = '{
                                    data: dec_cache_data,
                                    info: preread_task_q.task_pay.request.info
                                };

                                //5.4 Change status and stop preread for request
                                cache_status_d = RESP_STALL;
                                preread_allowed = 0;

                            end

                            //5.5 Write to LRU
                            bank_write_LRU_req_o = 1;

                        end else begin

                            //6. Process of write hit
                            //6.1 Update dirty bit
                            enc_cache_dirty = 1;

                            //6.2 Update dirty mask
                            enc_cache_mask = dec_cache_mask | preread_task_q.task_pay.request.wmask;

                            //6.3 Update data line
                            enc_mod_data_with_mask = 1'b1;
                            enc_mod_mask = preread_task_q.task_pay.request.wmask;
                            enc_mod_write_data = preread_task_q.task_pay.request.wdata;

                            //6.5 Write to bank
                            bank_write_req_o = 1;

                            //6.6 Write to LRU
                            bank_write_LRU_req_o = 1;

                        end

                        //6.7 update life-cycle counter
                            `ifndef TARGET_SYNTHESIS
                            if (LogLifeCycle) begin
                                life_cycle_scoreboard_d[{req_depth_tmp,req_way_tmp}].usefull_cnt = $time -
                                                                                                   life_cycle_scoreboard_q[{req_depth_tmp,req_way_tmp}].last_record_time +
                                                                                                   life_cycle_scoreboard_q[{req_depth_tmp,req_way_tmp}].usefull_cnt;
                                life_cycle_scoreboard_d[{req_depth_tmp,req_way_tmp}].current_state = LIFECYCLE_USELESS;
                                life_cycle_scoreboard_d[{req_depth_tmp,req_way_tmp}].last_record_time = $time;
                            end
                        `endif

                    end 
                    else if (req_is_hit_pend_tmp & ~req_is_write_through_ignore_tmp) begin

                        /*The request hit pending line with same access type*/
                        if (~req_is_write_tmp) begin
                            
                            //7. Process of read hit on read pend line
                            req_hit_pend_cache_payload_tmp.data = dec_cache_data;
`ifdef ENABLE_MULTI_READ_PEND
                            //Cases for multi-read-pending
                            //|-------------------------------------------------------------------------------------------------------------------|
                            //| situation |  need new entry | all way pending |                            action                                 |
                            //|-------------------------------------------------------------------------------------------------------------------|
                            //|     A     |        0        |        x        | directly merge into [dec_way]; update full signal if needed       |
                            //|-------------------------------------------------------------------------------------------------------------------|
                            //|     B     |        1        |        0        | link [dec_read_hit_pend_linkable_way] to new way [dec_way]        |
                            //|           |                 |                 | create a new read-pending line with full=0, prime=0, link=0, cnt=1|
                            //|           |                 |                 | update life-cycle counter                                         |
                            //|-------------------------------------------------------------------------------------------------------------------|
                            //|     C     |        1        |        1        | Goto MSHR_FULL_STALL: pending on [dec_read_hit_pend_prime_way]    |
                            //|-------------------------------------------------------------------------------------------------------------------|

                            if (~dec_is_hit_pend_new_entry) begin
                                /*Situation A*/
                                req_hit_pend_subarray_cnt_tmp = dec_cache_mask[SubarrayCntWidth-1:0];
                                //7.2 Update subarrays
                                enc_cache_mask = cache_mask_t'(req_hit_pend_subarray_cnt_tmp + 1'b1);
                                if (PartSplit > 1) begin
                                    enc_mod_data_with_mask = 1'b1;
                                    enc_mod_mask = mshr_subarray_mask(req_hit_pend_subarray_cnt_tmp);
                                    // Preserve neighbor bits in the same byte lane when info_t is not byte-aligned.
                                    // The byte mask can touch bytes that contain adjacent subarray fields.
                                    req_hit_pend_cache_payload_tmp.mshr.subarrays[req_hit_pend_subarray_cnt_tmp][InfoWidth-1:0] =
                                        preread_task_q.task_pay.request.info;
                                    enc_mod_write_data = req_hit_pend_cache_payload_tmp.data;
                                end else begin
                                    req_hit_pend_cache_payload_tmp.mshr.subarrays[req_hit_pend_subarray_cnt_tmp][InfoWidth-1:0] =
                                        preread_task_q.task_pay.request.info;
                                    enc_cache_data = req_hit_pend_cache_payload_tmp.data;
                                end

                                //7.3 update full signal if needed
                                if (enc_cache_mask >= NumSubarray) begin
                                    enc_cache_miss_meta.is_full = 1;
                                end

                                //7.4 Write to bank
                                bank_write_req_o = 1;

                                //7.5 Write to LRU
                                bank_write_LRU_req_o = 1;

                            end else if (~dec_is_all_pend) begin
                                /*Situation B*/
                                //7.6 linking
                                enc_link_exec = 1'b1;

                                //7.7 creat new read pend
                                enc_cache_status = READ_PEND;
                                enc_cache_dirty = 0;
                                enc_cache_mask = 1'b1;
                                enc_cache_tag = req_tag_tmp;
                                enc_cache_miss_meta = '0;

                                //7.7.1 update full signal if needed
                                if (enc_cache_mask >= NumSubarray) begin
                                    enc_cache_miss_meta.is_full = 1;
                                end

                                //7.8 generate MSHR info
                                req_hit_pend_cache_payload_tmp.mshr.subarrays = '0;
                                req_hit_pend_cache_payload_tmp.mshr.subarrays[0][InfoWidth-1:0] = preread_task_q.task_pay.request.info;
                                enc_cache_data = req_hit_pend_cache_payload_tmp.data;

                                //7.9 Write to Bank
                                bank_write_req_o = 1;

                                //7.10 Write to LRU
                                bank_write_LRU_req_o = 1;

                                //11.10 Update Life-Cycle Scoreboard
                                `ifndef TARGET_SYNTHESIS
                                    if (LogLifeCycle) begin
                                        if (dec_cache_status == INVALID) begin
                                            life_cycle_scoreboard_d[{req_depth_tmp,req_way_tmp}].invalid_cnt = $time -
                                                                                                               life_cycle_scoreboard_q[{req_depth_tmp,req_way_tmp}].last_record_time +
                                                                                                               life_cycle_scoreboard_q[{req_depth_tmp,req_way_tmp}].invalid_cnt;
                                        end else begin
                                            life_cycle_scoreboard_d[{req_depth_tmp,req_way_tmp}].useless_cnt = $time -
                                                                                                               life_cycle_scoreboard_q[{req_depth_tmp,req_way_tmp}].last_record_time +
                                                                                                               life_cycle_scoreboard_q[{req_depth_tmp,req_way_tmp}].useless_cnt;
                                        end

                                        life_cycle_scoreboard_d[{req_depth_tmp,req_way_tmp}].current_state = LIFECYCLE_READ_PENDING;
                                        life_cycle_scoreboard_d[{req_depth_tmp,req_way_tmp}].last_record_time = $time;
                                    end

                                    //multi-read-pend counter
                                    MRP_duplicat_cnt_d += 1'b1;
                                `endif
                            end else begin
                                /*Situation C*/
                                //7.6 Prepare meta-data
                                fsm_refill_stall_d = preread_task_q.task_pay.request;
                                fsm_refill_way_d = dec_read_hit_pend_prime_way;

                                //7.7 Change status and stop preread for request
                                cache_status_d = MSHR_FULL_STALL;
                                preread_allowed = 0;
                            end
`else //No -- ENABLE_MULTI_READ_PEND
                            begin : proc_read_hit_pend_no_mrp
                            //7.1 Check whether subarrays are full
                            req_hit_pend_subarray_cnt_tmp = dec_cache_mask[SubarrayCntWidth-1:0];
                            if (req_hit_pend_subarray_cnt_tmp < NumSubarray) begin

                                /*Allow merge subarray*/
                                //7.2 Update subarrays
                                enc_cache_mask = cache_mask_t'(req_hit_pend_subarray_cnt_tmp + 1'b1);
                                if (PartSplit > 1) begin
                                    enc_mod_data_with_mask = 1'b1;
                                    enc_mod_mask = mshr_subarray_mask(req_hit_pend_subarray_cnt_tmp);
                                    // Preserve neighbor bits in the same byte lane when info_t is not byte-aligned.
                                    // The byte mask can touch bytes that contain adjacent subarray fields.
                                    req_hit_pend_cache_payload_tmp.mshr.subarrays[req_hit_pend_subarray_cnt_tmp][InfoWidth-1:0] =
                                        preread_task_q.task_pay.request.info;
                                    enc_mod_write_data = req_hit_pend_cache_payload_tmp.data;
                                end else begin
                                    req_hit_pend_cache_payload_tmp.mshr.subarrays[req_hit_pend_subarray_cnt_tmp][InfoWidth-1:0] =
                                        preread_task_q.task_pay.request.info;
                                    enc_cache_data = req_hit_pend_cache_payload_tmp.data;
                                end

                                //7.4 Write to bank
                                bank_write_req_o = 1;

                                //7.5 Write to LRU
                                bank_write_LRU_req_o = 1;

                            end else begin

                                /*will stall of subarray full*/
                                //7.6 Prepare meta-data
                                fsm_refill_stall_d = preread_task_q.task_pay.request;
                                fsm_refill_way_d = req_way_tmp;

                                //7.7 Change status and stop preread for request
                                cache_status_d = MSHR_FULL_STALL;
                                preread_allowed = 0;

                            end
                            end
`endif //ENABLE_MULTI_READ_PEND
                        end else begin
                            
                            //8. Process of write hit on write pend line
                            //8.1 Update dirty bit
                            enc_cache_dirty = 1;

                            //8.2 Update dirty mask
                            enc_cache_mask = dec_cache_mask | preread_task_q.task_pay.request.wmask;

                            //8.3 Update data line
                            enc_mod_data_with_mask = 1'b1;
                            enc_mod_mask = preread_task_q.task_pay.request.wmask;
                            enc_mod_write_data = preread_task_q.task_pay.request.wdata;

                            //8.5 Write to bank
                            bank_write_req_o = 1;

                            //8.6 Write to LRU
                            bank_write_LRU_req_o = 1;

                        end
                    end
                    else if (req_is_hit_conflict_tmp) begin

                        /*The request hit pending line with opposite access type*/
                        //    ** NOTE! **
                        //      for write-through mode, we can not ignore the case
                        //      of write confliction on read pending lines, we need
                        //      to introduce stall here to make sure data is correct

                        //9. Process hit conflict
                        //9.1 Prepare request infomation
                        fsm_refill_stall_d = preread_task_q.task_pay.request;
                        fsm_refill_way_d = req_way_tmp;

                        //9.2 Change status and stop preread for request
                        cache_status_d = WR_CONFLICT_STALL;
                        preread_allowed = 0;

                    end
                    else if (~req_is_write_through_ignore_tmp) begin

                        /*The request is cache miss*/
                        //10. Prcoess of miss
                        if (~req_is_all_pend_tmp) begin

                            /*exist cache line to replace*/
                            //11. Generate pending line
                            //11.1 Update cache line status
                            enc_cache_status = req_is_write_tmp? WRITE_PEND: READ_PEND;

                            //11.2 Update dirty bit & mask
                            enc_cache_dirty = req_is_write_tmp? 1:0;
                            enc_cache_mask = req_is_write_tmp? preread_task_q.task_pay.request.wmask: '0;

                            //11.3 Update tag
                            enc_cache_tag = req_tag_tmp;

                            //11.4 Update payload
                            if (~req_is_write_tmp) begin
                                here_debug[0] = 1;
                                //11.5 Generate Subarrays
                                enc_cache_mask = 1;
                                miss_cache_payload_tmp = '0;
                                miss_cache_payload_tmp.mshr.subarrays = '0;
                                miss_cache_payload_tmp.mshr.subarrays[0][InfoWidth-1:0] = preread_task_q.task_pay.request.info;
                                enc_cache_data = miss_cache_payload_tmp.data;
`ifdef ENABLE_MULTI_READ_PEND
                                enc_cache_miss_meta = '0;
                                enc_cache_miss_meta.is_prime = 1'b1;
                                if (enc_cache_mask >= NumSubarray) begin
                                    enc_cache_miss_meta.is_full = 1;
                                end
`endif

                            end else begin

                                //11.6 Generate written data
                                enc_cache_data = '0;
                                enc_mod_data_with_mask = 1'b1;
                                enc_mod_mask = preread_task_q.task_pay.request.wmask;
                                enc_mod_write_data = preread_task_q.task_pay.request.wdata;

                            end

                            //11.7 Check whether the miss is a full masked write
                            //     If yes, we directly refill line w/o fetching
                            miss_is_full_masked_write_tmp = req_is_write_tmp & (&preread_task_q.task_pay.request.wmask);
                            if (miss_is_full_masked_write_tmp) begin
                                enc_cache_status = VALID;
                            end else begin
                                // Hold off the next preread for one cycle so the newly-created
                                // pending line is visible to the following bank lookup.
                                preread_allowed = 1'b0;
                            end

                            //11.8 Write to Bank
                            bank_write_req_o = 1;

                            //11.9 Write to LRU
                            bank_write_LRU_req_o = 1;

                            //11.10 Update Life-Cycle Scoreboard
                            `ifndef TARGET_SYNTHESIS
                                if (LogLifeCycle) begin
                                    if (dec_cache_status == INVALID) begin
                                        life_cycle_scoreboard_d[{req_depth_tmp,req_way_tmp}].invalid_cnt = $time -
                                                                                                           life_cycle_scoreboard_q[{req_depth_tmp,req_way_tmp}].last_record_time +
                                                                                                           life_cycle_scoreboard_q[{req_depth_tmp,req_way_tmp}].invalid_cnt;
                                    end else begin
                                        life_cycle_scoreboard_d[{req_depth_tmp,req_way_tmp}].useless_cnt = $time -
                                                                                                           life_cycle_scoreboard_q[{req_depth_tmp,req_way_tmp}].last_record_time +
                                                                                                           life_cycle_scoreboard_q[{req_depth_tmp,req_way_tmp}].useless_cnt;
                                    end

                                    if (enc_cache_status == WRITE_PEND) begin
                                        life_cycle_scoreboard_d[{req_depth_tmp,req_way_tmp}].current_state = LIFECYCLE_WRITE_PENDING;
                                    end else
                                    if (enc_cache_status == READ_PEND) begin
                                        life_cycle_scoreboard_d[{req_depth_tmp,req_way_tmp}].current_state = LIFECYCLE_READ_PENDING;
                                    end else
                                    if (enc_cache_status == VALID) begin
                                        life_cycle_scoreboard_d[{req_depth_tmp,req_way_tmp}].current_state = LIFECYCLE_USELESS;
                                    end

                                    life_cycle_scoreboard_d[{req_depth_tmp,req_way_tmp}].last_record_time = $time;
                                end
                            `endif

                            //12. Check to replace cache line dirty
                            if (dec_cache_status == VALID && dec_cache_dirty == 1 && miss_is_full_masked_write_tmp == 0) begin

                                //12.1 Need both eviction and send miss req to DRAM
                                if (~miss_fifo_full & ~evic_fifo_full) begin

                                    /*Push both miss + evic fifo*/
                                    //12.1.1 Prepare miss payload
                                    req_ofst_tmp = '0;
                                    miss_fifo_in.addr = {req_tag_tmp,req_depth_tmp,req_ofst_tmp};
                                    miss_fifo_in.info.for_write_pend = req_is_write_tmp;
                                    miss_fifo_in.info.way = req_way_tmp;
                                    miss_fifo_in.info.depth = req_depth_tmp;

                                    //12.1.2 Push miss fifo
                                    miss_fifo_push = 1;

                                    if (PartSplit > 1) begin
                                        fsm_evic_stall_d.addr = {dec_cache_tag,req_depth_tmp,req_ofst_tmp};
                                        fsm_evic_stall_d.wdata = '0;
                                        fsm_evic_stall_d.wmask = dec_cache_mask;
                                        evict_stall_way_d = req_way_tmp;
                                        evict_stall_depth_d = req_depth_tmp;
                                        evict_full_data_valid_d = 1'b0;
                                        evict_full_wait_d = 1'b0;
                                        cache_status_d = EVIC_STALL;
                                        preread_allowed = 0;
                                    end else begin
                                        //12.1.3 Prepare evic payload
                                        evic_fifo_in.addr = {dec_cache_tag,req_depth_tmp,req_ofst_tmp};
                                        evic_fifo_in.wdata = dec_cache_data;
                                        evic_fifo_in.wmask = dec_cache_mask;

                                        //12.1.4 Push evic fifo
                                        evic_fifo_push = 1;
                                    end

                                end else if (~miss_fifo_full & evic_fifo_full) begin

                                    /*Push miss fifo + will stall of evic*/
                                    //12.1.5 Prepare miss payload
                                    req_ofst_tmp = '0;
                                    miss_fifo_in.addr = {req_tag_tmp,req_depth_tmp,req_ofst_tmp};
                                    miss_fifo_in.info.for_write_pend = req_is_write_tmp;
                                    miss_fifo_in.info.way = req_way_tmp;
                                    miss_fifo_in.info.depth = req_depth_tmp;

                                    //12.1.6 Push miss fifo
                                    miss_fifo_push = 1;

                                    //12.1.7 Prepare evic meta-data
                                    fsm_evic_stall_d.addr = {dec_cache_tag,req_depth_tmp,req_ofst_tmp};
                                    fsm_evic_stall_d.wdata = dec_cache_data;
                                    fsm_evic_stall_d.wmask = dec_cache_mask;
                                    evict_stall_way_d = req_way_tmp;
                                    evict_stall_depth_d = req_depth_tmp;
                                    evict_full_data_valid_d = 1'b0;
                                    evict_full_wait_d = 1'b0;

                                    //12.1.8 Change status and stop preread for request
                                    cache_status_d = EVIC_STALL;
                                    preread_allowed = 0;

                                end else if (miss_fifo_full & ~evic_fifo_full) begin

                                    /*Will stall of miss + push evic fifo*/
                                    //12.1.9 Prepare miss meta-data
                                    req_ofst_tmp = '0;
                                    fsm_miss_stall_d.evic_stall = 0;
                                    fsm_miss_stall_d.evic = '0;
                                    fsm_miss_stall_d.evic_way = '0;
                                    fsm_miss_stall_d.evic_depth = '0;
                                    fsm_miss_stall_d.miss.addr = {req_tag_tmp,req_depth_tmp,req_ofst_tmp};
                                    fsm_miss_stall_d.miss.info.for_write_pend = req_is_write_tmp;
                                    fsm_miss_stall_d.miss.info.way = req_way_tmp;
                                    fsm_miss_stall_d.miss.info.depth = req_depth_tmp;

                                    //12.1.10 Change status and stop preread for request
                                    cache_status_d = MISS_STALL;
                                    preread_allowed = 0;

                                    if (PartSplit > 1) begin
                                        fsm_miss_stall_d.evic_stall = 1;
                                        fsm_miss_stall_d.evic.addr = {dec_cache_tag,req_depth_tmp,req_ofst_tmp};
                                        fsm_miss_stall_d.evic.wdata = '0;
                                        fsm_miss_stall_d.evic.wmask = dec_cache_mask;
                                        fsm_miss_stall_d.evic_way = req_way_tmp;
                                        fsm_miss_stall_d.evic_depth = req_depth_tmp;
                                    end else begin
                                        //12.1.11 Prepare evic payload
                                        evic_fifo_in.addr = {dec_cache_tag,req_depth_tmp,req_ofst_tmp};
                                        evic_fifo_in.wdata = dec_cache_data;
                                        evic_fifo_in.wmask = dec_cache_mask;

                                        //12.1.12 Push evic fifo
                                        evic_fifo_push = 1;
                                    end

                                end else begin

                                    /*Will stall of miss fifo*/
                                    //12.1.13 Prepare miss meta-data
                                    req_ofst_tmp = '0;
                                    fsm_miss_stall_d.evic_stall = 1;
                                    fsm_miss_stall_d.evic.addr = {dec_cache_tag,req_depth_tmp,req_ofst_tmp};
                                    fsm_miss_stall_d.evic.wdata = dec_cache_data;
                                    fsm_miss_stall_d.evic.wmask = dec_cache_mask;
                                    fsm_miss_stall_d.evic_way = req_way_tmp;
                                    fsm_miss_stall_d.evic_depth = req_depth_tmp;
                                    fsm_miss_stall_d.miss.addr = {req_tag_tmp,req_depth_tmp,req_ofst_tmp};
                                    fsm_miss_stall_d.miss.info.for_write_pend = req_is_write_tmp;
                                    fsm_miss_stall_d.miss.info.way = req_way_tmp;
                                    fsm_miss_stall_d.miss.info.depth = req_depth_tmp;

                                    //12.1.14 Change status and stop preread for request
                                    cache_status_d = MISS_STALL;
                                    preread_allowed = 0;

                                end
                            end else if (dec_cache_status == VALID && dec_cache_dirty == 1 && miss_is_full_masked_write_tmp == 1) begin

                                //12.2 Only eviction, we don't need to send miss request
                                if (~evic_fifo_full) begin

                                    /*Allow to push evic fifo*/
                                    if (PartSplit > 1) begin
                                        fsm_evic_stall_d.addr = {dec_cache_tag,req_depth_tmp,req_ofst_tmp};
                                        fsm_evic_stall_d.wdata = '0;
                                        fsm_evic_stall_d.wmask = dec_cache_mask;
                                        evict_stall_way_d = req_way_tmp;
                                        evict_stall_depth_d = req_depth_tmp;
                                        evict_full_data_valid_d = 1'b0;
                                        evict_full_wait_d = 1'b0;
                                        cache_status_d = EVIC_STALL;
                                        preread_allowed = 0;
                                    end else begin
                                        //12.2.1 Prepare evic payload
                                        evic_fifo_in.addr = {dec_cache_tag,req_depth_tmp,req_ofst_tmp};
                                        evic_fifo_in.wdata = dec_cache_data;
                                        evic_fifo_in.wmask = dec_cache_mask;

                                        //12.2.2 Push evic fifo
                                        evic_fifo_push = 1;
                                    end

                                end else begin

                                    /*Will stall of evic fifo*/
                                    //12.2.3 Prepare evic meta-data
                                    fsm_evic_stall_d.addr = {dec_cache_tag,req_depth_tmp,req_ofst_tmp};
                                    fsm_evic_stall_d.wdata = dec_cache_data;
                                    fsm_evic_stall_d.wmask = dec_cache_mask;
                                    evict_stall_way_d = req_way_tmp;
                                    evict_stall_depth_d = req_depth_tmp;
                                    evict_full_data_valid_d = 1'b0;
                                    evict_full_wait_d = 1'b0;

                                    //12.2.4 Change status and stop preread for request
                                    cache_status_d = EVIC_STALL;
                                    preread_allowed = 0;

                                end
                            end else if (~miss_is_full_masked_write_tmp) begin

                                //12.3 No eviction needed
                                if (~miss_fifo_full) begin

                                    /*Allow to push miss fifo*/
                                    //12.3.1 Prepare miss payload
                                    req_ofst_tmp = '0;
                                    miss_fifo_in.addr = {req_tag_tmp,req_depth_tmp,req_ofst_tmp};
                                    miss_fifo_in.info.for_write_pend = req_is_write_tmp;
                                    miss_fifo_in.info.way = req_way_tmp;
                                    miss_fifo_in.info.depth = req_depth_tmp;

                                    //12.3.2 Push miss fifo
                                    miss_fifo_push = 1;

                                end else begin

                                    /*Will stall of miss fifo*/
                                    //12.3.3 Prepare miss meta-data
                                    req_ofst_tmp = '0;
                                    fsm_miss_stall_d.evic_stall = 0;
                                    fsm_miss_stall_d.evic = '0;
                                    fsm_miss_stall_d.evic_way = '0;
                                    fsm_miss_stall_d.evic_depth = '0;
                                    fsm_miss_stall_d.miss.addr = {req_tag_tmp,req_depth_tmp,req_ofst_tmp};
                                    fsm_miss_stall_d.miss.info.for_write_pend = req_is_write_tmp;
                                    fsm_miss_stall_d.miss.info.way = req_way_tmp;
                                    fsm_miss_stall_d.miss.info.depth = req_depth_tmp;

                                    //12.3.4 Change status and stop preread for request
                                    cache_status_d = MISS_STALL;
                                    preread_allowed = 0;

                                end
                            end

                        end else begin

                            /*All way are pending*/
                            //13 Process to all-pend stall
                            //13.1 Prepare all-pend meta-data
                            fsm_refill_stall_d = preread_task_q.task_pay.request;

                            //13.2 Change status and stop preread for request
                            cache_status_d = ALL_PEND_STALL;
                            preread_allowed = 0;

                        end

                    end

`ifndef TARGET_SYNTHESIS
                    /*Set Debug Information*/
                    if (LogDebug) begin
                        debug_req_tag = req_tag_tmp;
                        debug_req_depth = req_depth_tmp;
                        debug_req_ofst = req_ofst_tmp;
                        debug_modway_read_data = dec_cache_data;
                        debug_modway_write_data = enc_cache_data;
                        debug_way = req_way_tmp;
                        debug_is_write_req = req_is_write_tmp;
                        debug_is_hit = req_is_hit_tmp;
                        debug_is_hit_pend = req_is_hit_pend_tmp;
                        debug_is_hit_conflit = req_is_hit_conflict_tmp;
                        debug_is_all_pend = req_is_all_pend_tmp;
                        debug_num_hit_d = req_is_hit_tmp ? debug_num_hit_q + 1'b1 : debug_num_hit_q;
                        debug_num_miss_d = req_is_hit_tmp ? debug_num_miss_q : debug_num_miss_q + 1'b1;
                        fsmcnt_REQ_PROC_d += 1;
                    end
`endif

                end : prec_req_process
`ifndef TARGET_SYNTHESIS
                if (LogDebug) fsmcnt_REQ_NOP_d += 1;
`endif
            end 

            /*Response stall due to fifo full*/
            RESP_STALL: begin
                preread_allowed = 0;
                if (~resp_fifo_full) begin
                    resp_fifo_in = fsm_resp_stall_q;
                    resp_fifo_push = 1;
                    preread_allowed = 1;
                    cache_status_d = REQ_PROC;
                end
`ifndef TARGET_SYNTHESIS
                if (LogDebug) fsmcnt_RESP_STALL_d += 1;
`endif
            end


            /*Miss stall due to fifo full*/
            MISS_STALL: begin
                preread_allowed = 0;
                if (~miss_fifo_full) begin
                    miss_fifo_in = fsm_miss_stall_q.miss;
                    miss_fifo_push = 1;
                    if (fsm_miss_stall_q.evic_stall) begin
                        if (PartSplit > 1) begin
                            fsm_evic_stall_d = fsm_miss_stall_q.evic;
                            evict_stall_way_d = fsm_miss_stall_q.evic_way;
                            evict_stall_depth_d = fsm_miss_stall_q.evic_depth;
                            evict_full_data_valid_d = 1'b0;
                            evict_full_wait_d = 1'b0;
                            preread_allowed = 0;
                            cache_status_d = EVIC_STALL;
                        end else begin
                            if (~evic_fifo_full) begin
                                evic_fifo_in = fsm_miss_stall_q.evic;
                                evic_fifo_push = 1;
                                preread_allowed = 1;
                                cache_status_d = REQ_PROC;
                            end else begin
                                fsm_evic_stall_d = fsm_miss_stall_q.evic;
                                preread_allowed = 0;
                                cache_status_d = EVIC_STALL;
                            end
                        end
                        
                    end else begin
                        preread_allowed = 1;
                        cache_status_d = REQ_PROC;
                    end
                end
`ifndef TARGET_SYNTHESIS
                if (LogDebug) fsmcnt_MISS_STALL_d += 1;
`endif
            end


            /*Evict stall due to fifo full*/
            EVIC_STALL: begin
                preread_allowed = 0;
                if (PartSplit > 1) begin
                    if (evict_full_wait_q) begin
                        evict_full_data_d = bank_read_cache_data_i[evict_stall_way_q];
                        evict_full_data_valid_d = 1'b1;
                        evict_full_wait_d = 1'b0;
                    end

                    if (~evict_full_wait_q && ~evict_full_data_valid_q) begin
                        evict_full_read_req = 1'b1;
                        evict_full_read_addr = evict_stall_depth_q;
                        if (bank_read_ready_i) begin
                            evict_full_wait_d = 1'b1;
                        end
                    end

                    if (evict_full_data_valid_q) begin
                        if (~evic_fifo_full) begin
                            evic_fifo_in.addr = fsm_evic_stall_q.addr;
                            evic_fifo_in.wdata = evict_full_data_q;
                            evic_fifo_in.wmask = fsm_evic_stall_q.wmask;
                            evic_fifo_push = 1;
                            evict_full_data_valid_d = 1'b0;
                            preread_allowed = 1;
                            cache_status_d = REQ_PROC;
                        end
                    end
                end else begin
                    if (~evic_fifo_full) begin
                        evic_fifo_in = fsm_evic_stall_q;
                        evic_fifo_push = 1;
                        preread_allowed = 1;
                        cache_status_d = REQ_PROC;
                    end
                end
`ifndef TARGET_SYNTHESIS
                if (LogDebug) fsmcnt_EVIC_STALL_d += 1;
`endif
            end


            /*Stall due to all way are in pending status*/
            ALL_PEND_STALL: begin
                preread_allowed = 0;
`ifndef TARGET_SYNTHESIS
                if (LogDebug) fsmcnt_ALL_PEND_STALL_d += 1;
`endif
            end


            /*Stall due to read request hit a read-pend cache line but subarrays are full*/
            MSHR_FULL_STALL: begin
                preread_allowed = 0;
`ifndef TARGET_SYNTHESIS
                if (LogDebug) fsmcnt_MSHR_FULL_STALL_d += 1;
`endif
            end


            /*Stall due to write(read) request hit on read-pend(write-pend) cache line*/
            WR_CONFLICT_STALL: begin
                preread_allowed = 0;
`ifndef TARGET_SYNTHESIS
                if (LogDebug) fsmcnt_WR_CONFLICT_STALL_d += 1;
`endif
            end

            default: begin
                cache_status_d = REQ_PROC;
            end
        endcase



        /************************/
        /* Cache Refill Process */
        /************************/

        if (preread_task_q.valid & preread_task_q.is_refill) begin : proc_refill
            {refill_req_tag_tmp, refill_req_depth_tmp, refill_req_ofst_tmp} = fsm_refill_stall_q.addr;
            refill_cache_payload_tmp.data = dec_cache_data;
            refill_is_stalled_req_write_tmp = fsm_refill_stall_q.write;

            //1. Check whether it is read refill, then we can push to retrieval fifo
            if (~refill_is_write_tmp) begin

                //1.1 Prepare retrieve payload
                refill_retr_subarray_cnt_tmp = dec_cache_mask[SubarrayCntWidth-1:0];
                retr_fifo_in.data = preread_task_q.task_pay.refill.data;
                retr_fifo_in.num_subarray = cache_mask_t'(refill_retr_subarray_cnt_tmp);
                retr_fifo_in.subarrays = refill_cache_payload_tmp.mshr.subarrays;
                retr_fifo_in.one_more = 0;
                retr_fifo_in.extra_subarray = '0;
                
                //1.2 Check whether a read request is pending due to the same line has full subarrays
                if (cache_status_q == MSHR_FULL_STALL && 
                    preread_task_q.task_pay.refill.info.depth == refill_req_depth_tmp &&
                    preread_task_q.task_pay.refill.info.way == fsm_refill_way_q) begin

                    //1.2.1 Update retrieve payload
                    refill_retr_subarray_cnt_tmp = refill_retr_subarray_cnt_tmp + 1'b1;
                    retr_fifo_in.num_subarray = cache_mask_t'(refill_retr_subarray_cnt_tmp);
                    retr_fifo_in.one_more = 1;
                    retr_fifo_in.extra_subarray = fsm_refill_stall_q.info;

                    //1.2.2 Return FSM status and allow preread
                    cache_status_d = REQ_PROC;
                    preread_allowed = 1;

                end

                //1.3 Push retrieve fifo
                retr_fifo_push = (refill_retr_subarray_cnt_tmp != '0);
            end

            //2. Process of cache line refilling
            //2.1 Update cache line status
            enc_cache_status = VALID;

            //2.2 Form cache line data
            refill_cache_data_in_words_tmp = preread_task_q.task_pay.refill.data;

            //2.3 Update accroding to dirty bits
            if (dec_cache_dirty) begin
                refill_write_data_in_words_tmp = dec_cache_data;
                refill_cache_data_in_bytes_tmp = refill_cache_data_in_words_tmp;
                refill_write_data_in_bytes_tmp = refill_write_data_in_words_tmp;
                refill_write_storb_tmp = dec_cache_mask;
                for (int bt = 0; bt < CacheLineWidth/ByteWidth; bt++ ) begin
                    if (refill_write_storb_tmp[bt]) begin
                        refill_cache_data_in_bytes_tmp[bt] = refill_write_data_in_bytes_tmp[bt];
                    end
                end
                refill_cache_data_in_words_tmp = refill_cache_data_in_bytes_tmp;
            end

            //2.4 Update cache line data
            enc_cache_data = refill_cache_data_in_words_tmp;
`ifdef ENABLE_MULTI_READ_PEND
            enc_cache_miss_meta = '0;

            //Process of Multi-Read-Pending Line
            if (~refill_is_write_tmp) begin
                if (dec_cache_miss_meta.is_prime == 1'b1) begin
                    enc_cache_status = VALID;
                    if (dec_cache_miss_meta.link_enable) begin
                        pesudo_refill_fifo_in = preread_task_q.task_pay.refill;
                        pesudo_refill_fifo_in.info.way = dec_cache_miss_meta.link_ptr;
                        pesudo_refill_fifo_push = 1'b1;
                        pesudo_refill_cnt_d += 1'b1;
                        multi_read_pend_break = 1'b1;
                    end
                end else begin
                    enc_cache_status = INVALID;
                    if (dec_cache_miss_meta.link_enable) begin
                        pesudo_refill_fifo_in = preread_task_q.task_pay.refill;
                        pesudo_refill_fifo_in.info.way = dec_cache_miss_meta.link_ptr;
                        pesudo_refill_fifo_push = 1'b1;
                    end else begin
                        pesudo_refill_cnt_d -= 1'b1;
                    end

                    `ifndef TARGET_SYNTHESIS
                        MRP_duplicat_cnt_d -= 1'b1;
                    `endif
                end
            end
`endif

            //2.5 Write to bank
            bank_write_req_o = 1;
            bank_write_addr_o = preread_task_q.task_pay.refill.info.depth;
            bank_write_way_o = refill_way_tmp;

            //2.6 Record data in case of eviction
            refill_evic_data_in_words_tmp = refill_cache_data_in_words_tmp;

            //3 Write to LRU
            bank_write_LRU_req_o = 1;

            //4 update life-cycle counter
            `ifndef TARGET_SYNTHESIS
                if (LogLifeCycle) begin
                    if (life_cycle_scoreboard_q[{bank_write_addr_o,bank_write_way_o}].current_state == LIFECYCLE_READ_PENDING) begin
                        life_cycle_scoreboard_d[{bank_write_addr_o,bank_write_way_o}].read_pending_cnt = $time -
                            life_cycle_scoreboard_q[{bank_write_addr_o,bank_write_way_o}].last_record_time +
                            life_cycle_scoreboard_q[{bank_write_addr_o,bank_write_way_o}].read_pending_cnt;
                    end else
                    if (life_cycle_scoreboard_q[{bank_write_addr_o,bank_write_way_o}].current_state == LIFECYCLE_WRITE_PENDING) begin
                        life_cycle_scoreboard_d[{bank_write_addr_o,bank_write_way_o}].write_pending_cnt = $time -
                            life_cycle_scoreboard_q[{bank_write_addr_o,bank_write_way_o}].last_record_time +
                            life_cycle_scoreboard_q[{bank_write_addr_o,bank_write_way_o}].write_pending_cnt;
                    end

                    life_cycle_scoreboard_d[{bank_write_addr_o,bank_write_way_o}].current_state = LIFECYCLE_USELESS;
                    if (enc_cache_status == INVALID) begin
                        life_cycle_scoreboard_d[{bank_write_addr_o,bank_write_way_o}].current_state = LIFECYCLE_INVALID;
                    end
                    life_cycle_scoreboard_d[{bank_write_addr_o,bank_write_way_o}].last_record_time = $time;
                end
            `endif

            /*
            ***
            Process WR conflict stall when refill exactly match pending request
            ***
            */
            if (cache_status_q == WR_CONFLICT_STALL && 
                preread_task_q.task_pay.refill.info.depth == refill_req_depth_tmp &&
                preread_task_q.task_pay.refill.info.way == fsm_refill_way_q) begin : proc_refill_wr_conflict

                /*The stalled request hit refill cache line*/
                if (~refill_is_stalled_req_write_tmp) begin

                    //5. Process of RAW
                    if (~resp_fifo_full) begin

                        //5.1 Prepare resp data to fifo
                        resp_fifo_in = '{
                            data: refill_cache_data_in_words_tmp,
                            info: fsm_refill_stall_q.info
                        };

                        //5.2 Push to resp fifo
                        resp_fifo_push = 1;

                        //5.3 Return status and continue preread for request
                        cache_status_d = REQ_PROC;
                        preread_allowed = 1;

                    end else begin

                        //5.4 Prepare resp data to reg
                        fsm_resp_stall_d = '{
                            data: refill_cache_data_in_words_tmp,
                            info: fsm_refill_stall_q.info
                        };

                        //5.5 Change status and stop preread for request
                        cache_status_d = RESP_STALL;
                        preread_allowed = 0;

                    end

                end else begin

                    //6. Process of WAR
                    //6.1 Update dirty bit
                    //    ** NOTE! **
                    //      for write-through mode, we don't set the dirty bit
                    //      because the write is already bypassed to downstream
                    enc_cache_dirty = ~WriteThroughMode;

                    //6.2 Update dirty mask
                    enc_cache_mask = dec_cache_mask | fsm_refill_stall_q.wmask;

                    //6.3 Update data line
                    enc_cache_data = refill_cache_data_in_words_tmp;
                    enc_mod_data_with_mask = 1'b1;
                    enc_mod_mask = fsm_refill_stall_q.wmask;
                    enc_mod_write_data = fsm_refill_stall_q.wdata;

                    //6.4 Return status and continue preread for request
                    cache_status_d = REQ_PROC;
                    preread_allowed = 1;

                end 
            end : proc_refill_wr_conflict



            /*
            ***
            Process ALL-Pend stall when refill match pending request depth
            ***
            */
            if (cache_status_q == ALL_PEND_STALL && 
                preread_task_q.task_pay.refill.info.depth == refill_req_depth_tmp ) begin : proc_refill_all_pend

                /*Use the refilled cache line to replace*/
                //7. Generate pending line
                //7.1 Update cache line status
                enc_cache_status = refill_is_stalled_req_write_tmp ? WRITE_PEND : READ_PEND;

                //7.2 Update dirty bit & mask
                enc_cache_dirty = refill_is_stalled_req_write_tmp ? 1 : 0;
                enc_cache_mask = refill_is_stalled_req_write_tmp ? fsm_refill_stall_q.wmask : '0;

                //7.3 Update tag
                enc_cache_tag = refill_req_tag_tmp;

                //7.4 Update payload
                if (~refill_is_stalled_req_write_tmp) begin

                    //7.4.1 Generate Subarrays
                    enc_cache_mask = 1;
                    refill_all_pend_cache_payload_tmp = '0;
                    refill_all_pend_cache_payload_tmp.mshr.subarrays = '0;
                    refill_all_pend_cache_payload_tmp.mshr.subarrays[0][InfoWidth-1:0] = fsm_refill_stall_q.info;
                    enc_cache_data = refill_all_pend_cache_payload_tmp.data;
`ifdef ENABLE_MULTI_READ_PEND
                    enc_cache_miss_meta = '0;
                    enc_cache_miss_meta.is_prime = 1'b1;
                    if (enc_cache_mask >= NumSubarray) begin
                        enc_cache_miss_meta.is_full = 1;
                    end
`endif

                end else begin

                    //7.4.2 Generate written data
                    enc_cache_data = refill_cache_data_in_words_tmp;
                    enc_mod_data_with_mask = 1'b1;
                    enc_mod_mask = fsm_refill_stall_q.wmask;
                    enc_mod_write_data = fsm_refill_stall_q.wdata;

                end

                //7.5 Check whether the miss is a full masked write
                //     If yes, we directly refill line w/o fetching
                refill_all_pend_is_full_masked_write_tmp = refill_is_stalled_req_write_tmp & (&fsm_refill_stall_q.wmask);
                if (refill_all_pend_is_full_masked_write_tmp) begin
                    enc_cache_status = VALID;
                end

                //7.7 Return status and continue preread for request
                cache_status_d = REQ_PROC;
                preread_allowed = 1;
                if (~refill_all_pend_is_full_masked_write_tmp) begin
                    // Hold off the next preread for one cycle so the newly-created
                    // pending line is visible to the following bank lookup.
                    preread_allowed = 1'b0;
                end

                //7.8 update life-cycle state
                `ifndef TARGET_SYNTHESIS
                    if (LogLifeCycle) begin
                        life_cycle_scoreboard_d[{refill_req_depth_tmp,refill_way_tmp}].current_state = refill_all_pend_is_full_masked_write_tmp ? LIFECYCLE_USELESS :
                                                                                                       refill_is_stalled_req_write_tmp       ? LIFECYCLE_WRITE_PENDING :
                                                                                                                                             LIFECYCLE_READ_PENDING;
                    end
                `endif

                //8. Check to replace cache line dirty
                if (dec_cache_dirty == 1 && refill_all_pend_is_full_masked_write_tmp == 0) begin

                    //8.1 Need both eviction and send miss req to DRAM
                    if (~miss_fifo_full & ~evic_fifo_full) begin

                        /*Push both miss + evic fifo*/
                        //8.1.1 Prepare miss payload
                        refill_req_ofst_tmp = '0;
                        miss_fifo_in.addr = {refill_req_tag_tmp,refill_req_depth_tmp,refill_req_ofst_tmp};
                        miss_fifo_in.info.for_write_pend = refill_is_stalled_req_write_tmp;
                        miss_fifo_in.info.way = refill_way_tmp;
                        miss_fifo_in.info.depth = refill_req_depth_tmp;

                        //8.1.2 Push miss fifo
                        miss_fifo_push = 1;

                        if (PartSplit > 1) begin
                            fsm_evic_stall_d.addr = {dec_cache_tag,refill_req_depth_tmp,refill_req_ofst_tmp};
                            fsm_evic_stall_d.wdata = '0;
                            fsm_evic_stall_d.wmask = dec_cache_mask;
                            evict_stall_way_d = refill_way_tmp;
                            evict_stall_depth_d = refill_req_depth_tmp;
                            evict_full_data_valid_d = 1'b0;
                            evict_full_wait_d = 1'b0;
                            cache_status_d = EVIC_STALL;
                            preread_allowed = 0;
                        end else begin
                            //8.1.3 Prepare evic payload
                            evic_fifo_in.addr = {dec_cache_tag,refill_req_depth_tmp,refill_req_ofst_tmp};
                            evic_fifo_in.wdata = refill_evic_data_in_words_tmp;
                            evic_fifo_in.wmask = dec_cache_mask;

                            //8.1.4 Push evic fifo
                            evic_fifo_push = 1;
                        end

                    end else if (~miss_fifo_full & evic_fifo_full) begin

                        /*Push miss fifo + will stall of evic*/
                        //8.1.5 Prepare miss payload
                        refill_req_ofst_tmp = '0;
                        miss_fifo_in.addr = {refill_req_tag_tmp,refill_req_depth_tmp,refill_req_ofst_tmp};
                        miss_fifo_in.info.for_write_pend = refill_is_stalled_req_write_tmp;
                        miss_fifo_in.info.way = refill_way_tmp;
                        miss_fifo_in.info.depth = refill_req_depth_tmp;

                        //8.1.6 Push miss fifo
                        miss_fifo_push = 1;

                        //8.1.7 Prepare evic meta-data
                        fsm_evic_stall_d.addr = {dec_cache_tag,refill_req_depth_tmp,refill_req_ofst_tmp};
                        fsm_evic_stall_d.wdata = refill_evic_data_in_words_tmp;
                        fsm_evic_stall_d.wmask = dec_cache_mask;
                        evict_stall_way_d = refill_way_tmp;
                        evict_stall_depth_d = refill_req_depth_tmp;
                        evict_full_data_valid_d = 1'b0;
                        evict_full_wait_d = 1'b0;

                        //8.1.8 Change status and stop preread for request
                        cache_status_d = EVIC_STALL;
                        preread_allowed = 0;

                    end else if (miss_fifo_full & ~evic_fifo_full) begin

                        /*Will stall of miss + push evic fifo*/
                        //8.1.9 Prepare miss meta-data
                        refill_req_ofst_tmp = '0;
                        fsm_miss_stall_d.evic_stall = 0;
                        fsm_miss_stall_d.evic = '0;
                        fsm_miss_stall_d.evic_way = '0;
                        fsm_miss_stall_d.evic_depth = '0;
                        fsm_miss_stall_d.miss.addr = {refill_req_tag_tmp,refill_req_depth_tmp,refill_req_ofst_tmp};
                        fsm_miss_stall_d.miss.info.for_write_pend = refill_is_stalled_req_write_tmp;
                        fsm_miss_stall_d.miss.info.way = refill_way_tmp;
                        fsm_miss_stall_d.miss.info.depth = refill_req_depth_tmp;

                        //8.1.10 Change status and stop preread for request
                        cache_status_d = MISS_STALL;
                        preread_allowed = 0;

                        if (PartSplit > 1) begin
                            fsm_miss_stall_d.evic_stall = 1;
                            fsm_miss_stall_d.evic.addr = {dec_cache_tag,refill_req_depth_tmp,refill_req_ofst_tmp};
                            fsm_miss_stall_d.evic.wdata = '0;
                            fsm_miss_stall_d.evic.wmask = dec_cache_mask;
                            fsm_miss_stall_d.evic_way = refill_way_tmp;
                            fsm_miss_stall_d.evic_depth = refill_req_depth_tmp;
                        end else begin
                            //8.1.11 Prepare evic payload
                            evic_fifo_in.addr = {dec_cache_tag,refill_req_depth_tmp,refill_req_ofst_tmp};
                            evic_fifo_in.wdata = refill_evic_data_in_words_tmp;
                            evic_fifo_in.wmask = dec_cache_mask;

                            //8.1.12 Push evic fifo
                            evic_fifo_push = 1;
                        end

                    end else begin

                        /*Will stall of miss fifo*/
                        //8.1.13 Prepare miss meta-data
                        refill_req_ofst_tmp = '0;
                        fsm_miss_stall_d.evic_stall = 1;
                        fsm_miss_stall_d.evic.addr = {dec_cache_tag,refill_req_depth_tmp,refill_req_ofst_tmp};
                        fsm_miss_stall_d.evic.wdata = refill_evic_data_in_words_tmp;
                        fsm_miss_stall_d.evic.wmask = dec_cache_mask;
                        fsm_miss_stall_d.evic_way = refill_way_tmp;
                        fsm_miss_stall_d.evic_depth = refill_req_depth_tmp;
                        fsm_miss_stall_d.miss.addr = {refill_req_tag_tmp,refill_req_depth_tmp,refill_req_ofst_tmp};
                        fsm_miss_stall_d.miss.info.for_write_pend = refill_is_stalled_req_write_tmp;
                        fsm_miss_stall_d.miss.info.way = refill_way_tmp;
                        fsm_miss_stall_d.miss.info.depth = refill_req_depth_tmp;

                        //8.1.14 Change status and stop preread for request
                        cache_status_d = MISS_STALL;
                        preread_allowed = 0;

                    end
                end else if (dec_cache_dirty == 1 && refill_all_pend_is_full_masked_write_tmp == 1) begin

                    //8.2 Only eviction, we don't need to send miss request
                    if (~evic_fifo_full) begin

                        /*Allow to push evic fifo*/
                        if (PartSplit > 1) begin
                            fsm_evic_stall_d.addr = {dec_cache_tag,refill_req_depth_tmp,refill_req_ofst_tmp};
                            fsm_evic_stall_d.wdata = '0;
                            fsm_evic_stall_d.wmask = dec_cache_mask;
                            evict_stall_way_d = refill_way_tmp;
                            evict_stall_depth_d = refill_req_depth_tmp;
                            evict_full_data_valid_d = 1'b0;
                            evict_full_wait_d = 1'b0;
                            cache_status_d = EVIC_STALL;
                            preread_allowed = 0;
                        end else begin
                            //12.2.1 Prepare evic payload
                            evic_fifo_in.addr = {dec_cache_tag,refill_req_depth_tmp,refill_req_ofst_tmp};
                            evic_fifo_in.wdata = refill_evic_data_in_words_tmp;
                            evic_fifo_in.wmask = dec_cache_mask;

                            //12.2.2 Push evic fifo
                            evic_fifo_push = 1;
                        end

                    end else begin

                        /*Will stall of evic fifo*/
                        //12.2.3 Prepare evic meta-data
                        fsm_evic_stall_d.addr = {dec_cache_tag,refill_req_depth_tmp,refill_req_ofst_tmp};
                        fsm_evic_stall_d.wdata = refill_evic_data_in_words_tmp;
                        fsm_evic_stall_d.wmask = dec_cache_mask;
                        evict_stall_way_d = refill_way_tmp;
                        evict_stall_depth_d = refill_req_depth_tmp;
                        evict_full_data_valid_d = 1'b0;
                        evict_full_wait_d = 1'b0;

                        //12.2.4 Change status and stop preread for request
                        cache_status_d = EVIC_STALL;
                        preread_allowed = 0;

                    end
                end else if (refill_all_pend_is_full_masked_write_tmp == 0) begin

                    //8.3 No eviction needed
                    if (~miss_fifo_full) begin

                        /*Allow to push miss fifo*/
                        //8.3.1 Prepare miss payload
                        refill_req_ofst_tmp = '0;
                        miss_fifo_in.addr = {refill_req_tag_tmp,refill_req_depth_tmp,refill_req_ofst_tmp};
                        miss_fifo_in.info.for_write_pend = refill_is_stalled_req_write_tmp;
                        miss_fifo_in.info.way = refill_way_tmp;
                        miss_fifo_in.info.depth = refill_req_depth_tmp;

                        //8.3.2 Push miss fifo
                        miss_fifo_push = 1;

                    end else begin

                        /*Will stall of miss fifo*/
                        //8.3.3 Prepare miss meta-data
                        refill_req_ofst_tmp = '0;
                        fsm_miss_stall_d.evic_stall = 0;
                        fsm_miss_stall_d.evic = '0;
                        fsm_miss_stall_d.evic_way = '0;
                        fsm_miss_stall_d.evic_depth = '0;
                        fsm_miss_stall_d.miss.addr = {refill_req_tag_tmp,refill_req_depth_tmp,refill_req_ofst_tmp};
                        fsm_miss_stall_d.miss.info.for_write_pend = refill_is_stalled_req_write_tmp;
                        fsm_miss_stall_d.miss.info.way = refill_way_tmp;
                        fsm_miss_stall_d.miss.info.depth = refill_req_depth_tmp;

                        //8.3.4 Change status and stop preread for request
                        cache_status_d = MISS_STALL;
                        preread_allowed = 0;

                    end
                end

            end : proc_refill_all_pend

`ifndef TARGET_SYNTHESIS
            /*Set Debug Information*/
            if (LogDebug) begin
                debug_stalled_req_depth = refill_req_depth_tmp;
                debug_stalled_req_tag = refill_req_tag_tmp;
                debug_stalled_way = fsm_refill_way_q;
            end
`endif

        end : proc_refill

    end : Cache_FSM


    //////////////////////////////
    //        Debugging         //
    //////////////////////////////
`ifndef TARGET_SYNTHESIS
if (LogDebug) begin
    final begin
        automatic real total_req = debug_num_hit_q + debug_num_miss_q;
        automatic real miss_req = debug_num_miss_q;
        automatic real miss_rate = (total_req == 0)? 0: miss_req/total_req;
        automatic real total_cycle = debug_num_cycle_q;
        automatic real num_resp = debug_num_resp_q;
        automatic real bus_uti = (total_cycle == 0)? 0: num_resp/total_cycle;
        automatic real bandwidth = bus_uti * 64;
        automatic string cache_name = (ModeleName == "none")? $sformatf("%m"): ModeleName;
        $display(" ");
        $display(" ");
        $display(" ");
        $display(" ");
        $display("   Cache: %s   ",cache_name);
        $display("           |");
        $display("           v");
        $display("*********************************************************************************************");
        $display("***                             Insitu-Cache Statistics                                   ***");
        $display("   ---------------------------------------------------------------------------------------   ");
        $display("        Number of Hits:  %8d |  Number of Misses: %8d |  Miss Rate: %0.6f   ",debug_num_hit_q, debug_num_miss_q, miss_rate);
        $display("        Bus Utilization: %0.6f |  Bandwidth(GB/s):   %2.4f |",bus_uti, bandwidth);
        $display("   ---------------------------------------------------------------------------------------   ");
        $display("        FSM State Counting");
        $display("        < REQ_PROC >:             %8d",fsmcnt_REQ_PROC_q);
        $display("        < REQ_NOP >:              %8d",fsmcnt_REQ_NOP_q);
        $display("        < RESP_STALL >:           %8d",fsmcnt_RESP_STALL_q);
        $display("        < MISS_STALL >:           %8d",fsmcnt_MISS_STALL_q);
        $display("        < EVIC_STALL >:           %8d",fsmcnt_EVIC_STALL_q);
        $display("        < ALL_PEND_STALL >:       %8d",fsmcnt_ALL_PEND_STALL_q);
        $display("        < MSHR_FULL_STALL >:      %8d",fsmcnt_MSHR_FULL_STALL_q);
        $display("        < WR_CONFLICT_STALL >:    %8d",fsmcnt_WR_CONFLICT_STALL_q);
`ifdef ENABLE_MULTI_READ_PEND
        $display("        Multi-Read-Pend Counting");
        $display("        < Max Duplicates >:       %8d",MRP_duplicat_max_q);
`endif
        $display("*********************************************************************************************");
    end
end

if (LogLifeCycle) begin
    final begin
        automatic real runtime_invalid_cnt = 0;
        automatic real runtime_writ_pd_cnt = 0;
        automatic real runtime_read_pd_cnt = 0;
        automatic real runtime_usefull_cnt = 0;
        automatic real runtime_useless_cnt = 0;

        automatic real endtime_invalid_cnt = 0;
        automatic real endtime_writ_pd_cnt = 0;
        automatic real endtime_read_pd_cnt = 0;
        automatic real endtime_useless_cnt = 0;

        for (int i = 0; i < NumCacheEntry; i++) begin
            runtime_invalid_cnt += life_cycle_scoreboard_q[i].invalid_cnt;
            runtime_writ_pd_cnt += life_cycle_scoreboard_q[i].write_pending_cnt;
            runtime_read_pd_cnt += life_cycle_scoreboard_q[i].read_pending_cnt;
            runtime_usefull_cnt += life_cycle_scoreboard_q[i].usefull_cnt;
            runtime_useless_cnt += life_cycle_scoreboard_q[i].useless_cnt;

            case (life_cycle_scoreboard_q[i].current_state)
                LIFECYCLE_INVALID:          endtime_invalid_cnt += $time - life_cycle_scoreboard_q[i].last_record_time;
                LIFECYCLE_WRITE_PENDING:    endtime_writ_pd_cnt += $time - life_cycle_scoreboard_q[i].last_record_time;
                LIFECYCLE_READ_PENDING:     endtime_read_pd_cnt += $time - life_cycle_scoreboard_q[i].last_record_time;
                LIFECYCLE_USELESS:          endtime_useless_cnt += $time - life_cycle_scoreboard_q[i].last_record_time;
            endcase
        end

        $display(" ");
        $display(" ");
        $display("*********************************************************************");
        $display("***                 Insitu-Cache Life-Cycle Report                ***");
        $display("   ---------------------------------------------------------------   ");
        $display("   Number of Invalid       Life:  %16d + %16d",runtime_invalid_cnt,endtime_invalid_cnt);
        $display("   Number of Write Pending Life:  %16d + %16d",runtime_writ_pd_cnt,endtime_writ_pd_cnt);
        $display("   Number of Read  Pending Life:  %16d + %16d",runtime_read_pd_cnt,endtime_read_pd_cnt);
        $display("   Number of Useless       Life:  %16d + %16d",runtime_useless_cnt,endtime_useless_cnt);
        $display("   Number of Usefull       Life:  %16d ",runtime_usefull_cnt);
        $display("*********************************************************************");
    end
end

`endif

endmodule
