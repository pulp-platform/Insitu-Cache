// Copyright 2023 ETH Zurich and 
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 12.Feb.2024

// Decoder for cache banks reads accroding to cache task

`include "common_cells/registers.svh"
module insitu_cache_decoder
  import insitu_cache_pkg::*;
  #(
    /// Address width of both upstream narrow request and downstream wide request
    parameter int unsigned ReqAddrWidth                     = 32,
    /// Word width of cache line (512b default)
    parameter int unsigned CacheLineWidth                   = 512,
    /// Information payload
    parameter type         task_payload_t                   = logic[7:0],
    /// Number of Cache entries
    parameter int unsigned NumCacheEntry                    = 512,
    /// Number of Associatity
    parameter int unsigned SetAssociativity                 = 16,
    /// Use deterministic hash-based way selection.
    parameter bit          UseHashWaySelect                 = 1'b0,
    /// Width of word (granularity of non-blocking write)
    parameter int unsigned WordWidth                        = 64,
    /// Width of byte (granularity of byte mask)
    parameter int unsigned ByteWidth                        = 8,
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
    // Dependent parameter, do not override. Downstream request payload.
    localparam type downstream_info_t                       = struct packed {logic for_write_pend; cache_bank_depth_ptr_t depth; way_ptr_t way;},
    // Dependent parameter, do not override. Downstream request payload.
    localparam type miss_meta_t                             = struct packed {logic is_full; logic is_prime; logic link_enable; way_ptr_t link_ptr;}
    )(

    /// Cache Banks Reads
    input  cache_status_t           [SetAssociativity-1:0]  bank_read_cache_status_i,
    input  logic                    [SetAssociativity-1:0]  bank_read_cache_dirty_i,
    input  miss_meta_t              [SetAssociativity-1:0]  bank_read_cache_miss_meta_i,
    input  cache_mask_t             [SetAssociativity-1:0]  bank_read_cache_mask_i,
    input  cache_tag_t              [SetAssociativity-1:0]  bank_read_cache_tag_i,
    input  cache_data_t             [SetAssociativity-1:0]  bank_read_cache_data_i,
    input  way_ptr_t                [SetAssociativity-1:0]  bank_read_cache_LRU_i,

    /// Cache Task
    input  task_payload_t                                   cache_task_i,

    /// Decoder output
    output way_ptr_t                                        dec_way_o,
    output logic                                            dec_is_write_req_o,
    output logic                                            dec_is_hit_o,
    output logic                                            dec_is_hit_pend_o,
`ifdef ENABLE_MULTI_READ_PEND
    output logic                                            dec_is_hit_pend_new_entry_o,
    output way_ptr_t                                        dec_read_hit_pend_prime_way_o,
    output way_ptr_t                                        dec_read_hit_pend_linkable_way_o,
`endif
    output logic                                            dec_is_hit_conflit_o,
    output logic                                            dec_is_all_pend_o,
    output cache_status_t                                   dec_cache_status_o,
    output logic                                            dec_cache_dirty_o,
    output miss_meta_t                                      dec_cache_miss_meta_o,
    output cache_mask_t                                     dec_cache_mask_o,
    output cache_tag_t                                      dec_cache_tag_o,
    output cache_data_t                                     dec_cache_data_o
);

// =====================================================================
//  Decoded address fields (combinational, debug-visible).
//
//  These are intermediate signals used by the decode FSM below.  They
//  USED to be declared `automatic` inside the always_comb block, which
//  meant the simulator allocated them on the call stack and they could
//  not be added to the waveform.  They are now module-scope `logic`
//  signals driven by continuous `assign` statements so they are
//  visible in QuestaSim and any other waveform viewer.
// =====================================================================
cache_tag_t              _tag;       // tag bits of the request addr
cache_bank_depth_ptr_t   _depth;     // depth bits of the request addr
byte_offset_t            _ofst;      // byte-offset bits of the request addr
way_ptr_t                _hash_way;  // hash-derived way (UseHashWaySelect mode)
// Flattened hash-way decode helpers (see proc_hash_way_req): the shared wide
// tag compare + parallel status decode, hoisted out of the old priority
// if/else-if cascade to shorten the meta-SRAM read -> dec_is_hit_* path.
logic                    _tag_hit;
logic                    _is_rpend_way;  // == READ_PEND, for the MULTI_READ_PEND append arm
logic                    _status_s1;     // status[1]: 1 => pending (READ_PEND/WRITE_PEND)
logic                    _status_s0;     // status[0]

assign {_tag, _depth, _ofst} = cache_task_i.task_pay.request.addr;

assign _hash_way = (SetAssociativity > 1)
    ? way_ptr_t'(
        cache_task_i.task_pay.request.addr[$clog2(CacheLineWidth/8) + $clog2(CacheBankDepth)
                                           +: $clog2(SetAssociativity)] ^
        cache_task_i.task_pay.request.addr[$clog2(CacheLineWidth/8)
                                           +: $clog2(SetAssociativity)])
    : '0;

assign dec_cache_status_o   = bank_read_cache_status_i[dec_way_o];
assign dec_cache_dirty_o    = bank_read_cache_dirty_i[dec_way_o];
assign dec_cache_miss_meta_o= bank_read_cache_miss_meta_i[dec_way_o];
assign dec_cache_mask_o     = bank_read_cache_mask_i[dec_way_o];
assign dec_cache_tag_o      = bank_read_cache_tag_i[dec_way_o];
assign dec_cache_data_o     = bank_read_cache_data_i[dec_way_o];

// Hash-way decode helpers: wide tag compare computed once; status (2-bit enum)
// decoded as raw bits so the late SRAM bit s0 reaches each dec_is_* output via a
// single XOR with the early dec_is_write_req_o (see proc_hash_way_req).
assign _tag_hit    = (bank_read_cache_tag_i[_hash_way] == _tag);
assign _status_s1  = bank_read_cache_status_i[_hash_way][1];
assign _status_s0  = bank_read_cache_status_i[_hash_way][0];
assign _is_rpend_way = _status_s1 & ~_status_s0;  // READ_PEND = 2'b10
// T1.5 relies on the cache_status_t bit-encoding (INVALID=00,VALID=01,READ_PEND=10,
// WRITE_PEND=11, insitu_cache_pkg.sv); guard against a future reorder.
initial assert (int'(INVALID)==0 && int'(VALID)==1 && int'(READ_PEND)==2 && int'(WRITE_PEND)==3)
    else $fatal(1, "insitu_cache_decoder: cache_status_t encoding changed; fix the status bit-decode");

always_comb begin : proc_bank_decode

    dec_way_o = '0;
    dec_is_write_req_o = '0;
    dec_is_hit_o = '0;
    dec_is_hit_pend_o = '0;
    dec_is_hit_conflit_o = '0;
    dec_is_all_pend_o = 1'b1;
`ifdef ENABLE_MULTI_READ_PEND
    dec_is_hit_pend_new_entry_o = 1'b1;
    dec_read_hit_pend_prime_way_o = '0;
    dec_read_hit_pend_linkable_way_o = '0;
`endif

    /*************************/
    /* Cache Request Process */
    /*************************/
    //1. Process when prereader got request task
    //   (_tag, _depth, _ofst, _hash_way are computed by continuous
    //    assigns at module scope above so they show up in the waveform.)
    if (cache_task_i.valid & ~cache_task_i.is_refill) begin : prec_req_process
        dec_is_write_req_o = cache_task_i.task_pay.request.write;


        if (UseHashWaySelect && (SetAssociativity > 1)) begin : proc_hash_way_req
            dec_way_o = _hash_way;

            // Flattened decode (was a priority if/else-if cascade on status):
            // status is a 1-hot enum so the arms are mutually exclusive --
            // form each output as a flat SOP over the precomputed _tag_hit /
            // _is_*_way helpers.  Behaviourally identical; removes the cascade
            // depth from the critical dec_is_hit_conflit_o output.
            // T1.5: status decoded as bit ops (s1=is-pending, s0); the late s0
            // reaches each output through one XOR with the early write bit.
            dec_is_hit_o         = _tag_hit & ~_status_s1 & _status_s0;
            dec_is_hit_pend_o    = _tag_hit &  _status_s1 & ~(_status_s0 ^ dec_is_write_req_o);
            dec_is_hit_conflit_o = _tag_hit &  _status_s1 &  (_status_s0 ^ dec_is_write_req_o);
            dec_is_all_pend_o    =  _status_s1;

`ifdef ENABLE_MULTI_READ_PEND
            // hit-under-miss append bookkeeping: only a READ to a READ_PEND line
            // (the old READ_PEND / not-write arm).
            if (_tag_hit & _is_rpend_way & ~dec_is_write_req_o) begin
                if (bank_read_cache_miss_meta_i[_hash_way].is_full == 0) begin
                    dec_is_hit_pend_new_entry_o = '0;
                end
                if (bank_read_cache_miss_meta_i[_hash_way].is_prime) begin
                    dec_read_hit_pend_prime_way_o = _hash_way;
                end
                if (bank_read_cache_miss_meta_i[_hash_way].link_enable == 0) begin
                    dec_read_hit_pend_linkable_way_o = _hash_way;
                end
            end
`endif
        end else begin : proc_full_assoc_req
            //2. Check request type
            for (int way = 0; way < SetAssociativity; way ++) begin : proc2_check_req_type

                //2.1 Check hit on valid line
                if (bank_read_cache_status_i[way] == VALID && (bank_read_cache_tag_i[way] == _tag)) begin
                    dec_is_hit_o = 1;
                    dec_way_o = way;
                end
                //2.2 Check hit on same type pend line/ on the opposite type pend line
                if (bank_read_cache_status_i[way] == READ_PEND && (bank_read_cache_tag_i[way] == _tag)) begin
                    if (dec_is_write_req_o) begin
                        dec_is_hit_conflit_o = 1;
                        dec_way_o = way;
                    end else begin
                        dec_is_hit_pend_o = 1;
`ifdef ENABLE_MULTI_READ_PEND
                        if (bank_read_cache_miss_meta_i[way].is_full == 0) begin
                            dec_is_hit_pend_new_entry_o = '0;
                            dec_way_o = way;
                        end
                        if (bank_read_cache_miss_meta_i[way].is_prime) begin
                            dec_read_hit_pend_prime_way_o = way;
                        end
                        if (bank_read_cache_miss_meta_i[way].link_enable == 0) begin
                            dec_read_hit_pend_linkable_way_o = way;
                        end
`else
                        dec_way_o = way;
`endif
                    end
                end
                if (bank_read_cache_status_i[way] == WRITE_PEND && (bank_read_cache_tag_i[way] == _tag)) begin
                    if (dec_is_write_req_o) begin
                        dec_is_hit_pend_o = 1;
                        dec_way_o = way;
                    end else begin
                        dec_is_hit_conflit_o = 1;
                        dec_way_o = way;
                    end
                end
                //2.3 Check if all lines are in pending status
                if (bank_read_cache_status_i[way] == VALID || bank_read_cache_status_i[way] == INVALID) begin
                    dec_is_all_pend_o = 0;
                end

            end : proc2_check_req_type
        end



        //3.Determine way if miss (LRU mode ONLY).
        //
        // BUG FIX (cache-coverage-min phase 06): when UseHashWaySelect=1,
        // dec_way_o was already set to _hash_way at line 132 above and must
        // NOT be overridden here.  Without this guard the LRU-victim loop
        // ran on every miss in hash mode too, picking the first way whose
        // unused-LRU bits were 0 (= typically way 0) and silently
        // overwriting the correct hash way.  That made dec_cache_status /
        // dec_cache_dirty / dec_cache_tag read from the wrong way, so the
        // miss FSM in insitu_cache_core (line 1837) never observed the
        // displaced VALID+dirty victim and skipped the writeback (the
        // bank_write commit still went to the correct hash way via
        // hash_way_fsm/req_way_tmp, silently dropping the dirty data).
`ifdef ENABLE_MULTI_READ_PEND
        if (!UseHashWaySelect &&
            ~ dec_is_hit_o & ~(dec_is_hit_pend_o & (dec_is_write_req_o | ~dec_is_hit_pend_new_entry_o)) &
            ~dec_is_hit_conflit_o & ~dec_is_all_pend_o) begin : proc3_find_miss_way
`else
        if (!UseHashWaySelect &&
            ~ dec_is_hit_o & ~dec_is_hit_pend_o & ~dec_is_hit_conflit_o & ~dec_is_all_pend_o) begin : proc3_find_miss_way
`endif

`ifdef USE_ORIGINAL_LRU
            for (int way = 0; way < SetAssociativity; way ++) begin
                if (bank_read_cache_status_i[way] == VALID || bank_read_cache_status_i[way] == INVALID) begin
                    if (bank_read_cache_status_i[dec_way_o] == VALID || bank_read_cache_status_i[dec_way_o] == INVALID) begin
                        dec_way_o = bank_read_cache_LRU_i[way] < bank_read_cache_LRU_i[dec_way_o]? way: dec_way_o;
                    end else begin
                        dec_way_o = way;
                    end
                end
            end
`else
            for (int way = 0; way < SetAssociativity; way ++) begin
                if (bank_read_cache_LRU_i[way] == '0) begin
                    dec_way_o = way;
                    break;
                end
            end
`endif
        end : proc3_find_miss_way

    end : prec_req_process




    /************************/
    /* Cache Refill Process */
    /************************/

    if (cache_task_i.valid & cache_task_i.is_refill) begin : proc_refill
        dec_way_o = cache_task_i.task_pay.refill.info.way;
    end : proc_refill

end : proc_bank_decode

    

endmodule
