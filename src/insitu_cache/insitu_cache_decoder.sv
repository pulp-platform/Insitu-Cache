// Copyright 2025 ETH Zurich and 
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
    /// Width of word (granularity of non-blocking write)
    parameter int unsigned WordWidth                        = 64,
    // Dependent parameter, do not override. Depth of cache bank.
    localparam int unsigned CacheBankDepth                  = NumCacheEntry/SetAssociativity,
    // Dependent parameter, do not override. way ptr type.
    localparam type way_ptr_t                               = logic [$clog2(SetAssociativity)-1:0],
    // Dependent parameter, do not override. Address type.
    localparam type addr_t                                  = logic [ReqAddrWidth-1:0],
    // Dependent parameter, do not override. Narrow word type.
    localparam type cache_data_t                            = logic [CacheLineWidth-1:0],
    // Dependent parameter, do not override. Byte mask type.
    localparam type cache_mask_t                            = logic [CacheLineWidth/WordWidth-1:0],
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

assign dec_cache_status_o   = bank_read_cache_status_i[dec_way_o];
assign dec_cache_dirty_o    = bank_read_cache_dirty_i[dec_way_o];
assign dec_cache_miss_meta_o= bank_read_cache_miss_meta_i[dec_way_o];
assign dec_cache_mask_o     = bank_read_cache_mask_i[dec_way_o];
assign dec_cache_tag_o      = bank_read_cache_tag_i[dec_way_o];
assign dec_cache_data_o     = bank_read_cache_data_i[dec_way_o];

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
    if (cache_task_i.valid & ~cache_task_i.is_refill) begin : prec_req_process
        automatic cache_tag_t _tag;
        automatic cache_bank_depth_ptr_t _depth;
        automatic byte_offset_t _ofst;


        {_tag,_depth,_ofst} = cache_task_i.task_pay.request.addr;
        dec_is_write_req_o = cache_task_i.task_pay.request.write;


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



        //3.Determine way if miss
`ifdef ENABLE_MULTI_READ_PEND
        if (~ dec_is_hit_o & ~(dec_is_hit_pend_o & (dec_is_write_req_o | ~dec_is_hit_pend_new_entry_o)) & ~dec_is_hit_conflit_o & ~dec_is_all_pend_o) begin : proc3_find_miss_way
`else
        if (~ dec_is_hit_o & ~dec_is_hit_pend_o & ~dec_is_hit_conflit_o & ~dec_is_all_pend_o) begin : proc3_find_miss_way
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
