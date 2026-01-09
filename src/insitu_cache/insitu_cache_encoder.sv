// Copyright 2023 ETH Zurich and 
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 12.Feb.2024

// Decoder for cache banks reads accroding to cache task

`include "common_cells/registers.svh"
module insitu_cache_encoder
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
    /// Clock, positive edge triggered.
    input  logic                                            clk_i,
    /// Reset, active low.
    input  logic                                            rst_ni,
    /// Indicate Pend Lines
    output logic                                            has_pend_line_o,

    input  logic                                            enc_LRU_update,
    input  way_ptr_t                                        enc_way_i,
`ifdef ENABLE_MULTI_READ_PEND
    input  logic                                            enc_link_exec_i,
    input  way_ptr_t                                        enc_link_src_way_i,
    input  way_ptr_t                                        enc_link_dst_way_i,
`endif
    input  cache_status_t                                   enc_cache_status_i,
    input  logic                                            enc_cache_dirty_i,
    input  miss_meta_t                                      enc_cache_miss_meta_i,
    input  cache_mask_t                                     enc_cache_mask_i,
    input  cache_tag_t                                      enc_cache_tag_i,
    input  cache_data_t                                     enc_cache_data_i,
    input  logic                                            enc_mod_data_with_mask_i,
    input  cache_mask_t                                     enc_mod_mask_i,
    input  cache_data_t                                     enc_mod_write_data_i,

    /// Cache Banks Reads
    input  cache_status_t           [SetAssociativity-1:0]  bank_read_cache_status_i,
    input  logic                    [SetAssociativity-1:0]  bank_read_cache_dirty_i,
    input  miss_meta_t              [SetAssociativity-1:0]  bank_read_cache_miss_meta_i,
    input  cache_mask_t             [SetAssociativity-1:0]  bank_read_cache_mask_i,
    input  cache_tag_t              [SetAssociativity-1:0]  bank_read_cache_tag_i,
    input  cache_data_t             [SetAssociativity-1:0]  bank_read_cache_data_i,
    input  way_ptr_t                [SetAssociativity-1:0]  bank_read_cache_LRU_i,

    /// Encoder output
    output cache_status_t           [SetAssociativity-1:0]  bank_write_cache_status_o,
    output logic                    [SetAssociativity-1:0]  bank_write_cache_dirty_o,
    output miss_meta_t              [SetAssociativity-1:0]  bank_write_cache_miss_meta_o,
    output cache_mask_t             [SetAssociativity-1:0]  bank_write_cache_mask_o,
    output cache_tag_t              [SetAssociativity-1:0]  bank_write_cache_tag_o,
    output cache_data_t             [SetAssociativity-1:0]  bank_write_cache_data_o,
    output cache_mask_t             [SetAssociativity-1:0]  bank_write_data_mask_o,
    output way_ptr_t                [SetAssociativity-1:0]  bank_write_cache_LRU_o
);

    //Byte packed cache data
    typedef logic [ByteWidth-1:0]                           cache_byte_t;
    typedef logic [CacheLineWidth/ByteWidth-1:0][ByteWidth-1:0] cache_data_in_bytes_t;

    way_ptr_t                                               max_lru_credit;

    //Pending Line Counter
    logic [$clog2(NumCacheEntry)-1:0]                       pendline_cnt_q,pendline_cnt_d;
    `FFARN (pendline_cnt_q,pendline_cnt_d,                  '0, clk_i, rst_ni)

    /////////////////////////////////////
    //        Function Utility         //
    /////////////////////////////////////

    function automatic void LRU_array_update(
        input way_ptr_t                                 _way,
        input cache_status_t    [SetAssociativity-1:0]  _way_status_in,
        input cache_status_t                            _way_status_before,
        input cache_status_t                            _way_status_after,
        input way_ptr_t         [SetAssociativity-1:0]  _LRU_array_in,
        output way_ptr_t        [SetAssociativity-1:0]  _LRU_array_out
        );
        automatic way_ptr_t LRU_original;
        LRU_original = _LRU_array_in[_way];
        _LRU_array_out = _LRU_array_in;

        if ((_way_status_before == READ_PEND || _way_status_before == WRITE_PEND) &&
            (_way_status_after  == READ_PEND || _way_status_after  == WRITE_PEND)) begin
            // Pend -> Pend
            // nothing to change
        end else
        if ((_way_status_before == VALID     || _way_status_before == INVALID) &&
            (_way_status_after  == READ_PEND || _way_status_after  == WRITE_PEND)) begin

            for (int way = 0; way < SetAssociativity; way ++) begin
                if (way == _way) begin
                    _LRU_array_out[way] = SetAssociativity - 1;
                end else if (_LRU_array_in[way] > LRU_original && (_way_status_in[way] == VALID || _way_status_in[way] == INVALID)) begin
                    _LRU_array_out[way] = _LRU_array_in[way] - 1;
                end
            end
            pendline_cnt_d = pendline_cnt_q + 1'b1;

        end else
        if ((_way_status_before == READ_PEND || _way_status_before == WRITE_PEND) &&
            (_way_status_after  == VALID     || _way_status_after  == INVALID)) begin
            _LRU_array_out[_way] = (_way_status_after  == INVALID)? '0 : max_lru_credit;
            pendline_cnt_d = pendline_cnt_q - 1'b1;
        end else begin

            for (int way = 0; way < SetAssociativity; way ++) begin
                if (way == _way) begin
                    _LRU_array_out[way] = (max_lru_credit=='0)? SetAssociativity - 1: max_lru_credit - 1'b1;
                end else if (_LRU_array_in[way] > LRU_original && (_way_status_in[way] == VALID || _way_status_in[way] == INVALID)) begin
                    _LRU_array_out[way] = _LRU_array_in[way] - 1;
                end
            end

        end

    endfunction

    always_comb begin : proc_max_lru
        max_lru_credit = '0;
        for (int way = 0; way < SetAssociativity; way ++) begin
            if (bank_read_cache_status_i[way] == VALID || bank_read_cache_status_i[way] == INVALID ) begin
                max_lru_credit =  max_lru_credit + 1'b1;
            end
        end
    end


    always_comb begin : proc_encode
        cache_data_t write_data_final;

        bank_write_cache_status_o = bank_read_cache_status_i;
        bank_write_cache_dirty_o = bank_read_cache_dirty_i;
        bank_write_cache_miss_meta_o = bank_read_cache_miss_meta_i;
        bank_write_cache_mask_o = bank_read_cache_mask_i;
        bank_write_cache_tag_o = bank_read_cache_tag_i;
        bank_write_cache_data_o = bank_read_cache_data_i;
        bank_write_cache_LRU_o = bank_read_cache_LRU_i;
        bank_write_data_mask_o = '{default: '1};

        pendline_cnt_d = pendline_cnt_q;
        has_pend_line_o = (pendline_cnt_q != '0);

        //deal with masked modification
        write_data_final = enc_cache_data_i;
        if (enc_mod_data_with_mask_i) begin
            automatic cache_data_in_bytes_t cache_data_in_bytes;
            automatic cache_data_in_bytes_t write_data_in_bytes;
            automatic cache_mask_t write_strob;
            cache_data_in_bytes = enc_cache_data_i;
            write_data_in_bytes = enc_mod_write_data_i;
            write_strob = enc_mod_mask_i;
            for (int bt = 0; bt < CacheLineWidth/ByteWidth; bt++ ) begin
                if (write_strob[bt]) begin
                    cache_data_in_bytes[bt] = write_data_in_bytes[bt];
                end
            end
            write_data_final = cache_data_in_bytes;
            bank_write_data_mask_o[enc_way_i] = enc_mod_mask_i;
        end

        //modify selescted way
        bank_write_cache_status_o[enc_way_i] = enc_cache_status_i; 
        bank_write_cache_dirty_o[enc_way_i] = enc_cache_dirty_i; 
        bank_write_cache_miss_meta_o[enc_way_i] = enc_cache_miss_meta_i;
        bank_write_cache_mask_o[enc_way_i] = enc_cache_mask_i; 
        bank_write_cache_tag_o[enc_way_i] = enc_cache_tag_i; 
        bank_write_cache_data_o[enc_way_i] = write_data_final;

        //deal with LRU
        if (enc_LRU_update) begin
            LRU_array_update(
                enc_way_i,
                bank_read_cache_status_i,
                bank_read_cache_status_i[enc_way_i],
                bank_write_cache_status_o[enc_way_i],
                bank_read_cache_LRU_i,
                bank_write_cache_LRU_o);
        end

`ifdef ENABLE_MULTI_READ_PEND
        if (enc_link_exec_i) begin
            bank_write_cache_miss_meta_o[enc_link_src_way_i].link_enable = 1;
            bank_write_cache_miss_meta_o[enc_link_src_way_i].link_ptr = enc_link_dst_way_i;
        end
`endif
    end 

endmodule
