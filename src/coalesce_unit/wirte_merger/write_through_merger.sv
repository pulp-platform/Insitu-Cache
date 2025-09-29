// Copyright 2023 ETH Zurich and 
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 29.Feb.2024

`include "common_cells/registers.svh"
module write_through_merger #(
    /// Address width of both upstream narrow request and downstream wide request
    parameter int unsigned ReqAddrWidth             = 32,
    /// Information payload needed for each narrow data
    parameter type         info_t                   = logic,
    /// Information payload needed for each narrow data
    parameter type         downstream_info_t        = logic,
    /// Word width of cache line (512b default)
    parameter int unsigned CacheLineWidth           = 512,
    /// Width of word (granularity of non-blocking write)
    parameter int unsigned WordWidth                = 64,
    /// Watchdog Counter
    parameter int unsigned WatchDogMax              = 4,
    /// Word width of narrow data to upstream
    parameter int unsigned UpstreamWidth            = CacheLineWidth,
    /// Word width of wide data from downsteam
    parameter int unsigned DownstreamWidth          = CacheLineWidth,
    // Dependent parameter, do not override. Depth of cache bank.
    localparam int unsigned NumWord                 = CacheLineWidth/WordWidth,
    // Dependent parameter, do not override. Address type.
    localparam type addr_t                          = logic [ReqAddrWidth-1:0],
    // Dependent parameter, do not override. Narrow word type.
    localparam type upstream_data_t                 = logic [UpstreamWidth-1:0],
    // Dependent parameter, do not override. Wide word type.
    localparam type downstream_data_t               = logic [DownstreamWidth-1:0],
    // Dependent parameter, do not override. Byte mask type.
    localparam type mask_t                          = logic [DownstreamWidth/WordWidth-1:0],
    // Dependent parameter, do not override. byte offset type.
    localparam type byte_of_t                       = logic [$clog2(DownstreamWidth/8)-1:0],
    // Dependent parameter, do not override. tag type.
    localparam type tag_t                           = logic [ReqAddrWidth-$clog2(DownstreamWidth/8)-1:0]
    )(
    /// Clock, positive edge triggered.
    input  logic                                    clk_i,
    /// Reset, active low.
    input  logic                                    rst_ni,

    /// Upstream request
    input  logic                                    upstream_req_valid_i,
    output logic                                    upstream_req_ready_o,
    input  addr_t                                   upstream_req_addr_i,
    input  upstream_data_t                          upstream_req_wdata_i,
    input  mask_t                                   upstream_req_wmask_i,

    /// Read Moniter
    input  logic                                    mon_read_handshaked_i,
    input  addr_t                                   mon_read_addr_i,

    /// Downstream request
    output logic                                    downstream_req_valid_o,
    input  logic                                    downstream_req_ready_i,
    output addr_t                                   downstream_req_addr_o,
    output downstream_info_t                        downstream_req_info_o,
    output logic                                    downstream_req_write_o,
    output downstream_data_t                        downstream_req_wdata_o,
    output mask_t                                   downstream_req_wmask_o
 
);

    //////////////////////////////////////
    //        Types Definition          //
    //////////////////////////////////////

    typedef logic [$clog2(WatchDogMax):0]           watch_dog_cnt_t;

    typedef logic [WordWidth-1:0]                   cache_word_t;
    typedef cache_word_t [NumWord-1:0]              cache_data_in_words_t;

    typedef struct packed {
        tag_t                                       tag;
        downstream_data_t                           wdata;
        mask_t                                      wmask;
    } coal_meta_t;

    /*******************/
    /*  Coalescer FSM  */
    /*******************/
    typedef enum logic [1:0] {
        IDLE = '0,
        WRITE_COAL,
        FLUSH
    } coal_fsm_status_t;

    //////////////////////////////////////
    //        Signal Definition         //
    //////////////////////////////////////

    coal_meta_t                                     coal_meta_q, coal_meta_d;
    coal_fsm_status_t                               state_q, state_d;
    watch_dog_cnt_t                                 dog_cnt_q, dog_cnt_d;
    `FFARN (coal_meta_q, coal_meta_d,               '0, clk_i, rst_ni)
    `FFARN (state_q, state_d,                       IDLE, clk_i, rst_ni)
    `FFARN (dog_cnt_q, dog_cnt_d,                   '0, clk_i, rst_ni)

    //////////////////////////////////
    //        Coalescer FSM         //
    //////////////////////////////////

    always_comb begin : coal_fsm
        /*****************/
        /* Defualt Value */
        /*****************/
        upstream_req_ready_o = '0;
        downstream_req_valid_o = '0;
        downstream_req_addr_o = '0;
        downstream_req_info_o = '0;
        downstream_req_write_o = '0;
        downstream_req_wdata_o = '0;
        downstream_req_wmask_o = '0;
        coal_meta_d = coal_meta_q;
        state_d = state_q;
        dog_cnt_d = dog_cnt_q;

        /************/
        /* Main FSM */
        /************/
        case (state_q)
            IDLE: begin
                //1.Check receive new request
                if (upstream_req_valid_i) begin
                    //1.1 goto WRITE_COAL state
                    state_d = WRITE_COAL;

                    //1.2 charge watchdog
                    dog_cnt_d = WatchDogMax;

                    //1.3 prepare meta
                    coal_meta_d.tag = upstream_req_addr_i >> $clog2(DownstreamWidth/8);
                    coal_meta_d.wdata = upstream_req_wdata_i;
                    coal_meta_d.wmask = upstream_req_wmask_i;

                    //1.4 accept req
                    upstream_req_ready_o = 1'b1;
                end
            end


            WRITE_COAL: begin
                automatic tag_t     _read_tag;
                automatic tag_t     _tag;
                automatic byte_of_t _bt_;

                automatic logic     dog_push;
                automatic logic     new_tag;
                automatic logic     coal_hit;
                automatic logic     flush_push;

                automatic logic     down_stall = '0;

                {_tag, _bt_}        = upstream_req_addr_i;
                {_read_tag, _bt_}   = mon_read_addr_i;
                dog_push            = (dog_cnt_q == '0)     & ~upstream_req_valid_i;
                flush_push          = mon_read_handshaked_i & (coal_meta_q.tag == _read_tag);
                new_tag             = upstream_req_valid_i  & (coal_meta_q.tag != _tag);
                coal_hit            = upstream_req_valid_i  & (coal_meta_q.tag == _tag);

                //3.1 conut down watchdog when no request come
                if (~upstream_req_valid_i && dog_cnt_q != '0) begin
                    dog_cnt_d = dog_cnt_q - 1'b1;
                end

                if (upstream_req_valid_i) begin
                    dog_cnt_d = WatchDogMax;
                end

                //3.2 send request downstream
                if (dog_push | new_tag | flush_push) begin

                    //3.2.1 prepare payload
                    _bt_ = '0;
                    downstream_req_addr_o = {coal_meta_q.tag, _bt_};
                    downstream_req_info_o = '0;
                    downstream_req_write_o = 1'b1;
                    downstream_req_wdata_o = coal_meta_q.wdata;
                    downstream_req_wmask_o = coal_meta_q.wmask;

                    //3.2.1 send to downstream
                    downstream_req_valid_o = 1'b1;
                    if (~downstream_req_ready_i) begin
                        down_stall = 1'b1;
                    end 
                end

                //3.3 update coal meta when successfully issue downstream:
                if (~down_stall) begin
                    if (new_tag) begin

                        /* update with miss */
                        state_d = WRITE_COAL;
                        dog_cnt_d = WatchDogMax;

                        //prepare meta
                        coal_meta_d.tag = upstream_req_addr_i >> $clog2(DownstreamWidth/8);
                        coal_meta_d.wdata = upstream_req_wdata_i;
                        coal_meta_d.wmask = upstream_req_wmask_i;

                        //accept req
                        upstream_req_ready_o = 1'b1;

                    end else if (coal_hit) begin
                        automatic cache_data_in_words_t write_data_in_words = upstream_req_wdata_i;
                        automatic cache_data_in_words_t cache_data_in_words = coal_meta_q.wdata;

                        /* merge a hit */
                        dog_cnt_d = WatchDogMax;

                        for (int wd = 0; wd < CacheLineWidth/WordWidth; wd++ ) begin
                            if (upstream_req_wmask_i[wd]) begin
                                cache_data_in_words[wd] = write_data_in_words[wd];
                            end
                        end

                        coal_meta_d.wdata = cache_data_in_words;
                        coal_meta_d.wmask = coal_meta_q.wmask | upstream_req_wmask_i;

                        //accept req
                        upstream_req_ready_o = 1'b1;

                    end else if (dog_push | flush_push) begin
                        
                        /* return to IDLE */
                        state_d = IDLE;
                        dog_cnt_d = '0;
                        coal_meta_d = '0;
                        
                    end
                end else if(flush_push) begin
                    state_d = FLUSH;
                end

            end


            FLUSH: begin
                automatic byte_of_t _bt_ = '0;
                downstream_req_addr_o = {coal_meta_q.tag, _bt_};
                downstream_req_info_o = '0;
                downstream_req_write_o = 1'b1;
                downstream_req_wdata_o = coal_meta_q.wdata;
                downstream_req_wmask_o = coal_meta_q.wmask;

                //3.2.1 send to downstream
                downstream_req_valid_o = 1'b1;
                if (downstream_req_ready_i) begin
                    if (upstream_req_valid_i) begin
                        //goto WRITE_COAL state
                        state_d = WRITE_COAL;

                        //charge watchdog
                        dog_cnt_d = WatchDogMax;

                        //prepare meta
                        coal_meta_d.tag = upstream_req_addr_i >> $clog2(DownstreamWidth/8);
                        coal_meta_d.wdata = upstream_req_wdata_i;
                        coal_meta_d.wmask = upstream_req_wmask_i;

                        //accept req
                        upstream_req_ready_o = 1'b1;
                    end else begin
                        /* return to IDLE */
                        state_d = IDLE;
                        dog_cnt_d = '0;
                        coal_meta_d = '0; 
                    end
                end 
            end
            
            
            default: state_d = IDLE;
        endcase
    end

endmodule