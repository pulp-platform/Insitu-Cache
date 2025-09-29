// Copyright 2023 ETH Zurich and 
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 29.Feb.2024

`include "common_cells/registers.svh"
module seq_coalescer_req_merger #(
    /// Address width of both upstream narrow request and downstream wide request
    parameter int unsigned ReqAddrWidth             = 32,
    /// Information payload needed for each narrow data
    parameter type         info_t                   = logic,
    /// Data width of upstream channel
    parameter int unsigned UpstreamDataWidth        = 32,
    /// Data width of downstream channel
    parameter int unsigned DownstreamDataWidth      = 512,
    /// Watchdog Counter
    parameter int unsigned WatchDogMax              = 4,
    // Dependent parameter, do not override. Depth of cache bank.
    localparam int unsigned NumWord                 = DownstreamDataWidth/UpstreamDataWidth,
    // Dependent parameter, do not override. Address type.
    localparam type addr_t                          = logic [ReqAddrWidth-1:0],
    // Dependent parameter, do not override. Narrow word type.
    localparam type upstream_data_t                 = logic [UpstreamDataWidth-1:0],
    // Dependent parameter, do not override. Wide word type.
    localparam type downstream_data_t               = logic [DownstreamDataWidth-1:0],
    // Dependent parameter, do not override. Word mask type.
    localparam type mask_t                          = logic [DownstreamDataWidth/UpstreamDataWidth-1:0],
    // Dependent parameter, do not override. tag type.
    localparam type tag_t                           = logic [ReqAddrWidth-$clog2(DownstreamDataWidth/8)-1:0],
    // Dependent parameter, do not override. byte offset type.
    localparam type offset_t                        = logic [$clog2(DownstreamDataWidth/UpstreamDataWidth)-1:0],
    // Dependent parameter, do not override. byte offset type.
    localparam type byte_of_t                       = logic [$clog2(UpstreamDataWidth/8)-1:0],
    // Dependent parameter, do not override. coalescer subentry.
    localparam type sub_t                           = struct packed {info_t info; offset_t ofst;},
    // Dependent parameter, do not override. Downstream request payload.
    localparam type downstream_info_t               = struct packed {offset_t num_sub; sub_t [NumWord-1:0] subs;}
    )(
    /// Clock, positive edge triggered.
    input  logic                                    clk_i,
    /// Reset, active low.
    input  logic                                    rst_ni,

    /// Upstream request
    input  logic                                    upstream_req_valid_i,
    output logic                                    upstream_req_ready_o,
    input  addr_t                                   upstream_req_addr_i,
    input  info_t                                   upstream_req_info_i,
    input  logic                                    upstream_req_write_i,
    input  upstream_data_t                          upstream_req_wdata_i,

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

    typedef upstream_data_t [NumWord-1:0]           wdata_in_words_t;

    typedef struct packed {
        tag_t                                       tag;
        downstream_info_t                           down_info;
        wdata_in_words_t                            wdata;
        mask_t                                      wmask;
    } coal_meta_t;

    /*******************/
    /*  Coalescer FSM  */
    /*******************/
    typedef enum logic[1:0] {
        IDLE = '0,
        READ_COAL,
        WRITE_COAL
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
                    if (upstream_req_write_i) begin
                        /*1.1 is write*/
                        automatic offset_t _ofst;

                        //1.1.1 goto WRITE_COAL state
                        state_d = WRITE_COAL;

                        //1.1.2 charge watchdog
                        dog_cnt_d = WatchDogMax;

                        //1.1.3 prepare meta
                        coal_meta_d.tag = upstream_req_addr_i >> $clog2(DownstreamDataWidth/8);
                        _ofst = upstream_req_addr_i[$clog2(DownstreamDataWidth/8)-1:$clog2(UpstreamDataWidth/8)];
                        coal_meta_d.down_info = '0;
                        coal_meta_d.down_info.subs[0].info = upstream_req_info_i;
                        coal_meta_d.wdata = '0;
                        coal_meta_d.wdata[_ofst] = upstream_req_wdata_i;
                        coal_meta_d.wmask = '0;
                        coal_meta_d.wmask[_ofst] = 1'b1;

                        //1.1.4 accept req
                        upstream_req_ready_o = 1'b1;

                    end else begin
                        /*1.2 is read*/
                        automatic offset_t _ofst;

                        //1.2.1 goto READ_COAL state
                        state_d = READ_COAL;

                        //1.2.2 charge watchdog
                        dog_cnt_d = WatchDogMax;

                        //1.2.3 prepare meta
                        coal_meta_d.tag = upstream_req_addr_i >> $clog2(DownstreamDataWidth/8);
                        _ofst = upstream_req_addr_i[$clog2(DownstreamDataWidth/8)-1:$clog2(UpstreamDataWidth/8)];
                        coal_meta_d.down_info = '0;
                        coal_meta_d.down_info.subs[0].info = upstream_req_info_i;
                        coal_meta_d.down_info.subs[0].ofst = _ofst;
                        coal_meta_d.wdata = '0;
                        coal_meta_d.wmask = '0;

                        //1.2.4 accept req
                        upstream_req_ready_o = 1'b1;

                    end
                end
            end






            READ_COAL: begin
                automatic tag_t     _tag;
                automatic offset_t  _ofst;
                automatic byte_of_t _bt_;
                automatic logic     dog_push;
                automatic logic     new_tag;
                automatic logic     coal_hit;
                automatic logic     full_refresh;
                automatic logic     down_stall = '0;

                {_tag, _ofst, _bt_} = upstream_req_addr_i;
                dog_push = (dog_cnt_q == '0) & ~upstream_req_valid_i;
                new_tag = upstream_req_valid_i & (upstream_req_write_i | (coal_meta_q.tag != _tag));
                coal_hit = upstream_req_valid_i & ~upstream_req_write_i & (coal_meta_q.tag == _tag);
                full_refresh = upstream_req_valid_i & coal_hit & (coal_meta_q.down_info.num_sub == NumWord - 1);

                //2.1 conut down watchdog when no request come
                if (~upstream_req_valid_i && dog_cnt_q != '0) begin
                    dog_cnt_d = dog_cnt_q - 1'b1;
                end

                if (upstream_req_valid_i) begin
                    dog_cnt_d = WatchDogMax;
                end

                //2.2 send request downstream
                if (dog_push | new_tag | full_refresh) begin

                    //2.2.1 prepare payload
                    _ofst = '0;
                    _bt_ = '0;
                    downstream_req_addr_o = {coal_meta_q.tag, _ofst, _bt_};
                    downstream_req_info_o = coal_meta_q.down_info;
                    downstream_req_write_o = '0;
                    downstream_req_wdata_o = '0;
                    downstream_req_wmask_o = '0;

                    //2.2.1 send to downstream
                    downstream_req_valid_o = 1'b1;
                    if (~downstream_req_ready_i) begin
                        down_stall = 1'b1;
                    end 
                end

                //2.3 update coal meta when successfully issue downstream:
                if (~down_stall) begin
                    if (new_tag) begin

                        /* update with miss */
                        if (upstream_req_write_i) begin

                            state_d = WRITE_COAL;
                            dog_cnt_d = WatchDogMax;

                            //prepare meta
                            coal_meta_d.tag = upstream_req_addr_i >> $clog2(DownstreamDataWidth/8);
                            _ofst = upstream_req_addr_i[$clog2(DownstreamDataWidth/8)-1:$clog2(UpstreamDataWidth/8)];
                            coal_meta_d.down_info = '0;
                            coal_meta_d.down_info.subs[0].info = upstream_req_info_i;
                            coal_meta_d.wdata = '0;
                            coal_meta_d.wdata[_ofst] = upstream_req_wdata_i;
                            coal_meta_d.wmask = '0;
                            coal_meta_d.wmask[_ofst] = 1'b1;

                            //1.1.4 accept req
                            upstream_req_ready_o = 1'b1;

                        end else begin
                            
                            state_d = READ_COAL;
                            dog_cnt_d = WatchDogMax;

                            //prepare meta
                            coal_meta_d.tag = upstream_req_addr_i >> $clog2(DownstreamDataWidth/8);
                            _ofst = upstream_req_addr_i[$clog2(DownstreamDataWidth/8)-1:$clog2(UpstreamDataWidth/8)];
                            coal_meta_d.down_info = '0;
                            coal_meta_d.down_info.subs[0].info = upstream_req_info_i;
                            coal_meta_d.down_info.subs[0].ofst = _ofst;
                            coal_meta_d.wdata = '0;
                            coal_meta_d.wmask = '0;

                            //accept req
                            upstream_req_ready_o = 1'b1;

                        end


                    end else if (full_refresh) begin

                        /* refresh when sub full */
                        dog_cnt_d = WatchDogMax;

                        _ofst = upstream_req_addr_i[$clog2(DownstreamDataWidth/8)-1:$clog2(UpstreamDataWidth/8)];
                        coal_meta_d.down_info = '0;
                        coal_meta_d.down_info.subs[0].info = upstream_req_info_i;
                        coal_meta_d.down_info.subs[0].ofst = _ofst;
                        coal_meta_d.wdata = '0;
                        coal_meta_d.wmask = '0;

                        //accept req
                        upstream_req_ready_o = 1'b1;


                    end else if (coal_hit) begin
                        automatic offset_t sub_ptr;

                        /* merge a hit */
                        dog_cnt_d = WatchDogMax;

                        _ofst = upstream_req_addr_i[$clog2(DownstreamDataWidth/8)-1:$clog2(UpstreamDataWidth/8)];
                        sub_ptr = coal_meta_q.down_info.num_sub + 1'b1;
                        coal_meta_d.down_info.subs[sub_ptr].info = upstream_req_info_i;
                        coal_meta_d.down_info.subs[sub_ptr].ofst = _ofst;
                        coal_meta_d.down_info.num_sub = sub_ptr;

                        //accept req
                        upstream_req_ready_o = 1'b1;

                    end else if (dog_push) begin

                        /* return to IDLE */
                        state_d = IDLE;
                        dog_cnt_d = '0;
                        coal_meta_d = '0;
                        
                    end
                end
            end





            WRITE_COAL: begin
                automatic tag_t     _tag;
                automatic offset_t  _ofst;
                automatic byte_of_t _bt_;
                automatic logic     dog_push;
                automatic logic     new_tag;
                automatic logic     coal_hit;
                automatic logic     full_refresh;
                automatic logic     down_stall = '0;

                {_tag, _ofst, _bt_} = upstream_req_addr_i;
                dog_push = (dog_cnt_q == '0) & ~upstream_req_valid_i;
                new_tag = upstream_req_valid_i & (~upstream_req_write_i | (coal_meta_q.tag != _tag));
                coal_hit = upstream_req_valid_i & upstream_req_write_i & (coal_meta_q.tag == _tag);
                full_refresh = upstream_req_valid_i & coal_hit & (coal_meta_q.down_info.num_sub == NumWord - 1);

                //3.1 conut down watchdog when no request come
                if (~upstream_req_valid_i && dog_cnt_q != '0) begin
                    dog_cnt_d = dog_cnt_q - 1'b1;
                end

                if (upstream_req_valid_i) begin
                    dog_cnt_d = WatchDogMax;
                end

                //3.2 send request downstream
                if (dog_push | new_tag | full_refresh) begin

                    //3.2.1 prepare payload
                    _ofst = '0;
                    _bt_ = '0;
                    downstream_req_addr_o = {coal_meta_q.tag, _ofst, _bt_};
                    downstream_req_info_o = coal_meta_q.down_info;
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
                        if (upstream_req_write_i) begin

                            state_d = WRITE_COAL;
                            dog_cnt_d = WatchDogMax;

                            //prepare meta
                            coal_meta_d.tag = upstream_req_addr_i >> $clog2(DownstreamDataWidth/8);
                            _ofst = upstream_req_addr_i[$clog2(DownstreamDataWidth/8)-1:$clog2(UpstreamDataWidth/8)];
                            coal_meta_d.down_info = '0;
                            coal_meta_d.down_info.subs[0].info = upstream_req_info_i;
                            coal_meta_d.wdata = '0;
                            coal_meta_d.wdata[_ofst] = upstream_req_wdata_i;
                            coal_meta_d.wmask = '0;
                            coal_meta_d.wmask[_ofst] = 1'b1;

                            //1.1.4 accept req
                            upstream_req_ready_o = 1'b1;

                        end else begin
                            
                            state_d = READ_COAL;
                            dog_cnt_d = WatchDogMax;

                            //prepare meta
                            coal_meta_d.tag = upstream_req_addr_i >> $clog2(DownstreamDataWidth/8);
                            _ofst = upstream_req_addr_i[$clog2(DownstreamDataWidth/8)-1:$clog2(UpstreamDataWidth/8)];
                            coal_meta_d.down_info = '0;
                            coal_meta_d.down_info.subs[0].info = upstream_req_info_i;
                            coal_meta_d.down_info.subs[0].ofst = _ofst;
                            coal_meta_d.wdata = '0;
                            coal_meta_d.wmask = '0;

                            //accept req
                            upstream_req_ready_o = 1'b1;

                        end


                    end else if (full_refresh) begin

                        /* refresh when sub full */
                        dog_cnt_d = WatchDogMax;

                        _ofst = upstream_req_addr_i[$clog2(DownstreamDataWidth/8)-1:$clog2(UpstreamDataWidth/8)];
                        coal_meta_d.down_info = '0;
                        coal_meta_d.down_info.subs[0].info = upstream_req_info_i;
                        coal_meta_d.wdata = '0;
                        coal_meta_d.wdata[_ofst] = upstream_req_wdata_i;
                        coal_meta_d.wmask = '0;
                        coal_meta_d.wmask[_ofst] = 1'b1;

                        //accept req
                        upstream_req_ready_o = 1'b1;


                    end else if (coal_hit) begin
                        automatic offset_t sub_ptr;

                        /* merge a hit */
                        dog_cnt_d = WatchDogMax;

                        _ofst = upstream_req_addr_i[$clog2(DownstreamDataWidth/8)-1:$clog2(UpstreamDataWidth/8)];
                        sub_ptr = coal_meta_q.down_info.num_sub + 1'b1;
                        coal_meta_d.down_info.subs[sub_ptr].info = upstream_req_info_i;
                        coal_meta_d.down_info.subs[sub_ptr].ofst = _ofst;
                        coal_meta_d.down_info.num_sub = sub_ptr;
                        coal_meta_d.wdata[_ofst] = upstream_req_wdata_i;
                        coal_meta_d.wmask[_ofst] = 1'b1;

                        //accept req
                        upstream_req_ready_o = 1'b1;

                    end else if (dog_push) begin
                        
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