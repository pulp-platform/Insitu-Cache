// Copyright 2023 ETH Zurich and 
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 22.Mar.2023

// force to synchronize decoupled queues
// TODO: watchdog

`include "common_cells/registers.svh"

module decouple_queue_sync #(
    /// number of queues
    parameter int unsigned NumQueues            = 128,
    /// datatype
    parameter type         data_t               = logic,
    /// watchdog counter limit
    parameter int unsigned WatchdogMax          = 16
    )(
    /// Clock, positive edge triggered.
    input  logic                                clk_i,
    /// Reset, active low.
    input  logic                                rst_ni,

    /// decoupled queueus
    input  data_t  [NumQueues-1:0]              decoupled_data_i,
    input  logic   [NumQueues-1:0]              decoupled_empty_i,
    output logic   [NumQueues-1:0]              decoupled_pop_o,

    /// synchronized queues 
    output  data_t [NumQueues-1:0]              sync_data_o,
    output  logic  [NumQueues-1:0]              sync_empty_o,
    input   logic  [NumQueues-1:0]              sync_pop_i

);

    //types definition
    typedef enum logic { SYNC = '0, BYPASS } sync_status_t;

    //signals definition
    sync_status_t sync_status_q, sync_status_d;

    `FFARN(sync_status_q, sync_status_d, SYNC, clk_i, rst_ni)

    logic [NumQueues-1:0] handshack_map_q, handshack_map_d, handshack_map_update;

    logic [NumQueues-1:0] lock_up_q, lock_up_d;//for BYPASS statue, stores which queus is locked up 

    logic [$clog2(WatchdogMax):0] watch_dog_q, watch_dog_d;//watch dog counter

    `FFARN(handshack_map_q, handshack_map_d, '0, clk_i, rst_ni)

    `FFARN(lock_up_q, lock_up_d, '0, clk_i, rst_ni)

    `FFARN(watch_dog_q, watch_dog_d, '0, clk_i, rst_ni)

    logic  has_complete_chunk, has_some_input, pop_all;


    // data path
    assign sync_data_o = decoupled_data_i;

    // control signal
    assign has_complete_chunk = &(~decoupled_empty_i);

    assign has_some_input = ~(&decoupled_empty_i);

    assign pop_all = &handshack_map_update & has_complete_chunk;

    assign handshack_map_update = handshack_map_q | (sync_pop_i & ~sync_empty_o);

    always_comb begin
        //default
        handshack_map_d = handshack_map_q;
        sync_status_d = sync_status_q;
        decoupled_pop_o = '0;
        sync_empty_o = {NumQueues{1'b1}};
        watch_dog_d = watch_dog_q;
        lock_up_d = lock_up_q;

        case (sync_status_q)
            SYNC : begin 
                if (has_complete_chunk) begin
                    handshack_map_d = pop_all? '0: handshack_map_update;
                    decoupled_pop_o = pop_all? {NumQueues{1'b1}} : '0;
                    sync_empty_o = handshack_map_q;
                    watch_dog_d = '0;
                end else begin 
                    handshack_map_d = '0;
                    decoupled_pop_o = '0;
                    sync_empty_o = {NumQueues{1'b1}};

                    if (has_some_input) begin
                        watch_dog_d = watch_dog_q + 1;
                        if (watch_dog_q > WatchdogMax) begin
                            sync_status_d = BYPASS;
                            lock_up_d = ~decoupled_empty_i;
                            handshack_map_d = decoupled_empty_i;
                            watch_dog_d = '0;
                        end
                    end else begin 
                        watch_dog_d = '0;
                    end
                end
            end
        
            BYPASS : begin 
                handshack_map_d = handshack_map_update;
                decoupled_pop_o = '0;
                sync_empty_o = handshack_map_q;
                if (&handshack_map_update) begin
                    sync_status_d = SYNC;
                    handshack_map_d = '0;
                    decoupled_pop_o = lock_up_q;
                    watch_dog_d = '0;
                    lock_up_d = '0;
                end
            end
        endcase
    end

    /////////////////////////////////////////
    //      Collect Metrices for Testing   //
    /////////////////////////////////////////

    // real SYNC_cnt;

    // logic [64-1:0] SYNC_cnt_q, SYNC_cnt_d;
    // `FFARN(SYNC_cnt_q, SYNC_cnt_d, '0, clk_i, rst_ni)

    // always_comb begin
    //     SYNC_cnt_d = SYNC_cnt_q;
    //     if (sync_status_q == SYNC) begin
    //         SYNC_cnt_d = SYNC_cnt_q + 1;
    //     end
    //     SYNC_cnt = SYNC_cnt_q;
    // end

    // final begin
    //     $display("sync cycles: %0d",SYNC_cnt);
    // end

endmodule