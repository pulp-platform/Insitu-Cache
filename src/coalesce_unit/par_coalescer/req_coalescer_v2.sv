// Copyright 2025 ETH Zurich and 
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 23.Mar.2023

//Coalescing Unit for AXI-Pack Dram Controller, version 2
//Notion
//1. upstream channels must be smaller than downstream channel
//2. the number of port can be parametrized
//3. the downstream valid waits for ready

//
//
//
// Upstream(TCDM)               Downstream(AXI)

//      port--> |-----------|
//          --> |           |
//          --> |           |         |
//          --> |   coal    | ------\ |
//          --> |   unit    | ------/ |
//          --> |           |         |
//          --> |           |
//          --> |-----------|
//


//Coalescing Process
//
//      --update_CSHR   --next_CSHR_addr                        |   <CSHR status>   <CSHR addr>
//
//      port    valid   chan_idx    current_hit     next_hit    |   CSHR_occupy_map     addr_ofst: addr_ofst 
//      --->      1                                             |       port0
//      --->      1                                             |       port1
//      --->      1                                             |       port2
//      --->      1                                             |       port3
//      --->      1                                             |       port4
//      --->      1                                             |       port5
//      --->      1                                             |       port6
//      --->      1                                             |       port7
//
//

`include "common_cells/registers.svh"
`include "axi/assign.svh"
`include "axi/typedef.svh"

module req_coalescer_v2 #(
    /// upstream data width
    parameter int unsigned UpstreamDataWidth                = 64,
    /// downstream data width.
    parameter int unsigned DownstreamDataWidth              = 512,
    /// Number of narrow request ports.
    parameter int unsigned NumPorts                         = 16,
    /// address width
    parameter int unsigned AddrWidth                        = 32,
    /// Choose stratege to determine next tag addr
    parameter int unsigned USE_ORDER_PRIOR                  = 1,
    /// Dependent parameter, do not override. number of address offsets of one coalesce block.
    parameter int unsigned NumAddrOfst                      = DownstreamDataWidth/UpstreamDataWidth,
    /// Dependent parameter, do not override. Axi DW Align.
    parameter int unsigned UpstreamDataAlign                = $clog2(UpstreamDataWidth/8),
    /// Dependent parameter, do not override. Mem DW Align.
    parameter int unsigned DownstreamDataAlign              = $clog2(DownstreamDataWidth/8),
    /// Dependent parameter, do not override. port align type.
    localparam type addr_ofst_t                             = logic [$clog2(NumAddrOfst)-1:0],
    /// Dependent parameter, do not override. upstream data type.
    localparam type upstream_data_t                         = logic [UpstreamDataWidth-1:0],
    /// Dependent parameter, do not override. downstream data type.
    localparam type downstream_data_t                       = logic [DownstreamDataWidth-1:0],
    /// Dependent parameter, do not override. address type.
    localparam type addr_t                                  = logic [AddrWidth-1:0]
    )(
    /// Clock, positive edge triggered.
    input  logic                            clk_i,
    /// Reset, active low.
    input  logic                            rst_ni,

    /// Upstream side
    input  addr_t [NumPorts-1:0]            upstream_addr_i,
    input  logic [NumPorts-1:0]             upstream_valid_i,
    output logic [NumPorts-1:0]             upstream_ready_o,

    /// Downstream side
    output  logic                           coal_valid_o,
    input   logic                           coal_ready_i,
    output  addr_t                          coal_addr_o,

    /// metadata fifo of valid port bitmap (strb)
    output  logic [NumPorts-1:0]            coal_strb_o,
    input   logic                           coal_strb_full_i,
    output  logic                           coal_strb_push_o,

    /// metadata fifos for every port
    output  addr_ofst_t [NumPorts-1:0]      coal_port_addr_ofst_o,
    input   logic       [NumPorts-1:0]      coal_port_addr_ofst_full_i,
    output  logic       [NumPorts-1:0]      coal_port_addr_ofst_push_o

);

    ////////////////////////////////////
    //  Types and Signals Definition  //
    ////////////////////////////////////

    ///** Coalescing Status Hold Register -- CSHR
    //types
    typedef enum logic { IDLE = '0, VALID } CSHR_status_t;

    typedef logic [NumPorts-1:0] port_bitmap_t;

    typedef logic [AddrWidth-DownstreamDataAlign-1:0] tag_addr_t;

    //registers
    CSHR_status_t CSHR_status_q, CSHR_status_d;

    tag_addr_t CSHR_addr_q, CSHR_addr_d;

    port_bitmap_t occupy_map_q, occupy_map_d;

    addr_ofst_t [NumPorts-1:0] addr_ofst_of_port_q, addr_ofst_of_port_d;

    //setup registers
    `FFARN(CSHR_status_q, CSHR_status_d, IDLE, clk_i, rst_ni)

    `FFARN(CSHR_addr_q, CSHR_addr_d, '0, clk_i, rst_ni)

    `FFARN(occupy_map_q, occupy_map_d, '0, clk_i, rst_ni)

    `FFARN(addr_ofst_of_port_q, addr_ofst_of_port_d, '0, clk_i, rst_ni)


    ///** Signals
    tag_addr_t [NumPorts-1:0] tag_addr_of_port;

    addr_ofst_t [NumPorts-1:0] addr_ofst_of_port;

    port_bitmap_t current_hit_of_port;

    port_bitmap_t next_hit_of_port;

    port_bitmap_t occupy_map_update_current_hit;

    port_bitmap_t occupy_map_update_next_hit;

    addr_ofst_t [NumPorts-1:0] addr_ofst_update_current_hit;

    addr_ofst_t [NumPorts-1:0] addr_ofst_update_next_hit;

    tag_addr_t next_CSHR_addr;

    logic update_CSHR;

    logic coal_addr_ofst_fifos_have_space;


    ////////////////////////
    //  Watchdog Counter  //
    ////////////////////////
    /*
        The idea is to release the CSHR when no requests come in a certain time.
        The watchdog counts up when CSHR is occupied but no valid input comes.
        When:
            1. the watchdog counter is equal or greater than a credit,
            2. the CSHR is in VALID state
            3. the downstream is ready to accept coalesced request
            4. fifo for coal strb is not full
            5. fifos for coal addr offest are not full
            4. currently still no valid input ports
        Then the CSHR is forced to release and return into IDLE state
        Defined signals:
            1. watchdog_cnt -- the counter
            2. watchdog_credit -- decide by the number of unoccupied channels in CSHR
            3. watchdog_flag -- tell that a CSHR release is needed 
        The block below defines signal circuits of `watchdog_credit` and `watchdog_flag`
        While `watchdog_cnt` is inside the `CSHR FSM` block
    */

    addr_ofst_t watchdog_cnt_q, watchdog_cnt_d;

    addr_ofst_t watchdog_credit;

    logic watchdog_flag;

    always_comb begin
        watchdog_credit = 0;
        for (int i = 0; i < NumPorts; i++) begin
            if (occupy_map_q[i] ==  0) begin
                watchdog_credit = watchdog_credit + 1;
            end
        end
    end

    assign watchdog_flag =  (watchdog_cnt_q == watchdog_credit) & 
                            (CSHR_status_q == VALID)    &
                            coal_ready_i &
                            ~coal_strb_full_i &
                            coal_addr_ofst_fifos_have_space &
                            ((|upstream_valid_i) == 0);

    `FFARN(watchdog_cnt_q, watchdog_cnt_d, '0, clk_i, rst_ni)




    /////////////////////////////////////
    //  CSHR Control Signals Circuits  //
    /////////////////////////////////////

    //Calculate every tag address of ports--
    for (genvar i = 0; i < NumPorts; i++) begin: gen_tag
        assign tag_addr_of_port[i] = upstream_addr_i[i][AddrWidth-1:DownstreamDataAlign];
    end

    //Calculate every slot index of ports--
    for (genvar i = 0; i < NumPorts; i++) begin: gen_addr_ofst
        assign addr_ofst_of_port[i] = upstream_addr_i[i][DownstreamDataAlign-1:UpstreamDataAlign];
    end

    //Hit map of inputs: --
    /*Hit = valid_port + 
            has equally tag addr + 
            port is not occupied in CSHR +
            CSHR is in VALID state
    */ 
    for (genvar i = 0; i < NumPorts; i++) begin: gen_current_hit
        assign current_hit_of_port[i] = upstream_valid_i[i] & 
                                        (tag_addr_of_port[i]==CSHR_addr_q) & 
                                        (~occupy_map_q[i]) &
                                        (CSHR_status_q == VALID);
    end

    //determine whether fifos for addr_ofst are ready
    /*
        that every addr ofst fifo should be either:
            1. not targeted
            2. not full
    */
    assign coal_addr_ofst_fifos_have_space = & (~occupy_map_update_current_hit | 
                                                ~coal_port_addr_ofst_full_i);


    //Calculate the next addr for coalescing --
if (USE_ORDER_PRIOR == 1) begin
    /*first order arbitration*/
    always_comb begin
        next_CSHR_addr = '0;
        for (int i = 0; i < NumPorts; i++) begin
            if (upstream_valid_i[i] & (~current_hit_of_port[i])) begin
                next_CSHR_addr = tag_addr_of_port[i];
                break;
            end
         end 
    end
end else begin
    /*Round-robin artribution among miss ports*/
    logic rr_arb_out_valid;
    rr_arb_tree #(
      .NumIn    ( NumPorts ),
      .DataType ( tag_addr_t   ),
      .AxiVldRdy( 1'b1       ),
      .LockIn   ( 1'b1       )
    ) i_next_CSHR_addr_mux (
      .clk_i,
      .rst_ni,
      .flush_i( 1'b0          ),
      .rr_i   ( '0            ),
      .req_i  ( upstream_valid_i & (~current_hit_of_port) ),
      .gnt_o  ( /*open*/ ),
      .data_i ( tag_addr_of_port   ),
      .gnt_i  ( update_CSHR   ),
      .req_o  ( rr_arb_out_valid   ),
      .data_o ( next_CSHR_addr  ),
      .idx_o  ( /*open*/ )
    );
end

    

    //Calculate the next hits to update --
    /*
        Next hits to be accepted =  find ports that (tag addr == next CSHR addr) Among all miss ports
    */
    for (genvar i = 0; i < NumPorts; i++) begin: gen_next_hit
        assign next_hit_of_port[i] = upstream_valid_i[i] & 
                                    (~current_hit_of_port[i]) &
                                    (tag_addr_of_port[i] == next_CSHR_addr); 
    end

    //Determine whether to update the CHSR --
    /*prerequisites are: 
        1.there are valid ports
        2.the downstream is ready to accept coalesced request
        3.fifo for coal strb is not full
        4.fifos for coal addr offest are not full
      Then either of follwoing two conditions can trigger CSHR update:
        1.the CSHR is in IDLE state
        2.there exists miss requests
    */
    always_comb begin: gen_update_CSHR
        update_CSHR = 0;
        if ((|upstream_valid_i) & coal_ready_i & ~coal_strb_full_i & coal_addr_ofst_fifos_have_space) begin
            if (CSHR_status_q == IDLE ) begin
                update_CSHR = 1;
            end else begin
                if ((upstream_valid_i & (~current_hit_of_port)) != '0 ) begin
                    update_CSHR = 1;
                end
            end
        end 
    end

    //update signals for CSHR --
    always_comb begin: gen_occupy_map_update_current_hit
        //default
        occupy_map_update_current_hit = occupy_map_q;
        addr_ofst_update_current_hit = addr_ofst_of_port_q;
        occupy_map_update_next_hit = '0;
        for (int i = 0; i < NumPorts; i++) begin
            addr_ofst_update_next_hit[i] = '0;
        end

        for (int i = 0; i < NumPorts; i++) begin
            if (current_hit_of_port[i]) begin
                occupy_map_update_current_hit[i] = 1;
                addr_ofst_update_current_hit[i] = addr_ofst_of_port[i];
            end
            if (next_hit_of_port[i]) begin
                occupy_map_update_next_hit[i] = 1;
                addr_ofst_update_next_hit[i] = addr_ofst_of_port[i];
            end 
        end
    end


    //CSHR FSM --
    always_comb begin
        //default
        CSHR_status_d = CSHR_status_q;
        CSHR_addr_d = CSHR_addr_q;
        occupy_map_d = occupy_map_q;
        addr_ofst_of_port_d = addr_ofst_of_port_q;

        // Upstream side
        upstream_ready_o = '0;

        // Downstream side
        coal_valid_o = 0;
        coal_addr_o = CSHR_addr_q << DownstreamDataAlign;

        // metadata fifo of valid port bitmap (strb)
        coal_strb_o = '0;
        coal_strb_push_o = '0;
        
        // metadata fifos for every port
        coal_port_addr_ofst_o = '0;
        coal_port_addr_ofst_push_o = '0;

        // watchdog
        watchdog_cnt_d = watchdog_cnt_q;

        case (CSHR_status_q)
            
            IDLE: begin
                if (update_CSHR) begin
                    CSHR_status_d = VALID;
                    CSHR_addr_d = next_CSHR_addr;
                    occupy_map_d = occupy_map_update_next_hit;
                    addr_ofst_of_port_d = addr_ofst_update_next_hit;
                    upstream_ready_o = next_hit_of_port;
                end

                //watchdog is always reset in this state
                watchdog_cnt_d = '0;
            end

            VALID: begin

                //update CSHR
                if (update_CSHR | watchdog_flag) begin
                    // internal signals
                    CSHR_addr_d = next_CSHR_addr;
                    occupy_map_d = occupy_map_update_next_hit;
                    addr_ofst_of_port_d = addr_ofst_update_next_hit;

                    // upstream
                    upstream_ready_o = current_hit_of_port | next_hit_of_port;

                    // downstream
                    coal_valid_o = 1;

                    // strb fifo
                    coal_strb_o = occupy_map_update_current_hit;
                    coal_strb_push_o = 1;

                    // addr ofst fifos
                    coal_port_addr_ofst_o = addr_ofst_update_current_hit;
                    coal_port_addr_ofst_push_o = occupy_map_update_current_hit;

                    //the case update CSHR by watchdag
                    if (watchdog_flag) begin
                        CSHR_status_d = IDLE;
                    end

                end else begin
                    //Normal current hit update
                    occupy_map_d = occupy_map_update_current_hit;
                    addr_ofst_of_port_d = addr_ofst_update_current_hit;
                    upstream_ready_o = current_hit_of_port ;
                end

                //Watchdog counter
                if ((|upstream_valid_i) == 0) begin
                    if (watchdog_cnt_q < watchdog_credit) begin
                        watchdog_cnt_d = watchdog_cnt_q + 1;
                    end
                end else begin
                    watchdog_cnt_d = '0;
                end

            end

            default: begin
                CSHR_status_d = IDLE;
            end
        endcase
    end


endmodule
