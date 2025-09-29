// Copyright 2023 ETH Zurich and 
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 22.Mar.2023

// The adapter converts N reqrsp channels to M reqrsp channels
// N and M should be power of 2 numbers (must not be 1)

`include "common_cells/registers.svh"

module decouple_channels_adapter #(
    /// channel number of channels
    parameter int unsigned NumUpstream          = 32,
    /// downstream number of queues
    parameter int unsigned NumDownstream            = 128,
    /// datatype
    parameter type         data_t               = logic
    )(
    /// Clock, positive edge triggered.
    input  logic                                clk_i,
    /// Reset, active low.
    input  logic                                rst_ni,

    /// channel side
    input  data_t  [NumUpstream-1:0]            upstream_data_i,
    input  logic   [NumUpstream-1:0]            upstream_valid_i,
    output logic   [NumUpstream-1:0]            upstream_ready_o,

    /// queue side
    output  data_t [NumDownstream-1:0]          downstream_data_o,
    output  logic  [NumDownstream-1:0]          downstream_valid_o,
    input   logic  [NumDownstream-1:0]          downstream_ready_i

);


if ( NumUpstream > NumDownstream ) begin : gen_downsize
    //////////////////////////////////////////
    //        Condition: Down Size          //
    //////////////////////////////////////////

    typedef logic [NumUpstream/NumDownstream-1:0] set_of_ready_t;
    typedef set_of_ready_t    [NumDownstream-1:0] group_up_ready_t;

    group_up_ready_t group_up_ready;

    //group up ready to upstream_ready_o
    always_comb begin : gen_upstream_ready
        for (int i = 0; i < NumDownstream; i++) begin
            for (int j = 0; j < (NumUpstream/NumDownstream); j++) begin
                upstream_ready_o[j*NumDownstream + i] = group_up_ready[i][j];
            end
        end
    end

    for (genvar i = 0; i < NumDownstream; i++) begin : gen_adapter

        logic [$clog2(NumUpstream/NumDownstream)-1:0] pointer_q, pointer_d;

        `FFARN(pointer_q, pointer_d, '0, clk_i, rst_ni)

        logic [$clog2(NumDownstream)-1:0] cnt_i;

        assign cnt_i = i;

        assign pointer_d = downstream_valid_o[i] & downstream_ready_i[i] ? pointer_q + 1 : pointer_q;

        assign downstream_valid_o[i] = upstream_valid_i[{pointer_q,cnt_i}];

        always_comb begin
            for (int j = 0; j < (NumUpstream/NumDownstream); j++) begin
                group_up_ready[i][j] = '0;
            end
            group_up_ready[i][pointer_q] = downstream_ready_i[i];
        end

        assign downstream_data_o[i] = upstream_data_i[{pointer_q,cnt_i}];
        
    end


end else if (NumUpstream < NumDownstream) begin : gen_upsize
    //////////////////////////////////////
    //        Condition: Up Size        //
    //////////////////////////////////////
    typedef logic [NumDownstream/NumUpstream-1:0] set_of_valid_t;
    typedef set_of_valid_t    [NumUpstream-1:0] group_up_valid_t;

    group_up_valid_t group_up_valid;

    //group up valid to downstream_valid_o
    always_comb begin : gen_downstream_valid
        for (int i = 0; i < NumUpstream; i++) begin
            for (int j = 0; j < (NumDownstream/NumUpstream); j++) begin
                downstream_valid_o[j*NumUpstream + i] = group_up_valid[i][j];
            end
        end
    end

    typedef data_t [NumDownstream/NumUpstream-1:0] set_of_data_t;
    typedef set_of_data_t    [NumUpstream-1:0] group_up_data_t;

    group_up_data_t group_up_data;

    //group up valid to downstream_data_o
    always_comb begin : gen_downstream_data
        for (int i = 0; i < NumUpstream; i++) begin
            for (int j = 0; j < (NumDownstream/NumUpstream); j++) begin
                downstream_data_o[j*NumUpstream + i] = group_up_data[i][j];
            end
        end
    end

    for (genvar i = 0; i < NumUpstream; i++) begin : gen_adapter

        logic [$clog2(NumDownstream/NumUpstream)-1:0] pointer_q, pointer_d;

        `FFARN(pointer_q, pointer_d, '0, clk_i, rst_ni)

        logic [$clog2(NumUpstream)-1:0] cnt_i;

        assign cnt_i = i;

        assign pointer_d = upstream_ready_o[i] & upstream_valid_i[i] ? pointer_q + 1 : pointer_q;

        always_comb begin
            for (int j = 0; j < (NumDownstream/NumUpstream); j++) begin
                group_up_valid[i][j] = '0;
            end
            group_up_valid[i][pointer_q] = upstream_valid_i[i];
        end

        assign upstream_ready_o[i] = downstream_ready_i[{pointer_q,cnt_i}];

        always_comb begin
            for (int j = 0; j < (NumDownstream/NumUpstream); j++) begin
                group_up_data[i][j] = '0;
            end
            group_up_data[i][pointer_q] = upstream_data_i[i];
        end

    end




end else begin : gen_equal
    //////////////////////////////////////////
    //        Condition: Equal Size         //
    //////////////////////////////////////////

    for (genvar i = 0; i < NumUpstream; i++) begin : gen_adapter

        assign downstream_valid_o[i] = upstream_valid_i[i];

        assign upstream_ready_o[i] = downstream_ready_i[i];

        assign downstream_data_o[i] = upstream_data_i[i];

    end


end

endmodule : decouple_channels_adapter