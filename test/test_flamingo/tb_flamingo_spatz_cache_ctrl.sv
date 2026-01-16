// Copyright 2023 ETH Zurich and
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 28.Feb.2024

`include "axi/assign.svh"
`include "axi/typedef.svh"
`include "insitu_cache/assign.svh"

`timescale 1ns/1ps

module tb_flamingo_spatz_cache_ctrl#(
    /* DUT Model Parameters */
    /// Address width of both upstream narrow request and downstream wide request
    parameter int unsigned ReqAddrWidth                                         = 32,
    /// Word width of cache line (512b default)
    parameter int unsigned CacheLineWidth                                       = 512,
    /// Number of Cache entries
    parameter int unsigned NumCacheEntry                                        = `NUM_CACHE_LINE,
    /// Number of Associatity
    parameter int unsigned SetAssociativity                                     = `NUM_CACHE_ASSO,
    /// Number of Pseudo-Dual Banks
    parameter int unsigned BankFactor                                           = `NUM_CACHE_BANK_FACTOR,
    /// Enable folded data banks for cache data RAM.
    parameter bit          UseFoldedDataBanks                                   = 1'b1,
    /// Ways per folded data bank (0 = auto: min(4, SetAssociativity)).
    parameter int unsigned FoldWayGroup                                         = 0,
    /// Width of word (granularity of non-blocking write)
    parameter int unsigned WordWidth                                            = `CACHE_WORD_WIDTH,
    /// Number of Latency of Offchip Link
    parameter int unsigned NumOffchipLatency                                    = `OFFCHIP_LATENCY,
    /// Counter cache line life cycle information for questa-sim.
    parameter int unsigned LogLifeCycle                                         = `LOG_LIFE_CYCLE,
    /// Width of Access MetaData
    parameter int unsigned AccessMetaWidth                                      = `ACCESS_META_WIDTH,
    /// Number of Core Access Ports
    parameter int unsigned NumPorts                                             = `ACCESS_CORE_PORTS,
     /// Coalescer Extned Factor
    parameter int unsigned CoalExtFactor                                        = `COAL_EXT_FACTOR,


    /* Test Parameters */
    parameter int unsigned NumTest                                              = `NUM_TEST,
    parameter int          TrafficLimit                                         = `TRAFFIC_LIMIT,

    // Dependent parameter, do not override. Depth of cache bank.
    localparam int unsigned CacheWaysEntry                                      = NumCacheEntry/SetAssociativity,
    localparam int unsigned NumDataBankPerWay                                   = BankFactor*(CacheLineWidth/WordWidth),
    // Dependent parameter, do not override. Number of data tcdm banks.
    localparam int unsigned NumDataBank                                         = SetAssociativity*BankFactor*(CacheLineWidth/WordWidth),
    // Dependent parameter, do not override. Number of meta tcdm banks.
    localparam int unsigned NumTagBank                                          = SetAssociativity*BankFactor,
    localparam int unsigned NumWays                                             = SetAssociativity,
    localparam int unsigned EffectiveFoldWayGroup                               = UseFoldedDataBanks ?
        ((FoldWayGroup == 0) ? ((SetAssociativity < 4) ? SetAssociativity : 4) : FoldWayGroup) :
        SetAssociativity,
    localparam int unsigned NumWayGroups                                        = SetAssociativity / EffectiveFoldWayGroup,
    localparam int unsigned DataBankLineSplit                                   = UseFoldedDataBanks ? EffectiveFoldWayGroup : 1,
    localparam int unsigned DataBankWordGroup                                   = (CacheLineWidth / DataBankLineSplit) / WordWidth,
    localparam int unsigned NumDataBankPerWayGrouped                            = NumDataBankPerWay / DataBankWordGroup,
    // Dependent parameter, do not override. Address type.
    localparam type tcdm_bank_addr_t                                            = logic [$clog2(CacheWaysEntry)-$clog2(BankFactor)-1:0],
    // Dependent parameter, do not override. Address type.
    localparam type addr_t                                                      = logic [ReqAddrWidth-1:0],
    // Dependent parameter, do not override. Wide word type.
    localparam type data_t                                                      = logic [CacheLineWidth-1:0],
    // Dependent parameter, do not override. Narrow word type.
    localparam type word_t                                                      = logic [WordWidth-1:0],
    // Dependent parameter, do not override. Byte strobe type for a word.
    localparam type word_strb_t                                                 = logic [WordWidth/8-1:0],
    // Dependent parameter, do not override. Word mask type.
    localparam type mask_t                                                      = logic [CacheLineWidth/WordWidth-1:0],
    // Dependent parameter, do not override. Byte strb type.
    localparam type strb_t                                                      = logic [CacheLineWidth/8-1:0],
    // Dependent parameter, do not override. set ptr type.
    localparam type way_ptr_t                                                   = logic [$clog2(SetAssociativity)-1:0],
    // Dependent parameter, do not override. bank depth ptr type.
    localparam type cache_ways_entry_ptr_t                                      = logic [$clog2(CacheWaysEntry)-1:0],
    /// Dependent parameter, do not override. word type
    localparam type tcdm_tag_data_t                                             = logic [WordWidth-1:0]
);

    localparam time ClkPeriod               = 1ns;
    localparam time ApplTime                = 0.2ns;
    localparam time TestTime                = 0.8ns;


    //////////////////////////////////////
    //        Types Definition          //
    //////////////////////////////////////

    typedef struct packed {
        logic [$clog2(NumPorts)-1:0]                        core_id;
        logic [AccessMetaWidth-1:0]                         access_id;
    } core_meta_t;

    // typedef logic [CoreMetaWidth-1:0]                       core_meta_t;


    typedef struct packed {
        addr_t                                              addr;
        core_meta_t                                         meta;
        logic                                               write;
        word_t                                              wdata;
        word_strb_t                                         strb;
    } core_req_t;

    typedef struct packed {
        logic                                               write;
        word_t                                              data;
        core_meta_t                                         meta;
    } core_resp_t;

    `AXI_TYPEDEF_ALL(dram_axi, addr_t, logic, data_t, strb_t, logic)

    //////////////////////////////////////
    //        Signal Definition         //
    //////////////////////////////////////

    logic  clk, rst_n;

    //core req/resp
    logic             [NumPorts-1:0]                        core_req_valid;
    logic             [NumPorts-1:0]                        core_req_ready;
    addr_t            [NumPorts-1:0]                        core_req_addr;
    core_meta_t       [NumPorts-1:0]                        core_req_meta;
    logic             [NumPorts-1:0]                        core_req_write;
    word_t            [NumPorts-1:0]                        core_req_wdata;
    word_strb_t       [NumPorts-1:0]                        core_req_wstrb;

    logic             [NumPorts-1:0]                        core_resp_valid;
    logic             [NumPorts-1:0]                        core_resp_ready;
    logic             [NumPorts-1:0]                        core_resp_write;
    word_t            [NumPorts-1:0]                        core_resp_data;
    core_meta_t       [NumPorts-1:0]                        core_resp_meta;

    //DRAM AXI
    dram_axi_req_t                                          dram_axi_req;
    dram_axi_resp_t                                         dram_axi_resp;
    dram_axi_req_t                                          dram_axi_offchip_req;
    dram_axi_resp_t                                         dram_axi_offchip_resp;

    /// Meta Banks
    logic             [NumTagBank-1:0]                      tcdm_tag_bank_req;
    logic             [NumTagBank-1:0]                      tcdm_tag_bank_we;
    tcdm_bank_addr_t  [NumTagBank-1:0]                      tcdm_tag_bank_addr;
    tcdm_tag_data_t   [NumTagBank-1:0]                      tcdm_tag_bank_wdata;
    logic             [NumTagBank-1:0]                      tcdm_tag_bank_be;
    tcdm_tag_data_t   [NumTagBank-1:0]                      tcdm_tag_bank_rdata;

    /// Data Banks
    logic             [NumDataBank-1:0]                     tcdm_data_bank_req;
    logic             [NumDataBank-1:0]                     tcdm_data_bank_we;
    tcdm_bank_addr_t  [NumDataBank-1:0]                     tcdm_data_bank_addr;
    word_t            [NumDataBank-1:0]                     tcdm_data_bank_wdata;
    logic             [NumDataBank-1:0][WordWidth/8-1:0]     tcdm_data_bank_be;
    word_t            [NumDataBank-1:0]                     tcdm_data_bank_rdata;

    /// Bank Grant for Cache
    logic             [NumDataBank-1:0]                     tcdm_data_bank_gnt;

    /// monitor signals
    core_req_t                                              monitor_core_request_queue [NumPorts][$];
    core_resp_t                                             monitor_core_respons_queue [NumPorts][$];



    //////////////////////////////////////
    //        Clock Generation          //
    //////////////////////////////////////
    initial begin
        rst_n = 0;
        clk   = 0;
        $display("start");
        repeat (10) begin
            #0.5ns clk = 0;
            #0.5ns clk = 1;
        end
        rst_n <= #0.5ns 1;
        $display("rst up");
        forever begin
            #0.5ns clk = 0;
            #0.5ns clk = 1;
        end
    end



    //////////////////////
    //        DUT       //
    //////////////////////

    flamingo_spatz_cache_ctrl #(
        .NumPorts               (NumPorts),
        .CoalExtFactor          (CoalExtFactor),
        .core_meta_t            (core_meta_t),
        .AddrWidth              (ReqAddrWidth),
        .WordWidth              (WordWidth),
        .NumCacheEntry          (NumCacheEntry),
        .CacheLineWidth         (CacheLineWidth),
        .SetAssociativity       (SetAssociativity),
        .BankFactor             (BankFactor),
        .axi_req_t              (dram_axi_req_t),
        .axi_resp_t             (dram_axi_resp_t)
    ) flamingo_spatz_cache_ctrl (
        .clk_i                  (clk                  ),
        .rst_ni                 (rst_n                ),
        .impl_i                 ('0),

        .cache_sync_valid_i     ('0),
        .cache_sync_ready_o     (),
        .cache_sync_insn_i      ('0),
        .bank_depth_for_SPM_i   ('0),

        .core_req_valid_i       (core_req_valid),
        .core_req_ready_o       (core_req_ready),
        .core_req_addr_i        (core_req_addr),
        .core_req_meta_i        (core_req_meta),
        .core_req_write_i       (core_req_write),
        .core_req_wdata_i       (core_req_wdata),
        .core_req_wstrb_i       (core_req_wstrb),
        .core_resp_valid_o      (core_resp_valid),
        .core_resp_ready_i      (core_resp_ready),
        .core_resp_write_o      (core_resp_write),
        .core_resp_data_o       (core_resp_data),
        .core_resp_meta_o       (core_resp_meta),

        .axi_req_o              (dram_axi_req),
        .axi_resp_i             (dram_axi_resp),

        .tcdm_tag_bank_req_o    (tcdm_tag_bank_req),
        .tcdm_tag_bank_we_o     (tcdm_tag_bank_we),
        .tcdm_tag_bank_addr_o   (tcdm_tag_bank_addr),
        .tcdm_tag_bank_wdata_o  (tcdm_tag_bank_wdata),
        .tcdm_tag_bank_be_o     (tcdm_tag_bank_be),
        .tcdm_tag_bank_rdata_i  (tcdm_tag_bank_rdata),

        .tcdm_data_bank_req_o   (tcdm_data_bank_req),
        .tcdm_data_bank_we_o    (tcdm_data_bank_we),
        .tcdm_data_bank_addr_o  (tcdm_data_bank_addr),
        .tcdm_data_bank_wdata_o (tcdm_data_bank_wdata),
        .tcdm_data_bank_be_o    (tcdm_data_bank_be),
        .tcdm_data_bank_rdata_i (tcdm_data_bank_rdata),

        .tcdm_data_bank_gnt_i   (tcdm_data_bank_gnt)
    );


    /*****************/
    /*  TCDM Banks   */
    /*****************/
    for (genvar i = 0; i < NumTagBank; i++) begin : gen_meta_banks
        tc_sram #(
            .NumWords(CacheWaysEntry/BankFactor),
            .DataWidth($bits(tcdm_tag_data_t)),
            .ByteWidth($bits(tcdm_tag_data_t)),
            .NumPorts(1),
            .Latency(1),
            .SimInit("zeros")
        ) i_meta_bank (
            .clk_i  (clk                    ),
            .rst_ni (rst_n                  ),
            .req_i  (tcdm_tag_bank_req[i]  ),
            .we_i   (tcdm_tag_bank_we[i]   ),
            .addr_i (tcdm_tag_bank_addr[i] ),
            .wdata_i(tcdm_tag_bank_wdata[i]),
            .be_i   (tcdm_tag_bank_be[i]   ),
            .rdata_o(tcdm_tag_bank_rdata[i])
        );
    end

    if (UseFoldedDataBanks) begin : gen_folded_data_banks
        localparam int unsigned BankDataWidth = WordWidth * DataBankWordGroup;
        localparam int unsigned BankByteCount = BankDataWidth / 8;
        for (genvar bank = 0; bank < NumDataBankPerWayGrouped; bank++) begin : gen_data_banks
            for (genvar group = 0; group < NumWayGroups; group++) begin : gen_way_groups
                logic [EffectiveFoldWayGroup-1:0] bank_req;
                logic [EffectiveFoldWayGroup-1:0] bank_we;
                tcdm_bank_addr_t                  bank_addr  [EffectiveFoldWayGroup];
                logic [BankDataWidth-1:0]         bank_wdata [EffectiveFoldWayGroup];
                logic [BankByteCount-1:0]         bank_be    [EffectiveFoldWayGroup];
                logic [BankDataWidth-1:0]         bank_rdata [EffectiveFoldWayGroup];

                for (genvar way = 0; way < EffectiveFoldWayGroup; way++) begin : gen_folded_ports
                    localparam int unsigned WayIdx = group * EffectiveFoldWayGroup + way;
                    assign bank_req[way] = |tcdm_data_bank_req[WayIdx*NumDataBankPerWay + bank*DataBankWordGroup +: DataBankWordGroup];
                    assign bank_we[way] = |tcdm_data_bank_we[WayIdx*NumDataBankPerWay + bank*DataBankWordGroup +: DataBankWordGroup];
                    assign bank_addr[way] = tcdm_data_bank_addr[WayIdx*NumDataBankPerWay + bank*DataBankWordGroup];

                    for (genvar g = 0; g < DataBankWordGroup; g++) begin : gen_group_words
                        localparam int unsigned FlatIdx = WayIdx * NumDataBankPerWay + bank * DataBankWordGroup + g;
                        assign bank_wdata[way][g*WordWidth +: WordWidth] = tcdm_data_bank_wdata[FlatIdx];
                        assign bank_be[way][g*(WordWidth/8) +: (WordWidth/8)] = tcdm_data_bank_be[FlatIdx];
                        assign tcdm_data_bank_rdata[FlatIdx] = bank_rdata[way][g*WordWidth +: WordWidth];
                        assign tcdm_data_bank_gnt[FlatIdx] = 1'b1;
                    end
                end

                folded_data_bank #(
                    .NumWays     (EffectiveFoldWayGroup),
                    .DepthPerWay (CacheWaysEntry/BankFactor),
                    .DataWidth   (BankDataWidth),
                    .ByteWidth   (8),
                    .Latency     (1),
                    .SimInit     ("zeros")
                ) i_data_bank (
                    .clk_i   (clk),
                    .rst_ni  (rst_n),
                    .req_i   (bank_req),
                    .we_i    (bank_we),
                    .addr_i  (bank_addr),
                    .wdata_i (bank_wdata),
                    .be_i    (bank_be),
                    .rdata_o (bank_rdata)
                );
            end
        end
    end else begin : gen_unfolded_data_banks
        localparam int unsigned BankDataWidth = WordWidth * DataBankWordGroup;
        localparam int unsigned BankByteCount = BankDataWidth / 8;
        for (genvar bank = 0; bank < NumDataBankPerWayGrouped; bank++) begin : gen_data_banks
            for (genvar way = 0; way < NumWays; way++) begin : gen_way_banks
                logic                     bank_req;
                logic                     bank_we;
                tcdm_bank_addr_t          bank_addr;
                logic [BankDataWidth-1:0] bank_wdata;
                logic [BankByteCount-1:0] bank_be;
                logic [BankDataWidth-1:0] bank_rdata;

                assign bank_req = |tcdm_data_bank_req[way*NumDataBankPerWay + bank*DataBankWordGroup +: DataBankWordGroup];
                assign bank_we  = |tcdm_data_bank_we [way*NumDataBankPerWay + bank*DataBankWordGroup +: DataBankWordGroup];
                assign bank_addr = tcdm_data_bank_addr[way*NumDataBankPerWay + bank*DataBankWordGroup];

                for (genvar g = 0; g < DataBankWordGroup; g++) begin : gen_group_words
                    localparam int unsigned FlatIdx = way * NumDataBankPerWay + bank * DataBankWordGroup + g;
                    assign bank_wdata[g*WordWidth +: WordWidth] = tcdm_data_bank_wdata[FlatIdx];
                    assign bank_be[g*(WordWidth/8) +: (WordWidth/8)] = tcdm_data_bank_be[FlatIdx];
                    assign tcdm_data_bank_rdata[FlatIdx] = bank_rdata[g*WordWidth +: WordWidth];
                    assign tcdm_data_bank_gnt[FlatIdx] = 1'b1;
                end

                tc_sram #(
                    .NumWords(CacheWaysEntry/BankFactor),
                    .DataWidth(BankDataWidth),
                    .ByteWidth(8),
                    .NumPorts(1),
                    .Latency(1),
                    .SimInit("zeros")
                ) i_data_bank (
                    .clk_i  (clk      ),
                    .rst_ni (rst_n    ),
                    .req_i  (bank_req ),
                    .we_i   (bank_we  ),
                    .addr_i (bank_addr),
                    .wdata_i(bank_wdata),
                    .be_i   (bank_be  ),
                    .rdata_o(bank_rdata)
                );
            end
        end
    end

    ////////////////////////
    //        Slave       //
    ////////////////////////


    /***********/
    /*  DRAM   */
    /***********/
    dram_sim_engine #(.ClkPeriod(1)) i_dram_sim_engine (.clk_i(clk), .rst_ni(rst_n));

    axi_dram_sim #(
        .AxiAddrWidth(ReqAddrWidth),
        .AxiDataWidth(CacheLineWidth),
        .AxiIdWidth  (1),
        .AxiUserWidth(1),
        .DRAMType    ("HBM2"),
        .BASE        ('0),
        .axi_req_t   (dram_axi_req_t),
        .axi_resp_t  (dram_axi_resp_t),
        .axi_ar_t    (dram_axi_ar_chan_t),
        .axi_r_t     (dram_axi_r_chan_t),
        .axi_aw_t    (dram_axi_aw_chan_t),
        .axi_w_t     (dram_axi_w_chan_t),
        .axi_b_t     (dram_axi_b_chan_t)
    ) i_axi_dram_sim (
        .clk_i(clk),
        .rst_ni(rst_n),
        .axi_req_i (dram_axi_req ),
        .axi_resp_o(dram_axi_resp)
    );

    //////////////////////////
    //        Monitor       //
    //////////////////////////

    class core_req_wrapper;
        rand core_req_t req;

        constraint valid_addr {
        req.addr >= 0;
        req.addr < `ADDR_RANG;
        }
    endclass

    task cycle_start;
      #TestTime;
    endtask

    task cycle_end;
      @(posedge clk);
    endtask

    task automatic monitor_record_core_req();
        for (int i = 0; i < NumPorts; i++) begin
            if (core_req_valid[i] && core_req_ready[i]) begin
                automatic core_req_t req;
                req.addr  = core_req_addr[i];
                req.meta  = core_req_meta[i];
                req.write = core_req_write[i];
                req.wdata = core_req_wdata[i];
                req.strb  = core_req_wstrb[i];
                monitor_core_request_queue[i].push_back(req);  // Push write signal into queue
            end
        end
    endtask

    task automatic monitor_record_core_resp();
        for (int i = 0; i < NumPorts; i++) begin
            if (core_resp_valid[i] && core_resp_ready[i]) begin
                automatic core_resp_t resp;
                resp.data = core_resp_data[i];
                resp.meta = core_resp_meta[i];
                resp.write = core_resp_write[i];
                monitor_core_respons_queue[i].push_back(resp);  // Push write signal into queue
            end
        end
    endtask

    task automatic monitor_log_queue_info();
        for (int i = 0; i < NumPorts; i++) begin
            $display("[Core %0d]: %d Req | %d Resp",i, monitor_core_request_queue[i].size(),monitor_core_respons_queue[i].size());
        end
    endtask

    task automatic monitor_wait_all_resp(input int unsigned n_access);
        automatic int done = 0;
        cycle_start();
        while(done == 0) begin
            cycle_end(); cycle_start();
            done = 1;
            for (int i = 0; i < NumPorts; i++) begin
                if (monitor_core_respons_queue[i].size() < n_access) begin
                    done = 0;
                end
            end
        end
        cycle_end();
    endtask

    initial begin
        @(posedge rst_n);
        @(posedge clk);
        forever begin
            cycle_start();
            monitor_record_core_req();
            monitor_record_core_resp();
            cycle_end();
        end
    end


    /////////////////////////////
    //        Scoreboard       //
    /////////////////////////////

    task scoreboard_check_correctness();
        for (int c = 0; c < NumPorts; c++) begin
            $display("[Scoreboard] Checking Core %0d",c);
            for (int q = 0; q < monitor_core_request_queue[c].size(); q++) begin
                automatic int find_out = 0;
                for (int p = 0; p < monitor_core_respons_queue[c].size(); p++) begin
                    automatic core_req_t req   = monitor_core_request_queue[c][q];
                    automatic core_resp_t resp = monitor_core_respons_queue[c][p];
                    if (req.meta == resp.meta) begin
                        if (req.write == resp.write) begin
                            if (find_out == 1) begin
                                $fatal(1,"[Scoreboard] duplicate match on req %p",monitor_core_request_queue[c][q]);
                            end
                            // $display("[Scoreboard][Core %0d] Match req id %0d <===> resp order %0d",c,q,p);
                            find_out = 1;
                        end
                    end
                end

                if (find_out == 0) begin
                    $fatal(1,"[Scoreboard] can not match core req %p",monitor_core_request_queue[c][q]);
                end
            end
        end
    endtask

    task scoreboard_reset();
        for (int i = 0; i < NumPorts; i++) begin
            monitor_core_request_queue[i].delete();
            monitor_core_respons_queue[i].delete();
        end
    endtask


    /////////////////////////
    //        Driver       //
    /////////////////////////

    task automatic init_core_req();
        for (int id = 0; id < NumPorts; id++) begin
            core_req_addr[id]   = '0;
            core_req_meta[id]   = '0;
            core_req_write[id]  = '0;
            core_req_wdata[id]  = '0;
            core_req_wstrb[id]  = '0;
            core_req_valid[id]  = '0;
            core_resp_ready[id] = 1'b1;
        end
    endtask

    task automatic send_req_to_cache (
      input core_req_t req,
      input int unsigned id
    );
        core_req_addr[id] = req.addr;
        core_req_meta[id] = req.meta;
        core_req_write[id] = req.write;
        core_req_wdata[id] = req.wdata;
        core_req_wstrb[id] = req.strb;
        core_req_valid[id] = 1'b1;
        cycle_start();
        while (core_req_ready[id] != 1'b1) begin cycle_end(); cycle_start(); end
        cycle_end();
        core_req_addr[id] = '0;
        core_req_meta[id] = '0;
        core_req_write[id] = '0;
        core_req_wdata[id] = '0;
        core_req_wstrb[id] = '0;
        core_req_valid[id] = '0;
    endtask

    task automatic recv_resp_from_cache (
      output core_resp_t resp,
      input int unsigned id
    );
        core_resp_ready[id] = 1'b1;
        cycle_start();
        while (core_resp_valid[id] != 1) begin cycle_end(); cycle_start(); end
        resp.write = core_resp_write[id];
        resp.data = core_resp_data[id];
        resp.meta = core_resp_meta[id];
        cycle_end();
        core_resp_ready[id] = '0;
    endtask

    task automatic random_sends(
        input int unsigned n_sends,
        input int unsigned id
    );
        automatic core_req_wrapper req_wrapper = new;
        automatic string test_type_str = "[Random Read/Write]";
        for (int i = 0; i < n_sends; i++) begin
            req_wrapper.randomize();
            req_wrapper.req.addr = req_wrapper.req.addr << $clog2(WordWidth/8);
            req_wrapper.req.meta.core_id = id;
            req_wrapper.req.meta.access_id = i;
            req_wrapper.req.strb = req_wrapper.req.write ? {WordWidth/8{1'b1}} : '0;
            send_req_to_cache(req_wrapper.req, id);
            $display("%s[Core %0d]    Send #%0d ",test_type_str,id,i);
        end
    endtask

    task automatic stride_reads(
        input int unsigned n_sends,
        input int unsigned id,
        input int unsigned base,
        input int unsigned stride
    );
        automatic core_req_wrapper req_wrapper = new;
        automatic string test_type_str = "[Random Read/Write]";
        for (int i = 0; i < n_sends; i++) begin
            req_wrapper.randomize();
            req_wrapper.req.addr = base + i * stride;
            req_wrapper.req.addr = req_wrapper.req.addr << $clog2(WordWidth/8);
            req_wrapper.req.meta.core_id = id;
            req_wrapper.req.meta.access_id = i;
            req_wrapper.req.write = '0;
            req_wrapper.req.strb = '0;
            send_req_to_cache(req_wrapper.req, id);
            $display("%s[Core %0d]    Send #%0d ",test_type_str,id,i);
        end
    endtask

    task automatic async_random_core_access_send(
        input int unsigned n_access
    );
        for (int i = 0; i < NumPorts; i++) begin
            int j = i;  // Make a local copy of loop variable
            fork
                random_sends(n_access,j);
            join_none  // Join_none allows the forked processes to run concurrently
        end
    endtask

    task automatic sync_stride_core_access_read(
        input int unsigned n_access,
        input int unsigned base,
        input int unsigned stride
    );
        for (int i = 0; i < NumPorts; i++) begin
            int j = i;  // Make a local copy of loop variable
            fork
                stride_reads(n_access,j,base,stride);
            join_none  // Join_none allows the forked processes to run concurrently
        end
    endtask




    //////////////////////////////////////
    //        TestBench Main Flow       //
    //////////////////////////////////////

    initial begin
        init_core_req();
        @(posedge rst_n);
        @(posedge clk);
        repeat(100) begin
            cycle_start(); cycle_end();
        end

        //Test with contigouse reads
        sync_stride_core_access_read(NumTest, 0, 8);
        monitor_wait_all_resp(NumTest);
        monitor_log_queue_info();
        scoreboard_check_correctness();

        //Test with random access
        scoreboard_reset();
        async_random_core_access_send(NumTest);
        monitor_wait_all_resp(NumTest);
        monitor_log_queue_info();
        scoreboard_check_correctness();

        $display("*************************************************************");
        $display("                       TEST PASSED !                         ");
        $display("*************************************************************");
        $finish;
    end

endmodule : tb_flamingo_spatz_cache_ctrl
