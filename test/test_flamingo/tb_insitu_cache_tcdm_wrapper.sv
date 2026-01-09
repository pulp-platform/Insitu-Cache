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

module tb_insitu_cache_tcdm_wrapper#(
    /* DUT Model Parameters */
    /// Address width of both upstream narrow request and downstream wide request
    parameter int unsigned ReqAddrWidth                                         = 32,
    /// Word width of cache line (512b default)
    parameter int unsigned CacheLineWidth                                       = 512,
    /// Number of Cache entries
    parameter int unsigned NumCacheEntry                                        = `NUM_CACHE_LINE,
    /// Number of Associatity
    parameter int unsigned SetAssociativity                                     = `NUM_CACHE_ASSO,
    /// If Use Dual-Port RF
    parameter bit          UseDualPortRF                                        = `USE_DUAL_PORT_RF,
    /// If Use Pseudo-Dual Banks
    parameter bit          UsePseudoDualBanks                                   = `USE_PSEUDO_DUAL_BANK,
    /// Number of Pseudo-Dual Banks
    parameter int unsigned NumPseudoDualBanks                                   = `NUM_CACHE_BANK_FACTOR,
    /// Width of word (granularity of non-blocking write)
    parameter int unsigned WordWidth                                            = `CACHE_WORD_WIDTH,
    /// Whether the cache is in Write-Through mode
    parameter bit          WriteThroughMode                                     = `WRITE_THROUGH_MODE,
    /// Whether using conventional cache for testing
    parameter bit          UseConventionalCache                                 = `USE_CONVENTIONAL_CACHE,
    /// Whether using bypass cache for testing
    parameter bit          UseBypassCache                                       = `USE_BYPASS_CACHE,
    /// Number of MSHR entries 
    parameter int unsigned NumMSHREntry                                         = `NUM_MSHR_ENTRY,
    /// Number of Write Buffer entries 
    parameter int unsigned NumWrtbfEntry                                        = `NUM_WRITE_BUFFER_ENTRY,
    /// Number of Latency of Offchip Link
    parameter int unsigned NumOffchipLatency                                    = `OFFCHIP_LATENCY,
    /// Width of Access MetaData
    parameter int unsigned AccessMetaWidth                                      = `ACCESS_META_WIDTH,
    /// Counter cache line life cycle information for questa-sim.
    parameter int unsigned LogLifeCycle                                         = `LOG_LIFE_CYCLE,


    /* Test Parameters */
    parameter int unsigned NumTest                                              = 1000,
    parameter int          TrafficLimit                                         = `TRAFFIC_LIMIT,

    // Dependent parameter, do not override. Depth of cache bank.
    localparam int unsigned CacheBankDepth                                      = NumCacheEntry/SetAssociativity,
    // Dependent parameter, do not override. Number of data tcdm banks.
    localparam int unsigned NumDataBank                                         = SetAssociativity*NumPseudoDualBanks*(CacheLineWidth/WordWidth),
    // Dependent parameter, do not override. Number of meta tcdm banks.
    localparam int unsigned NumMetaBank                                         = SetAssociativity*NumPseudoDualBanks,
    // Dependent parameter, do not override. Address type.
    localparam type tcdm_bank_addr_t                                            = logic [$clog2(CacheBankDepth)-$clog2(NumPseudoDualBanks)-1:0],
    // Dependent parameter, do not override. Address type.
    localparam type addr_t                                                      = logic [ReqAddrWidth-1:0],
    // Dependent parameter, do not override. Narrow word type.
    localparam type data_t                                                      = logic [CacheLineWidth-1:0],
    // Dependent parameter, do not override. Byte mask type.
    localparam type mask_t                                                      = logic [CacheLineWidth/8-1:0],
    // Dependent parameter, do not override. Byte strb type.
    localparam type strb_t                                                      = logic [CacheLineWidth/8-1:0],
    // Dependent parameter, do not override. set ptr type.
    localparam type way_ptr_t                                                   = logic [$clog2(SetAssociativity)-1:0],
    // Dependent parameter, do not override. bank depth ptr type.
    localparam type cache_bank_depth_ptr_t                                      = logic [$clog2(CacheBankDepth)-1:0],
    /// Dependent parameter, do not override. word type
    localparam type tcdm_meta_data_t                                            = logic [63:0],
    // Dependent parameter, do not override. Downstream request payload.
    localparam type downstream_info_t                                           = struct packed {logic for_write_pend; cache_bank_depth_ptr_t depth; way_ptr_t way;}
);

    localparam time ClkPeriod               = 1ns;
    localparam time ApplTime                = 0.2ns;
    localparam time TestTime                = 0.8ns;
    localparam type TestType_t              = enum logic[1:0] { TEST_RANDOM_READ_WRITE = '0, TEST_ALL_READS, TEST_ALL_WRITES};
    localparam TestType_t test_type         = TEST_RANDOM_READ_WRITE;
    localparam type TestPattern_t           = enum logic[1:0] { PATTERN_RANDOM = '0, PATTERN_STREAM, PATTERN_SPARSE, PATTERN_TRACES};
    localparam TestPattern_t test_pattern   = `TEST_PATTERN;


    //////////////////////////////////////
    //        Types Definition          //
    //////////////////////////////////////

    typedef logic [AccessMetaWidth-1:0]                     upstream_info_t;

    typedef logic [WordWidth-1:0]                           cache_word_t;
    typedef cache_word_t [CacheLineWidth/WordWidth-1:0]     cache_data_in_words_t;

    typedef struct packed {
        addr_t                                              addr;
        upstream_info_t                                     info;
        logic                                               write;
        data_t                                              wdata;
        mask_t                                              wmask;
        strb_t                                              wstrb;
    } upstream_req_t;

    typedef struct packed {
        logic                                               write;
        data_t                                              data;
        upstream_info_t                                     info;
    } upstream_resp_t;

    typedef struct packed {
        addr_t                                              addr;
        downstream_info_t                                   info;
        logic                                               write;
        data_t                                              wdata;
        mask_t                                              wmask;
        strb_t                                              wstrb;
    } downstream_req_t;

    typedef struct packed {
        logic                                               write;
        data_t                                              data;
        downstream_info_t                                   info;
    } downstream_resp_t;

    `AXI_TYPEDEF_ALL(monitor_axi, addr_t, upstream_info_t, data_t, strb_t, logic)
    `AXI_TYPEDEF_ALL(dram_axi, addr_t, downstream_info_t, data_t, strb_t, logic)

    /////////////////////////////////////
    //        Function Utility         //
    /////////////////////////////////////

    function automatic void mask_to_strb(input mask_t mask, output strb_t strb);
        strb = mask;
    endfunction

    function automatic void simplify_data(input data_t data_in, output data_t data_out);
        automatic cache_data_in_words_t _data_in = data_in;
        automatic cache_data_in_words_t _data_out = '0;
        for (int i = 0; i < CacheLineWidth/WordWidth ; i++) begin
            automatic cache_word_t _word = _data_in[i];

            for (int j = 0; j < WordWidth; j++) begin
                if (j< WordWidth - 8) begin
                    _word[j] = 1'b0;
                end
            end

            _data_out[i] = _word;
        end
        data_out = _data_out;
    endfunction

    //////////////////////////////////////
    //        Signal Definition         //
    //////////////////////////////////////

    logic  clk, rst_n;

    //cache sync ctrl
    logic                                                   cache_sync_valid;
    logic                                                   cache_sync_ready;
    logic [1:0]                                             cache_sync_insn;
    tcdm_bank_addr_t                                        cache_part_for_SPM;

    //upstream req/resp
    logic                                                   upstream_req_valid;
    logic                                                   upstream_req_ready;
    upstream_req_t                                          upstream_req;
    logic                                                   upstream_resp_valid;
    logic                                                   upstream_resp_ready;
    upstream_resp_t                                         upstream_resp;

    //Monitor AXI
    monitor_axi_req_t                                       monitor_axi_req;
    monitor_axi_resp_t                                      monitor_axi_resp;
    AXI_BUS_DV #(
        .AXI_ADDR_WIDTH ( ReqAddrWidth ),
        .AXI_DATA_WIDTH ( CacheLineWidth ),
        .AXI_ID_WIDTH   ( $bits(upstream_info_t)  ),
        .AXI_USER_WIDTH ( 1 )
    ) axi_mon_dv(clk);
    `AXI_ASSIGN_FROM_REQ(axi_mon_dv, monitor_axi_req)
    `AXI_ASSIGN_FROM_RESP(axi_mon_dv, monitor_axi_resp)


    //downstream req/resp
    logic                                                   downstream_req_valid;
    logic                                                   downstream_req_ready;
    downstream_req_t                                        downstream_req;
    logic                                                   downstream_resp_valid;
    logic                                                   downstream_resp_ready;
    downstream_resp_t                                       downstream_resp;

    //DRAM AXI
    dram_axi_req_t                                          dram_axi_req;
    dram_axi_resp_t                                         dram_axi_resp;
    dram_axi_req_t                                          dram_axi_offchip_req;
    dram_axi_resp_t                                         dram_axi_offchip_resp;

    /// Meta Banks
    logic             [NumMetaBank-1:0]                     tcdm_meta_bank_req;
    logic             [NumMetaBank-1:0]                     tcdm_meta_bank_we;
    tcdm_bank_addr_t  [NumMetaBank-1:0]                     tcdm_meta_bank_addr;
    tcdm_meta_data_t  [NumMetaBank-1:0]                     tcdm_meta_bank_wdata;
    logic             [NumMetaBank-1:0]                     tcdm_meta_bank_be;
    tcdm_meta_data_t  [NumMetaBank-1:0]                     tcdm_meta_bank_rdata;

    /// Data Banks
    logic             [NumDataBank-1:0]                     tcdm_data_bank_req;
    logic             [NumDataBank-1:0]                     tcdm_data_bank_we;
    tcdm_bank_addr_t  [NumDataBank-1:0]                     tcdm_data_bank_addr;
    cache_word_t      [NumDataBank-1:0]                     tcdm_data_bank_wdata;
    logic             [NumDataBank-1:0][WordWidth/8-1:0]     tcdm_data_bank_be;
    cache_word_t      [NumDataBank-1:0]                     tcdm_data_bank_rdata;

    /// Bank Grant for Cache
    logic             [NumDataBank-1:0]                     tcdm_data_bank_gnt;


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
    insitu_cache_tcdm_wrapper_partitionable_flushable #(
        .ReqAddrWidth           (ReqAddrWidth),
        .info_t                 (upstream_info_t),
        .CacheLineWidth         (CacheLineWidth),
        .NumCacheEntry          (NumCacheEntry),
        .SetAssociativity       (SetAssociativity),
        .NumPseudoDualBanks     (NumPseudoDualBanks),
        .WriteThroughMode       (WriteThroughMode),
        .WordWidth              (WordWidth)
    ) i_insitu_cache_tcdm_wrapper (
        .clk_i                  (clk                  ),
        .rst_ni                 (rst_n                ),

        .cache_sync_valid_i     (cache_sync_valid     ),
        .cache_sync_ready_o     (cache_sync_ready     ),
        .cache_sync_insn_i      (cache_sync_insn      ),

        .bank_depth_for_SPM_i   (cache_part_for_SPM),

        .upstream_req_valid_i   (upstream_req_valid   ),
        .upstream_req_ready_o   (upstream_req_ready   ),
        .upstream_req_addr_i    (upstream_req.addr    ),
        .upstream_req_info_i    (upstream_req.info    ),
        .upstream_req_write_i   (upstream_req.write   ),
        .upstream_req_wdata_i   (upstream_req.wdata   ),
        .upstream_req_wmask_i   (upstream_req.wmask   ),

        .upstream_resp_valid_o  (upstream_resp_valid  ),
        .upstream_resp_ready_i  (upstream_resp_ready  ),
        .upstream_resp_write_o  (upstream_resp.write  ),
        .upstream_resp_data_o   (upstream_resp.data   ),
        .upstream_resp_info_o   (upstream_resp.info   ),

        .downstream_req_valid_o (downstream_req_valid ),
        .downstream_req_ready_i (downstream_req_ready ),
        .downstream_req_addr_o  (downstream_req.addr  ),
        .downstream_req_info_o  (downstream_req.info  ),
        .downstream_req_write_o (downstream_req.write ),
        .downstream_req_wdata_o (downstream_req.wdata ),
        .downstream_req_wmask_o (downstream_req.wmask ),

        .downstream_resp_valid_i(downstream_resp_valid),
        .downstream_resp_ready_o(downstream_resp_ready),
        .downstream_resp_data_i (downstream_resp.data ),
        .downstream_resp_info_i (downstream_resp.info ),
        .downstream_resp_write_i(downstream_resp.write),

        .tcdm_meta_bank_req_o   (tcdm_meta_bank_req),
        .tcdm_meta_bank_we_o    (tcdm_meta_bank_we),
        .tcdm_meta_bank_addr_o  (tcdm_meta_bank_addr),
        .tcdm_meta_bank_wdata_o (tcdm_meta_bank_wdata),
        .tcdm_meta_bank_be_o    (tcdm_meta_bank_be),
        .tcdm_meta_bank_rdata_i (tcdm_meta_bank_rdata),

        .tcdm_data_bank_req_o   (tcdm_data_bank_req),
        .tcdm_data_bank_we_o    (tcdm_data_bank_we),
        .tcdm_data_bank_addr_o  (tcdm_data_bank_addr),
        .tcdm_data_bank_wdata_o (tcdm_data_bank_wdata),
        .tcdm_data_bank_be_o    (tcdm_data_bank_be),
        .tcdm_data_bank_rdata_i (tcdm_data_bank_rdata),

        .tcdm_data_bank_gnt_i   (tcdm_data_bank_gnt)
    );

    always_comb begin
        mask_to_strb(downstream_req.wmask, downstream_req.wstrb);
    end

    /*****************/
    /*  TCDM Banks   */
    /*****************/
    for (genvar i = 0; i < NumMetaBank; i++) begin : gen_meta_banks
        tc_sram #(
            .NumWords(CacheBankDepth/NumPseudoDualBanks),
            .DataWidth($bits(tcdm_meta_data_t)),
            .ByteWidth($bits(tcdm_meta_data_t)),
            .NumPorts(1),
            .Latency(1),
            .SimInit("zeros")
        ) i_meta_bank (
            .clk_i  (clk                    ),
            .rst_ni (rst_n                  ),
            .req_i  (tcdm_meta_bank_req[i]  ),
            .we_i   (tcdm_meta_bank_we[i]   ),
            .addr_i (tcdm_meta_bank_addr[i] ),
            .wdata_i(tcdm_meta_bank_wdata[i]),
            .be_i   (tcdm_meta_bank_be[i]   ),
            .rdata_o(tcdm_meta_bank_rdata[i])
        );
    end

    for (genvar i = 0; i < NumDataBank; i++) begin : gen_data_banks
        tc_sram #(
            .NumWords(CacheBankDepth/NumPseudoDualBanks),
            .DataWidth(WordWidth),
            .ByteWidth(8),
            .NumPorts(1),
            .Latency(1),
            .SimInit("zeros")
        ) i_data_bank (
            .clk_i  (clk                    ),
            .rst_ni (rst_n                  ),
            .req_i  (tcdm_data_bank_req[i]  ),
            .we_i   (tcdm_data_bank_we[i]   ),
            .addr_i (tcdm_data_bank_addr[i] ),
            .wdata_i(tcdm_data_bank_wdata[i]),
            .be_i   (tcdm_data_bank_be[i]   ),
            .rdata_o(tcdm_data_bank_rdata[i])
        );

        assign tcdm_data_bank_gnt[i] = 1'b1;
    end

    /*****************/
    /*  Montior AXI  */
    /*****************/

    logic                                                   mon_upstream_req_valid;
    logic                                                   mon_upstream_req_ready;
    upstream_req_t                                          mon_upstream_req;
    logic                                                   mon_upstream_resp_valid;
    logic                                                   mon_upstream_resp_ready;
    upstream_resp_t                                         mon_upstream_resp;

    upstream_info_t                                         mon_order_cnt;
    upstream_resp_t                                         mon_upstream_resp_queue[$];

    assign mon_upstream_req_valid = upstream_req_valid;
    assign mon_upstream_req_ready = upstream_req_ready;
    assign mon_upstream_req = upstream_req;

    always_ff @( posedge clk ) begin
        automatic int find_out = 0;
        automatic int i;
        if (~rst_n) begin
            mon_order_cnt = '0;
            mon_upstream_resp_valid = '0;
            mon_upstream_resp_ready = '0;
            mon_upstream_resp = '0;
        end
        // insert
        if (upstream_resp_valid & upstream_resp_ready) begin
            mon_upstream_resp_queue.push_back(upstream_resp);
        end
        // find resp
        for (i = 0; i<mon_upstream_resp_queue.size(); i++) begin
            // if (mon_upstream_resp_queue[i].info < mon_order_cnt) begin
            //     $fatal(1,"[Insitu-Cache] Info Response Error: check=%0d, bar=%0d",mon_upstream_resp_queue[i].info,mon_order_cnt);
            // end
            if (mon_upstream_resp_queue[i].info == mon_order_cnt) begin
                find_out = 1;
                mon_upstream_resp = mon_upstream_resp_queue[i];
                break;
            end
        end
        
        if (find_out) begin
            mon_upstream_resp_valid = 1'b1;
            mon_upstream_resp_ready = 1'b1;
            mon_upstream_resp_queue.delete(i);
            mon_order_cnt = mon_order_cnt + 1'b1;
        end else begin
            mon_upstream_resp_valid = '0;
            mon_upstream_resp_ready = '0;
            mon_upstream_resp = '0;
        end
    end

    assign monitor_axi_req.aw_valid = mon_upstream_req_valid & mon_upstream_req.write;
    assign monitor_axi_req.ar_valid = mon_upstream_req_valid & ~mon_upstream_req.write;
    assign monitor_axi_req.w_valid  = mon_upstream_req_valid & mon_upstream_req.write;

    assign monitor_axi_req.b_ready = mon_upstream_resp_ready & mon_upstream_resp.write;
    assign monitor_axi_req.r_ready = mon_upstream_resp_ready & ~mon_upstream_resp.write;


    assign monitor_axi_resp.aw_ready = mon_upstream_req_ready & mon_upstream_req.write;
    assign monitor_axi_resp.ar_ready = mon_upstream_req_ready & ~mon_upstream_req.write;
    assign monitor_axi_resp.w_ready  = mon_upstream_req_ready & mon_upstream_req.write;

    assign monitor_axi_resp.b_valid = mon_upstream_resp_valid & mon_upstream_resp.write;
    assign monitor_axi_resp.r_valid = mon_upstream_resp_valid & ~mon_upstream_resp.write;


    `AXI_AW_ASSIGN_FROM(mon_upstream_req, monitor_axi_req.aw, $clog2(CacheLineWidth))
    `AXI_AR_ASSIGN_FROM(mon_upstream_req, monitor_axi_req.ar, $clog2(CacheLineWidth))
    `AXI_W_ASSIGN_FROM(mon_upstream_req, monitor_axi_req.w, $clog2(CacheLineWidth))

    `AXI_R_ASSIGN_FROM(mon_upstream_resp, monitor_axi_resp.r, $clog2(CacheLineWidth))
    `AXI_B_ASSIGN_FROM(mon_upstream_resp, monitor_axi_resp.b, $clog2(CacheLineWidth))



    /**************/
    /*  DRAM AXI  */
    /**************/
    cache_to_axi #(
        .CacheLineWidth(CacheLineWidth), 
        .cache_req_t(downstream_req_t), 
        .cache_resp_t(downstream_resp_t), 
        .axi_req_t(dram_axi_req_t), 
        .axi_resp_t(dram_axi_resp_t)
    ) i_cache_to_axi (
        .clk_i             (clk                  ),
        .rst_ni            (rst_n                ),
        .cache_req_valid_i (downstream_req_valid ),
        .cache_req_ready_o (downstream_req_ready ),
        .cache_req_i       (downstream_req       ),
        .cache_resp_valid_o(downstream_resp_valid),
        .cache_resp_ready_i(downstream_resp_ready),
        .cache_resp_o      (downstream_resp      ),
        .axi_req_o         (dram_axi_req         ),
        .axi_resp_i        (dram_axi_resp        )
    );

    /*****************************/
    /*  Offchip Latency Modeling */
    /*****************************/

    axi_multicut #(
        .NoCuts    (NumOffchipLatency),
        .aw_chan_t (dram_axi_aw_chan_t),
        .w_chan_t  (dram_axi_w_chan_t),
        .b_chan_t  (dram_axi_b_chan_t),
        .ar_chan_t (dram_axi_ar_chan_t),
        .r_chan_t  (dram_axi_r_chan_t),
        .axi_req_t (dram_axi_req_t),
        .axi_resp_t(dram_axi_resp_t)
    ) i_offchip_latency_insert (
        .clk_i(clk),
        .rst_ni(rst_n),
        .slv_req_i (dram_axi_req ),
        .slv_resp_o(dram_axi_resp),
        .mst_req_o (dram_axi_offchip_req ),
        .mst_resp_i(dram_axi_offchip_resp)
    );


if (WriteThroughMode) begin

    dram_axi_req_t      dram_axi_latency_req;
    dram_axi_resp_t     dram_axi_latency_resp;

    axi_multicut #(
        .NoCuts    (32),
        .aw_chan_t (dram_axi_aw_chan_t),
        .w_chan_t  (dram_axi_w_chan_t),
        .b_chan_t  (dram_axi_b_chan_t),
        .ar_chan_t (dram_axi_ar_chan_t),
        .r_chan_t  (dram_axi_r_chan_t),
        .axi_req_t (dram_axi_req_t),
        .axi_resp_t(dram_axi_resp_t)
    ) i_model_DRAM_lat (
        .clk_i(clk),
        .rst_ni(rst_n),
        .slv_req_i (dram_axi_offchip_req ),
        .slv_resp_o(dram_axi_offchip_resp),
        .mst_req_o (dram_axi_latency_req ),
        .mst_resp_i(dram_axi_latency_resp)
    );


    axi_sim_mem #(
        .AddrWidth(ReqAddrWidth),
        .DataWidth(CacheLineWidth),
        .IdWidth($bits(downstream_info_t)),
        .UserWidth(1),
        .axi_req_t(dram_axi_req_t),
        .axi_rsp_t(dram_axi_resp_t),
        .ApplDelay(ApplTime),
        .AcqDelay(TestTime)
    ) i_axi_sim_mem (
        .clk_i(clk),
        .rst_ni(rst_n),
        .axi_req_i (dram_axi_latency_req ),
        .axi_rsp_o (dram_axi_latency_resp),
        .mon_w_valid_o (/*open*/),
        .mon_w_addr_o (/*open*/),
        .mon_w_data_o (/*open*/),
        .mon_w_id_o (/*open*/),
        .mon_w_user_o (/*open*/),
        .mon_w_beat_count_o (/*open*/),
        .mon_w_last_o (/*open*/),
        .mon_r_valid_o (/*open*/),
        .mon_r_addr_o (/*open*/),
        .mon_r_data_o (/*open*/),
        .mon_r_id_o (/*open*/),
        .mon_r_user_o (/*open*/),
        .mon_r_beat_count_o (/*open*/),
        .mon_r_last_o (/*open*/)
    );

end else begin

    dram_sim_engine #(.ClkPeriod(1)) i_dram_sim_engine (.clk_i(clk), .rst_ni(rst_n));

    axi_dram_sim #(
        .AxiAddrWidth(ReqAddrWidth),
        .AxiDataWidth(CacheLineWidth),
        .AxiIdWidth  ($bits(downstream_info_t)),
        .AxiUserWidth(1),
        .DRAMType    ("DDR4"),
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
        .axi_req_i (dram_axi_offchip_req ),
        .axi_resp_o(dram_axi_offchip_resp)
    );

end

    /////////////////////////////
    //        Scoreboard       //
    /////////////////////////////

    typedef axi_test::axi_scoreboard #(
        // AXI interface parameters
        .AW ( ReqAddrWidth ),
        .DW ( CacheLineWidth ),
        .IW ( $bits(upstream_info_t) ),
        .UW ( 1 ),
        // Stimuli application and test time
        .TT ( TestTime )
    ) axi_scoreboard_master_t;

    axi_scoreboard_master_t axi_scoreboard_master = new(axi_mon_dv);




    /////////////////////////
    //        Driver       //
    /////////////////////////

    class upstream_req_wrapper;
        rand upstream_req_t req;

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

    task automatic init_driver();
        upstream_req_valid  = '0;
        upstream_req        = '0;
        upstream_resp_ready = '0;
        cache_sync_valid    = '0;
        cache_sync_insn     = '0;
        cache_part_for_SPM  = '0;
    endtask

    task automatic cache_sync_init();
        cache_sync_insn = 2'b11;
        cache_sync_valid = 1'b1;
        cycle_start();
        while (cache_sync_ready != 1) begin cycle_end(); cycle_start(); end
        cycle_end();
        cache_sync_insn = '0;
        cache_sync_valid = '0;
        repeat(10) begin cycle_end(); cycle_start(); end
    endtask

    task automatic cache_sync_flu_inv();
        cache_sync_insn = 2'b00;
        cache_sync_valid = 1'b1;
        cycle_start();
        while (cache_sync_ready != 1) begin cycle_end(); cycle_start(); end
        cycle_end();
        cache_sync_insn = '0;
        cache_sync_valid = '0;
        repeat(10) begin cycle_end(); cycle_start(); end
    endtask

    task automatic set_SPM_part(
        tcdm_bank_addr_t part
    );
        cache_part_for_SPM = part;
        repeat(10) begin cycle_end(); cycle_start(); end
    endtask

    task send_req_to_cache (
      input upstream_req_t req
    );
        upstream_req = req;
        mask_to_strb(upstream_req.wmask,   upstream_req.wstrb  );
        upstream_req_valid = 1'b1;
        cycle_start();
        while (upstream_req_ready != 1) begin cycle_end(); cycle_start(); end
        cycle_end();
        upstream_req = '0;
        upstream_req_valid = '0;
    endtask

    task recv_resp_from_cache (
      output upstream_resp_t resp
    );
        upstream_resp_ready = 1'b1;
        cycle_start();
        while (upstream_resp_valid != 1) begin cycle_end(); cycle_start(); end
        resp = upstream_resp;
        cycle_end();
        upstream_resp_ready = '0;
    endtask

    task automatic sends(input int unsigned n_sends);
        automatic upstream_req_wrapper req_wrapper = new;
        automatic string test_type_str = "[Random Read/Write]";
        for (int i = 0; i < n_sends; i++) begin
            automatic upstream_req_t req;
            req_wrapper.randomize();
            req = req_wrapper.req;
            req.addr = (req_wrapper.req.addr << $clog2(CacheLineWidth/8));
            mask_to_strb(req.wmask, req.wstrb);
            if (test_type == TEST_ALL_READS) begin
                req.write = '0;
                test_type_str = "[    All Reads    ]";
            end else if (test_type == TEST_ALL_WRITES) begin
                req.write = 1'b1;
                test_type_str = "[    All Writes   ]";
            end
            req.info = i;
            if (~req.write) begin
                req.wdata = '0;
                req.wmask = '0;
                req.wstrb = '0;
            end
            send_req_to_cache(req);
            // $display(">>> Send #%d to Cache: %p",i, req);
            $display("%s    Send #%0d ",test_type_str,i);
        end
    endtask

    task automatic stream_sends(input int unsigned n_sends);
        automatic upstream_req_wrapper req_wrapper = new;
        automatic string test_type_str = "[Random Read/Write]";
        for (int i = 0; i < n_sends; i++) begin
            automatic upstream_req_t req;
            req_wrapper.randomize();
            req = req_wrapper.req;
            req.addr  = (i << $clog2(CacheLineWidth/8));
            req.wmask = '1;
            mask_to_strb(req.wmask, req.wstrb);
            if (test_type == TEST_ALL_READS) begin
                req.write = '0;
                test_type_str = "[    All Reads    ]";
            end else if (test_type == TEST_ALL_WRITES) begin
                req.write = 1'b1;
                test_type_str = "[    All Writes   ]";
            end
            req.info = i;
            if (~req.write) begin
                req.wdata = '0;
                req.wmask = '0;
                req.wstrb = '0;
            end
            send_req_to_cache(req);
            // $display(">>> Send #%d to Cache: %p",i, req);
            $display("%s    Send #%0d ",test_type_str,i);
        end
    endtask : stream_sends

    function int get_send_num(input string filename);
        // Declare variables
        int file;
        string line;
        int third_number;
        int dummy1, dummy2;

        // Open the file for reading
        file = $fopen(filename, "r");
        if (file == 0) begin
          $display("Error: Could not open file %s", filename);
          return -1; // Return -1 to indicate an error
        end

        // Read the first line from the file
        if (!$feof(file)) begin
          line = "";
          void'($fgets(line, file));
          // Extract the third number from the first line
          if ($sscanf(line, "%d %d %d", dummy1, dummy2, third_number) == 3) begin
            // Close the file
            $fclose(file);
            // Return the third number
            return third_number;
          end
        end

        // Close the file if not already closed
        $fclose(file);
        // Return -1 to indicate an error if the extraction failed
        return -1;
    endfunction

    task automatic sparse_sends(input string filename, input int traffic_limit);
        // Declare variables
        automatic int                   file;
        automatic string                line;
        automatic int                   number;
        automatic int                   i = 0;

        automatic upstream_req_wrapper  req_wrapper = new;
        automatic string                test_type_str = "[Random Read/Write]";

        // Open the file for reading
        file = $fopen(filename, "r");
        if (file == 0) begin
          $display("Error: Could not open file %s", filename);
          return;
        end

        // Read the file line by line
        while (!$feof(file)) begin
          // Read a line from the file
          line = "";
          void'($fgets(line, file));

          // Extract the first number from the line
          if ($sscanf(line, "%d", number) == 1) begin
            
            /**********************************
            * generate sparse access requests *
            **********************************/
            automatic int unsigned offset;
            automatic upstream_req_t req;
            offset = number;
            req_wrapper.randomize();
            req = req_wrapper.req;

            req.addr  = (number << $clog2(WordWidth/8));
            req.wmask = '0;
            for (int b = 0; b < WordWidth/8; b++) begin
                req.wmask[(offset * (WordWidth/8)) + b] = 1'b1;
            end
            mask_to_strb(req.wmask, req.wstrb);
            if (test_type == TEST_ALL_READS) begin
                req.write = '0;
                test_type_str = "[    All Reads    ]";
            end else if (test_type == TEST_ALL_WRITES) begin
                req.write = 1'b1;
                test_type_str = "[    All Writes   ]";
            end
            req.info = i;
            if (~req.write) begin
                req.wdata = '0;
            end
            send_req_to_cache(req);

            // $display("%s    Send #%0d ",test_type_str,i);
            i++;

            //traffic limit
            if ((traffic_limit > 0) && (i>=traffic_limit)) begin
                break;
            end
          end
        end

        // Close the file
        $fclose(file);
    endtask

    task automatic traces_sends(input string filename, input int traffic_limit);
        // Declare variables
        automatic int                   file;
        automatic string                line;
        automatic int                   i = 0;

        automatic upstream_req_wrapper  req_wrapper = new;

        // Open the file for reading
        file = $fopen(filename, "r");
        if (file == 0) begin
          $display("Error: Could not open file %s", filename);
          return;
        end

        // Read the file line by line
        while (!$feof(file)) begin
          automatic addr_t addr;
          automatic logic  write;
          automatic mask_t mask;
          automatic strb_t strb;

          // Read a line from the file
          line = "";
          void'($fgets(line, file));

          // Extract the first number from the line
          if ($sscanf(line, "%h %h %b %h", addr, write, mask, strb) == 4) begin

            /**********************************
            * generate traces access requests *
            **********************************/
            automatic upstream_req_t req;
            req_wrapper.randomize();
            req = req_wrapper.req;

            req.addr  = addr;
            req.wmask = mask;
            req.wstrb = strb;
            req.write = write;
            req.info = i;
            if (~req.write) begin
                req.wdata = '0;
            end
            send_req_to_cache(req);
            i++;

            //traffic limit
            if ((traffic_limit > 0) && (i>=traffic_limit)) begin
                break;
            end
          end
        end

        // Close the file
        $fclose(file);
    endtask

    task automatic recvs(input int unsigned n_recvs);
        automatic string test_type_str = "[Random Read/Write]";
        automatic int last_progress = -1;
        if (test_type == TEST_ALL_READS) begin
            test_type_str = "[    All Reads    ]";
        end else if (test_type == TEST_ALL_WRITES) begin
            test_type_str = "[    All Writes   ]";
        end
        for (int i = 0; i < n_recvs; i++) begin
            automatic upstream_resp_t resp;
            recv_resp_from_cache(resp);
            
            if ((test_pattern == PATTERN_SPARSE) || (test_pattern == PATTERN_TRACES)) begin
                automatic int progress = (i*100)/n_recvs;
                if (progress > last_progress) begin
                    last_progress = progress;
                    if (test_pattern == PATTERN_SPARSE) begin
                        $display("%s Sparse Test Progress: %0d/100 | Nrecv = %0d | Time = %0t",test_type_str,progress,i,$time);
                    end else begin
                        $display("Traces Test Progress: %0d/100 | Nrecv = %0d | Time = %0t",progress,i,$time);
                    end

                    if (progress == 99) begin
                        $finish;
                    end
                end
            end else begin
                $display("%s                   Recive #%0d",test_type_str,i);
            end
        end
    endtask

    ////////////////////////////////
    //        Traffic Tasks       //
    ////////////////////////////////

    task simple_read_after_write_test();
        fork
            begin
                automatic upstream_req_wrapper req_wrapper = new;
                automatic upstream_req_t req;
                automatic logic [CacheLineWidth/WordWidth-1:0] word_mask;
                req.addr = '0;
                req.wstrb = '0;
                word_mask = 'b11101011;
                req.wmask = '0;
                for (int w = 0; w < CacheLineWidth/WordWidth; w++) begin
                    if (word_mask[w]) begin
                        for (int b = 0; b < WordWidth/8; b++) begin
                            req.wmask[(w * (WordWidth/8)) + b] = 1'b1;
                        end
                    end
                end

                req_wrapper.randomize();
                req.wdata = req_wrapper.req.wdata;

                req.info = '0;
                req.write = 1;
                send_req_to_cache(req);

                req.wmask = '0;
                req.wdata = '0;
                req.info = 'd1;
                req.write = 0;
                send_req_to_cache(req);
            end
            begin
                automatic upstream_resp_t resp;
                recv_resp_from_cache(resp);
                recv_resp_from_cache(resp);
            end
        join
    endtask : simple_read_after_write_test



    task ramdom_read_write_test(input int unsigned n_tests);
        fork
            sends(n_tests);
            recvs(n_tests);
        join
    endtask : ramdom_read_write_test


    task publication_test(input int unsigned n_tests);
        if (test_pattern == PATTERN_STREAM) begin
            fork
                stream_sends(n_tests);
                recvs(n_tests);
            join
        end else
        if (test_pattern == PATTERN_TRACES) begin
            automatic int num_traces = get_send_num("data.trace");
            num_traces = ((TrafficLimit <= 0) || (num_traces < TrafficLimit)) ? num_traces : TrafficLimit;
            fork
                traces_sends("data.trace", TrafficLimit);
                recvs(num_traces);
            join
        end else
        if (test_pattern == PATTERN_SPARSE) begin
            automatic int num_sparse_test = get_send_num("data.mtx")+1;
            num_sparse_test = ((TrafficLimit <= 0) || (num_sparse_test < TrafficLimit)) ? num_sparse_test : TrafficLimit;
            fork
                sparse_sends("data.mtx", TrafficLimit);
                recvs(num_sparse_test);
            join
        end else begin
            fork
                sends(n_tests);
                recvs(n_tests);
            join
        end
    endtask : publication_test

    task cache_flush_and_partition_test();
        cache_sync_init();

        set_SPM_part('d0);
        ramdom_read_write_test(NumTest);
        cache_sync_flu_inv();
        ramdom_read_write_test(NumTest);
        cache_sync_flu_inv();
        
        set_SPM_part('d10);
        ramdom_read_write_test(NumTest);
        cache_sync_flu_inv();
        ramdom_read_write_test(NumTest);
        cache_sync_flu_inv();

        set_SPM_part('d20);
        ramdom_read_write_test(NumTest);
        cache_sync_flu_inv();
        ramdom_read_write_test(NumTest);
        cache_sync_flu_inv();

        set_SPM_part('d30);
        ramdom_read_write_test(NumTest);
        cache_sync_flu_inv();
        ramdom_read_write_test(NumTest);
        cache_sync_flu_inv();

        set_SPM_part('d40);
        ramdom_read_write_test(NumTest);
        cache_sync_flu_inv();
        ramdom_read_write_test(NumTest);
        cache_sync_flu_inv();

        set_SPM_part('d50);
        ramdom_read_write_test(NumTest);
        cache_sync_flu_inv();
        ramdom_read_write_test(NumTest);
        cache_sync_flu_inv();

    endtask : cache_flush_and_partition_test

    //////////////////////////////////////
    //        TestBench Main Flow       //
    //////////////////////////////////////

    initial begin
        axi_scoreboard_master.enable_all_checks();
        axi_scoreboard_master.monitor();
        init_driver();
        $display("*************************************************************");
        if (UseConventionalCache) begin
            if (WriteThroughMode) begin
              $display("            Using Convenional Cache -- Write Through   ");
            end else begin
              $display("            Using Convenional Cache -- Write Back      ");
            end
        end else begin
            if (WriteThroughMode) begin
              $display("            Using Insitu Cache -- Write Through        ");
            end else begin
              $display("            Using Insitu Cache -- Write Back           ");
            end
        end
        $display("   -------------------------------------------------------   ");
        $display("        Cache lines = %0d  SetAsso = %0d  WordWidth = %0d",NumCacheEntry, SetAssociativity, WordWidth);
        $display("*************************************************************");
        @(posedge rst_n);
        @(posedge clk);
        cache_flush_and_partition_test();
        $display("*************************************************************");
        $display("                       TEST PASSED !                         ");
        $display("*************************************************************");
        $finish;
    end

    // initial begin
    //     #300000;
    //     $finish;
    // end

endmodule : tb_insitu_cache_tcdm_wrapper
