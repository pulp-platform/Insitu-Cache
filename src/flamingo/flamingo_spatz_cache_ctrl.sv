// Copyright 2023 ETH Zurich and
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 30.Sep.2024


`include "common_cells/registers.svh"
module flamingo_spatz_cache_ctrl #(
    /*************************
    * Core Access Parameters *
    *************************/
    /// Number of spatz core complex
    parameter int unsigned NumPorts                                         = 10,
    /// Coalescer Extned Factor
    parameter int unsigned CoalExtFactor                                    = 1,
    /// Meta information payload from spatz/snitch
    parameter type         core_meta_t                                      = logic[7:0],
    /// Address width of both narrow request from spatz
    parameter int unsigned AddrWidth                                        = 32,
    /// Width of word (granularity of non-blocking write)
    parameter int unsigned WordWidth                                        = 64,
    /// Width of Tag Bank data
    parameter int unsigned TagWidth                                         = 64,

    /**********************
    * Cache Configuration *
    **********************/
    /// Number of Cache entries
    parameter int unsigned NumCacheEntry                                    = 512,
    /// Word width of cache line (512b default)
    parameter int unsigned CacheLineWidth                                   = 512,
    /// Number of Associatity
    parameter int unsigned SetAssociativity                                 = 4,
    /// Number of Pseudo-Dual Banks
    parameter int unsigned BankFactor                                       = 2,

    /************************
    * AXI Bus Configuration *
    ************************/
    // AXI interface
    parameter type          axi_req_t                                       = logic,
    parameter type          axi_resp_t                                      = logic,

    /**************************
    * FIFO SRAM Configuration *
    **************************/
    parameter type          impl_in_t                                       = logic,

    /***********************
    * Dependent Parameters *
    ***********************/
    // Dependent parameter, do not override. Depth of cache bank.
    localparam int unsigned CacheWaysEntry                                  = NumCacheEntry/SetAssociativity,
    // Dependent parameter, do not override. Number of data bank per way.
    localparam int unsigned NumDataBankPerWay                               = BankFactor * (CacheLineWidth/WordWidth),
    // Dependent parameter, do not override. Number of meta bank per way.
    localparam int unsigned NumTagBankPerWay                                = BankFactor,
    // Dependent parameter, do not override. Address type.
    localparam type         addr_t                                          = logic [AddrWidth-1:0],
    // Dependent parameter, do not override. Address type.
    localparam type         tcdm_bank_addr_t                                = logic [$clog2(CacheWaysEntry)-$clog2(BankFactor)-1:0],
    // Dependent parameter, do not override. TCDM Tag type.
    localparam type         tcdm_tag_data_t                                 = logic [TagWidth-1:0],
    // Dependent parameter, do not override. word type.
    localparam type         word_data_t                                     = logic [WordWidth-1:0]
    )(
    /// Clock, positive edge triggered.
    input  logic                                                            clk_i,
    /// Reset, active low.
    input  logic                                                            rst_ni,

    /// Sync Control Signals
    input  logic                                                            cache_sync_valid_i,
    output logic                                                            cache_sync_ready_o,
    /*  00-> flush+invalidation
        01-> flush only
        10-> invalidation only
        11-> all tag initialization*/
    input  logic [1:0]                                                      cache_sync_insn_i,

    /// Cache Partitioning Signals
    input  tcdm_bank_addr_t                                                 bank_depth_for_SPM_i,

    /// spatz requests
    input  logic            [NumPorts-1:0]                                  core_req_valid_i,
    output logic            [NumPorts-1:0]                                  core_req_ready_o,
    input  addr_t           [NumPorts-1:0]                                  core_req_addr_i,
    input  core_meta_t      [NumPorts-1:0]                                  core_req_meta_i,
    input  logic            [NumPorts-1:0]                                  core_req_write_i,
    input  word_data_t      [NumPorts-1:0]                                  core_req_wdata_i,

    /// spatz responses
    output logic            [NumPorts-1:0]                                  core_resp_valid_o,
    input  logic            [NumPorts-1:0]                                  core_resp_ready_i,
    output logic            [NumPorts-1:0]                                  core_resp_write_o,
    output word_data_t      [NumPorts-1:0]                                  core_resp_data_o,
    output core_meta_t      [NumPorts-1:0]                                  core_resp_meta_o,

    /// AXI port
    output  axi_req_t                                                       axi_req_o,
    input   axi_resp_t                                                      axi_resp_i,

    /// FIFO SRAM Configuration
    input   impl_in_t       [1:0]                                           impl_i,

    /// SRAM bank interfaces
    /// Tag Banks
    output logic            [SetAssociativity-1:0][NumTagBankPerWay-1:0]    tcdm_tag_bank_req_o,
    output logic            [SetAssociativity-1:0][NumTagBankPerWay-1:0]    tcdm_tag_bank_we_o,
    output tcdm_bank_addr_t [SetAssociativity-1:0][NumTagBankPerWay-1:0]    tcdm_tag_bank_addr_o,
    output tcdm_tag_data_t  [SetAssociativity-1:0][NumTagBankPerWay-1:0]    tcdm_tag_bank_wdata_o,
    output logic            [SetAssociativity-1:0][NumTagBankPerWay-1:0]    tcdm_tag_bank_be_o,
    input  tcdm_tag_data_t  [SetAssociativity-1:0][NumTagBankPerWay-1:0]    tcdm_tag_bank_rdata_i,

    /// Data Banks
    output logic            [SetAssociativity-1:0][NumDataBankPerWay-1:0]   tcdm_data_bank_req_o,
    output logic            [SetAssociativity-1:0][NumDataBankPerWay-1:0]   tcdm_data_bank_we_o,
    output tcdm_bank_addr_t [SetAssociativity-1:0][NumDataBankPerWay-1:0]   tcdm_data_bank_addr_o,
    output word_data_t      [SetAssociativity-1:0][NumDataBankPerWay-1:0]   tcdm_data_bank_wdata_o,
    output logic            [SetAssociativity-1:0][NumDataBankPerWay-1:0]   tcdm_data_bank_be_o,
    input  word_data_t      [SetAssociativity-1:0][NumDataBankPerWay-1:0]   tcdm_data_bank_rdata_i,

    /// Data Bank Request GNT for Cache
    input  logic            [SetAssociativity-1:0][NumDataBankPerWay-1:0]   tcdm_data_bank_gnt_i

);

    //////////////////////////////////////
    //        Types Definition          //
    //////////////////////////////////////

    typedef logic [CacheLineWidth-1:0]                                      coalescing_data_t;
    typedef logic [CacheLineWidth/WordWidth-1:0]                            coalescing_mask_t;
    typedef logic [$clog2(CacheLineWidth/WordWidth)-1:0]                    coal_ofst_t;

    typedef struct packed {
        logic                                                               id;
        logic       [NumPorts * CoalExtFactor - 1:0]                        hitmap;
        coal_ofst_t [NumPorts * CoalExtFactor - 1:0]                        ofsts;
        core_meta_t [NumPorts * CoalExtFactor - 1:0]                        infos;
    } coalescing_info_t;

    typedef logic [CacheLineWidth-1:0]                                      cache_data_t;
    typedef logic [CacheLineWidth/WordWidth-1:0]                            cache_mask_t;
    typedef logic [CacheLineWidth/8-1:0]                                    cache_strb_t;
    typedef logic [$clog2(SetAssociativity)-1:0]                            way_ptr_t;
    typedef logic [$clog2(CacheWaysEntry)-1:0]                              cache_ways_entry_ptr_t;

    typedef struct packed {
        logic                                                               for_write_pend;
        cache_ways_entry_ptr_t                                              depth;
        way_ptr_t                                                           way;
    } cache_info_t;

    typedef struct packed {
        addr_t                                                              addr;
        logic                                                               info;
        logic                                                               write;
        cache_data_t                                                        wdata;
        cache_strb_t                                                        wstrb;
    } cache_noninfo_req_t;

    typedef struct packed {
        logic                                                               write;
        cache_data_t                                                        data;
        logic                                                               info;
    } cache_noninfo_resp_t;


    //////////////////////////////////////
    //        Signal Definition         //
    //////////////////////////////////////

    /// Coalesced request
    logic                                                                   coalescing_req_valid;
    logic                                                                   coalescing_req_ready;
    addr_t                                                                  coalescing_req_addr;
    coalescing_info_t                                                       coalescing_req_info;
    logic                                                                   coalescing_req_write;
    coalescing_data_t                                                       coalescing_req_wdata;
    coalescing_mask_t                                                       coalescing_req_wmask;

    /// Coalesced response
    logic                                                                   coalescing_resp_valid;
    logic                                                                   coalescing_resp_ready;
    coalescing_data_t                                                       coalescing_resp_data;
    coalescing_info_t                                                       coalescing_resp_info;
    logic                                                                   coalescing_resp_write;

    /// Cache request
    logic                                                                   cache_req_valid;
    logic                                                                   cache_req_ready;
    addr_t                                                                  cache_req_addr;
    cache_info_t                                                            cache_req_info;
    logic                                                                   cache_req_write;
    cache_data_t                                                            cache_req_wdata;
    cache_mask_t                                                            cache_req_wmask;

    /// Cache response
    logic                                                                   cache_resp_valid;
    logic                                                                   cache_resp_ready;
    cache_data_t                                                            cache_resp_data;
    cache_info_t                                                            cache_resp_info;
    logic                                                                   cache_resp_write;

    logic                                                                   cache_noninfo_req_valid;
    logic                                                                   cache_noninfo_req_ready;
    cache_noninfo_req_t                                                     cache_noninfo_req;
    logic                                                                   cache_noninfo_resp_valid;
    logic                                                                   cache_noninfo_resp_ready;
    cache_noninfo_resp_t                                                    cache_noninfo_resp;

    cache_info_t                                                            info_fifo_in;
    logic                                                                   info_fifo_full;
    logic                                                                   info_fifo_push;
    cache_info_t                                                            info_fifo_out;
    logic                                                                   info_fifo_empty;
    logic                                                                   info_fifo_pop;


    /////////////////////////////////////
    //        Function Utility         //
    /////////////////////////////////////

    function automatic cache_strb_t mask_to_strb(input cache_mask_t mask);
        automatic cache_strb_t strb;
        for (int i = 0; i < CacheLineWidth/8 ; i++) begin
            strb[i] = mask[i/(WordWidth/8)];
        end
        return strb;
    endfunction


    /////////////////////////////////////
    //        Instance Modules         //
    /////////////////////////////////////

    //1.Coalescer
    par_coalescer_top #(
        .ReqAddrWidth       (AddrWidth),
        .NumPorts           (NumPorts),
        .ExtFactor          (CoalExtFactor),
        .info_t             (core_meta_t),
        .down_id_t          (logic),
        .UpstreamDataWidth  (WordWidth),
        .DownstreamDataWidth(CacheLineWidth)
    ) i_par_coalescer_for_spatz (
        .clk_i,
        .rst_ni,
        .id_i                   ('0                   ),

        .upstream_req_valid_i   (core_req_valid_i     ),
        .upstream_req_ready_o   (core_req_ready_o     ),
        .upstream_req_addr_i    (core_req_addr_i      ),
        .upstream_req_info_i    (core_req_meta_i      ),
        .upstream_req_write_i   (core_req_write_i     ),
        .upstream_req_wdata_i   (core_req_wdata_i     ),

        .upstream_resp_valid_o  (core_resp_valid_o    ),
        .upstream_resp_ready_i  (core_resp_ready_i    ),
        .upstream_resp_write_o  (core_resp_write_o    ),
        .upstream_resp_data_o   (core_resp_data_o     ),
        .upstream_resp_info_o   (core_resp_meta_o     ),

        .downstream_req_valid_o (coalescing_req_valid ),
        .downstream_req_ready_i (coalescing_req_ready ),
        .downstream_req_addr_o  (coalescing_req_addr  ),
        .downstream_req_info_o  (coalescing_req_info  ),
        .downstream_req_write_o (coalescing_req_write ),
        .downstream_req_wdata_o (coalescing_req_wdata ),
        .downstream_req_wmask_o (coalescing_req_wmask ),

        .downstream_resp_valid_i(coalescing_resp_valid),
        .downstream_resp_ready_o(coalescing_resp_ready),
        .downstream_resp_data_i (coalescing_resp_data ),
        .downstream_resp_info_i (coalescing_resp_info ),
        .downstream_resp_write_i(coalescing_resp_write)
    );

    //2.Insitu-Cache controller
    insitu_cache_tcdm_wrapper_partitionable_flushable #(
        .ReqAddrWidth           (AddrWidth),
        .info_t                 (coalescing_info_t),
        .CacheLineWidth         (CacheLineWidth),
        .NumCacheEntry          (NumCacheEntry),
        .SetAssociativity       (SetAssociativity),
        .NumPseudoDualBanks     (BankFactor),
        .WriteThroughMode       (0),
        .WordWidth              (WordWidth)
    ) i_insitu_cache_tcdm_wrapper (
        .clk_i,
        .rst_ni,

        .cache_sync_valid_i,
        .cache_sync_ready_o,
        .cache_sync_insn_i,
        .bank_depth_for_SPM_i,

        .upstream_req_valid_i   (coalescing_req_valid   ),
        .upstream_req_ready_o   (coalescing_req_ready   ),
        .upstream_req_addr_i    (coalescing_req_addr    ),
        .upstream_req_info_i    (coalescing_req_info    ),
        .upstream_req_write_i   (coalescing_req_write   ),
        .upstream_req_wdata_i   (coalescing_req_wdata   ),
        .upstream_req_wmask_i   (coalescing_req_wmask   ),

        .upstream_resp_valid_o  (coalescing_resp_valid  ),
        .upstream_resp_ready_i  (coalescing_resp_ready  ),
        .upstream_resp_write_o  (coalescing_resp_write  ),
        .upstream_resp_data_o   (coalescing_resp_data   ),
        .upstream_resp_info_o   (coalescing_resp_info   ),

        .downstream_req_valid_o (cache_req_valid        ),
        .downstream_req_ready_i (cache_req_ready        ),
        .downstream_req_addr_o  (cache_req_addr         ),
        .downstream_req_info_o  (cache_req_info         ),
        .downstream_req_write_o (cache_req_write        ),
        .downstream_req_wdata_o (cache_req_wdata        ),
        .downstream_req_wmask_o (cache_req_wmask        ),

        .downstream_resp_valid_i(cache_resp_valid       ),
        .downstream_resp_ready_o(cache_resp_ready       ),
        .downstream_resp_data_i (cache_resp_data        ),
        .downstream_resp_info_i (cache_resp_info        ),
        .downstream_resp_write_i(cache_resp_write       ),

        .tcdm_meta_bank_req_o   (tcdm_tag_bank_req_o    ),
        .tcdm_meta_bank_we_o    (tcdm_tag_bank_we_o     ),
        .tcdm_meta_bank_addr_o  (tcdm_tag_bank_addr_o   ),
        .tcdm_meta_bank_wdata_o (tcdm_tag_bank_wdata_o  ),
        .tcdm_meta_bank_be_o    (tcdm_tag_bank_be_o     ),
        .tcdm_meta_bank_rdata_i (tcdm_tag_bank_rdata_i  ),

        .tcdm_data_bank_req_o   (tcdm_data_bank_req_o   ),
        .tcdm_data_bank_we_o    (tcdm_data_bank_we_o    ),
        .tcdm_data_bank_addr_o  (tcdm_data_bank_addr_o  ),
        .tcdm_data_bank_wdata_o (tcdm_data_bank_wdata_o ),
        .tcdm_data_bank_be_o    (tcdm_data_bank_be_o    ),
        .tcdm_data_bank_rdata_i (tcdm_data_bank_rdata_i ),

        .tcdm_data_bank_gnt_i   (tcdm_data_bank_gnt_i   )
    );

    //3.Cache -> AXI
    always_comb begin
        //Control Path
        cache_noninfo_req_valid = (info_fifo_full & ~cache_req_write)? '0 : cache_req_valid;
        cache_req_ready = (info_fifo_full & ~cache_req_write)? '0 : cache_noninfo_req_ready;

        cache_resp_valid = (info_fifo_empty & ~cache_noninfo_resp.write)? '0 : cache_noninfo_resp_valid;
        cache_noninfo_resp_ready = (info_fifo_empty & ~cache_noninfo_resp.write)? '0 : cache_resp_ready;

        info_fifo_push = cache_noninfo_req_valid & cache_noninfo_req_ready & ~cache_req_write;
        info_fifo_pop = cache_resp_valid & cache_resp_ready & ~cache_noninfo_resp.write;

        //Data path
        cache_noninfo_req.addr  = cache_req_addr;
        cache_noninfo_req.info  = '0;
        cache_noninfo_req.write = cache_req_write;
        cache_noninfo_req.wdata = cache_req_wdata;
        cache_noninfo_req.wstrb = mask_to_strb(cache_req_wmask);
        info_fifo_in = cache_req_info;

        cache_resp_data  = cache_noninfo_resp.data;
        cache_resp_info  = info_fifo_out;
        cache_resp_write = cache_noninfo_resp.write;
    end

    pseudo_dual_port_fifo #(
        .DEPTH                   (512                        ),
        .dtype                   (cache_info_t               ),
        .impl_in_t               (impl_in_t                  )
    ) i_cache_info_fifo (
        .clk_i,
        .rst_ni,
        .impl_i                  (impl_i                     ),
        .full_o                  (info_fifo_full             ),
        .empty_o                 (info_fifo_empty            ),
        .usage_o                 (/*open*/                   ),
        .data_i                  (info_fifo_in               ),
        .push_i                  (info_fifo_push             ),
        .data_o                  (info_fifo_out              ),
        .pop_i                   (info_fifo_pop              )
    );

    cache_to_axi #(
        .CacheLineWidth(CacheLineWidth),
        .cache_req_t(cache_noninfo_req_t),
        .cache_resp_t(cache_noninfo_resp_t),
        .axi_req_t(axi_req_t),
        .axi_resp_t(axi_resp_t)
    ) i_cache_to_axi (
        .clk_i,
        .rst_ni,
        .cache_req_valid_i (cache_noninfo_req_valid ),
        .cache_req_ready_o (cache_noninfo_req_ready ),
        .cache_req_i       (cache_noninfo_req       ),
        .cache_resp_valid_o(cache_noninfo_resp_valid),
        .cache_resp_ready_i(cache_noninfo_resp_ready),
        .cache_resp_o      (cache_noninfo_resp      ),
        .axi_req_o,
        .axi_resp_i
    );

    //////////////////////////////////////
    //        Parameter Assertion       //
    //////////////////////////////////////
`ifndef TARGET_SYNTHESIS
    function automatic bit is_pow2(input int unsigned x);
        return (x > 0) && ((x & (x - 1)) == 0);
    endfunction

    initial begin
        assert (NumPorts > 0)
            else $fatal(1,"NumPorts must be greater than 0. Current value: %0d", NumPorts);
        assert (CoalExtFactor > 0 && is_pow2(CoalExtFactor))
            else $fatal(1,"CoalExtFactor must be greater than 0 and a power of 2. Current value: %0d", CoalExtFactor);
        assert (AddrWidth >= 2 && is_pow2(AddrWidth))
            else $fatal(1,"AddrWidth must be greater than or equal to 2 and a power of 2. Current value: %0d", AddrWidth);
        assert (WordWidth >= 8 && is_pow2(WordWidth) && WordWidth < CacheLineWidth)
            else $fatal(1,"WordWidth must be greater than or equal to 8, a power of 2, and less than CacheLineWidth. Current value: %0d", WordWidth);
        assert (NumCacheEntry >= 2 && is_pow2(NumCacheEntry) && NumCacheEntry > (SetAssociativity * BankFactor))
            else $fatal(1,"NumCacheEntry must be greater than or equal to 2 and a power of 2, NumCacheEntry must be greater than SetAssociativity * BankFactor. Current value: %0d", NumCacheEntry);
        assert (CacheLineWidth > $bits(core_meta_t) && is_pow2(CacheLineWidth))
            else $fatal(1,"CacheLineWidth must be greater than width of core_meta_t and a power of 2. Current value: %0d", CacheLineWidth);
        assert (SetAssociativity >= 2 && is_pow2(SetAssociativity))
            else $fatal(1,"SetAssociativity must be greater than or equal to 2 and a power of 2. Current value: %0d", SetAssociativity);
        assert (BankFactor >= 2 && is_pow2(BankFactor))
            else $fatal(1,"BankFactor must be greater than or equal to 2 and a power of 2. Current value: %0d", BankFactor);
        assert ($bits(axi_req_o.aw.addr) >= AddrWidth)
            else $fatal(1,"axi_req_o.aw.addr field width must be equal to AddrWidth. Current width: %0d, AddrWidth: %0d", $bits(axi_req_o.aw.addr), AddrWidth);
        assert ($bits(axi_req_o.ar.addr) >= AddrWidth)
            else $fatal(1,"axi_req_o.ar.addr field width must be equal to AddrWidth. Current width: %0d, AddrWidth: %0d", $bits(axi_req_o.ar.addr), AddrWidth);
        assert ($bits(axi_resp_i.r.data) == CacheLineWidth)
            else $fatal(1,"axi_resp_i.r.data field width must be equal to CacheLineWidth. Current width: %0d, CacheLineWidth: %0d", $bits(axi_resp_i.r.data), CacheLineWidth);
        assert ($bits(axi_req_o.w.data) == CacheLineWidth)
            else $fatal(1,"axi_req_o.w.data field width must be equal to CacheLineWidth. Current width: %0d, CacheLineWidth: %0d", $bits(axi_req_o.w.data), CacheLineWidth);
    end
`endif


endmodule : flamingo_spatz_cache_ctrl
