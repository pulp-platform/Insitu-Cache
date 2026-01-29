// Copyright 2023 ETH Zurich and
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang  <chizhang@iis.ee.ethz.ch>, ETH Zurich
//         Diyou Shen <dishen@iis.ee.ethz.ch>, ETH Zurich
// Date: 30.Sep.2024


`include "common_cells/registers.svh"
module cachepool_cache_ctrl #(
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
  /// Width of word
  parameter int unsigned WordWidth                                        = 64,
  /// Width of strb (byte enable) for each word
  parameter int unsigned ByteWidth                                        = 8,
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

  /***************************
  * ReqRsp Bus Configuration *
  ***************************/
  /// ReqRsp data width
  parameter int unsigned  RefillDataWidth                                 = 128,
  parameter type          refill_req_t                                    = logic,
  parameter type          refill_rsp_t                                    = logic,
  parameter type          burst_req_t                                     = logic,

  /**************************
  * FIFO SRAM Configuration *
  **************************/
  parameter type          impl_in_t                                       = logic,

  /***********************
  * Dependent Parameters *
  ***********************/
  // Dependent parameter, do not override. Burst length of each visit.
  localparam int unsigned BurstLength                                     = CacheLineWidth/RefillDataWidth,
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
  localparam type         word_data_t                                     = logic [WordWidth-1:0],
  // Dependent parameter, do not override. byte strobe type.
  localparam type         strb_t                                          = logic [WordWidth/ByteWidth-1:0]
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
  input  strb_t           [NumPorts-1:0]                                  core_req_wstrb_i,

  /// spatz responses
  output logic            [NumPorts-1:0]                                  core_resp_valid_o,
  input  logic            [NumPorts-1:0]                                  core_resp_ready_i,
  output logic            [NumPorts-1:0]                                  core_resp_write_o,
  output word_data_t      [NumPorts-1:0]                                  core_resp_data_o,
  output core_meta_t      [NumPorts-1:0]                                  core_resp_meta_o,

  /// Refill port
  output refill_req_t                                                     refill_req_o,
  output burst_req_t                                                      refill_burst_o,
  output logic                                                            refill_req_valid_o,
  input  logic                                                            refill_req_ready_i,

  input  refill_rsp_t                                                     refill_rsp_i,
  input  logic                                                            refill_rsp_valid_i,
  output logic                                                            refill_rsp_ready_o,

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
  output logic            [SetAssociativity-1:0][NumDataBankPerWay-1:0][WordWidth/ByteWidth-1:0] tcdm_data_bank_be_o,
  input  word_data_t      [SetAssociativity-1:0][NumDataBankPerWay-1:0]   tcdm_data_bank_rdata_i,

  /// Data Bank Request GNT for Cache
  input  logic            [SetAssociativity-1:0][NumDataBankPerWay-1:0]   tcdm_data_bank_gnt_i

);

  //////////////////////////////////////
  //        Types Definition          //
  //////////////////////////////////////

  typedef logic [CacheLineWidth-1:0]                                      coalescing_data_t;
  typedef logic [CacheLineWidth/ByteWidth-1:0]                            coalescing_mask_t;
  typedef logic [$clog2(CacheLineWidth/WordWidth)-1:0]                    coal_ofst_t;

  typedef struct packed {
    logic                                                                 id;
    logic       [(NumPorts-1) * CoalExtFactor - 1:0]                      hitmap;
    coal_ofst_t [(NumPorts-1) * CoalExtFactor - 1:0]                      ofsts;
    core_meta_t [(NumPorts-1) * CoalExtFactor - 1:0]                      infos;
    logic                                                                 bypass_coalescer;
  } coalescing_info_t;

  typedef logic [CacheLineWidth-1:0]                                      cache_data_t;
  typedef logic [CacheLineWidth/ByteWidth-1:0]                            cache_mask_t;
  typedef logic [CacheLineWidth/8-1:0]                                    cache_strb_t;
  typedef logic [$clog2(SetAssociativity)-1:0]                            way_ptr_t;
  typedef logic [$clog2(CacheWaysEntry)-1:0]                              cache_ways_entry_ptr_t;

  typedef logic [BurstLength-1:0]                                         burst_cnt_t;

  typedef struct packed {
    logic                                                                 for_write_pend;
    cache_ways_entry_ptr_t                                                depth;
    way_ptr_t                                                             way;
  } cache_info_t;


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

  // Bypass xbar signals
  logic                                                                   bypass_xbar_req_valid;
  logic                                                                   bypass_xbar_req_ready;
  addr_t                                                                  bypass_xbar_req_addr;
  coalescing_info_t                                                       bypass_xbar_req_info;
  logic                                                                   bypass_xbar_req_write;
  coalescing_data_t                                                       bypass_xbar_req_wdata;
  coalescing_mask_t                                                       bypass_xbar_req_wmask;

  logic                                                                   bypass_xbar_resp_valid;
  logic                                                                   bypass_xbar_resp_ready;
  coalescing_data_t                                                       bypass_xbar_resp_data;
  coalescing_info_t                                                       bypass_xbar_resp_info;
  logic                                                                   bypass_xbar_resp_write;

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


  /////////////////////////////////////
  //        Function Utility         //
  /////////////////////////////////////

  function automatic cache_strb_t mask_to_strb(input cache_mask_t mask);
    automatic cache_strb_t strb;
    for (int i = 0; i < CacheLineWidth/8 ; i++) begin
        strb[i] = mask[i/(ByteWidth/8)];
    end
    return strb;
  endfunction

  coalescing_data_t bypass_pad_data;
  logic [$clog2(CacheLineWidth/WordWidth)-1:0] bypass_word_index;

  assign bypass_word_index =
    core_req_addr_i[NumPorts-1][($clog2(CacheLineWidth/8)-1):$clog2(WordWidth/8)];

  always_comb begin
    bypass_pad_data = '0;
    // Data from the core is already aligned to the byte lane indicated by strb.
    bypass_pad_data[bypass_word_index * WordWidth +: WordWidth] =
      core_req_wdata_i[NumPorts-1];
  end


  /////////////////////////////////////
  //        Instance Modules         //
  /////////////////////////////////////

  //0.Coalescer
  par_coalescer_top #(
    .ReqAddrWidth           (AddrWidth            ),
    .NumPorts               (NumPorts - 1         ), // Only spatz vlsu goes through coalescer
    .ExtFactor              (CoalExtFactor        ),
    .info_t                 (core_meta_t          ),
    .down_id_t              (logic                ),
    .UpstreamDataWidth      (WordWidth            ),
    .DownstreamDataWidth    (CacheLineWidth       ),
    .ByteWidth              (ByteWidth            )
  ) i_par_coalescer_for_spatz (
    .clk_i,
    .rst_ni,
    .id_i                   ('0                   ),

    .upstream_req_valid_i   (core_req_valid_i [NumPorts-2:0]    ),
    .upstream_req_ready_o   (core_req_ready_o [NumPorts-2:0]    ),
    .upstream_req_addr_i    (core_req_addr_i  [NumPorts-2:0]    ),
    .upstream_req_info_i    (core_req_meta_i  [NumPorts-2:0]    ),
    .upstream_req_write_i   (core_req_write_i [NumPorts-2:0]    ),
    .upstream_req_wdata_i   (core_req_wdata_i [NumPorts-2:0]    ),
    .upstream_req_wstrb_i   (core_req_wstrb_i [NumPorts-2:0]    ),

    .upstream_resp_valid_o  (core_resp_valid_o[NumPorts-2:0]    ),
    .upstream_resp_ready_i  (core_resp_ready_i[NumPorts-2:0]    ),
    .upstream_resp_write_o  (core_resp_write_o[NumPorts-2:0]    ),
    .upstream_resp_data_o   (core_resp_data_o [NumPorts-2:0]    ),
    .upstream_resp_info_o   (core_resp_meta_o [NumPorts-2:0]    ),

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

  //1.mux/demux to divide snitch and spatz req/resp
  localparam int unsigned BypassAddrOfstWidth = $clog2(CacheLineWidth/WordWidth);
  typedef logic [BypassAddrOfstWidth-1:0]    bypass_addr_ofst_t;
  typedef struct packed {
    logic [$bits(coalescing_info_t)-$bits(core_meta_t)-BypassAddrOfstWidth-1-1:0] padding;
    core_meta_t                       core_meta;
    bypass_addr_ofst_t                addr_offset;
    logic                             bypass_coalescer;
  } bypass_info_t;

  typedef union packed {
    coalescing_info_t   coalescer;
    bypass_info_t       bypass;
  } coalescer_xbar_info_union_t;

  // Ensure the packed union members overlay cleanly
  initial begin
    if ($bits(bypass_info_t) != $bits(coalescing_info_t)) begin
      $error("Width mismatch: bypass_info_t=%0d, coalescing_info_t=%0d",
            $bits(bypass_info_t), $bits(coalescing_info_t));
    end
  end

  typedef struct packed {
    addr_t              addr;
    coalescer_xbar_info_union_t   info;
    logic               write;
    coalescing_data_t   wdata;
    cache_mask_t        wmask;
  } dreq_chan_t;

  typedef struct packed {
    coalescing_data_t           data;
    coalescer_xbar_info_union_t meta;
    logic                       write;
  } drsp_chan_t;

  dreq_chan_t coalescer_req, bypass_req, bypass_xbar_req;
  drsp_chan_t bypass_xbar_resp, coalescer_resp, bypass_resp;

  assign coalescer_req = '{
    addr    : coalescing_req_addr,
    info    : coalescer_xbar_info_union_t'(coalescing_req_info),
    write   : coalescing_req_write,
    wdata   : coalescing_req_wdata,
    wmask   : coalescing_req_wmask
  };

  // bypass_info_t _binfo = '{
  //   core_meta   : core_req_meta_i[NumPorts-1],
  //   addr_offset : core_req_addr_i[NumPorts-1][($clog2(CacheLineWidth/8)-1):0]
  // };

  assign bypass_req = '{
    addr    : core_req_addr_i [NumPorts-1] & ~(CacheLineWidth/8-1),
    info    : coalescer_xbar_info_union_t'(
      bypass_info_t'{
        padding    : '0,
        core_meta  : core_req_meta_i[NumPorts-1],
        addr_offset: core_req_addr_i[NumPorts-1][($clog2(CacheLineWidth/8)-1):$clog2(WordWidth/8)], // Snitch always accept word-width aligned response
        bypass_coalescer: 1'b1
      }
    ),
    write   : core_req_write_i[NumPorts-1],
    wdata   : bypass_pad_data,
    wmask   :
      core_req_wstrb_i[NumPorts-1][WordWidth/ByteWidth-1:0] <<
        (WordWidth/ByteWidth * bypass_word_index)
  };

  assign bypass_xbar_resp = '{
    data    : bypass_xbar_resp_data,
    write   : bypass_xbar_resp_write,
    meta    : bypass_xbar_resp_info
  };

  reqrsp_xbar #(
    .NumInp           (2                ),
    .NumOut           (1                ),
    .PipeReg          (1'b0             ),
    .ExtReqPrio       (1'b0             ),
    .ExtRspPrio       (1'b0             ),
    .tcdm_req_chan_t  (dreq_chan_t      ),
    .tcdm_rsp_chan_t  (drsp_chan_t      )
  ) i_bypass_xbar (
    .clk_i            (clk_i            ),
    .rst_ni           (rst_ni           ),
    .slv_req_i        ({bypass_req                   , coalescer_req       } ),
    .slv_req_valid_i  ({core_req_valid_i[NumPorts-1] , coalescing_req_valid} ),
    .slv_req_ready_o  ({core_req_ready_o[NumPorts-1] , coalescing_req_ready} ),
    .slv_rsp_o        ({bypass_resp                  , coalescer_resp      } ),
    .slv_rsp_valid_o  ({core_resp_valid_o[NumPorts-1], coalescing_resp_valid} ),
    .slv_rsp_ready_i  ({core_resp_ready_i[NumPorts-1], coalescing_resp_ready} ),
    .slv_sel_i        ('0               ),
    .slv_rr_i         ('0               ),
    .slv_selected_o   (                 ),
    .mst_req_o        (bypass_xbar_req          ),
    .mst_req_valid_o  (bypass_xbar_req_valid    ),
    .mst_req_ready_i  (bypass_xbar_req_ready    ),
    .mst_rsp_i        (bypass_xbar_resp         ),
    .mst_rsp_valid_i  (bypass_xbar_resp_valid    ),
    .mst_rsp_ready_o  (bypass_xbar_resp_ready    ),
    .mst_sel_i        (bypass_xbar_resp_info.bypass_coalescer),
    .mst_rr_i         ('0               )
  );

    // resp xbar to coalescer
  assign coalescing_resp_data  = coalescer_resp.data;
  assign coalescing_resp_info  = coalescer_resp.meta.coalescer;
  assign coalescing_resp_write = coalescer_resp.write;
    // resp xbar to snitch
  assign core_resp_write_o[NumPorts-1]    = bypass_resp.write;
  assign core_resp_data_o [NumPorts-1]    = bypass_resp.data[bypass_resp.meta.bypass.addr_offset * WordWidth +: WordWidth];
  assign core_resp_meta_o [NumPorts-1]    = bypass_resp.meta.bypass.core_meta;

  //2.Insitu-Cache controller
  insitu_cache_tcdm_wrapper #(
    .ReqAddrWidth           (AddrWidth              ),
    .TagWidth               (TagWidth               ),
    .info_t                 (coalescing_info_t      ),
    .CacheLineWidth         (CacheLineWidth         ),
    .NumCacheEntry          (NumCacheEntry          ),
    .SetAssociativity       (SetAssociativity       ),
    .NumPseudoDualBanks     (BankFactor             ),
    .WriteThroughMode       (0                      ),
    .WordWidth              (WordWidth              ),
    .ByteWidth              (ByteWidth              ),
    .LogDebug               (1                      ),
    .LogLifeCycle           (0                      ),
    .AddrHashLength         (0                      )
  ) i_insitu_cache_tcdm_wrapper (
    .clk_i,
    .rst_ni,

    .cache_sync_valid_i     (cache_sync_valid_i     ),
    .cache_sync_ready_o     (cache_sync_ready_o     ),
    .cache_sync_insn_i      (cache_sync_insn_i      ),
    .cache_part_base_i      ('0                     ),

    .upstream_req_valid_i   (bypass_xbar_req_valid  ),
    .upstream_req_ready_o   (bypass_xbar_req_ready  ),
    .upstream_req_addr_i    (bypass_xbar_req.addr   ),
    .upstream_req_info_i    (bypass_xbar_req.info   ),
    .upstream_req_write_i   (bypass_xbar_req.write  ),
    .upstream_req_wdata_i   (bypass_xbar_req.wdata  ),
    .upstream_req_wmask_i   (bypass_xbar_req.wmask  ),

    .upstream_resp_valid_o  (bypass_xbar_resp_valid  ),
    .upstream_resp_ready_i  (bypass_xbar_resp_ready  ),
    .upstream_resp_write_o  (bypass_xbar_resp_write  ),
    .upstream_resp_data_o   (bypass_xbar_resp_data   ),
    .upstream_resp_info_o   (bypass_xbar_resp_info   ),

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

  cache_strb_t  cache_req_strb;
  assign        cache_req_strb = mask_to_strb(cache_req_wmask);

  coalescing_data_t   refill_data_d, refill_data_q;
  burst_cnt_t         refill_cnt_d,  refill_cnt_q;
  cache_info_t        refill_info_d, refill_info_q;

  coalescing_data_t   write_data_d, write_data_q;
  cache_strb_t        write_strb_d, write_strb_q;
  addr_t              write_addr_d, write_addr_q;
  burst_cnt_t         write_cnt_d,  write_cnt_q;

  `FF(refill_data_q, refill_data_d, '0)
  `FF(refill_cnt_q,  refill_cnt_d,  '0)
  `FF(refill_info_q, refill_info_d, '0)

  `FF(write_data_q, write_data_d, '0)
  `FF(write_strb_q, write_strb_d, '0)
  `FF(write_addr_q, write_addr_d, '0)
  `FF(write_cnt_q,  write_cnt_d,  '0)

  typedef enum logic [1:0] {
    // idle until response comes
    Idle,
    // have partial results
    Partial,
    // have full results, refill into cachelines
    Refill
  } refill_rsp_fsm_e;

  typedef enum logic {
    // idle until response comes
    Read,
    // sending write requests
    Write
  } refill_req_fsm_e;

  refill_rsp_fsm_e refill_rsp_state_d, refill_rsp_state_q;
  `FF(refill_rsp_state_q, refill_rsp_state_d, Idle)

  refill_req_fsm_e refill_req_state_d, refill_req_state_q;
  `FF(refill_req_state_q, refill_req_state_d, Read)

  logic write_strb_is_zero;



  if (BurstLength == 1) begin
    // Bandwidth matched, no burst needed
    always_comb begin
      refill_req_o = '{
        addr : cache_req_addr,
        write: cache_req_write,
        wdata: cache_req_wdata,
        wstrb: cache_req_strb,
        info : cache_req_info,
        default: '0
      };
      refill_req_valid_o  = cache_req_valid;
      cache_req_ready     = refill_req_ready_i;

      cache_resp_data     = refill_rsp_i.data;
      cache_resp_info     = refill_rsp_i.info;
      cache_resp_write    = refill_rsp_i.write;

      refill_rsp_ready_o  = cache_resp_ready;
      cache_resp_valid    = refill_rsp_valid_i;

      refill_burst_o      = '{
        // Send burst if the request is valid
        is_burst : cache_req_valid,
        burst_len: '0
      };
    end
  end else begin
    // Important Note:
    // Here we assume that the response from one burst should come back continuously
    // No other response should be inside during this procedure
    // No out-of-order is supported within a burst

    always_comb begin
      // Write does not support burst
      // Need to separate into multiple write requests
      refill_data_d       = refill_data_q;
      refill_cnt_d        = refill_cnt_q;
      refill_rsp_state_d  = refill_rsp_state_q;
      refill_info_d       = refill_info_q;

      refill_req_state_d  = refill_req_state_q;
      write_data_d        = write_data_q;
      write_strb_d        = write_strb_q;
      write_addr_d        = write_addr_q;
      write_cnt_d         = write_cnt_q;

      write_strb_is_zero  = 1'b0;

      /***********************/
      /***** Request FSM *****/
      /***********************/

      case (refill_req_state_q)
        Read: begin
          refill_req_o        = '0;
          refill_req_valid_o  = '0;
          cache_req_ready     = '0;
          refill_burst_o      = '0;


          // Judge if it is a read or write request
          // If read: send out burst
          // If write: send out single req and switch mode
          if (cache_req_valid) begin
            // By default, send these info for valid request
            refill_req_o = '{
              addr : cache_req_addr,
              write: cache_req_write,
              wdata: cache_req_wdata[RefillDataWidth-1    :0],
              wstrb: cache_req_strb [(RefillDataWidth/8)-1:0],
              info : cache_req_info,
              default: '0
            };

            if (cache_req_write) begin
              // Write data, we do not need the part already consumed
              write_data_d = cache_req_wdata >> RefillDataWidth;
              // Write strobe
              write_strb_d = cache_req_strb  >> (RefillDataWidth/8);
              // Write address
              write_addr_d = cache_req_addr + (RefillDataWidth/8);
              // Reset counter
              write_cnt_d  = 'b0;
              // Do we actually need this request?
              write_strb_is_zero  = (refill_req_o.wstrb == '0);
              refill_req_valid_o  = write_strb_is_zero ? 1'b0 : cache_req_valid;

              // We are not yet finished for requesting
              cache_req_ready     = 1'b0;

              if (refill_req_ready_i | write_strb_is_zero) begin
                // The first write is accepted, switch state
                refill_req_state_d = Write;
                write_cnt_d        = 1'b1;
              end

              // No burst on write
              // Maybe we need to add information here for filtering response?
              refill_burst_o      = '{
                // Send burst if the request is valid
                is_burst : cache_req_valid,
                burst_len: '0
              };
            end else begin
              // read request side
              cache_req_ready     = refill_req_ready_i;
              refill_req_valid_o  = cache_req_valid;

              refill_burst_o      = '{
                // Send burst if the request is valid
                is_burst : cache_req_valid,
                burst_len: (BurstLength-1)
              };
            end
          end
        end

        Write: begin
          // Only write request should enter this state
          refill_req_o = '{
            addr : write_addr_q,
            write: 1'b1,
            wdata: write_data_q [RefillDataWidth-1    :0],
            wstrb: write_strb_q [(RefillDataWidth/8)-1:0],
            info : cache_req_info,
            default: '0
          };
          write_strb_is_zero  = (refill_req_o.wstrb == '0);
          // No need to send write if no strb
          refill_req_valid_o  = write_strb_is_zero ? 1'b0 : cache_req_valid;
          cache_req_ready     = 1'b0;
          refill_burst_o      = '{
            // Send burst if the request is valid
            is_burst : cache_req_valid,
            burst_len: '0
          };


          if (refill_req_ready_i | write_strb_is_zero) begin
            if (write_cnt_q == BurstLength-1) begin
              // This is the last request need to send
              // We restore the state after the request is accepted
              cache_req_ready     = 1'b1;
              write_data_d        = '0;
              write_strb_d        = '0;
              write_addr_d        = '0;
              write_cnt_d         = '0;

              refill_req_state_d  = Read;
            end else begin
              // Write data, we do not need the part already consumed
              write_data_d = write_data_q >> RefillDataWidth;
              // Write strobe
              write_strb_d = write_strb_q >> (RefillDataWidth/8);
              // Write address
              write_addr_d = write_addr_q + (RefillDataWidth/8);
              // Counter
              write_cnt_d  = write_cnt_q + 1;
            end
          end
        end
      endcase

      /************************/
      /***** Response FSM *****/
      /************************/

      cache_resp_data     = refill_rsp_i.data;
      cache_resp_info     = refill_rsp_i.info;
      cache_resp_write    = refill_rsp_i.write;
      cache_resp_info     = refill_rsp_i.info;

      refill_rsp_ready_o  = cache_resp_ready;
      cache_resp_valid    = refill_rsp_valid_i;

      case (refill_rsp_state_q)
        Idle: begin
          if (refill_rsp_valid_i) begin
            // We got a valid response, is it from write?
            if (refill_rsp_i.write == 1'b0) begin
              // Count how much we got
              refill_cnt_d        = 1'b1;
              // Clear the data
              refill_data_d       = '0;
              // Indicate a read response, fill in the data in the top region
              refill_data_d[(CacheLineWidth-1)-:RefillDataWidth] = refill_rsp_i.data;
              // Acknowledge the acceptance of the data
              refill_rsp_ready_o  = 1'b1;
              // Fill the refill info
              refill_info_d       = refill_rsp_i.info;
              // Response not yet ready
              cache_resp_valid    = 1'b0;
              // move to the next state
              if (refill_cnt_q == BurstLength) begin
                // This actually should never happen
                refill_rsp_state_d = Refill;
              end else begin
                refill_rsp_state_d = Partial;
              end
            end
          end
        end
        Partial: begin
          if (refill_rsp_valid_i) begin
            // We got a valid response, is it from write?
            if (refill_rsp_i.write == 1'b0) begin
              // Add counter
              refill_cnt_d        = refill_cnt_q + 1;
              // Move data to right to add new data
              refill_data_d       = refill_data_q >> RefillDataWidth;
              // Indicate a read response, fill in the data in the top region
              refill_data_d[(CacheLineWidth-1)-:RefillDataWidth] = refill_rsp_i.data;
              // Acknowledge the acceptance of the data
              refill_rsp_ready_o  = 1'b1;
              // The refill info should be the same, raise a warning if not
            `ifndef TARGET_SYNTHESIS
              if (refill_rsp_i.info != refill_info_q) begin
                $warning("[L1 D$ Ctrl] Info mismatch!");
              end
            `endif
              // Response not yet ready
              cache_resp_valid    = 1'b0;
              // move to the next state if the entire cacheline is assembled
              if (refill_cnt_q == BurstLength-1) begin
                refill_rsp_state_d = Refill;
              end
            end
          end
        end
        Refill: begin
          // The refill data
          cache_resp_data     = refill_data_q;
          // The refill info
          cache_resp_info     = refill_info_q;
          // Write response is not handled here
          cache_resp_write    = 1'b0;
          // raise the valid flag
          cache_resp_valid    = 1'b1;
          // not yet ready to accept the next response
          // TODO: In theory it can be pipelined
          refill_rsp_ready_o  = 1'b0;

          if (cache_resp_ready) begin
            // The refill is accepted
            // Clear all FF and reset the state
            if (refill_rsp_valid_i & (refill_rsp_i.write == 1'b0)) begin
              // If we already have a valid response
              // Then we can skip the Idle state to save one cycle
              // Already have first element back, count one
              refill_cnt_d        = 1'b1;
              // Clear the data
              refill_data_d       = '0;
              // Indicate a read response, fill in the data in the top region
              refill_data_d[(CacheLineWidth-1)-:RefillDataWidth] = refill_rsp_i.data;
              // Acknowledge the acceptance of the data
              refill_rsp_ready_o  = 1'b1;
              // Fill the refill info
              refill_info_d       = refill_rsp_i.info;
              // move to the next state
              refill_rsp_state_d  = Partial;
            end else begin
              refill_cnt_d      = '0;
              refill_data_d     = '0;
              refill_rsp_state_d    = Idle;
            end
          end
        end
      endcase
    end
  end

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
    assert (BurstLength >= 1)
      else $fatal(1,"ReqRsp DataWidth must be less or equal to cacheline width. Current DataWidth: %0d, CacheLineWidth: %d", RefillDataWidth, CacheLineWidth);
    // assert ($bits(axi_req_o.aw.addr) >= AddrWidth)
    //     else $fatal(1,"axi_req_o.aw.addr field width must be equal to AddrWidth. Current width: %0d, AddrWidth: %0d", $bits(axi_req_o.aw.addr), AddrWidth);
    // assert ($bits(axi_req_o.ar.addr) >= AddrWidth)
    //     else $fatal(1,"axi_req_o.ar.addr field width must be equal to AddrWidth. Current width: %0d, AddrWidth: %0d", $bits(axi_req_o.ar.addr), AddrWidth);
    // assert ($bits(axi_resp_i.r.data) == CacheLineWidth)
    //     else $fatal(1,"axi_resp_i.r.data field width must be equal to CacheLineWidth. Current width: %0d, CacheLineWidth: %0d", $bits(axi_resp_i.r.data), CacheLineWidth);
    // assert ($bits(axi_req_o.w.data) == CacheLineWidth)
    //     else $fatal(1,"axi_req_o.w.data field width must be equal to CacheLineWidth. Current width: %0d, CacheLineWidth: %0d", $bits(axi_req_o.w.data), CacheLineWidth);
  end
`endif


endmodule : cachepool_cache_ctrl
