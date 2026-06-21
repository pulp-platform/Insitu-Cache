// Simple 1-cycle SRAM model for the forwarding-buffer unit testbench.
//
// Layout matches what the buffer talks to downstream:
//   - 1-cycle read latency.  Read data is registered.
//   - Byte-mask write, single port (read OR write per cycle).
//   - On reset, every row N is initialised to {NumWordsPerLine{N+1}}
//     so the testbench can tell "this came from SRAM (refill)" from
//     "this came from the buffer's write data" by inspection.
//
// This is intentionally simpler than `pseudo_dual_port_*` -- the buffer
// only needs to see a single SRAM-like interface for read-back.

module sram_model #(
    parameter int unsigned Depth           = 64,
    parameter int unsigned NumWordsPerLine = 4,
    parameter int unsigned WordWidth       = 32,
    parameter int unsigned ByteWidth       = 8,
    localparam int unsigned DataWidth = WordWidth * NumWordsPerLine,
    localparam int unsigned MaskBits  = DataWidth / ByteWidth,
    localparam type addr_t = logic [$clog2(Depth)-1:0],
    localparam type data_t = logic [DataWidth-1:0],
    localparam type mask_t = logic [MaskBits-1:0]
)(
    input  logic    clk_i,
    input  logic    rst_ni,

    // Read port
    input  logic    rd_req_i,
    input  addr_t   rd_addr_i,
    output data_t   rd_data_o,
    output logic    rd_valid_o,

    // Write port (byte mask)
    input  logic    wr_req_i,
    input  addr_t   wr_addr_i,
    input  data_t   wr_data_i,
    input  mask_t   wr_mask_i
);

    data_t mem [Depth];

    // Combined storage process so vopt doesn't see `mem` driven from
    // multiple always blocks: reset-time initialisation, 1-cycle read
    // and byte-masked write all live here.
    //
    // Init pattern: row N = {NumWordsPerLine{N + 1}}.  +1 so row 0 is non-zero.
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            for (int i = 0; i < Depth; i++) begin
                for (int w = 0; w < NumWordsPerLine; w++) begin
                    mem[i][w*WordWidth +: WordWidth] <= WordWidth'(i + 1);
                end
            end
            rd_data_o  <= '0;
            rd_valid_o <= 1'b0;
        end else begin
            // Read
            rd_valid_o <= rd_req_i;
            if (rd_req_i)
                rd_data_o <= mem[rd_addr_i];
            // Byte-masked write
            if (wr_req_i) begin
                for (int b = 0; b < MaskBits; b++) begin
                    if (wr_mask_i[b])
                        mem[wr_addr_i][b*ByteWidth +: ByteWidth] <=
                            wr_data_i[b*ByteWidth +: ByteWidth];
                end
            end
        end
    end

`ifndef SYNTHESIS
    // Single-port: read and write same cycle = configuration error in this TB.
    sram_no_simul_rw: assert property (
        @(posedge clk_i) disable iff (!rst_ni)
        !(rd_req_i && wr_req_i)
    ) else $error("sram_model: simultaneous rd/wr on single-port memory");
`endif

endmodule
