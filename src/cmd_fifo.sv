// =============================================================
// Command FIFO: stores {instr_id, payload_len}
// Width = 16 bits (simplified)
// =============================================================
module cmd_fifo #(
    parameter int DEPTH = 32
)(
    input  logic        clk,
    input  logic        rst,

    // Write interface
    input  logic        wr_en,
    input  logic [3:0]  instr,
    input  logic [7:0]  payload_len,
    output logic        full,
    output logic [8:0]  free,

    // Read interface
    input  logic        rd_en,
    output logic [3:0]  out_instr,
    output logic [7:0]  out_len,
    output logic        empty
);

    // internal packed word
    logic [15:0] wr_word;
    logic [15:0] rd_word;

    assign wr_word = {4'h0, instr, payload_len}; // upper nibble unused

    // unpack on read
    assign out_instr = rd_word[11:8];
    assign out_len   = rd_word[7:0];

    sync_fifo #(
        .WIDTH(16),
        .DEPTH(DEPTH)
    ) fifo_i (
        .clk(clk),
        .rst(rst),

        .wr_en(wr_en),
        .wr_data(wr_word),
        .full(full),

        .rd_en(rd_en),
        .rd_data(rd_word),
        .empty(empty),

        .free(free)
    );

endmodule
