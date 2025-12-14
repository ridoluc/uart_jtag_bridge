// =============================================================
// Payload FIFO (byte-wide FIFO)
// =============================================================
module payload_fifo #(
    parameter int DEPTH = 256
)(
    input  logic        clk,
    input  logic        rst,

    // Write interface
    input  logic        wr_en,
    input  logic [7:0]  wr_data,
    output logic        full,
    output logic [8:0]  free,     // up to 256 entries → 9 bits

    // Read interface
    input  logic        rd_en,
    output logic [7:0]  rd_data,
    output logic        empty
);

    sync_fifo #(
        .WIDTH(8),
        .DEPTH(DEPTH)
    ) fifo_i (
        .clk(clk),
        .rst(rst),

        .wr_en(wr_en),
        .wr_data(wr_data),
        .full(full),

        .rd_en(rd_en),
        .rd_data(rd_data),
        .empty(empty),

        .free(free)
    );

endmodule
