// =============================================================
// Generic synchronous FIFO with free-space counter
// Parametrizable width and depth
// =============================================================

`timescale 1ns/1ps


module sync_fifo #(
    parameter  WIDTH = 8,
    parameter  DEPTH = 16,
    parameter  ADDR_BITS = $clog2(DEPTH)
)(
    input  logic                 clk,
    input  logic                 rst,

    // Write
    input  logic                 wr_en,
    input  logic [WIDTH-1:0]     wr_data,
    output logic                 full,

    // Read
    input  logic                 rd_en,
    output logic [WIDTH-1:0]     rd_data,
    output logic                 empty,

    // Free space for flow control
    output logic [ADDR_BITS:0]   free      // number of free entries (0..DEPTH)
);

    // ---------------------------------------------------------
    // FIFO memory
    // ---------------------------------------------------------
    logic [WIDTH-1:0] mem [0:DEPTH-1];

    logic [ADDR_BITS-1:0] wr_ptr;
    logic [ADDR_BITS-1:0] rd_ptr;
    logic [ADDR_BITS:0]   count;  

    assign empty = (count == 0);
    assign full  = (count == DEPTH);
    assign free  = DEPTH - count;

    // read data (registered read)
    assign rd_data = mem[rd_ptr];

    // ---------------------------------------------------------
    // Sequential logic
    // ---------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            wr_ptr <= '0;
            rd_ptr <= '0;
            count  <= '0;
        end else begin

            // WRITE
            if (wr_en && !full) begin
                mem[wr_ptr] <= wr_data;
                wr_ptr <= wr_ptr + 1;
            end

            // READ
            if (rd_en && !empty) begin
                rd_ptr <= rd_ptr + 1;
            end

            // COUNT update
            unique case ({wr_en && !full, rd_en && !empty})
                2'b10: count <= count + 1; // write only
                2'b01: count <= count - 1; // read only
                default: /* no change */ ;
            endcase
        end
    end

endmodule
