`timescale 1ns/1ps

module rx_fifo #(
   parameter DEPTH = 256,
   localparam  ADDR_BITS = $clog2(DEPTH)
)(
      input  logic        clk,
      input  logic        rst,
   
      // Write interface
      input  logic [7:0]  rx_data,
      input  logic        rx_valid,
      output logic        rx_ready,

      // Read interface
      input  logic        rd_en,
      output logic [7:0]  rd_data,
      output logic        empty,
      output logic [ADDR_BITS:0]  free     // up to 256 entries
   );
   
      logic        full;

      always_ff @(posedge clk) begin
         if (rst) begin
            rx_ready <= 1'b0;
         end
         else begin
            // Ready to accept new data if FIFO is not full
            rx_ready <= !full;
         end
      end


      sync_fifo #(
         .WIDTH(8),
         .DEPTH(DEPTH)
      ) fifo_i (
         .clk(clk),
         .rst(rst),

         .wr_en(rx_valid && rx_ready),
         .wr_data(rx_data),
         .full(full),

         .rd_en(rd_en),
         .rd_data(rd_data),
         .empty(empty),

         .free(free)
      );

endmodule