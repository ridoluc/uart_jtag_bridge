// =============================================================
// uart_tx_from_fifo.sv
// Drains a reply FIFO and feeds uart_tx using ready/valid
// =============================================================

module uart_tx_from_fifo (
    input  logic       clk,
    input  logic       rst,

    // -------- Reply FIFO interface --------
    input  logic       fifo_empty,
    output logic       fifo_rd_en,
    input  logic [7:0] fifo_rd_data,

    // -------- UART TX interface --------
    output logic [7:0] tx_data,
    output logic       tx_valid,
    input  logic       tx_ready
);

    // ---------------------------------------------------------
    // Default outputs
    // ---------------------------------------------------------
    always_comb begin
        fifo_rd_en = 1'b0;
        tx_valid   = 1'b0;
        tx_data    = fifo_rd_data;
    end

    // ---------------------------------------------------------
    // Control logic
    // ---------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            fifo_rd_en <= 1'b0;
            tx_valid   <= 1'b0;
        end
        else begin
            fifo_rd_en <= 1'b0;
            tx_valid   <= 1'b0;

            // If UART is ready and FIFO has data, transmit next byte
            if (tx_ready && !fifo_empty) begin
                fifo_rd_en <= 1'b1;    // pop FIFO
                tx_valid   <= 1'b1;    // start UART TX
                // tx_data is driven combinationally from fifo_rd_data
            end
        end
    end

endmodule
