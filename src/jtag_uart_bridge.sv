`timescale 1ns/1ps

module jtag_uart_bridge #(
    parameter int CLK_FREQ  = 10_000_000,
    parameter int BAUD_RATE = 115200
)(
    input  logic clk,        // 10 MHz
    input  logic rst,

    // UART pins
    input  logic uart_rx,
    output logic uart_tx,

    // JTAG pins
    output logic TCK,
    output logic TMS,
    output logic TDI,
    input  logic TDO
);

    localparam TX_FIFO_DEPTH = 256;
    localparam RX_FIFO_DEPTH = 256;

    // =========================================================
    // UART RX signals
    // =========================================================
    logic       rx_valid;
    logic [7:0] rx_data;
    logic       rx_ready;
    
    // =========================================================
    // RX FIFO signals
    // =========================================================
    logic       rx_fifo_rd_en;
    logic [7:0] rx_fifo_rd_data;
    logic       rx_fifo_empty;


    // =========================================================
    // TX FIFO
    // =========================================================
    logic       tx_fifo_empty;
    logic       tx_fifo_full;
    logic [$clog2(TX_FIFO_DEPTH):0] tx_fifo_free;
    logic       tx_fifo_rd_en;
    logic       tx_fifo_wr_en;
    logic [7:0] tx_fifo_wr_data;
    logic [7:0] tx_fifo_rd_data;

    // =========================================================
    // UART TX signals
    // =========================================================
    logic       tx_ready;
    logic       tx_valid;
    logic [7:0] tx_data;

    // =========================================================
    // UART RX
    // =========================================================
    uart_rx #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) uart_rx_i (
        .clk      (clk),
        .rst      (rst),
        .rx       (uart_rx),
        .rx_ready (rx_ready),
        .rx_valid (rx_valid),
        .rx_data  (rx_data)
    );

    // =========================================================
    // RX FIFO
    // =========================================================
    rx_fifo #(
        .DEPTH(RX_FIFO_DEPTH)
    ) rx_fifo_i (
        .clk       (clk),
        .rst       (rst),
        .rx_data   (rx_data),
        .rx_valid  (rx_valid),
        .rx_ready  (rx_ready),
        .rd_en     (rx_fifo_rd_en),
        .rd_data   (rx_fifo_rd_data),
        .empty     (rx_fifo_empty),
        .free      ()
    );

    // =========================================================
    // JTAG Engine
    // =========================================================
    jtag_engine jtag_engine_i (
        .clk            (clk),
        .rst            (rst),
        // RX FIFO interface (UART → JTAG)
        .rx_empty       (rx_fifo_empty),
        .rx_rd_en       (rx_fifo_rd_en),
        .rx_rd_data     (rx_fifo_rd_data),
        // TX FIFO interface (JTAG → UART)
        .tx_full        (tx_fifo_full),
        .tx_wr_en       (tx_fifo_wr_en),
        .tx_wr_data     (tx_fifo_wr_data),
        // JTAG pins
        .TCK            (TCK),
        .TMS            (TMS),
        .TDI            (TDI),
        .TDO            (TDO)
    );

    // =========================================================
    // TX FIFO
    // =========================================================
    tx_fifo #(
        .DEPTH(TX_FIFO_DEPTH)
    ) tx_fifo_i (
        .clk        (clk),
        .rst        (rst),
        // Write interface
        .wr_en      (tx_fifo_wr_en),
        .wr_data    (tx_fifo_wr_data),
        .full       (tx_fifo_full),
        .free       (tx_fifo_free),
        // Read interface
        .rd_en      (tx_fifo_rd_en),
        .rd_data    (tx_fifo_rd_data),
        .empty      (tx_fifo_empty)
    );

    // =========================================================
    // UART TX FIFO drain
    // =========================================================
    uart_tx_from_fifo tx_sched_i (
        .clk          (clk),
        .rst          (rst),

        .fifo_empty   (tx_fifo_empty),
        .fifo_rd_en   (tx_fifo_rd_en),
        .fifo_rd_data (tx_fifo_rd_data),

        .tx_data      (tx_data),
        .tx_valid     (tx_valid),
        .tx_ready     (tx_ready)
    );

    // =========================================================
    // UART TX
    // =========================================================
    uart_tx #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) uart_tx_i (
        .clk      (clk),
        .rst      (rst),

        .tx_data  (tx_data),
        .tx_valid (tx_valid),
        .tx_ready (tx_ready),

        .tx       (uart_tx)
    );



endmodule
