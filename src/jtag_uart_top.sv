module jtag_uart_top (
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

    localparam int CLK_FREQ  = 10_000_000;
    localparam int BAUD_RATE = 115200;

    // =========================================================
    // UART RX signals
    // =========================================================
    logic       rx_valid;
    logic [7:0] rx_data;
    logic       rx_ready;
    // =========================================================
    // Parser → FIFOs
    // =========================================================
    logic       cmd_wr_en;
    logic [3:0] cmd_instr;
    logic [7:0] cmd_len;

    logic       payload_wr_en;
    logic [7:0] payload_wr_data;

    // =========================================================
    // Command FIFO
    // =========================================================
    logic       cmd_empty;
    logic       cmd_full;
    logic       cmd_rd_en;
    logic [7:0] cmd_free;

    logic [3:0] cmd_fifo_instr;
    logic [7:0] cmd_fifo_len;

    // =========================================================
    // Payload FIFO
    // =========================================================
    logic       payload_empty;
    logic [7:0] payload_free;
    logic       payload_rd_en;
    logic [7:0] payload_rd_data;

    // =========================================================
    // Reply FIFO
    // =========================================================
    logic       reply_empty;
    logic       reply_full;
    logic [7:0] reply_free;
    logic       reply_rd_en;
    logic       reply_wr_en;
    logic [7:0] reply_wr_data;
    logic [7:0] reply_rd_data;

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
    // Parser
    // =========================================================
    uart_packet_parser parser (
        .clk            (clk),
        .rst            (rst),

        .rx_valid       (rx_valid),
        .rx_data        (rx_data),
        .rx_ready       (rx_ready),

        .cmd_wr         (cmd_wr_en),
        .cmd_instr      (cmd_instr),
        .cmd_len        (cmd_len),
        .cmd_free       (cmd_free),

        .data_wr        (payload_wr_en),
        .data_byte      (payload_wr_data),
        .data_free      (payload_free)
    );

    // =========================================================
    // Command FIFO
    // =========================================================
    cmd_fifo cmd_fifo_i (
        .clk        (clk),
        .rst        (rst),

        .wr_en      (cmd_wr_en),
        .instr      (cmd_instr),
        .payload_len(cmd_len),
        .free       (cmd_free),

        .rd_en      (cmd_rd_en),
        .out_instr  (cmd_fifo_instr),
        .out_len    (cmd_fifo_len),
        .empty      (cmd_empty)
    );

    // =========================================================
    // Payload FIFO
    // =========================================================
    payload_fifo payload_fifo_i (
        .clk      (clk),
        .rst      (rst),

        .wr_en    (payload_wr_en),
        .wr_data  (payload_wr_data),
        .free     (payload_free),
        .full     (),  // unused

        .rd_en    (payload_rd_en),
        .rd_data  (payload_rd_data),
        .empty    (payload_empty)
    );

    // =========================================================
    // Reply FIFO
    // =========================================================
    payload_fifo reply_fifo_i (
        .clk      (clk),
        .rst      (rst),

        .wr_en    (reply_wr_en),
        .wr_data  (reply_wr_data),
        .full     (reply_full),
        .free     (reply_free),

        .rd_en    (reply_rd_en),
        .rd_data  (reply_rd_data),
        .empty    (reply_empty)
    );

    // =========================================================
    // JTAG Engine (fixed 32-bit read)
    // =========================================================
    jtag_engine_simple jtag_i (
        .clk        (clk),
        .rst        (rst),

        .cmd_empty  (cmd_empty),
        .cmd_rd_en  (cmd_rd_en),
        .cmd_instr  (cmd_fifo_instr),

        .data_empty (payload_empty),
        .data_rd_en (payload_rd_en),
        .data_in    (payload_rd_data),

        .reply_full (reply_full),
        .reply_wr_en(reply_wr_en),
        .reply_wr_data(reply_wr_data),

        .TCK        (TCK),
        .TMS        (TMS),
        .TDI        (TDI),
        .TDO        (TDO)
    );

    // =========================================================
    // UART TX
    // =========================================================
    uart_tx #(
        .CLK_FREQ(clk_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) uart_tx_i (
        .clk      (clk),
        .rst      (rst),

        .tx_data  (tx_data),
        .tx_valid (tx_valid),
        .tx_ready (tx_ready),

        .tx       (uart_tx)
    );

    // =========================================================
    // UART TX FIFO drain
    // =========================================================
    uart_tx_from_fifo tx_sched_i (
        .clk          (clk),
        .rst          (rst),

        .fifo_empty   (reply_empty),
        .fifo_rd_en   (reply_rd_en),
        .fifo_rd_data (reply_rd_data),

        .tx_data      (tx_data),
        .tx_valid     (tx_valid),
        .tx_ready     (tx_ready)
    );

endmodule
