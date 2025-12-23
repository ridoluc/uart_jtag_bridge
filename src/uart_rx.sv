
`timescale 1ns/1ps

module uart_rx #(
    parameter CLK_FREQ  = 10_000_000,
    parameter BAUD_RATE = 115200
)(
    input  logic clk,
    input  logic rst,

    input  logic rx,             // serial input line
    input  logic rx_ready,       // NEW: downstream ready / back-pressure

    output logic [7:0] rx_data,  // received byte
    output logic       rx_valid  // stays high until rx_ready seen
);

    // ------------------------------------------------------------
    // Baud timing
    // ------------------------------------------------------------
    localparam BAUD_DIV  = CLK_FREQ / BAUD_RATE;     // ~87 for 10MHz/115200
    localparam HALF_BAUD = BAUD_DIV / 2;

    logic [31:0] baud_cnt;

    // ------------------------------------------------------------
    // State machine
    // ------------------------------------------------------------
    typedef enum logic [1:0] {
        IDLE,
        START,
        DATA,
        STOP
    } rx_state_t;

    rx_state_t  state;
    logic [3:0] bit_index;
    logic [7:0] shift_reg;

    // ------------------------------------------------------------
    // UART Receive Logic
    // ------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            state     <= IDLE;
            rx_valid  <= 1'b0;
            baud_cnt  <= '0;
            bit_index <= '0;
            shift_reg <= '0;
        end
        else begin
            // Default: hold rx_valid; clear only when downstream accepts (rx_ready)
            if (rx_valid && rx_ready)
                rx_valid <= 1'b0;

            unique case (state)

                // ------------------------------------------------
                // IDLE: wait for start bit (rx goes low)
                // Do not start a new frame while previous data is unconsumed.
                // ------------------------------------------------
                IDLE: begin
                    if ((rx == 1'b0) && !rx_valid) begin
                        state    <= START;
                        baud_cnt <= HALF_BAUD;  // sample mid start bit
                    end
                end

                // ------------------------------------------------
                // START BIT: sample mid bit, must be 0
                // ------------------------------------------------
                START: begin
                    if (baud_cnt == 0) begin
                        if (rx == 1'b0) begin
                            // valid start bit
                            state     <= DATA;
                            bit_index <= 0;
                            baud_cnt  <= BAUD_DIV - 1;
                        end
                        else begin
                            // false start, return to idle
                            state <= IDLE;
                        end
                    end
                    else
                        baud_cnt <= baud_cnt - 1;
                end

                // ------------------------------------------------
                // DATA BITS: sample 8 bits LSB first
                // ------------------------------------------------
                DATA: begin
                    if (baud_cnt == 0) begin
                        shift_reg[bit_index[2:0]] <= rx;

                        if (bit_index == 7) begin
                            state <= STOP;
                        end

                        bit_index <= bit_index + 1;
                        baud_cnt  <= BAUD_DIV - 1;
                    end
                    else
                        baud_cnt <= baud_cnt - 1;
                end

                // ------------------------------------------------
                // STOP BIT
                // ------------------------------------------------
                STOP: begin
                    if (baud_cnt == 0) begin
                        if (rx == 1'b1) begin
                            // produce valid data; keep rx_valid high until rx_ready
                            rx_data  <= shift_reg;
                            rx_valid <= 1'b1;
                        end
                        // ignore framing error for now
                        state <= IDLE;
                    end
                    else
                        baud_cnt <= baud_cnt - 1;
                end

            endcase
        end
    end

endmodule
