`timescale 1ns/1ps
module jtag_engine (
    input  logic clk,
    input  logic rst,

    // RX FIFO interface (UART → JTAG)
    input  logic       rx_empty,
    output logic       rx_rd_en,
    input  logic [7:0] rx_rd_data,

    // TX FIFO interface (JTAG → UART)
    input  logic       tx_full,
    output logic       tx_wr_en,
    output logic [7:0] tx_wr_data,

    // JTAG pins
    output logic TCK,
    output logic TMS,
    output logic TDI,
    input  logic TDO
);

    // ---------------------------------------------------------
    // Internal registers
    // ---------------------------------------------------------
    logic [3:0] tms_shift;
    logic [3:0] tdi_shift;

    logic [1:0] bit_idx;
    logic       busy;

    logic [3:0] tdo_nibble;
    logic       nibble_half;   // 0 = lower, 1 = upper
    logic [7:0] tdo_byte;

    // ---------------------------------------------------------
    // FSM
    // ---------------------------------------------------------
    typedef enum logic [1:0] {
        IDLE,
        LOAD,
        SHIFT,
        COMMIT
    } state_t;

    state_t state;

    // ---------------------------------------------------------
    // Sequential logic
    // ---------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state        <= IDLE;
            rx_rd_en     <= 1'b0;
            tx_wr_en     <= 1'b0;

            TCK          <= 1'b0;
            TMS          <= 1'b0;
            TDI          <= 1'b0;

            bit_idx      <= 2'd0;
            tdo_nibble   <= 4'd0;
            tdo_byte     <= 8'd0;
            nibble_half  <= 1'b0;
        end else begin
            rx_rd_en <= 1'b0;
            tx_wr_en <= 1'b0;

            case (state)

                // ---------------------------------------------
                IDLE: begin
                    if (!rx_empty) begin
                        rx_rd_en <= 1'b1;
                        state    <= LOAD;
                    end
                end

                // ---------------------------------------------
                LOAD: begin
                    tms_shift <= rx_rd_data[7:4];
                    tdi_shift <= rx_rd_data[3:0];
                    bit_idx   <= 2'd0;
                    state     <= SHIFT;
                end

                // ---------------------------------------------
                SHIFT: begin
                    // Drive JTAG
                    TMS <= tms_shift[bit_idx];
                    TDI <= tdi_shift[bit_idx];

                    // Toggle TCK
                    TCK <= ~TCK;

                    // Sample TDO on rising edge
                    if (TCK == 1'b0) begin
                        tdo_nibble[bit_idx] <= TDO;
                        bit_idx <= bit_idx + 1'b1;

                        if (bit_idx == 2'd3) begin
                            state <= COMMIT;
                        end
                    end
                end

                // ---------------------------------------------
                COMMIT: begin
                    if (!tx_full) begin
                        if (!nibble_half) begin
                            tdo_byte[3:0] <= tdo_nibble;
                            nibble_half   <= 1'b1;
                        end else begin
                            tdo_byte[7:4] <= tdo_nibble;
                            tx_wr_data    <= tdo_byte;
                            tx_wr_en      <= 1'b1;
                            nibble_half   <= 1'b0;
                        end
                        state <= IDLE;
                    end
                end

            endcase
        end
    end

endmodule
