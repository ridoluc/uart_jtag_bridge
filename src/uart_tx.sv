// =============================================================
// uart_tx.sv
// UART transmitter: 8 data bits, 1 start bit, 1 stop bit
// =============================================================

module uart_tx #(
    parameter int CLK_FREQ  = 10_000_000,
    parameter int BAUD_RATE = 115200
)(
    input  logic       clk,
    input  logic       rst,

    // Transmit interface
    input  logic [7:0] tx_data,
    input  logic       tx_valid,
    output logic       tx_ready,

    // UART output pin
    output logic       tx
);

    // ---------------------------------------------------------
    // Baud timing
    // ---------------------------------------------------------
    localparam int BAUD_DIV = CLK_FREQ / BAUD_RATE;

    logic [$clog2(BAUD_DIV)-1:0] baud_cnt;
    logic baud_tick;

    // ---------------------------------------------------------
    // FSM
    // ---------------------------------------------------------
    typedef enum logic [1:0] {
        TX_IDLE,
        TX_START,
        TX_DATA,
        TX_STOP
    } tx_state_t;

    tx_state_t state;

    logic [7:0] shift_reg;
    logic [3:0] bit_index;

    // ---------------------------------------------------------
    // Baud tick generator
    // ---------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            baud_cnt  <= '0;
            baud_tick <= 1'b0;
        end else begin
            if (state == TX_IDLE) begin
                baud_cnt  <= '0;
                baud_tick <= 1'b0;
            end else if (baud_cnt == BAUD_DIV - 1) begin
                baud_cnt  <= '0;
                baud_tick <= 1'b1;
            end else begin
                baud_cnt  <= baud_cnt + 1;
                baud_tick <= 1'b0;
            end
        end
    end

    // ---------------------------------------------------------
    // UART TX FSM
    // ---------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state     <= TX_IDLE;
            tx        <= 1'b1; // idle line is high
            bit_index <= '0;
            shift_reg <= '0;
        end else begin
            case (state)

            // ---------------------------------------------
            TX_IDLE: begin
                tx <= 1'b1;
                bit_index <= 0;
                if (tx_valid) begin
                    shift_reg <= tx_data;
                    state <= TX_START;
                end
            end

            // ---------------------------------------------
            TX_START: begin
                tx <= 1'b0; // start bit
                if (baud_tick) begin
                    state <= TX_DATA;
                end
            end

            // ---------------------------------------------
            TX_DATA: begin
                tx <= shift_reg[bit_index];
                if (baud_tick) begin
                    bit_index <= bit_index + 1;
                    if (bit_index == 7)
                        state <= TX_STOP;
                end
            end

            // ---------------------------------------------
            TX_STOP: begin
                tx <= 1'b1; // stop bit
                if (baud_tick) begin
                    state <= TX_IDLE;
                end
            end

            endcase
        end
    end

    // ---------------------------------------------------------
    // Ready when idle
    // ---------------------------------------------------------
    assign tx_ready = (state == TX_IDLE);

endmodule
