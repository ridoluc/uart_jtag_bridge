// =============================================================
// jtag_engine.sv
// Minimal JTAG command executor
// Supports Shift-IR and Shift-DR with arbitrary lengths
// Reads commands from cmd_fifo and data from payload_fifo
// -------------------------------------------------------------
// Expects:
//   cmd_fifo: one entry containing {instr_id, bit_length}
//   payload_fifo: bitstream for TDI
// Produces:
//   TDO bits → written back to payload_fifo
// =============================================================

module jtag_engine #(
    parameter int DATA_WIDTH = 32,
    parameter int ADDR_WIDTH = 10,
)(
    input  logic clk,
    input  logic rst,

    // ------------- Command FIFO interface ---------------------
    input  logic         cmd_empty,
    output logic         cmd_rd_en,
    input  logic [3:0]   cmd_instr,
    input  logic [3:0]   cmd_len,     // number of bytes in the payload.

    // ------------- Data FIFO interface (TDI source) ----------
    input  logic         data_empty,
    output logic         data_rd_en,
    input  logic [7:0]   data_in,     // assume LSB-first bits

    // ------------- Data FIFO write-back (TDO sink) -----------
    output logic         data_wr_en,
    output logic [7:0]   data_out,
    input  logic         data_full,

    // ------------- JTAG signals ------------------------------
    output logic TCK,
    output logic TMS,
    output logic TDI,
    input  logic TDO
);

    // =========================================================
    // TAP controller sequences for IR and DR scans
    // =========================================================

    typedef enum logic [3:0] {
        IDLE,
        FETCH_CMD,
        TAP_GOTO_SHIFT_IR_1,
        TAP_GOTO_SHIFT_IR_2,
        TAP_GOTO_SHIFT_IR_3,

        TAP_GOTO_SHIFT_DR_1,
        TAP_GOTO_SHIFT_DR_2,

        SHIFTING,
        EXIT_SHIFT,
        UPDATE,
        DONE
    } state_t;

    state_t state;

    logic [7:0]  byte_cnt;      // how many bytes left to shift
    logic        last_bit;

    // For output bit packing (TDO captured)
    logic [7:0]  tdo_shift;
    logic [2:0]  tdo_count;

    // TCK generator: toggle every clk
    logic tck_int;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) tck_int <= 1'b0;
        else     tck_int <= ~tck_int;
    end

    assign TCK = tck_int;

    // =========================================================
    // Main JTAG engine FSM
    // =========================================================

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state       <= IDLE;
            cmd_rd_en   <= 0;
            data_rd_en  <= 0;
            data_wr_en  <= 0;
            byte_cnt     <= 0;
            tdo_shift   <= 0;
            tdo_count   <= 0;
            TMS         <= 1;
            TDI         <= 0;
        end
        else begin
            // default strobes
            cmd_rd_en  <= 0;
            data_rd_en <= 0;
            data_wr_en <= 0;

            unique case (state)

            // --------------------------------------------------
            IDLE:
            begin
                TMS <= 0;
                if (!cmd_empty)
                    state <= FETCH_CMD;
            end

            // --------------------------------------------------
            FETCH_CMD:
            begin
                cmd_rd_en <= 1;
                byte_cnt  <= cmd_len;
                tdo_count <= 0;

                



                // if (cmd_instr == 4'h0) begin
                //     // Shift-IR
                //     state <= TAP_GOTO_SHIFT_IR_1;
                // end
                // else if (cmd_instr == 4'h1) begin
                //     // Shift-DR
                //     state <= TAP_GOTO_SHIFT_DR_1;
                // end
                // else begin
                //     // unknown / idle
                //     state <= DONE;
                // end
            end

            // ==================================================
            // IR path: Move from Run-Test/Idle to Shift-IR
            // ==================================================
            TAP_GOTO_SHIFT_IR_1: begin
                TMS <= 1; // SELECT-DR-SCAN
                state <= TAP_GOTO_SHIFT_IR_2;
            end
            TAP_GOTO_SHIFT_IR_2: begin
                TMS <= 1; // SELECT-IR-SCAN
                state <= TAP_GOTO_SHIFT_IR_3;
            end
            TAP_GOTO_SHIFT_IR_3: begin
                TMS <= 0; // CAPTURE-IR, then SHIFT-IR
                state <= SHIFTING;
            end

            // ==================================================
            // DR path: Move from Run-Test/Idle to Shift-DR
            // ==================================================
            TAP_GOTO_SHIFT_DR_1: begin
                TMS <= 1; // SELECT-DR-SCAN
                state <= TAP_GOTO_SHIFT_DR_2;
            end
            TAP_GOTO_SHIFT_DR_2: begin
                TMS <= 0; // CAPTURE-DR → SHIFT-DR
                state <= SHIFTING;
            end

            // ==================================================
            // SHIFTING: shift IR/DR for byte_cnt bits
            // ==================================================
            SHIFTING: begin
                if (tck_int == 0) begin  
                    // ----- Rising edge of TCK -----

                    if (data_empty) begin
                        // No TDI available — stall
                        data_rd_en <= 0;
                        TDI <= 0;
                    end
                    else begin
                        data_rd_en <= 1;
                        TDI <= data_in[0];   // LSB-first
                    end

                    last_bit = (byte_cnt == 1);
                    TMS <= last_bit ? 1 : 0;
                end
                else begin
                    // ----- Falling edge of TCK -----
                    // Capture TDO on falling edge
                    tdo_shift <= {TDO, tdo_shift[7:1]};
                    tdo_count <= tdo_count + 1;

                    if (tdo_count == 7) begin
                        if (!data_full) begin
                            data_out   <= tdo_shift;
                            data_wr_en <= 1;
                        end
                        tdo_count <= 0;
                    end

                    if (last_bit)
                        state <= EXIT_SHIFT;
                    else
                        byte_cnt <= byte_cnt - 1;
                end
            end

            // --------------------------------------------------
            EXIT_SHIFT: begin
                TMS <= 1; 
                state <= UPDATE;
            end

            UPDATE: begin
                TMS <= 0;  
                state <= DONE;
            end

            DONE: begin
                state <= IDLE;
            end

            endcase
        end
    end

endmodule
