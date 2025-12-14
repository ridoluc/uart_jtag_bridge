// SystemVerilog parser for 1-byte header: [7:4] instr_id, [3:0] payload_len
// Two cases:
//   - payload_len == 0 : instruction-only -> push cmd immediately
//   - payload_len > 0  : instruction + payload -> push payload bytes to data FIFO
//
// Handshake with UART RX: rx_valid (input) + rx_ready (output).
// The module checks cmd_free and data_free before starting a packet to guarantee atomicity.

module uart_packet_parser (
    input  logic         clk,
    input  logic         rst,

    // ------ UART receive handshake (byte-level) ------
    // rx_valid: 1-cycle pulse when a byte is available from UART RX.
    // rx_ready: when low, UART RX must withhold asserting rx_valid (backpressure).
    input  logic  [7:0]  rx_data,
    input  logic         rx_valid,
    output logic         rx_ready,

    // ------ Command FIFO write interface (one descriptor per command) ------
    // cmd_wr: asserted for one cycle when a command descriptor is available.
    // cmd_instr/cmd_len: command descriptor data.
    output logic         cmd_wr,
    output logic [3:0]   cmd_instr,
    output logic [7:0]   cmd_len,
    input  logic [7:0]   cmd_free,   // number of free entries in cmd FIFO (>=1 to accept)

    // ------ Data FIFO write interface (payload bytes) ------
    // data_wr asserted for each payload byte (one cycle each)
    output logic         data_wr,
    output logic [7:0]   data_byte,
    input  logic [7:0]   data_free,  // number of free bytes available in data FIFO

    // ------ Status / error reporting (optional) ------
    output logic         pkt_error   // set if something bad happens (kept simple)
);

    // FSM states
    typedef enum logic [1:0] {
        WAIT_HEADER,      // waiting for header byte
        WAIT_FOR_SPACE,   // header consumed; waiting for enough FIFO space
        READ_PAYLOAD      // reading payload bytes from UART RX
    } state_t;

    state_t state;
    logic [3:0] curr_instr;
    logic [3:0] curr_len;          // zero-extended payload length (0..7)
    logic [3:0] bytes_remaining;   // counts down payload bytes left

    // default outputs
    always_comb begin
        rx_ready   = 1'b1;    // optimistic default: accept bytes
        cmd_wr     = 1'b0;
        cmd_instr  = 4'h0;
        cmd_len    = 8'h0;
        data_wr    = 1'b0;
        data_byte  = 8'h00;
        pkt_error  = 1'b0;
    end

    // Main sequential logic
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state           <= WAIT_HEADER;
            curr_instr      <= 4'h0;
            curr_len        <= 4'h0;
            bytes_remaining <= 4'h0;
        end
        else begin
            // Default stay in current state; outputs driven in combinational block
            unique case (state)

                // ------------------------------
                // WAIT_HEADER
                // ------------------------------
                WAIT_HEADER: begin
                    // Wait for header byte to arrive from UART RX
                    if (rx_valid) begin
                        // parse header: bits[7:4] instr, bits[3:0] len
                        curr_instr <= rx_data[7:4];
                        curr_len   <= rx_data[3:0]; // zero-extend to 8 bits

                        // If payload_len is 0 -> instruction-only
                        if (rx_data[3:0] == 4'h0) begin
                            // need at least 1 slot in cmd FIFO
                            if (cmd_free >= 1) begin
                                // write cmd descriptor immediately next cycle (combinational outputs)
                                // We'll assert cmd_wr for one cycle here (synchronous)
                                cmd_instr <= rx_data[7:4];
                                cmd_len   <= 8'h0;
                                cmd_wr    <= 1'b1;  // pulse to user; combinational override will take place for one cycle
                                // stay in WAIT_HEADER to accept next packet
                                state <= WAIT_HEADER;
                            end
                            else begin
                                // no space in cmd FIFO; we must hold header until space becomes available.
                                // We transition to WAIT_FOR_SPACE to re-try when cmd_free>0.
                                state <= WAIT_FOR_SPACE;
                                // bytes_remaining remains 0
                                bytes_remaining <= 4'h0;
                            end
                        end
                        else begin
                            // payload_len > 0: we must ensure there is enough room in data_fifo AND at least one cmd slot
                            // If not enough space, go to WAIT_FOR_SPACE; else start reading payload
                            if ((data_free >= rx_data[3:0]) && (cmd_free >= 1)) begin
                                // we can start reading payload immediately
                                bytes_remaining <= rx_data[3:0]; // number of bytes to read
                                state <= READ_PAYLOAD;
                                // rx_ready remains 1 so UART RX may present next byte
                            end
                            else begin
                                // not enough space; wait until space available
                                curr_len <= rx_data[3:0];
                                bytes_remaining <= rx_data[3:0];
                                state <= WAIT_FOR_SPACE;
                            end
                        end
                    end
                    else begin
                        // no byte - remain idle
                        state <= WAIT_HEADER;
                    end
                end // WAIT_HEADER

                // ------------------------------
                // WAIT_FOR_SPACE
                // ------------------------------
                // Header already consumed and latched in curr_instr/curr_len.
                // Wait until cmd_free >= 1 AND data_free >= curr_len,
                // then either issue a zero-len cmd or go to READ_PAYLOAD.
                // ------------------------------
                WAIT_FOR_SPACE: begin
                    // If waiting for a zero-len command (curr_len==0)
                    if (curr_len == 0) begin
                        if (cmd_free >= 1) begin
                            // push cmd descriptor
                            cmd_instr <= curr_instr;
                            cmd_len   <= 8'h0;
                            cmd_wr    <= 1'b1;
                            // return to WAIT_HEADER
                            state <= WAIT_HEADER;
                        end
                        else begin
                            // continue waiting
                            state <= WAIT_FOR_SPACE;
                        end
                    end
                    else begin
                        // waiting for payload space
                        if ((cmd_free >= 1) && (data_free >= curr_len)) begin
                            // now start reading payload
                            bytes_remaining <= curr_len;
                            state <= READ_PAYLOAD;
                        end
                        else begin
                            // still not enough space; keep rx_ready low by combinational override
                            state <= WAIT_FOR_SPACE;
                        end
                    end
                end // WAIT_FOR_SPACE

                // ------------------------------
                // READ_PAYLOAD
                // ------------------------------
                // Consume payload_len bytes from UART (rx_valid pulses), push them to data FIFO,
                // when last byte consumed, emit cmd descriptor (instr + len) and go back to WAIT_HEADER.
                // ------------------------------
                READ_PAYLOAD: begin
                    if (rx_valid) begin
                        // push incoming byte into data FIFO
                        data_byte <= rx_data;
                        data_wr   <= 1'b1;

                        if (bytes_remaining > 0)
                            bytes_remaining <= bytes_remaining - 8'd1;

                        if (bytes_remaining == 8'd1) begin
                            // This was the last payload byte; now issue the command descriptor
                            // Command will be written in the same cycle as last data byte completion
                            // Note: if cmd_free became zero in the meanwhile, this would be a problem;
                            // but we ensured earlier that cmd_free >= 1 before entering READ_PAYLOAD.
                            cmd_instr <= curr_instr;
                            cmd_len   <= {4'd0, curr_len};
                            cmd_wr    <= 1'b1;

                            // done with packet
                            state <= WAIT_HEADER;
                        end
                        else begin
                            // more bytes to read
                            state <= READ_PAYLOAD;
                        end
                    end
                    else begin
                        // no byte this cycle; stay and wait
                        state <= READ_PAYLOAD;
                    end
                end // READ_PAYLOAD

                default: state <= WAIT_HEADER;
            endcase
        end
    end

    // -------------------------
    // rx_ready combinational logic
    // -------------------------
    // When in WAIT_FOR_SPACE we must prevent the UART from sending additional bytes.
    // rx_ready is held low to request UART to pause (UART RX must implement handshake).
    always_comb begin
        // default
        rx_ready = 1'b1;

        // if waiting for FIFO space, deassert ready to prevent UART RX from presenting further bytes
        if (state == WAIT_FOR_SPACE) begin
            rx_ready = 1'b0;
        end

        // Safety: if in READ_PAYLOAD we must ensure we still have space for at least one byte
        // (data_free was checked before entering READ_PAYLOAD; however re-check here)
        if (state == READ_PAYLOAD) begin
            if (data_free == 0) begin
                // This should not happen (we pre-checked), but if it does, stop accepting bytes.
                rx_ready = 1'b0;
                pkt_error = 1'b1;
            end
        end

        // Also check for cmd_free: if it is zero and we are about to finish a packet, block.
        // But we ensured cmd_free>=1 before starting.
        if ((state == READ_PAYLOAD) && (cmd_free == 0) && (bytes_remaining == 8'd1)) begin
            rx_ready = 1'b0;
            pkt_error = 1'b1;
        end
    end

endmodule
