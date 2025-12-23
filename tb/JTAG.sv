`timescale 1ns/1ps

module JTAG #(
    parameter MEM_ADDR_WIDTH = 10,
    parameter MEM_DATA_WIDTH = 32
)(
    input  wire        tck,
    input  wire        tms,
    input  wire        tdi,
    output wire        tdo,
    input  wire        rst_n,
    // Memory interface
    output reg  [MEM_ADDR_WIDTH-1:0] jtag_addr,
    output reg  [MEM_DATA_WIDTH-1:0] jtag_wdata,
    input  wire [MEM_DATA_WIDTH-1:0] jtag_rdata,
    output reg                       jtag_we,
    output reg                       jtag_req_pulse,
    output reg                       jtag_en,
    input  wire                      jtag_ack   // TODO: Not implemented yet
);

    // TAP Controller States (IEEE 1149.1)
    typedef enum logic [3:0] {
        TEST_LOGIC_RESET = 4'd0,
        RUN_TEST_IDLE    = 4'd1,
        SELECT_DR_SCAN   = 4'd2,
        CAPTURE_DR       = 4'd3,
        SHIFT_DR         = 4'd4,
        EXIT1_DR         = 4'd5,
        PAUSE_DR         = 4'd6,
        EXIT2_DR         = 4'd7,
        UPDATE_DR        = 4'd8,
        SELECT_IR_SCAN   = 4'd9,
        CAPTURE_IR       = 4'd10,
        SHIFT_IR         = 4'd11,
        EXIT1_IR         = 4'd12,
        PAUSE_IR         = 4'd13,
        EXIT2_IR         = 4'd14,
        UPDATE_IR        = 4'd15
    } tap_state_t;

    tap_state_t tap_state, tap_state_next;

    // IR and DR registers
    reg [3:0] ir;
    reg [MEM_ADDR_WIDTH+MEM_DATA_WIDTH-1:0] dr;
    reg dr_shift, dr_capture, dr_update;
    reg ir_shift, ir_capture, ir_update;

    // Instruction set
    localparam [3:0] IR_NOP      = 4'b0000;
    localparam [3:0] IR_MEM_CTRL = 4'b0001; // Isolate memory from CPU for safe programming
    localparam [3:0] IR_WRITE    = 4'b0010;
    localparam [3:0] IR_READ     = 4'b0011;
    localparam [3:0] IR_DONE     = 4'b0100; // Close JTAG interface and reset CPU

    // TAP Controller State Machine
    always_ff @(posedge tck or negedge rst_n) begin
        if (!rst_n) begin
            tap_state <= TEST_LOGIC_RESET;
        end else begin
            tap_state <= tap_state_next;
        end
    end

    always_comb begin
        tap_state_next = tap_state;
        dr_shift = 0; dr_capture = 0; dr_update = 0;
        ir_shift = 0; ir_capture = 0; ir_update = 0;
        case (tap_state)
            TEST_LOGIC_RESET: tap_state_next = tms ? TEST_LOGIC_RESET : RUN_TEST_IDLE;
            RUN_TEST_IDLE:    tap_state_next = tms ? SELECT_DR_SCAN : RUN_TEST_IDLE;
            SELECT_DR_SCAN:   tap_state_next = tms ? SELECT_IR_SCAN : CAPTURE_DR;
            CAPTURE_DR:       begin 
                              dr_capture = 1; 
                              tap_state_next = tms ? EXIT1_DR : SHIFT_DR; 
                              end
            SHIFT_DR:         begin 
                              dr_shift = 1; 
                              tap_state_next = tms ? EXIT1_DR : SHIFT_DR; 
                              end
            EXIT1_DR:         tap_state_next = tms ? UPDATE_DR : PAUSE_DR;
            PAUSE_DR:         tap_state_next = tms ? EXIT2_DR : PAUSE_DR;
            EXIT2_DR:         tap_state_next = tms ? UPDATE_DR : SHIFT_DR;
            UPDATE_DR:        begin 
                              dr_update = 1; 
                              tap_state_next = tms ? SELECT_DR_SCAN : RUN_TEST_IDLE; 
                              end
            SELECT_IR_SCAN:   tap_state_next = tms ? TEST_LOGIC_RESET : CAPTURE_IR;
            CAPTURE_IR:       begin 
                              ir_capture = 1; 
                              tap_state_next = tms ? EXIT1_IR : SHIFT_IR; 
                              end
            SHIFT_IR:         begin 
                              ir_shift = 1; 
                              tap_state_next = tms ? EXIT1_IR : SHIFT_IR; 
                              end
            EXIT1_IR:         tap_state_next = tms ? UPDATE_IR : PAUSE_IR;
            PAUSE_IR:         tap_state_next = tms ? EXIT2_IR : PAUSE_IR;
            EXIT2_IR:         tap_state_next = tms ? UPDATE_IR : SHIFT_IR;
            UPDATE_IR:        begin 
                              ir_update = 1; 
                              tap_state_next = tms ? SELECT_DR_SCAN : RUN_TEST_IDLE; 
                              end
            default:          tap_state_next = TEST_LOGIC_RESET;
        endcase
    end

    // IR shift/capture/update
    always_ff @(posedge tck or negedge rst_n) begin
        if (!rst_n) begin
            ir <= IR_NOP;
        end else if (ir_capture) begin  //tap_state_next==CAPTURE_IR ??
            ir <= IR_NOP;
        end else if (tap_state_next==SHIFT_IR) begin
            ir <= {tdi, ir[3:1]};
        end else if (ir_update) begin   // tap_state_next==UPDATE_IR ??
            if (ir == IR_DONE) begin
                ir <= IR_NOP; // Reset instruction register
            end else begin
                ir <= ir;
            end
        end
    end

    // DR shift/capture/update
    always_ff @(posedge tck or negedge rst_n) begin
        if (!rst_n) begin
            dr <= '0;
        end else if (dr_capture) begin
            if (ir == IR_READ)
                dr <= {jtag_addr, jtag_rdata};
            else
                dr <= '0;
        end else if (tap_state_next == SHIFT_DR) begin
            dr <= {tdi, dr[MEM_ADDR_WIDTH+MEM_DATA_WIDTH-1:1]};
        end else if (dr_update) begin
            dr <= dr;
        end
    end

    // Memory write logic
    always_ff @(posedge tck or negedge rst_n) begin
        if (!rst_n) begin
            jtag_addr <= {MEM_ADDR_WIDTH{1'b0}};
            jtag_wdata <= {MEM_DATA_WIDTH{1'b0}};
            jtag_we <= 1'b0;
            jtag_req_pulse <= 1'b0; // Ensure pulse is low on reset
        end else if (dr_update) begin
            jtag_addr <= dr[MEM_ADDR_WIDTH+MEM_DATA_WIDTH-1:MEM_DATA_WIDTH];
            jtag_wdata <= dr[MEM_DATA_WIDTH-1:0];
            
            jtag_we <= ir == IR_WRITE ? 1'b1 : 1'b0;
            jtag_req_pulse <= 1'b1; // Pulse the request signal
        end else begin
            jtag_we <= 1'b0;
            jtag_req_pulse <= 1'b0; // Reset request pulse
        end
    end

    // Memory control enable logic
    always_ff @(posedge tck or negedge rst_n) begin
        if (!rst_n) begin
            jtag_en <= 1'b0;
        end else if (ir_update) begin
            if(ir == IR_MEM_CTRL) begin
                jtag_en <= 1'b1; // Enable memory control for JTAG operations
            end else if (ir == IR_DONE) begin
                jtag_en <= 1'b0; // Disable memory control when done
            end
        end
    end 


    // TDO output
    assign tdo = (tap_state == SHIFT_IR) ? ir[0] :
                 (tap_state == SHIFT_DR) ? dr[0] : 1'b0;  // Should be high impedance when not shifting

endmodule