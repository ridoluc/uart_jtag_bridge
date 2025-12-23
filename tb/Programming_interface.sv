module Programming_controller #(
    parameter MEM_ADDR_WIDTH = 10,
    parameter MEM_DATA_WIDTH = 32
)(
    input  wire        clk,
    input  wire        rst_n,
    
    // JTAG Clock for synchronization
    input  wire        tck,

    // JTAG-side (JTAG clcok domain inputs)
    input  wire        jtag_en,         // JTAG mode active
    input  wire        jtag_req_pulse,  // New request pulse
    input  wire        jtag_we,         // Write enable
    input  wire [MEM_ADDR_WIDTH-1:0] jtag_addr,
    input  wire [MEM_DATA_WIDTH-1:0] jtag_wdata,

    // Memory interface
    output reg  [MEM_ADDR_WIDTH-1:0] mem_addr,
    output reg  [MEM_DATA_WIDTH-1:0] mem_wdata,
    input  wire [MEM_DATA_WIDTH-1:0] mem_rdata,
    output reg         mem_we,
    output reg         mem_control_enable,

    // CPU reset on done state
    output reg        jtag_rst_n,

    // JTAG side outputs
    output wire [MEM_DATA_WIDTH-1:0] jtag_rdata,
    output reg        jtag_ack
);


    localparam S_NORMAL_OPS = 3'b000;
    localparam S_JTAG_CTRL  = 3'b001;
    localparam S_WRITE      = 3'b010;
    localparam S_READ       = 3'b011;
    localparam S_WAIT_READ  = 3'b100; 
    localparam S_ACK        = 3'b101; 
    localparam S_DONE       = 3'b110;


    reg [2:0] state, next_state;
    reg [1:0] reset_counter;

    ////////////////////////////////////////////////////////////////////////////
    // Synchronization of JTAG request pulse to CPU clock domain    
    ////////////////////////////////////////////////////////////////////////////

    wire jtag_en_sync;
    wire jtag_req_pulse_sync;
    wire jtag_we_sync;

    cdc_signal_sync #(1) u_cdc_en_sync (rst_n, tck, clk, jtag_en, jtag_en_sync);
    cdc_pulse_sync u_cdc_pulse_sync (rst_n, tck, clk, jtag_req_pulse, jtag_req_pulse_sync);
    cdc_pulse_sync u_cdc_we_sync (rst_n, tck, clk, jtag_we, jtag_we_sync);


    reg [MEM_DATA_WIDTH-1:0] jtag_wdata_sync;
    reg [MEM_ADDR_WIDTH-1:0] jtag_addr_sync;

    cdc_signal_sync #( MEM_ADDR_WIDTH) u_cdc_multi_bit_sync_addr (rst_n, tck, clk, jtag_addr, jtag_addr_sync);
    cdc_signal_sync #(MEM_DATA_WIDTH) u_cdc_multi_bit_sync_data (rst_n, tck, clk, jtag_wdata, jtag_wdata_sync);



    reg [MEM_DATA_WIDTH-1:0] jtag_rdata_sync;
    cdc_signal_sync #(MEM_DATA_WIDTH) u_cdc_rdata_sync (rst_n, clk, tck, jtag_rdata_sync, jtag_rdata);




    ////////////////////////////////////////////////////////////////////////////
    // State Machine for JTAG Control
    ////////////////////////////////////////////////////////////////////////////

    always_ff @(posedge clk) begin
        if(!rst_n) begin
            state <= S_NORMAL_OPS;
        end else begin
            state <= next_state;
        end
    end

    always_comb begin
        case(state)
            S_NORMAL_OPS: next_state = (jtag_en_sync) ? S_JTAG_CTRL : S_NORMAL_OPS;
            S_JTAG_CTRL: begin
                if(!jtag_en_sync) begin
                    next_state = S_DONE; // Exit JTAG mode
                end else begin
                    if(jtag_req_pulse_sync) begin
                        if(jtag_we_sync) begin
                            next_state = S_WRITE;
                        end else begin
                            next_state = S_READ;
                        end
                    end else begin
                        next_state = S_JTAG_CTRL; // Wait for next request
                    end
                end
            end

            S_WRITE: next_state = S_ACK;
            S_READ: next_state = S_WAIT_READ; // Wait for memory read data
            S_WAIT_READ: next_state = S_ACK; // Wait until mem_rdata is valid. The wait can be linked to memory valid signals.
            S_ACK: next_state = S_JTAG_CTRL; 
            S_DONE: next_state = (reset_counter == 2'b11) ? S_NORMAL_OPS : S_DONE;
            default: next_state = S_NORMAL_OPS;
        endcase
    end

    ////////////////////////////////////////////////////////////////////////////
    // Memory and JTAG Control Logic
    ////////////////////////////////////////////////////////////////////////////

    always_ff @(posedge clk) begin
        if(!rst_n) begin
            reset_counter       <= 2'b00;
            mem_addr            <= {MEM_ADDR_WIDTH{1'b0}};
            mem_wdata           <= {MEM_DATA_WIDTH{1'b0}};
            mem_we              <= 1'b0;
            mem_control_enable  <= 1'b0;
            jtag_rst_n          <= 1'b1;
            jtag_rdata_sync     <= '0;
            jtag_ack            <= 1'b0;
        end else begin
            mem_addr            <= mem_addr;
            mem_wdata           <= mem_wdata;
            mem_we              <= 1'b0;
            mem_control_enable  <= (state == S_NORMAL_OPS) ? 1'b0 : 1'b1;
            jtag_rst_n          <= 1'b1;
            jtag_rdata_sync     <= jtag_rdata_sync;  
            jtag_ack            <= 1'b0;     // TODO: sync this properly
            reset_counter       <= reset_counter;

            case(next_state)
                S_NORMAL_OPS: begin

                end

                S_JTAG_CTRL: begin
                end

                S_WRITE: begin
                    mem_addr <= jtag_addr_sync;
                    mem_wdata <= jtag_wdata_sync;
                    mem_we <= 1'b1; // Enable write
                end

                S_READ: begin
                    mem_addr <= jtag_addr_sync; // Set address for read
                end

                S_WAIT_READ: begin
                    // Wait for memory read data to be valid
                    // This state is used to ensure that we don't proceed until mem_rdata is ready
                    // In a real implementation, this could be linked to a memory valid signal
                end


                S_ACK: begin
                    mem_we <= 1'b0; // Disable write
                    jtag_rdata_sync <= mem_rdata; // Read data from memory  
                    jtag_ack <= 1'b1; // Acknowledge operation
                end

                S_DONE: begin
                    reset_counter <= reset_counter + 1; // Increment reset counter
                    jtag_rst_n <= 1'b0; // Assert CPU reset during done state
                end

                default: begin

                end
            endcase
        end
    end



endmodule


module cdc_pulse_sync (
    input  wire rst_n,

    input  wire src_clk,       // Source clock (e.g., TCK)
    input  wire dst_clk,       // Destination clock (e.g., CPU clk)
    input  wire src_pulse,     // 1-cycle pulse in source domain
    output wire dst_pulse      // 1-cycle pulse in destination domain
);

    // Stage 1: Generate toggle flag in source domain
    reg toggle_src;
    always @(posedge src_clk) begin
        if (!rst_n) 
            toggle_src <= 1'b0; // Reset toggle flag
        if (src_pulse)
            toggle_src <= ~toggle_src;
    end

    // Stage 2: Synchronize toggle into destination domain
    reg [2:0] sync_reg;
    always @(posedge dst_clk) begin
        if (!rst_n) begin
            sync_reg <= 3'b0; // Reset synchronization register
        end else begin 
            sync_reg <= {sync_reg[1:0], toggle_src};
        end
    end

    // Stage 3: Detect toggle change → generate single pulse in destination
    assign dst_pulse = (sync_reg[2] ^ sync_reg[1]);

endmodule


module cdc_signal_sync #(
    parameter WIDTH = 32 // Width of the signal to synchronize
)(
    input  wire rst_n,

    input  wire src_clk,              // Source clock (e.g., TCK)
    input  wire dst_clk,              // Destination clock (e.g., CPU clk)
    input  wire [WIDTH-1:0] src_data, // Data in source domain
    output reg  [WIDTH-1:0] dst_data  // Synchronized data in destination domain
);

    // Intermediate registers for synchronization
    reg [WIDTH-1:0] sync_stage1;
    reg [WIDTH-1:0] sync_stage2;

    // Synchronize data into destination domain
    always @(posedge dst_clk) begin
        if (!rst_n) begin
            sync_stage1 <= '0; // Reset stage 1
            sync_stage2 <= '0; // Reset stage 2
            dst_data <= '0;    // Reset output data
        end
        else begin
            sync_stage1 <= src_data;      // First stage
            sync_stage2 <= sync_stage1;  // Second stage
            dst_data <= sync_stage2;     // Output synchronized data
        end
    end

endmodule