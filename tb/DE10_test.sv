`timescale 1ns / 1ps


module DE10Top #(
    parameter MEM_ADDR_WIDTH = 10, // number of words addressable
    parameter MEM_DATA_WIDTH = 32 // 32 bits per word
)

(
    input logic pll_clk,
    input logic rst_n,

    input logic uart_rx,
    output logic uart_tx,

    output logic [6:0] seg_out1,
    output logic [6:0] seg_out2,
    output logic [6:0] seg_out3,
    output logic [6:0] seg_out4,
    output logic [6:0] seg_out5,
    output logic [6:0] seg_out6,

    output logic led,
    output logic mem_ctrl
);

    localparam MEM_SIZE = 128;

    logic [MEM_ADDR_WIDTH-1:0] jtag_addr;
    logic [31:0] jtag_wdata;
    logic [31:0] jtag_rdata;
    logic        jtag_we;
    logic        jtag_req_pulse;
    logic        jtag_en;
    logic        jtag_ack; 
    
    reg  [31:0] mem_out;
    logic [MEM_ADDR_WIDTH-1:0] mem_addr;
    logic [MEM_ADDR_WIDTH-1:0] address;
    logic [31:0] mem_wdata;
    logic [31:0] mem_rdata;
    logic        mem_we;
    logic        mem_control_enable;
    logic        cpu_rst_n;

    logic tck;
    logic tms;
    logic tdi;
    logic tdo;

    logic [MEM_ADDR_WIDTH-1:0] in_address;
    assign in_address = 0; // Not used in this testbench, but required for module interface
    

    logic clk;

	pll_clk clkgen_i(
		.inclk0(pll_clk),
		.c0(clk)
	);

    reg [31:0] memory[0:MEM_SIZE-1] /* verilator public */; 

    // initial begin
    //     $readmemh("data1.hex", memory);
    // end

    // Instruction memory read logic
    always_ff @(posedge clk) begin
        if (mem_we) begin
            memory[address] <= mem_wdata;
        end else begin
            mem_out <= memory[address];
        end
    end

    // Instantiate the JTAG-UART bridge
    jtag_uart_bridge jtag_uart_inst (
        .clk      (clk),
        .rst      (~rst_n),
        .uart_rx  (uart_rx),
        .uart_tx  (uart_tx),
        .TCK      (tck),
        .TMS      (tms),
        .TDI      (tdi),
        .TDO      (tdo)
    );

    // Instantiate the JTAG controller
    JTAG #(
        .MEM_ADDR_WIDTH(MEM_ADDR_WIDTH),
        .MEM_DATA_WIDTH(MEM_DATA_WIDTH)
    ) jtag (
        .tck(tck),
        .tms(tms),
        .tdi(tdi),
        .tdo(tdo),
        .rst_n(rst_n),

        .jtag_addr(jtag_addr),
        .jtag_wdata(jtag_wdata),
        .jtag_rdata(jtag_rdata),
        .jtag_we(jtag_we),
        .jtag_req_pulse(jtag_req_pulse),
        .jtag_en(jtag_en),
        .jtag_ack(jtag_ack)
    );

        
    // Instantiate the Programming controller
    Programming_controller #(
        .MEM_ADDR_WIDTH(MEM_ADDR_WIDTH),
        .MEM_DATA_WIDTH(MEM_DATA_WIDTH)
    )prog_ctrl(
        .clk(clk),
        .rst_n(rst_n),
        .tck(tck),
        .jtag_en(jtag_en),
        .jtag_req_pulse(jtag_req_pulse),
        .jtag_we(jtag_we),
        .jtag_addr(jtag_addr),
        .jtag_wdata(jtag_wdata),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .mem_rdata(mem_out),
        .mem_we(mem_we),
        .mem_control_enable(mem_control_enable),
        .jtag_rst_n(cpu_rst_n),
        .jtag_rdata(jtag_rdata),
        .jtag_ack(jtag_ack)
    );




    assign out_data = mem_control_enable ? mem_out : 32'b0;

    assign address = mem_control_enable ?  mem_addr : in_address;


    // Seven-segment display logic

    hexTo7seg seg1 (.hex(memory[0][3:0]  ),.seg(seg_out1));
    hexTo7seg seg2 (.hex(memory[0][31:28]),.seg(seg_out2));
    hexTo7seg seg3 (.hex(memory[1][3:0]  ),.seg(seg_out3));
    hexTo7seg seg4 (.hex(memory[1][31:28]),.seg(seg_out4));
    hexTo7seg seg5 (.hex(memory[2][3:0]  ),.seg(seg_out5));
    hexTo7seg seg6 (.hex(memory[2][31:28]),.seg(seg_out6));


    // LED signals (for debugging)

    localparam CLK_FREQ = 10_000_000; // 10 MHz

    logic [$clog2(CLK_FREQ/2)-1:0] counter;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter <= 0;
            led <= 0;
        end else begin
            counter <= counter + 1;
            if(counter == CLK_FREQ/2 - 1) begin
                counter <= 0;
                led <= ~led; // Toggle LED every 0.5 seconds
            end
        end
    end

    assign mem_ctrl = mem_control_enable;

endmodule

module hexTo7seg (
    input  logic [3:0] hex,
    output logic [6:0] seg
);

    always_comb begin
        case (hex)
            // seg bits are indexed as follows:
            // seg[0] = top (a)
            // seg[1] = top-right (b)
            // seg[2] = bottom-right (c)
            // seg[3] = bottom (d)
            // seg[4] = bottom-left (e)
            // seg[5] = top-left (f)
            // seg[6] = middle (g)
            // Using active-low encoding (0 lights a segment), seg[0]=top .. seg[6]=middle
            4'h0: seg = 7'b1000000; // 0
            4'h1: seg = 7'b1111001; // 1
            4'h2: seg = 7'b0100100; // 2
            4'h3: seg = 7'b0110000; // 3
            4'h4: seg = 7'b0011001; // 4
            4'h5: seg = 7'b0010010; // 5
            4'h6: seg = 7'b0000010; // 6
            4'h7: seg = 7'b1111000; // 7
            4'h8: seg = 7'b0000000; // 8
            4'h9: seg = 7'b0010000; // 9
            4'hA: seg = 7'b0001000; // A
            4'hB: seg = 7'b0000011; // b
            4'hC: seg = 7'b0100111; // C
            4'hD: seg = 7'b0100001; // d
            4'hE: seg = 7'b0000110; // E
            4'hF: seg = 7'b0001110; // F
            default: seg = 7'b0111111; // invalid input
        endcase
    end
    
endmodule