`timescale 1ns / 1ps


module Top #(
    parameter MEM_ADDR_WIDTH = 10, // number of words addressable
    parameter MEM_DATA_WIDTH = 32 // 32 bits per word
)

(
    input wire clk,
    input wire rst_n,

    input wire uart_rx,
    output wire uart_tx,

    input wire [MEM_ADDR_WIDTH-1:0] in_address,
    output wire [31:0] out_data
);


   wire [MEM_ADDR_WIDTH-1:0] jtag_addr;
   wire [31:0] jtag_wdata;
   wire [31:0] jtag_rdata;
   wire        jtag_we;
   wire        jtag_req_pulse;
   wire        jtag_en;
   wire        jtag_ack; 

   reg  [31:0] mem_out;
   wire [MEM_ADDR_WIDTH-1:0] mem_addr;
   wire [MEM_ADDR_WIDTH-1:0] address;
   wire [31:0] mem_wdata;
   wire [31:0] mem_rdata;
   wire        mem_we;
   wire        mem_control_enable;
   wire        cpu_rst_n;

   logic tck;
   logic tms;
   logic tdi;
   logic tdo;


   reg [31:0] memory[0:(1<<MEM_ADDR_WIDTH)-1]; 

   // initial begin
   //     $readmemb("memory_load.bin", memory);
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
      .MEM_DATA_WIDTH(32)
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
      .MEM_DATA_WIDTH(32)
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




   assign out_data = mem_control_enable ? 32'b0: mem_out;

   assign address = mem_control_enable ?  mem_addr : in_address;


endmodule