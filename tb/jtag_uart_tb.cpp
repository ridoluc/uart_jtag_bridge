
#include "VTop.h"
#include "verilated.h"
#include <verilated_vcd_c.h> // Add this line
#include "VTop___024root.h"
#include <iostream>
#include <iomanip> 
#include <vector>
#include <cassert>
#include <fstream>
#include <vector>
#include <cstdint>
#include "jtag_driver.h"

#define CLK_DIV 2

// Instruction Register (IR) values
#define IR_NOP   0x0
#define IR_MEM_CTRL 0x1 
#define IR_WRITE 0x2
#define IR_READ  0x3
#define IR_DONE  0x4 
// Parameters as compiler defines
#define ADDR_W 10
#define DATA_W 32
#define DR_W (ADDR_W + DATA_W)

#define UART_BAUD_COUNT 87

vluint64_t main_time = 0;

VerilatedVcdC* tfp = nullptr; 

JTAGDriver jtag_driver;

void clk_tick(VTop* top) {
    top->eval();
    if (tfp && main_time > 0) tfp->dump(main_time*10-2);
    top->clk = 1;
    top->eval();
    if (tfp) tfp->dump(main_time*10);

    top->clk = 0;
    top->eval();
    if (tfp) {
        tfp->dump(main_time*10+5); 
        tfp->flush();
    }
    main_time++;
}



bool uart_rx(VTop* top, char* _data) {
    char data = 0;
    int count = 0;


    int start_time = main_time;
    bool res = false;
    bool prev_uart_tx = top->uart_tx;

    // Wait for the UART TX start signal (falling edge detection)
    while ((prev_uart_tx == top->uart_tx || top->uart_tx == 1) && main_time - start_time < 1000) {
        prev_uart_tx = top->uart_tx;
        clk_tick(top); // Continue clocking until data is received
    }
    if (main_time - start_time >= 5000) {
        std::cerr << "UART RX Error: No start signal detected within timeout." << std::endl;
        return 0; // Error condition
    }

    // Read start bit
    while(count++ < UART_BAUD_COUNT) {
        clk_tick(top);

        if(count == UART_BAUD_COUNT / 2 && top->uart_tx != 0) {
            std::cerr << "UART RX Error: Start bit not detected." << std::endl;
            return 0; // Error condition
        }
    }
    // Read data bits
    for(int i = 0; i < 8; i++) {
        count = 0;
        while(count++ < UART_BAUD_COUNT) {
            clk_tick(top);
            if(count == UART_BAUD_COUNT / 2){
                data |= (top->uart_tx << i);
            }
        }
    }

    *_data = data;

    return 1;
}

void uart_tx(VTop* top, char data) {
    int count = 0;
    top->uart_rx = 1; // Ensure UART RX is high before starting transmission
    clk_tick(top);
    clk_tick(top);

    // Send start bit
    top->uart_rx = 0; // Start bit is low
    while (count++ < UART_BAUD_COUNT) {
        clk_tick(top);
    }
    
    // Send data bits
    for (int i = 0; i < 8; i++) {
        top->uart_rx = (data >> i) & 1;
        count = 0;
        while (count++ < UART_BAUD_COUNT) {
            clk_tick(top);
        }
    }

    // Send stop bit
    top->uart_rx = 1; // Stop bit is high
    count = 0;
    while (count++ < UART_BAUD_COUNT) {
        clk_tick(top);

    }

    std::cout << "Sent byte: 0b" << std::bitset<8>(data) << std::endl;

}

void prog_mode_on(VTop* top) {
	std::vector<uint8_t> jtag_stream;

	jtag_driver.shift_instruction(IR_MEM_CTRL, 4); 
	jtag_stream = jtag_driver.get_stream();

	for (auto byte : jtag_stream) {
		uart_tx(top, byte);
	}

	jtag_driver.clear_stream();
}

void prog_mode_off(VTop* top) {
	std::vector<uint8_t> jtag_stream;

	jtag_driver.shift_instruction(IR_DONE, 4); 
	jtag_stream = jtag_driver.get_stream();

	for (auto byte : jtag_stream) {
		uart_tx(top, byte);
	}

	jtag_driver.clear_stream();
}

void write_mem(VTop* top, uint32_t addr, uint32_t data) {
	std::vector<uint8_t> jtag_stream;

	jtag_driver.build_write_mem(IR_WRITE, 4, addr, ADDR_W, data, DATA_W); 
	jtag_stream = jtag_driver.get_stream();

	for (auto byte : jtag_stream) {
		uart_tx(top, byte);
	}

	jtag_driver.clear_stream();
}

void read_mem(VTop* top, uint32_t addr) {
	std::vector<uint8_t> jtag_stream;

	jtag_driver.build_read_mem(IR_READ, 4, addr, ADDR_W); 
	jtag_stream = jtag_driver.get_stream();

	for (auto byte : jtag_stream) {
		uart_tx(top, byte);
	}

	jtag_driver.clear_stream();

    char data_byte;
    std::cout << "Read Data: 0x";
    for (int i = 0; i < 4; i++) {
        if (!uart_rx(top, &data_byte)) {
            std::cerr << "UART RX Error during memory read." << std::endl;
            return;
        }
        std::cout << std::hex << std::setw(2) << std::setfill('0') << (static_cast<uint32_t>(data_byte) & 0xFF);
    }
    std::cout << std::dec << std::endl;
}

void reset_jtag(VTop* top) {
    std::vector<uint8_t> jtag_stream;
    // Move to Test-Logic-Reset state
    jtag_driver.append_repeat(1, 0, 5); // 5 TMS high cycles
    // Move to Run-Test/Idle state
    jtag_driver.append_repeat(0, 0, 1); // 1 TMS low cycle

    jtag_stream = jtag_driver.get_stream();

    for (auto byte : jtag_stream) {
        uart_tx(top, byte);
    }
    jtag_driver.clear_stream();
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true); // Enable tracing
    VTop* top = new VTop;
    

    tfp = new VerilatedVcdC; // Create trace object
    top->trace(tfp, 99);     // Trace 99 levels of hierarchy
    tfp->open("waveform.vcd"); // Open VCD file


    // Reset sequence
    top->rst_n = 0;
    for (int i = 0; i < 5; i++) {
        clk_tick(top);
    }
    top->rst_n = 1; 

    // Reset JTAG TAP controller
    std::cout << "Resetting JTAG TAP controller..." << std::endl;
    reset_jtag(top);

    // Enter programming mode
    std::cout << "Entering programming mode..." << std::endl;
    prog_mode_on(top);  

    // Write some data to memory
    std::cout << "Writing data to memory..." << std::endl;
    write_mem(top, 0x10, 0xDEADBEEF);
    write_mem(top, 0x14, 0xCAFEBABE);
    // Read back the data
    std::cout << "Reading data from memory..." << std::endl;
    read_mem(top, 0x10);
    read_mem(top, 0x14);    

    // Exit programming mode
    std::cout << "Exiting programming mode..." << std::endl;
    prog_mode_off(top);

    tfp->close(); // Close VCD file
    delete tfp;
    delete top;
    return 0;
}
