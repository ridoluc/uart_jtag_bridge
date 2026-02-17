
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
    std::cout<<std::endl<< std::dec<<"Te: "<<main_time<<std::endl;

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
    if (main_time - start_time >= 1000) {
        std::cerr << "\nUART RX Error: No start signal detected within timeout." << std::endl;
        std::cerr << "Current time: " << main_time << ", UART TX: " << (int)top->uart_tx << std::endl;
        return 0; // Error condition
    }
    std::cout << "Start bit detected at time: " << main_time << std::endl;

    // Read start bit
    while(count++ < UART_BAUD_COUNT) {
        clk_tick(top);

        if(count == UART_BAUD_COUNT / 2 && top->uart_tx != 0) {
            std::cerr << "\nUART RX Error: Start bit not detected." << std::endl;
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

    // std::cout << "Sent byte: 0b" << std::bitset<8>(data) << std::endl;
    // for(int i =0; i<1000; i++) clk_tick(top); // small gap after transmission
}

// Transfer a byte with value 0 while checking for the RX start bit
void uart_read_byte(VTop* top, char* data) {
    bool prev_uart_tx = top->uart_tx;

    top->uart_rx = 1; // Ensure UART RX is high before starting transmission
    clk_tick(top);
    clk_tick(top);

    // Send first zero byte 
    int rx_baud_count = UART_BAUD_COUNT*10; // Start bit + 8 data bits + stop bit
    top->uart_rx = 0; // Start bit
    while(rx_baud_count-- > 0) {
        clk_tick(top);
        if(rx_baud_count < UART_BAUD_COUNT) top->uart_rx = 1; // Stop bit
    }

    // std::cout << "Time after sending first zero byte: " << main_time << std::endl;
    //Send second zero byte to ensure we are in the middle of the stop bit of the first byte
    rx_baud_count = UART_BAUD_COUNT*10;
    top->uart_rx = 0; // Start bit
    while(rx_baud_count-- > 0) {
        clk_tick(top);
        if(rx_baud_count < UART_BAUD_COUNT){ 
            top->uart_rx = 1; // Stop bit
            if(prev_uart_tx == 1 && top->uart_tx == 0) {
                // std::cout << "Start bit detected at time: " << main_time << std::endl;
                break; // Start bit detected
            }
            prev_uart_tx = top->uart_tx;
        }
    }   

    // std::cout << "Time after sending second zero byte: " << main_time << std::endl;
    int tx_baud_count = UART_BAUD_COUNT;
    // Read start bit
    while(tx_baud_count-- > 0) {
        clk_tick(top);
    }
    // Read data bits
    char data_byte = 0;
    for(int i = 0; i < 8; i++) {
        tx_baud_count = UART_BAUD_COUNT;
        while(tx_baud_count-- > 0) {
            clk_tick(top);
            if(tx_baud_count == UART_BAUD_COUNT / 2) {
                data_byte |= (top->uart_tx << i);
            }
        }
    }
    *data = data_byte;

    // Wait for stop bit
    tx_baud_count = UART_BAUD_COUNT;
    while(tx_baud_count-- > 0) {
        clk_tick(top);
    }
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

void read_mem(VTop* top, uint32_t addr, std::vector<char>* received_bytes_ptr = nullptr) {
	std::vector<uint8_t> jtag_stream;

	jtag_driver.build_read_mem(IR_READ, 4, addr, ADDR_W); 
	jtag_stream = jtag_driver.get_stream();

	for (auto byte : jtag_stream) {
		uart_tx(top, byte);
	}

	jtag_driver.clear_stream();

    jtag_driver.shift_out_data(DR_W);
    jtag_driver.shift_out_data_exit();
    jtag_stream = jtag_driver.get_stream(); 

    char data_byte;
    
    uart_tx(top, jtag_stream[0]);
    uart_tx(top, jtag_stream[1]);

    std::vector<char> received_bytes;
    for (size_t i = 0; i < 6; i++)
    {
        uart_read_byte(top, &data_byte);
        received_bytes.push_back(data_byte);
    }

    if (received_bytes_ptr != nullptr) {
        *received_bytes_ptr = received_bytes;
    }



    uart_tx(top, jtag_stream[2]);
	jtag_driver.clear_stream();

}

void print_received_data(const std::vector<char>& received_bytes) {
    int data_address = ((uint8_t)received_bytes[received_bytes.size()-1] << 8); 
    data_address |= (uint8_t)received_bytes[received_bytes.size()-2];

    uint32_t data_value = 0;
    for (int i = 3; i >= 0; i--)    {
        data_value <<= 8;
        data_value |= (uint8_t)received_bytes[i];
    }

    std::cout<< "ADDR: 0x" << data_address << "\tDATA: 0x" << std::hex << data_value << std::dec << std::endl;

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
    // // Read back the data
    std::cout << "Reading data from memory..." << std::endl;
    std::vector<char> received_bytes;
    read_mem(top, 0x10, &received_bytes);
    print_received_data(received_bytes);
    read_mem(top, 0x14, &received_bytes);
    print_received_data(received_bytes);    


    // generate random data for the first 100 addresses and read them back checking for correctness
    std::cout << "Testing random data writes and reads..." << std::endl;
    std::vector<int> test_data;
    for (uint32_t addr = 0; addr < 100; addr++) {
        uint32_t data = rand(); // Generate random data
        test_data.push_back(data);
        write_mem(top, addr, data); // Write to memory
    }

    int errors = 0;
    for (uint32_t addr = 0; addr < 30; addr++) {
        std::cout << "Reading from address: 0x" << std::hex << addr << std::dec << std::endl;
        std::vector<char> received_bytes;
        read_mem(top, addr, &received_bytes); // Read from memory

        uint32_t expected_data = test_data[addr];
        uint32_t received_data = 0;
        for (int i = 3; i >= 0; i--)    {
            received_data <<= 8;
            received_data |= (uint8_t)received_bytes[i];
        }

        if (received_data != expected_data) {
            std::cout << "Error: Expected 0x" << std::hex << expected_data << ", got 0x" << received_data << std::dec << std::endl;
            errors++;
        }
    }

    std::cout << "Test completed with " << errors << " errors." << std::endl;

    // Exit programming mode
    std::cout << "Exiting programming mode..." << std::endl;
    prog_mode_off(top);

    tfp->close(); // Close VCD file
    delete tfp;
    delete top;
    return 0;
}


