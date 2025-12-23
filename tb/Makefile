# Project Name
PROJECT = Top


# Verilog Source Files (you can add more Verilog files here)
VERILOG_SOURCES = ../src/jtag_engine.sv \
						../src/jtag_uart_bridge.sv \
						../src/rx_fifo.sv \
						../src/sync_fifo.sv \
						../src/tx_fifo.sv \
						../src/uart_rx.sv \
						../src/uart_tx.sv \
						../src/uart_tx_from_fifo.sv \
						./TOP.sv	 \
						./JTAG.sv	 \
						./Programming_interface.sv	

# C++ Testbench File
TESTBENCH_CPP = ./jtag_uart_tb.cpp 

# Top module (this should match the name of top module in Verilog)
TOP_MODULE = $(PROJECT)

# Verilator Executable
VERILATOR = verilator

# Compiler Options
CXXFLAGS = -Wall -O2

# Verilator options
VOPTIONS = --public-flat-rw --public --trace 

# Directory for Verilator output files
OBJ_DIR = obj_dir

# The final executable name
TARGET = $(OBJ_DIR)/$(PROJECT)

# Detect OS (uname will return 'Darwin' for macOS, 'Linux' for WSL/Linux)
UNAME_S := $(shell uname -s)

# Verilator include paths
VERILATOR_INCLUDE = $(shell $(VERILATOR) --getenv VERILATOR_ROOT)/include
VERILATOR_VLTSTD_INCLUDE = $(VERILATOR_INCLUDE)/vltstd

# Add Verilator include paths to CXXFLAGS
CXXFLAGS += -I$(VERILATOR_INCLUDE) -I$(VERILATOR_VLTSTD_INCLUDE) -Wno-unused-variable

# Default rule to build the project
all: $(TARGET)

# Rule to run the simulation
run: all
	./$(TARGET)


# Compilation rule depending on platform
$(TARGET): $(VERILOG_SOURCES) $(TESTBENCH_CPP)
ifeq ($(UNAME_S), Darwin)
    # macOS specific rules
	$(VERILATOR) --cc $(VERILOG_SOURCES) --exe $(TESTBENCH_CPP) $(VOPTIONS) -CFLAGS "$(CXXFLAGS)" --top-module $(TOP_MODULE)
	$(MAKE) -j -C $(OBJ_DIR) -f V$(PROJECT).mk V$(PROJECT)
else ifeq ($(UNAME_S), Linux)
    # WSL/Linux specific rules
	$(VERILATOR) --cc $(VERILOG_SOURCES) --exe $(TESTBENCH_CPP) $(VOPTIONS) -CFLAGS "$(CXXFLAGS)" --top-module $(TOP_MODULE)
	$(MAKE) -j -C $(OBJ_DIR) -f V$(PROJECT).mk V$(PROJECT)
endif
	mv $(OBJ_DIR)/V$(PROJECT) $(TARGET)
	touch $(TARGET)

# Rule to generate the waveform
.PHONY:waves
waves: waveform.vcd
	@echo
	@echo "### WAVES ###"
	gtkwave waveform.vcd &


# Clean rule to remove generated files
clean:
	-rm -rf $(OBJ_DIR)
	-rm -f *.vcd

# Phony targets (not real files)
.PHONY: all clean run waves assembly