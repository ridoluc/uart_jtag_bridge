"""
JTAGProg.py

Python port of the JTAG driver and a small UART interface using pyserial.

Features:
- JTAGDriver: builds TMS/TDI bitstreams and packs them into bytes (4-bit nibbles
  per the C++ testbench packing: high nibble = TMS(3..0), low nibble = TDI(3..0)).
- JTAGProg: opens a serial port and sends/receives bytes to/from the UART bridge
  (works with FTDI or CP210x devices exposed as serial ports via drivers).
- helper to load 32-bit hex words from a text file.

This is intended to be used from the host side to drive the DUT's UART-JTAG
bridge.
"""

import time
import serial
import argparse
from typing import List, Optional


# JTAG constants matching the C++ defines
IR_NOP = 0x0
IR_MEM_CTRL = 0x1
IR_WRITE = 0x2
IR_READ = 0x3
IR_DONE = 0x4

ADDR_W = 10
DATA_W = 32
DR_W = ADDR_W + DATA_W


class JTAGDriver:
    def __init__(self):
        self.tms_bits: List[bool] = []
        self.tdi_bits: List[bool] = []

    def clear_stream(self):
        self.tms_bits.clear()
        self.tdi_bits.clear()

    @staticmethod
    def _pack_jtag_nibble(tms: int, tdi: int) -> int:
        return ((tms & 0x0F) << 4) | (tdi & 0x0F)

    def get_stream(self) -> List[int]:
        if len(self.tms_bits) != len(self.tdi_bits):
            raise RuntimeError("TMS/TDI size mismatch")

        out = []
        n = len(self.tms_bits)
        for i in range(0, n, 4):
            tms = 0
            tdi = 0
            for b in range(4):
                if i + b < n:
                    if self.tms_bits[i + b]:
                        tms |= (1 << b)
                    if self.tdi_bits[i + b]:
                        tdi |= (1 << b)
            out.append(self._pack_jtag_nibble(tms, tdi))

        return out

    def append_bits(self, tms_bits: List[bool], tdi_bits: List[bool]):
        if len(tms_bits) != len(tdi_bits):
            raise RuntimeError("TMS/TDI size mismatch")
        self.tms_bits.extend(tms_bits)
        self.tdi_bits.extend(tdi_bits)

    def append_repeat(self, tms: bool, tdi: bool, count: int):
        for _ in range(count):
            self.tms_bits.append(bool(tms))
            self.tdi_bits.append(bool(tdi))

    def shift_value(self, value: int, bitlen: int, exit_after: bool = True):
        for i in range(bitlen):
            bit = bool((value >> i) & 1)
            is_last_bit = (i == bitlen - 1)
            self.tms_bits.append(bool(exit_after and is_last_bit))
            self.tdi_bits.append(bit)

    def shift_instruction(self, ir: int, ir_len: int):
        # Move to Shift-IR
        self.append_repeat(1, 0, 2)  # Select-DR -> Select-IR
        self.append_repeat(0, 0, 1)  # Capture-IR
        self.append_repeat(0, 0, 1)  # Shift-IR

        # Shift IR LSB first
        self.shift_value(ir, ir_len, False)

        self.append_repeat(1, 0, 1)  # Exit IR
        self.append_repeat(1, 0, 1)  # Update-IR
        self.append_repeat(0, 0, 1)  # Run-Test/Idle

    def build_write_mem(self, ir: int, ir_len: int, dr_addr: int, dr_addr_len: int, dr_data: int, dr_data_len: int):
        self.shift_instruction(ir, ir_len)
        self.append_repeat(1, 0, 1)  # Select-DR
        self.append_repeat(0, 0, 1)  # Capture-DR
        self.append_repeat(0, 0, 1)  # Shift-DR

        # Shift DR data then address (LSB first as in the C++ driver)
        self.shift_value(dr_data, dr_data_len, False)
        self.shift_value(dr_addr, dr_addr_len, False)

        # Return to Run-Test/Idle
        self.append_repeat(1, 0, 1)  # Exit DR
        self.append_repeat(1, 0, 1)  # Update-DR
        self.append_repeat(0, 0, 1)  # Run-Test/Idle

    def build_read_mem(self, ir: int, ir_len: int, dr_addr: int, dr_addr_len: int):
        self.shift_instruction(ir, ir_len)
        self.append_repeat(1, 0, 1)  # Select-DR
        self.append_repeat(0, 0, 1)  # Capture-DR
        self.append_repeat(0, 0, 1)  # Shift-DR

        # Shift DR address
        self.shift_value(dr_addr, dr_addr_len, False)

        # Return to Run-Test/Idle
        self.append_repeat(1, 0, 1)  # Exit DR
        self.append_repeat(1, 0, 1)  # Update-DR
        self.append_repeat(0, 0, 1)  # Run-Test/Idle

    def shift_out_data(self, bitlen: int):
        # Wait for memory to process read (arbitrary cycles)
        self.append_repeat(0, 0, 5)

        # Move to Shift-DR to read data
        self.append_repeat(1, 0, 1)  # Select-DR
        self.append_repeat(0, 0, 1)  # Capture-DR
        self.append_repeat(0, 0, 1)  # Shift-DR

        # It's up to the testbench/host to clock out dummy bits if needed.

    def shift_out_data_exit(self):
        # Return to Run-Test/Idle after reading data
        self.append_repeat(1, 0, 1)  # Exit DR
        self.append_repeat(1, 0, 1)  # Update-DR
        self.append_repeat(0, 0, 2)  # Run-Test/Idle


class JTAGProg:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 1.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.01)

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send_bytes(self, data: bytes):
        # Write and flush
        self.ser.write(data)
        self.ser.flush()

    def send_jtag_driver(self, driver: JTAGDriver):
        stream = driver.get_stream()
        data = bytes(stream)
        print(f"Sending {len(data)} bytes to {self.port}")
        self.send_bytes(data)

    def read_bytes(self, count: int, timeout_s: float = 1.0) -> bytes:
        deadline = time.time() + timeout_s
        buf = bytearray()
        while len(buf) < count and time.time() < deadline:
            chunk = self.ser.read(count - len(buf))
            if chunk:
                buf.extend(chunk)
        return bytes(buf)

    # High level helpers mirroring the C++ testbench actions
    def reset_jtag(self):
        drv = JTAGDriver()
        drv.append_repeat(1, 0, 5)
        drv.append_repeat(0, 0, 1)
        self.send_jtag_driver(drv)

    def prog_mode_on(self):
        drv = JTAGDriver()
        drv.shift_instruction(IR_MEM_CTRL, 4)
        self.send_jtag_driver(drv)

    def prog_mode_off(self):
        drv = JTAGDriver()
        drv.shift_instruction(IR_DONE, 4)
        self.send_jtag_driver(drv)

    def write_mem(self, addr: int, data: int):
        drv = JTAGDriver()
        drv.build_write_mem(IR_WRITE, 4, addr, ADDR_W, data, DATA_W)
        self.send_jtag_driver(drv)

    def read_mem(self, addr: int, expect_response_bytes: int = 6, resp_timeout: float = 1.0) -> bytes:
        # Build read command and send
        print(f"Requesting read from addr=0x{addr:02X}")
        drv = JTAGDriver()
        drv.build_read_mem(IR_READ, 4, addr, ADDR_W)
        self.send_jtag_driver(drv)
        # Prepare to shift out read data: send pre-shift control bytes, then
        # send dummy bytes (0x00) while reading one response byte per dummy
        # byte, then send the post-shift (exit) control bytes.

        # time.sleep(0.015)


        # pre-shift (enter Shift-DR and any wait cycles)
        drv_pre = JTAGDriver()
        drv_pre.shift_out_data(DR_W)
        pre_stream = drv_pre.get_stream()
        if pre_stream:
            self.send_bytes(bytes(pre_stream))
        else:
            print("Warning: no pre-shift stream generated")


        self.ser.reset_input_buffer()
        # send dummy bytes one at a time and read one response byte per dummy
        # time.sleep(0.005)
        # self.ser.reset_input_buffer()


        resp_buf = bytearray()
        for i in range(expect_response_bytes):
            # send two dummy bytes (TMS=0,TDI=0 for 4 cycles packed as 0x00)
            # self.ser.reset_input_buffer()

          

            b = self.read_bytes(1, timeout_s=resp_timeout)

            self.send_bytes(bytes([0x00]))
            self.send_bytes(bytes([0x00]))  

            print(f"Received byte {i}: {b.hex() if b else 'timeout'}")
            if not b:
                # timed out, stop early
                break
            resp_buf.extend(b)

        # post-shift (exit DR back to Run-Test/Idle)
        drv_post = JTAGDriver()
        drv_post.shift_out_data_exit()
        post_stream = drv_post.get_stream()
        if post_stream:
            self.send_bytes(bytes(post_stream))
        else:
            print("Warning: no post-shift stream generated")

        return bytes(resp_buf)


    # def read_mem(self, addr: int, expect_response_bytes: int = 6, resp_timeout: float = 1.0) -> bytes:
    #     # 1. Clear any junk left from previous failed runs
    #     self.ser.reset_input_buffer()
        
    #     # 2. Send Command
    #     drv = JTAGDriver()
    #     drv.build_read_mem(IR_READ, 4, addr, ADDR_W)
    #     self.send_jtag_driver(drv)

    #     # 3. Enter Shift State
    #     drv_pre = JTAGDriver()
    #     drv_pre.shift_out_data(DR_W)
    #     self.send_bytes(bytes(drv_pre.get_stream()))

    #     # 4. Batch request: Send all dummy bytes at once
    #     # If your protocol requires two 0x00 per 1 byte of response:
    #     dummy_payload = bytes([0x00] * (expect_response_bytes * 2))
    #     self.send_bytes(dummy_payload)

    #     # 5. Block until all data is received
    #     # This is much more robust than a loop
    #     resp_buf = self.ser.read(expect_response_bytes)

    #     # 6. Exit Shift State
    #     drv_post = JTAGDriver()
    #     drv_post.shift_out_data_exit()
    #     self.send_bytes(bytes(drv_post.get_stream()))

    #     return resp_buf

def load_32bit_hex_file(path: str) -> List[int]:
    out: List[int] = []
    with open(path, "r") as f:
        for ln in f:
            s = ln.strip()
            if not s:
                continue
            # allow 0x prefix or plain hex
            if s.startswith("0x") or s.startswith("0X"):
                s = s[2:]
            # remove comments after a space
            if " " in s:
                s = s.split(" ", 1)[0]
            try:
                val = int(s, 16) & 0xFFFFFFFF
            except ValueError:
                raise ValueError(f"Invalid hex in data file: {ln.strip()}")
            out.append(val)
    return out


def reconstruct_data_from_response(resp: bytes) -> int:
    # Reconstruct data as in the C++ testbench (bytes 0..3 are data LSB..MSB)
    # return value and address separately
    data_val = 0
    for j in range(3, -1, -1):
        data_val <<= 8
        data_val |= resp[j]
    addr_val = (resp[5] << 8) | resp[4]
    return data_val, addr_val

def main():
    parser = argparse.ArgumentParser(description="Host JTAG UART programmer")
    parser.add_argument("port", help="Serial port to open (e.g. /dev/ttyUSB0)")
    parser.add_argument("datafile", help="Text file containing 32-bit hex words, one per line")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate for the serial port")
    parser.add_argument("--start-addr", type=int, default=0)
    parser.add_argument("--verify-count", type=int, default=8, help="How many words to read back and print")
    args = parser.parse_args()

    words = load_32bit_hex_file(args.datafile)
    p = JTAGProg(args.port, baud=args.baud)
    try:
        print("Resetting JTAG TAP...")
        p.reset_jtag()
        time.sleep(0.05)
        print("Entering programming mode...")
        p.prog_mode_on()

        # write words sequentially starting at start-addr
        # for i, w in enumerate(words):
        #     addr = args.start_addr + i
        #     print(f"Writing addr=0x{addr:02X} data=0x{w:08X}")
        #     p.write_mem(addr, w)
        #     time.sleep(0.002)

        print("Verifying first entries...")
        for i in range(min(args.verify_count, len(words))):
            addr = args.start_addr + i
            resp = p.read_mem(addr, expect_response_bytes=6, resp_timeout=0.5)
            if len(resp) < 6:
                print(f"Addr 0x{addr:02X}: no response (len={len(resp)})")
                continue
            # Reconstruct data as in the C++ testbench (bytes 0..3 are data LSB..MSB)
            data_val, addr_resp = reconstruct_data_from_response(resp)
            print(f"Addr read: 0x{addr_resp:04X} Data: 0x{data_val:08X}")


        # Read from address 0x02 and print the response
        # resp = p.read_mem(2, expect_response_bytes=6, resp_timeout=0.5)
        # resp = p.read_mem(3, expect_response_bytes=6, resp_timeout=0.5)
        # if len(resp) < 6:
        #     print(f"Addr 3: no response (len={len(resp)})")
        # else:
        #     data_val, addr_resp = reconstruct_data_from_response(resp)
        #     print(f"Addr read: 0x{addr_resp:04X} Data: 0x{data_val:08X}")
        # # resp = p.read_mem(0, expect_response_bytes=6, resp_timeout=0.5)

        # resp = p.read_mem(1, expect_response_bytes=6, resp_timeout=0.5)
        # if len(resp) < 6:
        #     print(f"Addr 1: no response (len={len(resp)})")
        # else:
        #     data_val, addr_resp = reconstruct_data_from_response(resp)
        #     print(f"Addr read: 0x{addr_resp:04X} Data: 0x{data_val:08X}")


        print("Exiting programming mode...")
        p.prog_mode_off()
    finally:
        p.close()


if __name__ == "__main__":
    main()
