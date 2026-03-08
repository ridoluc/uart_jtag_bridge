#!/usr/bin/env python3
"""
memory_rw_test.py

Chunked memory write/read tester for the UART-JTAG bridge.

Writes a large amount of 32-bit words in chunks that fit the target
memory (default 1024 words), measures write/read elapsed times, and
counts verification errors.

Example:
  python3 ProgrammerAPI/memory_rw_test.py /dev/ttyUSB0 --total-bytes 1048576

"""

import argparse
import time
import math
import random
import sys
from typing import List

from ProgrammerAPI.JTAGProg import JTAGProg, reconstruct_data_from_response, ADDR_W


def generate_chunk(seed: int, count: int) -> List[int]:
    r = random.Random(seed)
    return [r.getrandbits(32) for _ in range(count)]


def parse_args():
    parser = argparse.ArgumentParser(description="Chunked memory write/read tester")
    parser.add_argument("port", help="Serial port to open (e.g. /dev/ttyUSB0)")
    parser.add_argument("--total-bytes", type=int, default=4096, help="Total bytes to test (default 4KB)")
    parser.add_argument("--chunk-words", type=int, default=(1 << ADDR_W), help="Words per chunk (max memory).")
    parser.add_argument("--start-addr", type=int, default=0, help="Start address in target memory (wraps modulo memory size)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--per-write-delay", type=float, default=0.0, help="Optional delay (s) after each write")
    parser.add_argument("--per-read-delay", type=float, default=0.0, help="Optional delay (s) after each read")
    parser.add_argument("--base-seed", type=int, default=0, help="Base seed for deterministic test patterns")
    parser.add_argument("--no-verify", dest="verify", action="store_false", help="Skip read/verify step")
    parser.add_argument("--resp-timeout", type=float, default=0.5, help="Read response timeout per word (s)")
    return parser.parse_args()


def human_bytes(n: int) -> str:
    for unit in ["B", "KB", "MB", "GB"]:
        if n < 1024.0:
            return f"{n:.2f}{unit}"
        n /= 1024.0
    return f"{n:.2f}TB"


def main():
    args = parse_args()

    total_words = (args.total_bytes + 3) // 4
    max_mem_words = 1 << ADDR_W

    if args.chunk_words <= 0:
        args.chunk_words = max_mem_words
    if args.chunk_words > max_mem_words:
        print(f"Chunk size {args.chunk_words} exceeds device memory {max_mem_words}, capping to max")
        args.chunk_words = max_mem_words

    num_chunks = math.ceil(total_words / args.chunk_words)

    print(f"Testing {total_words} words ({human_bytes(total_words*4)}) in {num_chunks} chunk(s) of up to {args.chunk_words} words")
    print(f"Device memory size: {max_mem_words} words (ADDR_W={ADDR_W})")

    p = JTAGProg(args.port, baud=args.baud)
    try:
        print("Resetting JTAG TAP...")
        p.reset_jtag()
        time.sleep(0.05)
        print("Entering programming mode...")
        p.prog_mode_on()

        total_write_time = 0.0
        total_read_time = 0.0
        total_errors = 0
        total_written = 0

        for chunk_id in range(num_chunks):
            remaining = total_words - chunk_id * args.chunk_words
            this_count = min(args.chunk_words, remaining)
            # compute address offset for this chunk (wrap in device memory)
            address_offset = (args.start_addr + (chunk_id * args.chunk_words)) % max_mem_words

            # deterministic pattern per chunk
            chunk_seed = args.base_seed + chunk_id
            data_chunk = generate_chunk(chunk_seed, this_count)

            # Write chunk
            t0 = time.time()
            for i, w in enumerate(data_chunk):
                addr = (address_offset + i) % max_mem_words
                p.write_mem(addr, w)
                if args.per_write_delay > 0:
                    time.sleep(args.per_write_delay)
            t1 = time.time()
            write_time = t1 - t0
            total_write_time += write_time
            total_written += this_count

            print(f"Chunk {chunk_id+1}/{num_chunks}: wrote {this_count} words in {write_time:.3f}s ({(this_count*4)/write_time/1024:.2f} KB/s)")

            # Verify chunk if requested
            if args.verify:
                err_chunk = 0
                rt0 = time.time()
                for i, expected in enumerate(data_chunk):
                    addr = (address_offset + i) % max_mem_words
                    resp = p.read_mem(addr, expect_response_bytes=6, resp_timeout=args.resp_timeout)
                    if len(resp) < 6:
                        err_chunk += 1
                    else:
                        data_val, addr_resp = reconstruct_data_from_response(resp)
                        if addr_resp != addr or data_val != expected:
                            err_chunk += 1
                    if args.per_read_delay > 0:
                        time.sleep(args.per_read_delay)
                rt1 = time.time()
                read_time = rt1 - rt0
                total_read_time += read_time
                total_errors += err_chunk
                print(f"Chunk {chunk_id+1}/{num_chunks}: verify {this_count} words in {read_time:.3f}s, errors={err_chunk}")

        print("Exiting programming mode...")
        p.prog_mode_off()

        print("\nSummary:")
        print(f"  Total words: {total_written}")
        print(f"  Total bytes: {total_written*4} ({human_bytes(total_written*4)})")
        print(f"  Total write time: {total_write_time:.3f}s")
        if total_write_time > 0:
            print(f"  Write throughput: {(total_written*4)/1024/total_write_time:.2f} KB/s")
        print(f"  Total read time: {total_read_time:.3f}s")
        if total_read_time > 0:
            print(f"  Read throughput: {(total_written*4)/1024/total_read_time:.2f} KB/s")
        print(f"  Total errors: {total_errors}")

        if total_errors:
            print("One or more verification errors detected.")
            sys.exit(2)
        else:
            print("All data verified successfully.")

    finally:
        try:
            p.prog_mode_off()
        except Exception:
            pass
        p.close()


if __name__ == "__main__":
    main()
