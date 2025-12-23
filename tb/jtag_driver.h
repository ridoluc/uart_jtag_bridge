#ifndef JTAG_DRIVER_H
#define JTAG_DRIVER_H

#include <cstdint>
#include <vector>

class JTAGDriver
{

    std::vector<uint8_t> stream;
    inline uint8_t pack_jtag_nibble(uint8_t tms, uint8_t tdi) { return ((tms & 0x0F) << 4) | (tdi & 0x0F); }

public:
    JTAGDriver() = default;
    ~JTAGDriver() = default;

    const std::vector<uint8_t> &get_stream() const { return stream; }

    void clear_stream() { stream.clear(); } 

    void append_bits(const std::vector<bool> &tms_bits,
                     const std::vector<bool> &tdi_bits);

    void append_repeat(bool tms, bool tdi,
                       size_t count);

    void shift_value(uint32_t value,
                     int bitlen,
                     bool exit_after);

    void shift_instruction(uint32_t ir,
                           int ir_len);

    void build_write_mem(uint32_t ir,
                         int ir_len,
                         uint32_t dr_addr,
                         int dr_addr_len,
                         uint32_t dr_data,
                         int dr_data_len);

    void build_read_mem(uint32_t ir,
                        int ir_len,
                        uint32_t dr_addr,
                        int dr_addr_len);
};

void JTAGDriver::append_bits(
    const std::vector<bool> &tms_bits,
    const std::vector<bool> &tdi_bits)
{
    size_t n = tms_bits.size();
    if (tdi_bits.size() != n)
        throw std::runtime_error("TMS/TDI size mismatch");

    for (size_t i = 0; i < n; i += 4)
    {
        uint8_t tms = 0;
        uint8_t tdi = 0;

        for (int b = 0; b < 4; b++)
        {
            if (i + b < n)
            {
                if (tms_bits[i + b])
                    tms |= (1 << b);
                if (tdi_bits[i + b])
                    tdi |= (1 << b);
            }
        }

        stream.push_back(pack_jtag_nibble(tms, tdi));
    }
}

void JTAGDriver::append_repeat(
    bool tms, bool tdi,
    size_t count)
{
    while (count > 0)
    {
        uint8_t tms_n = 0;
        uint8_t tdi_n = 0;

        for (int i = 0; i < 4 && count > 0; i++, count--)
        {
            if (tms)
                tms_n |= (1 << i);
            if (tdi)
                tdi_n |= (1 << i);
        }

        stream.push_back(pack_jtag_nibble(tms_n, tdi_n));
    }
}

void JTAGDriver::shift_value(uint32_t value,
                             int bitlen,
                             bool exit_after = true)
{
    std::vector<bool> tms;
    std::vector<bool> tdi;

    for (int i = 0; i < bitlen; i++)
    {
        tdi.push_back((value >> i) & 1);
        tms.push_back(exit_after && (i == bitlen - 1));
    }

    append_bits(tms, tdi);
}

void JTAGDriver::shift_instruction(uint32_t ir,
                                     int ir_len)
{
    // Move to Shift-IR
    append_repeat(1, 0, 2); // Select-DR → Select-IR
    append_repeat(0, 0, 1); // Capture-IR
    append_repeat(0, 0, 1); // Shift-IR

    // Shift IR
    shift_value(ir, ir_len, true);
    // Exit IR → Shift-DR
    append_repeat(1, 0, 1);
    append_repeat(0, 0, 1);
}

void JTAGDriver::build_write_mem(uint32_t ir,
                                 int ir_len,
                                 uint32_t dr_addr,
                                 int dr_addr_len,
                                 uint32_t dr_data,
                                 int dr_data_len)
{
    // Move to Shift-IR
    append_repeat(1, 0, 2); // Select-DR → Select-IR
    append_repeat(0, 0, 1); // Capture-IR
    append_repeat(0, 0, 1); // Shift-IR
    // Shift IR
    shift_value(ir, ir_len, true);
    // Exit IR → Shift-DR
    append_repeat(1, 0, 1);
    append_repeat(0, 0, 1);

    // Shift DR address
    shift_value(dr_addr, dr_addr_len, false);
    // Shift DR data
    shift_value(dr_data, dr_data_len, true);
    // Return to Run-Test/Idle
    append_repeat(1, 0, 1);
    append_repeat(0, 0, 1);
}

void JTAGDriver::build_read_mem(uint32_t ir,
                                int ir_len,
                                uint32_t dr_addr,
                                int dr_addr_len)
{
    // Move to Shift-IR
    append_repeat(1, 0, 2); // Select-DR → Select-IR
    append_repeat(0, 0, 1); // Capture-IR

    // Shift IR
    shift_value(ir, ir_len, true);
    // Exit IR → Shift-DR
    append_repeat(1, 0, 1);
    append_repeat(0, 0, 1);

    // Shift DR address
    shift_value(dr_addr, dr_addr_len, true);
    // Return to Run-Test/Idle
    append_repeat(1, 0, 1);
    append_repeat(0, 0, 1);

    shift_value(0, 32, true); // Dummy read to get data out
    shift_value(0, dr_addr_len, true); // Dummy read to get data out

    // Return to Run-Test/Idle
    append_repeat(1, 0, 1);
    append_repeat(0, 0, 1);
}

#endif // JTAG_DRIVER_H