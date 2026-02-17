#ifndef JTAG_DRIVER_H
#define JTAG_DRIVER_H

#include <cstdint>
#include <vector>

class JTAGDriver
{

    // std::vector<uint8_t> stream;
    std::vector<bool> tms_bits;
    std::vector<bool> tdi_bits;
    inline uint8_t pack_jtag_nibble(uint8_t tms, uint8_t tdi) { return ((tms & 0x0F) << 4) | (tdi & 0x0F); }

public:
    JTAGDriver() = default;
    ~JTAGDriver() = default;

    const std::vector<uint8_t> get_stream();

    void clear_stream() { tms_bits.clear(); tdi_bits.clear(); } 

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

    void shift_out_data(int bitlen);
    void shift_out_data_exit();
};

const std::vector<uint8_t> JTAGDriver::get_stream()
{
    std::vector<uint8_t> byte_stream;
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

        byte_stream.push_back(pack_jtag_nibble(tms, tdi));
    }

    return byte_stream;
}

void JTAGDriver::append_bits(
    const std::vector<bool> &tms_bits,
    const std::vector<bool> &tdi_bits)
{
    if (tms_bits.size() != tdi_bits.size())
        throw std::runtime_error("TMS/TDI size mismatch");

    this->tms_bits.insert(this->tms_bits.end(),
                          tms_bits.begin(), tms_bits.end());
    this->tdi_bits.insert(this->tdi_bits.end(),
                          tdi_bits.begin(), tdi_bits.end());
}

void JTAGDriver::append_repeat(
    bool tms, bool tdi,
    size_t count)
{
    for (size_t i = 0; i < count; i++)
    {
        this->tms_bits.push_back(tms);
        this->tdi_bits.push_back(tdi);
    }
}

void JTAGDriver::shift_value(uint32_t value,
                             int bitlen,
                             bool exit_after = true)
{
    for (int i = 0; i < bitlen; i++)
    {
        bool bit = (value >> i) & 1;
        bool is_last_bit = (i == bitlen - 1);
        this->tms_bits.push_back(exit_after && is_last_bit);
        this->tdi_bits.push_back(bit);
    }
}

void JTAGDriver::shift_instruction(uint32_t ir,
                                     int ir_len)
{
    // Move to Shift-IR
    append_repeat(1, 0, 2); // Select-DR → Select-IR
    append_repeat(0, 0, 1); // Capture-IR
    append_repeat(0, 0, 1); // Shift-IR

    // Shift IR
    shift_value(ir, ir_len, false);

    append_repeat(1, 0, 1); // Exit IR
    append_repeat(1, 0, 1); // Update-IR 
    append_repeat(0, 0, 1); // Run-Test/Idle
}

void JTAGDriver::build_write_mem(uint32_t ir,
                                 int ir_len,
                                 uint32_t dr_addr,
                                 int dr_addr_len,
                                 uint32_t dr_data,
                                 int dr_data_len)
{
    // Move to Shift-IR
    // append_repeat(1, 0, 2); // Select-DR → Select-IR
    // append_repeat(0, 0, 1); // Capture-IR
    // append_repeat(0, 0, 1); // Shift-IR
    // // Shift IR
    // shift_value(ir, ir_len, false);
    // // Exit IR → Shift-DR
    // append_repeat(1, 0, 1);
    // append_repeat(0, 0, 1);

    shift_instruction(ir, ir_len);
    append_repeat(1, 0, 1); // Select-DR
    append_repeat(0, 0, 1); // Capture-DR
    append_repeat(0, 0, 1); // Shift-DR


    // Shift DR data
    shift_value(dr_data, dr_data_len, false);
    // Shift DR address
    shift_value(dr_addr, dr_addr_len, false);
    // Return to Run-Test/Idle
    append_repeat(1, 0, 1); // Exit DR
    append_repeat(1, 0, 1); // Update-DR
    append_repeat(0, 0, 1); // Run-Test/Idle
}

void JTAGDriver::build_read_mem(uint32_t ir,
                                int ir_len,
                                uint32_t dr_addr,
                                int dr_addr_len)
{
    shift_instruction(ir, ir_len);
    append_repeat(1, 0, 1); // Select-DR
    append_repeat(0, 0, 1); // Capture-DR
    append_repeat(0, 0, 1); // Shift-DR

    // Shift DR address
    shift_value(dr_addr, dr_addr_len, false);
    // Return to Run-Test/Idle
    append_repeat(1, 0, 1); // Exit DR
    append_repeat(1, 0, 1); // Update-DR
    append_repeat(0, 0, 1); // Run-Test/Idle

}


void JTAGDriver::shift_out_data(int bitlen)
{

    // Wait for memory to process read
    append_repeat(0, 0, 5); // Arbitrary wait cycles
    
    // Move to Shift-DR to read data
    append_repeat(1, 0, 1); // Select-DR
    append_repeat(0, 0, 1); // Capture-DR
    append_repeat(0, 0, 1); // Shift-DR

    // Shift out dummy values to read data
    // shift_value(0, bitlen, false); // Dummy read to get data out
    // // shift_value(0, dr_addr_len, true); // Dummy read to get data out

    // // Return to Run-Test/Idle
    // append_repeat(1, 0, 1); // Exit DR
    // append_repeat(1, 0, 1); // Update-DR
    // append_repeat(0, 0, 1); // Run-Test/Idle

}

// Return to Run-Test/Idle after reading data
void JTAGDriver::shift_out_data_exit()
{
    append_repeat(1, 0, 1); // Exit DR
    append_repeat(1, 0, 1); // Update-DR
    append_repeat(0, 0, 2); // Run-Test/Idle
}

#endif // JTAG_DRIVER_H
