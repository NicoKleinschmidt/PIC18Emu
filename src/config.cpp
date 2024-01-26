#include "config.hpp"

bool configuration_check_bit(const config_bit_t &bit, bus_reader_t<uint32_t, uint8_t> read_bus)
{
    uint8_t value = read_bus(bit.file_address);
    bool set = (value & (1 << bit.bit));

    if (bit.invert)
        return !set;
    else
        return set;
}
