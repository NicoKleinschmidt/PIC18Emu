#pragma once

#include "bus.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

template <size_t SIZE> struct memory_t
{
    uint16_t start_address;
    std::unique_ptr<std::array<uint8_t, SIZE>> data;
};

template <size_t SIZE> addr_read_result_t mem_read(const memory_t<SIZE> &mem, uint16_t addr)
{
    if (addr < mem.start_address || addr >= mem.start_address + SIZE)
        return addr_read_result_none;

    uint8_t value = (*mem.data)[addr - mem.start_address];
    return addr_read_result_t{.data = value, .mask = 0xFF};
}

template <size_t SIZE> addr_bit_mask_t mem_write(memory_t<SIZE> &mem, uint16_t addr, uint8_t val)
{
    if (addr < mem.start_address || addr >= mem.start_address + SIZE)
        return 0x00;

    (*mem.data)[addr - mem.start_address] = val;
    return 0xFF;
}
