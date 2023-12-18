#pragma once

#include <cstdint>
#include <functional>

template <typename ADDR, typename DATA> using bus_reader_t = std::function<DATA(ADDR addr)>;
template <typename ADDR, typename DATA> using bus_writer_t = std::function<void(ADDR addr, DATA val)>;

/// @brief Specifies which bits where handled by an addr_space_reader_t or addr_space_writer_t.
using addr_bit_mask_t = uint8_t;

struct addr_read_result_t
{
    uint8_t data;
    addr_bit_mask_t mask;
};

template <typename ADDR, typename DATA> using addr_space_reader_t = std::function<addr_read_result_t(ADDR addr)>;
template <typename ADDR, typename DATA> using addr_space_writer_t = std::function<addr_bit_mask_t(ADDR addr, DATA val)>;

const addr_read_result_t addr_read_result_none = {
    .data = 0,
    .mask = 0,
};
