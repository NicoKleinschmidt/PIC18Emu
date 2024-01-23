#pragma once

#include "bus.hpp"

#include <cstdint>

struct tbl_ctx_t
{
    uint16_t TABLAT;
    uint16_t TBLPTRL;
    uint16_t TBLPTRH;
    uint16_t TBLPTRU;

    uint32_t pointer;
    uint8_t latch;
};

enum class tblptr_action_t
{
    none,
    postinc,
    postdec,
    preinc,
};

/// @brief Puts the value stored in TABLAT onto the bus, at the address defined by the table pointer.
void tbl_write(tbl_ctx_t &ctx, bus_writer_t<uint32_t, uint8_t> write_bus, tblptr_action_t action);

/// @brief Reads a value from the address defined by table pointer into TABLAT.
void tbl_read(tbl_ctx_t &ctx, bus_reader_t<uint32_t, uint8_t> read_bus, tblptr_action_t action);

void tbl_initialize(tbl_ctx_t &ctx);
addr_read_result_t tbl_bus_read(const tbl_ctx_t &ctx, uint16_t address);
addr_bit_mask_t tbl_bus_write(tbl_ctx_t &ctx, uint16_t address, uint8_t value);
