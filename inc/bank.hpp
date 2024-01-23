#pragma once

#include "bus.hpp"

#include <cstdint>
#include <functional>

struct bank_known_sfrs_t
{
    uint16_t BSR;
    uint16_t FSR0H;
    uint16_t FSR1H;
    uint16_t FSR2H;
    uint16_t FSR0L;
    uint16_t FSR1L;
    uint16_t FSR2L;
    uint16_t INDF0;
    uint16_t INDF1;
    uint16_t INDF2;
    uint16_t PREINC0;
    uint16_t PREINC1;
    uint16_t PREINC2;
    uint16_t POSTINC0;
    uint16_t POSTINC1;
    uint16_t POSTINC2;
    uint16_t POSTDEC0;
    uint16_t POSTDEC1;
    uint16_t POSTDEC2;
    uint16_t PLUSW0;
    uint16_t PLUSW1;
    uint16_t PLUSW2;
};

struct bank_ctx_t
{
    uint8_t bsr;
    uint16_t fsr0;
    uint16_t fsr1;
    uint16_t fsr2;

    bank_known_sfrs_t sfr;
    std::function<uint8_t()> read_wreg;
    bus_reader_t<uint16_t, uint8_t> read_bus;
    bus_writer_t<uint16_t, uint8_t> write_bus;
};

void bank_initialize(bank_ctx_t &ctx);
addr_read_result_t bank_bus_read(bank_ctx_t &ctx, uint16_t addr);
addr_bit_mask_t bank_bus_write(bank_ctx_t &ctx, uint16_t addr, uint8_t val);
