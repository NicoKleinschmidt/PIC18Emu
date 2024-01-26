#pragma once

#include "bus.hpp"
#include <cstdint>

enum configuration_register_t
{
    CONFIG1L = 0x300000,
    CONFIG1H = 0x300001,
    CONFIG2L = 0x300002,
    CONFIG2H = 0x300003,
    CONFIG3H = 0x300005,
    CONFIG4L = 0x300006,
    CONFIG5L = 0x300008,
    CONFIG5H = 0x300009,
    CONFIG6L = 0x30000A,
    CONFIG6H = 0x30000B,
    CONFIG7L = 0x30000C,
    CONFIG7H = 0x30000D,
    DEVID1 = 0x3FFFFE,
    DEVID2 = 0x3FFFFF,
};

struct config_bit_t
{
    uint32_t file_address;
    uint8_t bit;
    bool invert;
};

namespace CONFIG
{
const config_bit_t XINST = {.file_address = CONFIG1L, .bit = 6, .invert = false};
const config_bit_t SOSCSEL1 = {.file_address = CONFIG1L, .bit = 4, .invert = false};
const config_bit_t SOSCSEL0 = {.file_address = CONFIG1L, .bit = 3, .invert = false};
const config_bit_t INTOSCSEL = {.file_address = CONFIG1L, .bit = 2, .invert = false};
const config_bit_t RETEN = {.file_address = CONFIG1L, .bit = 0, .invert = true};
const config_bit_t IESO = {.file_address = CONFIG1H, .bit = 7, .invert = false};
const config_bit_t FCMEN = {.file_address = CONFIG1H, .bit = 6, .invert = false};
const config_bit_t PLLCFG = {.file_address = CONFIG1H, .bit = 4, .invert = false};
const config_bit_t FOSC3 = {.file_address = CONFIG1H, .bit = 3, .invert = false};
const config_bit_t FOSC2 = {.file_address = CONFIG1H, .bit = 2, .invert = false};
const config_bit_t FOSC1 = {.file_address = CONFIG1H, .bit = 1, .invert = false};
const config_bit_t FOSC0 = {.file_address = CONFIG1H, .bit = 0, .invert = false};
const config_bit_t BORPWR1 = {.file_address = CONFIG2L, .bit = 6, .invert = false};
const config_bit_t BORPWR0 = {.file_address = CONFIG2L, .bit = 5, .invert = false};
const config_bit_t BORV1 = {.file_address = CONFIG2L, .bit = 4, .invert = false};
const config_bit_t BORV0 = {.file_address = CONFIG2L, .bit = 3, .invert = false};
const config_bit_t BOREN1 = {.file_address = CONFIG2L, .bit = 2, .invert = false};
const config_bit_t BOREN0 = {.file_address = CONFIG2L, .bit = 1, .invert = false};
const config_bit_t PWRTEN = {.file_address = CONFIG2L, .bit = 0, .invert = true};
const config_bit_t WDTPS4 = {.file_address = CONFIG2H, .bit = 6, .invert = false};
const config_bit_t WDTPS3 = {.file_address = CONFIG2H, .bit = 5, .invert = false};
const config_bit_t WDTPS2 = {.file_address = CONFIG2H, .bit = 4, .invert = false};
const config_bit_t WDTPS1 = {.file_address = CONFIG2H, .bit = 3, .invert = false};
const config_bit_t WDTPS0 = {.file_address = CONFIG2H, .bit = 2, .invert = false};
const config_bit_t WDTEN1 = {.file_address = CONFIG2H, .bit = 1, .invert = false};
const config_bit_t WDTEN0 = {.file_address = CONFIG2H, .bit = 0, .invert = false};
const config_bit_t MCLRE = {.file_address = CONFIG3H, .bit = 7, .invert = false};
const config_bit_t MSSPMSK = {.file_address = CONFIG3H, .bit = 3, .invert = false};
const config_bit_t T3CKMX = {.file_address = CONFIG3H, .bit = 2, .invert = false};
const config_bit_t T0CKMX = {.file_address = CONFIG3H, .bit = 1, .invert = false};
const config_bit_t CANMX = {.file_address = CONFIG3H, .bit = 0, .invert = false};
const config_bit_t DEBUG = {.file_address = CONFIG4L, .bit = 7, .invert = true};
const config_bit_t BBSIZ0 = {.file_address = CONFIG4L, .bit = 4, .invert = false};
const config_bit_t STVREN = {.file_address = CONFIG4L, .bit = 0, .invert = false};
const config_bit_t CP3 = {.file_address = CONFIG5L, .bit = 3, .invert = false};
const config_bit_t CP2 = {.file_address = CONFIG5L, .bit = 2, .invert = false};
const config_bit_t CP1 = {.file_address = CONFIG5L, .bit = 1, .invert = false};
const config_bit_t CP0 = {.file_address = CONFIG5L, .bit = 0, .invert = false};
const config_bit_t CPD = {.file_address = CONFIG5H, .bit = 7, .invert = false};
const config_bit_t CPB = {.file_address = CONFIG5H, .bit = 6, .invert = false};
const config_bit_t WRT3 = {.file_address = CONFIG6L, .bit = 3, .invert = false};
const config_bit_t WRT2 = {.file_address = CONFIG6L, .bit = 2, .invert = false};
const config_bit_t WRT1 = {.file_address = CONFIG6L, .bit = 1, .invert = false};
const config_bit_t WRT0 = {.file_address = CONFIG6L, .bit = 0, .invert = false};
const config_bit_t WRTD = {.file_address = CONFIG6H, .bit = 7, .invert = false};
const config_bit_t WRTB = {.file_address = CONFIG6H, .bit = 6, .invert = false};
const config_bit_t WRTC = {.file_address = CONFIG6H, .bit = 5, .invert = false};
const config_bit_t EBTR3 = {.file_address = CONFIG7L, .bit = 3, .invert = false};
const config_bit_t EBTR2 = {.file_address = CONFIG7L, .bit = 2, .invert = false};
const config_bit_t EBTR1 = {.file_address = CONFIG7L, .bit = 1, .invert = false};
const config_bit_t EBTR0 = {.file_address = CONFIG7L, .bit = 0, .invert = false};
const config_bit_t EBTRB = {.file_address = CONFIG7H, .bit = 6, .invert = false};
} // namespace CONFIG

bool configuration_check_bit(const config_bit_t &bit, bus_reader_t<uint32_t, uint8_t> read_bus);
