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

// clang-format off
enum class configuration_bit_t
{
    XINST,  SOSCSEL1, SOSCSEL0, INTOSCSEL, RETEN,
    OSCSEN, FOSC2,    FOSC1,    FOSC0,
    BORV1,  BORV0,    BOREN,    PWRTEN,
    WDTPS2, WDTPS1,   WDTPS0,   WDTEN,
    CCP2MX, DEBUG,    LVP,      STVREN,
    CP3,    CP2,      CP1,      CP0,
    CPD,    CPB,    
    WRT3,   WRT2,     WRT1,     WRT0,
    WRTD,   WRTB,     WRTC,
    EBTR3,  EBTR2,    EBTR1,    EBTR0,
    EBTRB,
};
// clang-format on

struct configuration_bit_info_t
{
    configuration_register_t file;
    uint8_t bit;
    bool invert;
};

bool configuration_check_bit(configuration_bit_t bit, bus_reader_t<uint32_t, uint8_t> read_bus);
