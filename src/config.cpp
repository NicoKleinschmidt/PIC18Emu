#include "config.hpp"

// clang-format off
configuration_bit_info_t configuration_bits[] = {
    { .file = CONFIG1L, .bit = 6, .invert = false,  },
    { .file = CONFIG1L, .bit = 4, .invert = false, },
    { .file = CONFIG1L, .bit = 3, .invert = false, },
    { .file = CONFIG1L, .bit = 2, .invert = false, },
    { .file = CONFIG2L, .bit = 0, .invert = true, },

    { .file = CONFIG1H, .bit = 5, .invert = true,  },
    { .file = CONFIG1H, .bit = 2, .invert = false, },
    { .file = CONFIG1H, .bit = 1, .invert = false, },
    { .file = CONFIG1H, .bit = 0, .invert = false, },
    { .file = CONFIG2L, .bit = 3, .invert = false, },
    { .file = CONFIG2L, .bit = 2, .invert = false, },
    { .file = CONFIG2L, .bit = 1, .invert = false, },
    { .file = CONFIG2L, .bit = 0, .invert = true,  },
    { .file = CONFIG2H, .bit = 3, .invert = false, },
    { .file = CONFIG2H, .bit = 2, .invert = false, },
    { .file = CONFIG2H, .bit = 1, .invert = false, },
    { .file = CONFIG2H, .bit = 0, .invert = false, },
    { .file = CONFIG3H, .bit = 0, .invert = false, },
    { .file = CONFIG4L, .bit = 7, .invert = false, },
    { .file = CONFIG4L, .bit = 2, .invert = false, },
    { .file = CONFIG4L, .bit = 1, .invert = false, },
    { .file = CONFIG5L, .bit = 3, .invert = false, },
    { .file = CONFIG5L, .bit = 2, .invert = false, },
    { .file = CONFIG5L, .bit = 1, .invert = false, },
    { .file = CONFIG5L, .bit = 0, .invert = false, },
    { .file = CONFIG5H, .bit = 7, .invert = false, },
    { .file = CONFIG5H, .bit = 6, .invert = false, },
    { .file = CONFIG6L, .bit = 3, .invert = false, },
    { .file = CONFIG6L, .bit = 2, .invert = false, },
    { .file = CONFIG6L, .bit = 1, .invert = false, },
    { .file = CONFIG6L, .bit = 0, .invert = false, },
    { .file = CONFIG6H, .bit = 7, .invert = false, },
    { .file = CONFIG6H, .bit = 6, .invert = false, },
    { .file = CONFIG6H, .bit = 5, .invert = false, },
    { .file = CONFIG7L, .bit = 3, .invert = false, },
    { .file = CONFIG7L, .bit = 2, .invert = false, },
    { .file = CONFIG7L, .bit = 1, .invert = false, },
    { .file = CONFIG7L, .bit = 0, .invert = false, },
    { .file = CONFIG7H, .bit = 6, .invert = false, },
};
// clang-format on

bool configuration_check_bit(configuration_bit_t bit, bus_reader_t<uint32_t, uint8_t> read_bus)
{
    configuration_bit_info_t bit_info = configuration_bits[static_cast<uint8_t>(bit)];

    uint8_t register_val = read_bus(bit_info.file);
    bool is_set = register_val & (1 << bit_info.bit);

    if (bit_info.invert)
        return !is_set;
    else
        return is_set;
}
