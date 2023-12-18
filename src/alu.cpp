#include "alu.hpp"
#include "reg.hpp"

uint8_t alu_add(uint8_t a, uint8_t b, uint8_t &status)
{
    uint16_t result = a + b;
    uint8_t result8bit = static_cast<uint8_t>(result);
    uint8_t new_status = 0;

    if (result & 0b10000000)
        new_status |= alu_status_N;
    if ((status & alu_status_N) != (new_status & alu_status_N))
        new_status |= alu_status_OV;
    if (result8bit == 0)
        new_status |= alu_status_Z;
    if (result & 0b100000000)
        new_status |= alu_status_C;

    status = new_status;
    return result8bit;
}

uint8_t alu_add(uint8_t a, uint8_t b, uint8_t c, uint8_t &status)
{
    uint16_t result = a + b + c;
    uint8_t result8bit = static_cast<uint8_t>(result);
    uint8_t new_status = 0;

    if (result & 0b10000000)
        new_status |= alu_status_N;
    if ((status & alu_status_N) != (new_status & alu_status_N))
        new_status |= alu_status_OV;
    if (result8bit == 0)
        new_status |= alu_status_Z;
    if (result & 0b100000000)
        new_status |= alu_status_C;

    status = new_status;
    return result8bit;
}

uint8_t alu_sub(uint8_t a, uint8_t b, uint8_t &status)
{
    return alu_add(a, -b, status);
}

uint8_t alu_sub(uint8_t a, uint8_t b, uint8_t c, uint8_t &status)
{
    return alu_add(a, -b, -c, status);
}

uint8_t alu_and(uint8_t a, uint8_t b, uint8_t &status)
{
    uint8_t result = a & b;
    uint8_t new_status = 0;

    if (result & 0b10000000)
        new_status |= alu_status_N;
    if (result == 0)
        new_status |= alu_status_Z;

    status = new_status;
    return result;
}

uint8_t alu_or(uint8_t a, uint8_t b, uint8_t &status)
{
    uint8_t result = a | b;
    uint8_t new_status = 0;

    if (result & 0b10000000)
        new_status |= alu_status_N;
    if (result == 0)
        new_status |= alu_status_Z;

    status = new_status;
    return result;
}

uint8_t alu_xor(uint8_t a, uint8_t b, uint8_t &status)
{
    uint8_t result = a ^ b;
    uint8_t new_status = 0;

    if (result & 0b10000000)
        new_status |= alu_status_N;
    if (result == 0)
        new_status |= alu_status_Z;

    status = new_status;
    return result;
}

uint8_t alu_complement(uint8_t a, uint8_t &status)
{
    return -static_cast<unsigned int>(a);
}

uint8_t alu_negate(uint8_t a, uint8_t &status)
{
    uint16_t result = (~a) + 1;
    uint8_t result8bit = static_cast<uint8_t>(result);
    uint8_t new_status = 0;

    if (result & 0b10000000)
        new_status |= alu_status_N;
    if ((status & alu_status_N) != (new_status & alu_status_N))
        new_status |= alu_status_OV;
    if (result8bit == 0)
        new_status |= alu_status_Z;
    if (result & 0b100000000)
        new_status |= alu_status_C;

    status = new_status;
    return result8bit;
}

uint8_t alu_rotate_left_carry(uint8_t a, uint8_t &status)
{
    uint16_t value = static_cast<uint16_t>(a);
    value <<= 1;
    if (status & alu_status_C)
        value |= 1;

    status &= ~alu_status_C;
    status &= ~alu_status_Z;
    status &= ~alu_status_N;

    if (value & 0b100000000)
        status |= alu_status_C;

    uint8_t result = static_cast<uint8_t>(value);
    if (result == 0)
        status |= alu_status_Z;
    if (result & 0b10000000)
        status |= alu_status_N;

    return result;
}

uint8_t alu_rotate_left(uint8_t a, uint8_t &status)
{
    uint16_t value = static_cast<uint16_t>(a);
    value <<= 1;
    if (value & (1 << 8))
        value |= 1;

    status &= ~alu_status_Z;
    status &= ~alu_status_N;

    uint8_t result = static_cast<uint8_t>(value);
    if (result == 0)
        status |= alu_status_Z;
    if (result & 0b10000000)
        status |= alu_status_N;

    return result;
}

uint8_t alu_rotate_right_carry(uint8_t a, uint8_t &status)
{
    status &= ~alu_status_C;

    uint8_t result = a >> 1;
    if (a & 1)
        status |= alu_status_C;

    status &= ~alu_status_Z;
    status &= ~alu_status_N;

    if (result == 0)
        status |= alu_status_Z;
    if (result & 0b10000000)
        status |= alu_status_N;

    return result;
}

uint8_t alu_rotate_right(uint8_t a, uint8_t &status)
{
    status &= ~alu_status_C;

    uint8_t result = a >> 1;
    if (a & 1)
        result |= (1 << 7);

    status &= ~alu_status_Z;
    status &= ~alu_status_N;

    if (result == 0)
        status |= alu_status_Z;
    if (result & 0b10000000)
        status |= alu_status_N;

    return result;
}

uint8_t alu_decimal_adjust(uint8_t a, uint8_t &status)
{
    uint8_t low = a & 0x0F;
    uint8_t high = a & 0xF0;
    uint16_t result = 0;

    if (low > 9 || (status & alu_status_DC))
        result = low + 6;
    else
        result = low;

    if (high > 9 || (status & alu_status_C))
        result |= ((high + 6) << 4);
    else
        result |= (high << 4);

    status &= ~alu_status_C;

    if (result & (1 << 8))
        status |= alu_status_C;

    return result;
}
