#pragma once

#include <cstdint>

enum alu_status_mask
{
    alu_status_C = 1 << 0,
    alu_status_DC = 1 << 1,
    alu_status_Z = 1 << 2,
    alu_status_OV = 1 << 3,
    alu_status_N = 1 << 4,
};

uint8_t alu_add(uint8_t a, uint8_t b, uint8_t &status);
uint8_t alu_add(uint8_t a, uint8_t b, uint8_t c, uint8_t &status);
uint8_t alu_sub(uint8_t a, uint8_t b, uint8_t &status);
uint8_t alu_sub(uint8_t a, uint8_t b, uint8_t c, uint8_t &status);
uint8_t alu_and(uint8_t a, uint8_t b, uint8_t &status);
uint8_t alu_or(uint8_t a, uint8_t b, uint8_t &status);
uint8_t alu_xor(uint8_t a, uint8_t b, uint8_t &status);
uint8_t alu_complement(uint8_t a, uint8_t &status);
uint8_t alu_negate(uint8_t a, uint8_t &status);
uint8_t alu_rotate_left_carry(uint8_t a, uint8_t &status);
uint8_t alu_rotate_left(uint8_t a, uint8_t &status);
uint8_t alu_rotate_right_carry(uint8_t a, uint8_t &status);
uint8_t alu_rotate_right(uint8_t a, uint8_t &status);
uint8_t alu_decimal_adjust(uint8_t a, uint8_t &status);
