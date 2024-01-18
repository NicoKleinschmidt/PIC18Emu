#pragma once

#include "cpu.hpp"
#include <cstdint>

void env_init();
void env_cpu_current_instruction(decode_result_t instruction, uint32_t pc);
void env_bus_read(uint16_t address, uint8_t val);
void env_bus_write(uint16_t address, uint8_t val);
