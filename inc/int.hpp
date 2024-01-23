#pragma once

#include "bus.hpp"

#include <array>
#include <functional>

using vector_func_t = std::function<void(bool high_priority)>;
using wakeup_func_t = std::function<void()>;

struct int_source_t
{
    bool enabled;
    bool high_priority;
    bool requested;
    bool is_peripheral;
    bool flag_read_only;
};

template <size_t SRC_COUNT> struct int_state_t
{
    bool IPEN;
    bool GIEH;
    bool GIEL;
    std::array<int_source_t, SRC_COUNT> sources;
};

/// @brief Should be called before every cpu tick to check if an interrupt occured.
/// @param read_prog_bus
/// @param read_data_bus
/// @param write_data_bus
/// @param vector This callback function should cause an interrupt to be raised on the cpu.
///               The parameter address is the interrupt vector the cpu should branch to.
template <size_t SRC_COUNT>
void interrupt_tick(bus_reader_t<uint32_t, uint8_t> read_prog_bus, int_state_t<SRC_COUNT> &state, vector_func_t vector,
                    wakeup_func_t wake);

#include "int.tcc"

struct int_known_sfrs_18f66k80_t
{
    uint16_t INTCON;
    uint16_t INTCON2;
    uint16_t INTCON3;
    uint16_t PIR1;
    uint16_t PIE1;
    uint16_t IPR1;
    uint16_t PIR2;
    uint16_t PIE2;
    uint16_t IPR2;
    uint16_t PIR3;
    uint16_t PIE3;
    uint16_t IPR3;
    uint16_t PIR4;
    uint16_t PIE4;
    uint16_t IPR4;
    uint16_t PIR5;
    uint16_t PIE5;
    uint16_t IPR5;
    uint16_t RCON;
};

addr_read_result_t int_addr_space_read_18f66k80(int_state_t<39> &state, const int_known_sfrs_18f66k80_t &sfr,
                                                uint16_t addr);
addr_bit_mask_t int_addr_space_write_18f66k80(int_state_t<39> &state, const int_known_sfrs_18f66k80_t &sfr,
                                              uint16_t addr, uint8_t value);
void int_set_flag(int_state_t<39> &state, const int_known_sfrs_18f66k80_t &sfr, uint16_t reg, uint8_t bit, bool b);
void int_state_initialize_18f66k80(int_state_t<39> &state);
void int_state_reset_18f66k80(int_state_t<39> &state);