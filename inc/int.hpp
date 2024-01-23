#pragma once

#include "bus.hpp"

#include <functional>
#include <vector>

using vector_func_t = std::function<void(bool high_priority)>;
using wakeup_func_t = std::function<void()>;

struct int_source_t
{
    uint16_t enabled_reg;
    uint16_t priority_reg;
    uint16_t flag_reg;
    uint8_t enabled_bit;
    uint8_t priority_bit;
    uint8_t flag_bit;

    bool enabled;
    bool high_priority;
    bool flagged;
    bool is_peripheral;
    bool flag_read_only;
};

struct int_known_sfrs_t
{
    uint16_t INTCON;
    uint16_t RCON;
};

struct int_ctx_t
{
    bool IPEN;
    bool GIEH;
    bool GIEL;

    int_known_sfrs_t sfr;
    std::vector<int_source_t> sources;

    vector_func_t vector;
    wakeup_func_t wakeup;
};

/// @brief Should be called before every cpu tick to check if an interrupt occurred.
void interrupt_tick(int_ctx_t &ctx);

addr_read_result_t interrupt_bus_read(int_ctx_t &state, uint16_t addr);
addr_bit_mask_t interrupt_bus_write(int_ctx_t &ctx, uint16_t addr, uint8_t value);
void interrupt_set_flag(int_source_t &src, bool b);
void interrupt_initialize(int_ctx_t &ctx);
void interrupt_reset(int_ctx_t &ctx);

enum class int_make_source_flags_t
{
    none = 0,
    default_enable = 1 << 0,
    default_low_prio = 1 << 1,
    default_flagged = 1 << 2,
    peripheral = 1 << 3,
    flag_ro = 1 << 4,
    remember_flagged = 1 << 5,
};

inline constexpr int_make_source_flags_t operator|(int_make_source_flags_t a, int_make_source_flags_t b)
{
    return static_cast<int_make_source_flags_t>(static_cast<int>(a) | static_cast<int>(b));
}

inline constexpr int_make_source_flags_t operator&(int_make_source_flags_t a, int_make_source_flags_t b)
{
    return static_cast<int_make_source_flags_t>(static_cast<int>(a) & static_cast<int>(b));
}

constexpr int_source_t int_make_source(uint16_t ie_reg, uint8_t ie_bit, uint16_t if_reg, uint8_t if_bit,
                                       uint16_t ip_reg, uint8_t ip_bit,
                                       int_make_source_flags_t flags = int_make_source_flags_t::none)
{
    return int_source_t{
        .enabled_reg = ie_reg,
        .priority_reg = ip_reg,
        .flag_reg = if_reg,
        .enabled_bit = ie_bit,
        .priority_bit = ip_bit,
        .flag_bit = if_bit,
        .enabled = (flags & int_make_source_flags_t::default_enable) != int_make_source_flags_t::none,
        .high_priority = (flags & int_make_source_flags_t::default_low_prio) == int_make_source_flags_t::none,
        .flagged = (flags & int_make_source_flags_t::default_flagged) != int_make_source_flags_t::none,
        .is_peripheral = (flags & int_make_source_flags_t::peripheral) != int_make_source_flags_t::none,
        .flag_read_only = (flags & int_make_source_flags_t::flag_ro) != int_make_source_flags_t::none,
    };
}
