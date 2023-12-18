#pragma once

#include "bus.hpp"
#include "io.hpp"

#include <cstdint>
#include <functional>

enum class ccp_mode_t
{
    disabled,
    unknown0,
    compare_toggle,
    unknown1,
    capture_falling,
    capture_rising,
    capture_4th_rising,
    capture_16th_rising,
    compare_force_high,
    compare_force_low,
    compare_software,
    compare_special_event,
    pwm_ac_high_bd_high,
    pwm_ac_high_bd_low,
    pwm_ac_low_bd_high,
    pwm_ac_low_bd_low,
};

enum class ccp_timer_selection_t
{
    timer_1_2,
    timer_3_4,
};

struct ccp_common_known_sfrs_t
{
    uint16_t PMD0;
    uint16_t CCPTMRS;
    uint16_t TMR1L;
    uint16_t TMR1H;
    uint16_t TMR2;
    uint16_t TMR3L;
    uint16_t TMR3H;
    uint16_t TMR4;
};

struct ccp_known_sfrs_t
{
    uint16_t CCPxCON;
    uint16_t CCPRxL;
    uint16_t CCPRxH;
    ccp_common_known_sfrs_t common;
};

struct ccp_ctx_t
{
    ccp_mode_t mode;
    ccp_timer_selection_t timer;
    uint8_t ccp_num;
    uint16_t ccp_register;
    uint16_t duty_cycle_latch;
    uint8_t duty_cycle_lower : 2;
    uint8_t prescaler_counter;
    bool output_latch;
    bool open_drain;
    std::function<void(bool set)> interrupt;
    std::function<void(uint8_t timer_num)> special_event_timer;
    std::function<void(io_state_t state)> set_pin;
    ccp_known_sfrs_t known_sfrs;
};

addr_read_result_t ccp_bus_read(ccp_ctx_t &ctx, uint16_t address);
addr_bit_mask_t ccp_bus_write(ccp_ctx_t &ctx, uint16_t address, uint8_t value);
void ccp_tick(ccp_ctx_t &ctx, bus_reader_t<uint16_t, uint8_t> read_bus);
void ccp_pwm_match_input(ccp_ctx_t &ctx, uint8_t timer_num, bus_reader_t<uint16_t, uint8_t> read_bus);
void ccp_pin_input(ccp_ctx_t &ctx, bool high, bus_reader_t<uint16_t, uint8_t> read_bus);
void ccp_can_msg_received(ccp_ctx_t &ctx, bus_reader_t<uint16_t, uint8_t> read_bus);
void ccp_initialize(ccp_ctx_t &ctx, uint8_t ccp_num, const ccp_known_sfrs_t &sfrs);
void ccp_reset(ccp_ctx_t &ctx);
