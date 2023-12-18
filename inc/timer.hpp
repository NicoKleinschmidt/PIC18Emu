#pragma once

#include "bus.hpp"
#include <functional>

using timer_interrupt_cb = std::function<void(bool set)>;

struct timer0_known_sfrs_t
{
    uint16_t TMR0L;
    uint16_t TMR0H;
    uint16_t T0CON;
};

enum class timer0_prescaler_t
{
    value_1_2,
    value_1_4,
    value_1_8,
    value_1_16,
    value_1_32,
    value_1_64,
    value_1_128,
    value_1_256,
};

struct timer0_t
{
    uint16_t counter;
    uint8_t high_buffer;
    bool enable;
    bool use_16_bit;
    bool use_external_clock;
    bool external_clock_falling_edge;
    bool enable_prescaler;
    uint8_t prescaler_tick_counter;
    timer0_prescaler_t prescaler;
    timer_interrupt_cb overflow_interrupt;
};

struct timer1_known_sfrs_t
{
    uint16_t TMR1L;
    uint16_t TMR1H;
    uint16_t T1CON;
};

enum class timer1_prescaler_t
{
    value_1_1,
    value_1_2,
    value_1_4,
    value_1_8,
};

struct timer1_t
{
    uint16_t counter;
    uint8_t high_buffer;
    bool enable;
    bool enable_16_bit_rw;
    bool enable_oscillator;
    bool sync_external_clock;
    bool use_external_clock;
    uint8_t prescaler_tick_counter;
    timer1_prescaler_t prescaler;
    timer_interrupt_cb overflow_interrupt;
};

enum class timer2_prescaler_t
{
    value_1_1,
    value_1_4,
    value_1_16,
};

enum class timer2_postscaler_t
{
    value_1_1,
    value_1_2,
    value_1_3,
    value_1_4,
    value_1_5,
    value_1_6,
    value_1_7,
    value_1_8,
    value_1_9,
    value_1_10,
    value_1_11,
    value_1_12,
    value_1_13,
    value_1_14,
    value_1_15,
    value_1_16,
};

struct timer2_4_known_sfrs_t
{
    uint16_t TMR;
    uint16_t PR;
    uint16_t TCON;
};

struct timer2_t
{
    uint8_t counter;
    uint8_t period;
    bool enable;
    uint8_t prescaler_tick_counter;
    uint8_t postscaler_tick_counter;
    timer2_prescaler_t prescaler;
    timer2_postscaler_t postscaler;
    std::function<void()> output;
    timer_interrupt_cb period_match_interrupt;
};

enum timer3_prescaler_t
{
    value_1_1,
    value_1_2,
    value_1_4,
    value_1_8,
};

struct timer3_known_sfrs_t
{
    uint16_t TMR3L;
    uint16_t TMR3H;
    uint16_t T3CON;
};

struct timer3_t
{
    uint16_t counter;
    uint8_t high_buffer;
    bool enable;
    bool enable_16_bit_rw;
    bool sync_external_clock;
    bool use_external_clock;
    uint8_t prescaler_tick_counter;
    timer3_prescaler_t prescaler;
    timer_interrupt_cb overflow_interrupt;
};

addr_read_result_t timer0_bus_read(timer0_t &timer, const timer0_known_sfrs_t &sfr, uint16_t addr);
addr_bit_mask_t timer0_bus_write(timer0_t &timer, const timer0_known_sfrs_t &sfr, uint16_t addr, uint8_t value);
void timer0_tick(timer0_t &timer, bus_reader_t<uint16_t, uint8_t> read_bus);
void timer0_external_clock_input(timer0_t &timer, bool level_high, bus_reader_t<uint16_t, uint8_t> read_bus);
void timer0_reset(timer0_t &timer);
void timer0_initialize(timer0_t &timer);

addr_read_result_t timer1_bus_read(timer1_t &timer, const timer1_known_sfrs_t &sfr, uint16_t addr);
addr_bit_mask_t timer1_bus_write(timer1_t &timer, const timer1_known_sfrs_t &sfr, uint16_t addr, uint8_t value);
void timer1_tick(timer1_t &timer, bus_reader_t<uint16_t, uint8_t> read_bus);
void timer1_external_clock_input(timer1_t &timer, bool level_high, bus_reader_t<uint16_t, uint8_t> read_bus);
void timer1_special_event_trigger(timer1_t &timer);
void timer1_reset(timer1_t &timer);
void timer1_initialize(timer1_t &timer);

addr_read_result_t timer2_bus_read(timer2_t &timer, const timer2_4_known_sfrs_t &sfr, uint16_t addr);
addr_bit_mask_t timer2_bus_write(timer2_t &timer, const timer2_4_known_sfrs_t &sfr, uint16_t addr, uint8_t value);
void timer2_tick(timer2_t &timer, bus_reader_t<uint16_t, uint8_t> read_bus);
void timer2_reset(timer2_t &timer);
void timer2_initialize(timer2_t &timer);

addr_read_result_t timer3_bus_read(timer3_t &timer, const timer3_known_sfrs_t &sfr, uint16_t addr);
addr_bit_mask_t timer3_bus_write(timer3_t &timer, const timer3_known_sfrs_t &sfr, uint16_t addr, uint8_t value);
void timer3_tick(timer3_t &timer, bus_reader_t<uint16_t, uint8_t> read_bus);
void timer3_external_clock_input(timer3_t &timer, bool level_high, bus_reader_t<uint16_t, uint8_t> read_bus);
void timer3_special_event_trigger(timer3_t &timer);
void timer3_reset(timer3_t &timer);
void timer3_initialize(timer3_t &timer);
