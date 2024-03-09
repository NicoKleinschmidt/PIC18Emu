#include "timer.hpp"

addr_read_result_t timer0_bus_read(timer0_t &timer, const timer0_known_sfrs_t &sfr, uint16_t addr)
{
    if (addr == sfr.TMR0L)
    {
        timer.high_buffer = static_cast<uint8_t>(timer.counter >> 8);
        return addr_read_result_t{.data = static_cast<uint8_t>(timer.counter), .mask = 0xFF};
    }
    if (addr == sfr.TMR0H && timer.use_16_bit)
        return addr_read_result_t{.data = timer.high_buffer, .mask = 0xFF};

    if (addr == sfr.T0CON)
    {
        uint8_t value = 0;

        value |= (static_cast<uint8_t>(timer.prescaler) & 0b111);
        if (timer.enable_prescaler)
            value |= (1 << 3);
        if (timer.external_clock_falling_edge)
            value |= (1 << 4);
        if (timer.use_external_clock)
            value |= (1 << 5);
        if (!timer.use_16_bit)
            value |= (1 << 6);
        if (timer.enable)
            value |= (1 << 7);

        return addr_read_result_t{.data = value, .mask = 0xFF};
    }

    return addr_read_result_none;
}

addr_bit_mask_t timer0_bus_write(timer0_t &timer, const timer0_known_sfrs_t &sfr, uint16_t addr, uint8_t value)
{
    if (addr == sfr.TMR0L)
    {
        timer.prescaler_tick_counter = 0;
        timer.counter &= 0xFF00;
        timer.counter |= value;
        if (timer.use_16_bit)
        {
            timer.counter &= 0x00FF;
            timer.counter |= static_cast<uint16_t>(timer.high_buffer) << 8;
        }

        return 0xFF;
    }
    if (addr == sfr.TMR0H && timer.use_16_bit)
    {
        timer.high_buffer = value;
        return 0xFF;
    }

    if (addr == sfr.T0CON)
    {
        timer.prescaler = static_cast<timer0_prescaler_t>(value & 0b111);
        timer.enable_prescaler = !(value & (1 << 3));
        timer.external_clock_falling_edge = value & (1 << 4);
        timer.use_external_clock = value & (1 << 5);
        timer.use_16_bit = !(value & (1 << 6));
        timer.enable = value & (1 << 7);
        return 0xFF;
    }

    return 0x00;
}

static void timer0_tick(timer0_t &timer)
{
    timer.prescaler_tick_counter++;

    if (!timer.enable)
        return;

    uint_fast16_t div = 2 << static_cast<uint8_t>(timer.prescaler);
    if (timer.enable_prescaler && timer.prescaler_tick_counter % div != 0)
        return;

    timer.counter++;

    if (!timer.use_16_bit)
        timer.counter &= 0xFF;

    if (timer.counter == 0 && timer.overflow_interrupt)
    {
        timer.overflow_interrupt(true);
    }
}

void timer0_tick_fosc(timer0_t &timer)
{
    if (timer.use_external_clock)
        return;

    timer0_tick(timer);
}

void timer0_external_clock_input(timer0_t &timer, bool level_high)
{
    if (!timer.use_external_clock || level_high == timer.external_clock_falling_edge)
        return;

    timer0_tick(timer);
}

void timer0_reset(timer0_t &)
{
    // TODO:
}

void timer0_initialize(timer0_t &timer)
{
    timer.enable_prescaler = false;
    timer.counter = 0;
    timer.enable = false;
    timer.external_clock_falling_edge = false;
    timer.overflow_interrupt = nullptr;
    timer.prescaler = timer0_prescaler_t::value_1_2;
    timer.use_16_bit = false;
    timer.use_external_clock = false;
}

addr_read_result_t timer1_bus_read(timer1_t &timer, const timer1_known_sfrs_t &sfr, uint16_t addr)
{
    if (addr == sfr.TMR1L)
    {
        timer.high_buffer = static_cast<uint8_t>(timer.counter >> 8);
        return addr_read_result_t{.data = static_cast<uint8_t>(timer.counter), .mask = 0xFF};
    }
    if (addr == sfr.TMR1H)
    {
        if (timer.enable_16_bit_rw)
            return addr_read_result_t{.data = timer.high_buffer, .mask = 0xFF};
        else
            return addr_read_result_t{.data = static_cast<uint8_t>(timer.counter >> 8), .mask = 0xFF};
    }

    if (addr == sfr.T1CON)
    {
        uint8_t value = 0;

        if (timer.enable)
            value |= (1 << 0);
        if (timer.use_external_clock)
            value |= (1 << 1);
        if (!timer.sync_external_clock)
            value |= (1 << 2);
        if (timer.enable_oscillator)
            value |= (1 << 3);
        if (static_cast<uint8_t>(timer.prescaler) & 0b01)
            value |= (1 << 4);
        if (static_cast<uint8_t>(timer.prescaler) & 0b10)
            value |= (1 << 5);
        if (timer.enable_16_bit_rw)
            value |= (1 << 7);

        return addr_read_result_t{.data = value, .mask = 0xFF};
    }

    return addr_read_result_none;
}

addr_bit_mask_t timer1_bus_write(timer1_t &timer, const timer1_known_sfrs_t &sfr, uint16_t addr, uint8_t value)
{
    if (addr == sfr.TMR1L)
    {
        timer.prescaler_tick_counter = 0;
        timer.counter &= 0xFF00;
        timer.counter |= value;

        if (timer.enable_16_bit_rw)
        {
            timer.counter &= 0x00FF;
            timer.counter |= static_cast<uint16_t>(timer.high_buffer) << 8;
        }

        return 0xFF;
    }
    if (addr == sfr.TMR1H)
    {
        if (timer.enable_16_bit_rw)
        {
            timer.high_buffer = value;
        }
        else
        {
            timer.counter &= 0x00FF;
            timer.counter |= static_cast<uint16_t>(value) << 8;
        }
        return 0xFF;
    }

    if (addr == sfr.T1CON)
    {
        timer.enable = value & (1 << 0);
        timer.use_external_clock = value & (1 << 1);
        timer.sync_external_clock = !(value & (1 << 2));
        timer.enable_oscillator = value & (1 << 3);
        timer.enable_16_bit_rw = value & (1 << 7);
        timer.prescaler = static_cast<timer1_prescaler_t>((value >> 4) & 0b11);
        return 0xFF;
    }

    return 0x00;
}

static void timer1_tick(timer1_t &timer)
{
    timer.prescaler_tick_counter++;

    if (!timer.enable)
        return;

    uint_fast16_t div = 1 << static_cast<uint8_t>(timer.prescaler);
    if (timer.prescaler_tick_counter % div != 0)
        return;

    timer.counter++;

    if (timer.counter == 0 && timer.overflow_interrupt)
    {
        timer.overflow_interrupt(true);
    }
}

void timer1_tick_fosc(timer1_t &timer)
{
    if (timer.use_external_clock)
        return;

    timer1_tick(timer);
}

void timer1_external_clock_input(timer1_t &timer, bool level_high)
{
    if (!timer.use_external_clock || !level_high)
        return;

    timer1_tick(timer);
}

void timer1_special_event_trigger(timer1_t &timer)
{
    timer.counter = 0;
}

void timer1_reset(timer1_t &)
{
    // TODO:
}

void timer1_initialize(timer1_t &timer)
{
    timer.counter = 0;
    timer.enable = false;
    timer.enable_16_bit_rw = false;
    timer.enable_oscillator = false;
    timer.overflow_interrupt = nullptr;
    timer.prescaler = timer1_prescaler_t::value_1_1;
    timer.sync_external_clock = false;
    timer.use_external_clock = false;
}

addr_read_result_t timer2_bus_read(timer2_t &timer, const timer2_4_known_sfrs_t &sfr, uint16_t addr)
{
    if (addr == sfr.TMR)
        return addr_read_result_t{.data = timer.counter, .mask = 0xFF};
    if (addr == sfr.PR)
        return addr_read_result_t{.data = timer.period, .mask = 0xFF};

    if (addr == sfr.TCON)
    {
        uint8_t value = 0;

        value |= static_cast<uint8_t>(timer.prescaler) & 0b11;
        value |= (static_cast<uint8_t>(timer.postscaler) & 0b1111) << 3;
        value |= timer.enable << 2;
        return addr_read_result_t{.data = value, .mask = 0x7F};
    }

    return addr_read_result_none;
}

addr_bit_mask_t timer2_bus_write(timer2_t &timer, const timer2_4_known_sfrs_t &sfr, uint16_t addr, uint8_t value)
{
    if (addr == sfr.TMR)
    {
        timer.prescaler_tick_counter = 0;
        timer.postscaler_tick_counter = 0;
        timer.counter = value;
        return 0xFF;
    }

    if (addr == sfr.PR)
    {
        timer.period = value;
        return 0xFF;
    }

    if (addr == sfr.TCON)
    {
        timer.prescaler_tick_counter = 0;
        timer.postscaler_tick_counter = 0;
        timer.prescaler = static_cast<timer2_prescaler_t>(value & 0b11);
        timer.postscaler = static_cast<timer2_postscaler_t>((value >> 3) & 0b1111);
        timer.enable = value & (1 << 2);
        return 0x7F;
    }

    return 0x00;
}

void timer2_tick_fosc(timer2_t &timer)
{
    timer.prescaler_tick_counter++;

    if (!timer.enable)
        return;

    uint_fast16_t div = 1 << static_cast<uint8_t>(timer.prescaler);
    if (timer.prescaler_tick_counter % div != 0)
        return;

    timer.counter++;

    if (timer.counter == timer.period)
    {
        timer.counter = 0;
        timer.postscaler_tick_counter++;
        bool scaler_out = timer.postscaler_tick_counter % static_cast<uint8_t>(timer.postscaler);

        if (scaler_out && timer.period_match_interrupt != nullptr)
            timer.period_match_interrupt(true);
    }

    if (timer.output != nullptr)
        timer.output();
}

void timer2_reset(timer2_t &timer)
{
    timer.period = 0xFF;
}

void timer2_initialize(timer2_t &timer)
{
    timer.counter = 0;
    timer.period = 0;
    timer.enable = false;
    timer.output = nullptr;
    timer.period_match_interrupt = nullptr;
    timer.postscaler = timer2_postscaler_t::value_1_1;
    timer.prescaler = timer2_prescaler_t::value_1_1;
}

addr_read_result_t timer3_bus_read(timer3_t &timer, const timer3_known_sfrs_t &sfr, uint16_t addr)
{
    if (addr == sfr.TMR3L)
    {
        timer.high_buffer = static_cast<uint8_t>(timer.counter >> 8);
        return addr_read_result_t{.data = static_cast<uint8_t>(timer.counter), .mask = 0xFF};
    }
    if (addr == sfr.TMR3H)
    {
        if (timer.enable_16_bit_rw)
            return addr_read_result_t{.data = timer.high_buffer, .mask = 0xFF};
        else
            return addr_read_result_t{.data = static_cast<uint8_t>(timer.counter >> 8), .mask = 0xFF};
    }

    if (addr == sfr.T3CON)
    {
        uint8_t value = 0;

        if (timer.enable)
            value |= (1 << 0);
        if (timer.use_external_clock)
            value |= (1 << 1);
        if (!timer.sync_external_clock)
            value |= (1 << 2);
        if (timer.enable_16_bit_rw)
            value |= (1 << 7);

        value |= (static_cast<uint8_t>(timer.prescaler) & 0b11) << 4;
        return addr_read_result_t{.data = value, .mask = 0xB7};
    }

    return addr_read_result_none;
}

addr_bit_mask_t timer3_bus_write(timer3_t &timer, const timer3_known_sfrs_t &sfr, uint16_t addr, uint8_t value)
{
    if (addr == sfr.TMR3L)
    {
        timer.prescaler_tick_counter = 0;
        timer.counter &= 0xFF00;
        timer.counter |= value;

        if (timer.enable_16_bit_rw)
        {
            timer.counter &= 0x00FF;
            timer.counter |= static_cast<uint16_t>(timer.high_buffer) << 8;
        }

        return 0xFF;
    }
    if (addr == sfr.TMR3H)
    {
        if (timer.enable_16_bit_rw)
        {
            timer.high_buffer = value;
        }
        else
        {
            timer.counter &= 0x00FF;
            timer.counter |= static_cast<uint16_t>(value) << 8;
        }
        return 0xFF;
    }

    if (addr == sfr.T3CON)
    {
        timer.enable = value & (1 << 0);
        timer.use_external_clock = value & (1 << 1);
        timer.sync_external_clock = !(value & (1 << 2));
        timer.enable_16_bit_rw = value & (1 << 7);
        timer.prescaler = static_cast<timer3_prescaler_t>((value >> 4) & 0b11);
        return 0xB7;
    }

    return 0x00;
}

static void timer3_tick(timer3_t &timer)
{
    timer.prescaler_tick_counter++;

    if (!timer.enable)
        return;

    uint_fast16_t div = 1 << static_cast<uint8_t>(timer.prescaler);
    if (timer.prescaler_tick_counter % div != 0)
        return;

    timer.counter++;

    if (timer.counter == 0 && timer.overflow_interrupt)
    {
        timer.overflow_interrupt(true);
    }
}

void timer3_tick_fosc(timer3_t &timer)
{
    if (timer.use_external_clock)
        return;

    timer3_tick(timer);
}

void timer3_external_clock_input(timer3_t &timer, bool level_high)
{
    if (!timer.use_external_clock || !level_high)
        return;

    timer3_tick(timer);
}

void timer3_special_event_trigger(timer3_t &timer)
{
    timer.counter = 0;
}

void timer3_reset(timer3_t &)
{
    // TODO:
}

void timer3_initialize(timer3_t &timer)
{
    timer.counter = 0;
    timer.enable = false;
    timer.enable_16_bit_rw = false;
    timer.overflow_interrupt = nullptr;
    timer.prescaler = timer3_prescaler_t::value_1_1;
    timer.sync_external_clock = false;
    timer.use_external_clock = false;
}
