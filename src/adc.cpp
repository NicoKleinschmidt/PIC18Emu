#include "adc.hpp"
#include <cassert>
#include <cmath>

static void start_conversion(adc_ctx_t &ctx)
{
    if (ctx.running)
        return;

    ctx.tad_counter = 0;
    switch (ctx.acquisition_time)
    {
    case adc_acquisition_time_t::tad_0: ctx.acquisition_cycles = 0; break;
    case adc_acquisition_time_t::tad_2: ctx.acquisition_cycles = 2; break;
    case adc_acquisition_time_t::tad_4: ctx.acquisition_cycles = 4; break;
    case adc_acquisition_time_t::tad_6: ctx.acquisition_cycles = 6; break;
    case adc_acquisition_time_t::tad_8: ctx.acquisition_cycles = 8; break;
    case adc_acquisition_time_t::tad_12: ctx.acquisition_cycles = 12; break;
    case adc_acquisition_time_t::tad_16: ctx.acquisition_cycles = 16; break;
    case adc_acquisition_time_t::tad_20: ctx.acquisition_cycles = 20; break;
    default: assert(false); return;
    }

    ctx.running = true;
}

static void abort_conversion(adc_ctx_t &ctx)
{
    if (!ctx.running)
        return;

    ctx.running = false;
}

static double clip(double n, double lower, double upper)
{
    return std::max(lower, std::min(n, upper));
}

static void finish_conversion(adc_ctx_t &ctx)
{
    if (!ctx.running)
        return;

    // C++20 guarantees 2's complement
    int16_t clipped_signed = static_cast<int16_t>(clip(ctx.adjusted_measurement, -1, 1) * 0x0FFF);
    uint16_t value = *reinterpret_cast<uint16_t *>(&clipped_signed);

    if (ctx.result_format == adc_result_format_t::justify_left)
        value <<= 4;

    if (clipped_signed < 0 && ctx.result_format == adc_result_format_t::justify_left)
        value |= 0x000F; // Extend sign bit
    else if (clipped_signed < 0 && ctx.result_format == adc_result_format_t::justify_right)
        value |= 0xF000; // Extend sign bit

    ctx.result = value;
    ctx.running = false;

    if (ctx.done_interrupt)
        ctx.done_interrupt(true);
}

addr_read_result_t adc_bus_read(adc_ctx_t &ctx, uint16_t address)
{
    if (address == ctx.sfr.ADCON0)
    {
        uint8_t value = 0;

        value |= ((static_cast<uint8_t>(ctx.enabled) & 0x01) << 0);
        value |= ((static_cast<uint8_t>(ctx.running) & 0x01) << 1);

        if (static_cast<uint8_t>(ctx.negative_channel) <= static_cast<uint8_t>(adc_chan_t::an10))
        {
            uint8_t channel = static_cast<uint8_t>(ctx.positive_channel);
            value |= (((channel - 1) & 0x05) << 2);
        }
        else if (ctx.positive_channel == adc_chan_t::mux_disconnect)
            value |= (28 << 2);
        else if (ctx.positive_channel == adc_chan_t::temperature)
            value |= (29 << 2);
        else if (ctx.positive_channel == adc_chan_t::avddcore)
            value |= (30 << 2);
        else if (ctx.positive_channel == adc_chan_t::band_gap)
            value |= (31 << 2);

        return addr_read_result_t{.data = value, .mask = 0x7F};
    }

    if (address == ctx.sfr.ADCON1)
    {
        uint8_t value = 0;
        value |= (static_cast<uint8_t>(ctx.trigger_select) << 6);
        value |= ((static_cast<uint8_t>(ctx.positive_vref) & 0x03) << 4);
        value |= ((static_cast<uint8_t>(ctx.negative_vref) & 0x01) << 3);
        value |= (static_cast<uint8_t>(ctx.negative_channel) & 0x07);
        return addr_read_result_t{.data = value, .mask = 0xFF};
    }

    if (address == ctx.sfr.ADCON2)
    {
        uint8_t value = 0;
        value |= (static_cast<uint8_t>(ctx.result_format) << 7);
        value |= ((static_cast<uint8_t>(ctx.acquisition_time) & 0x03) << 3);
        value |= (static_cast<uint8_t>(ctx.clock_source) & 0x03);
        return addr_read_result_t{.data = value, .mask = 0xBF};
    }

    if (address == ctx.sfr.ADRESL)
    {
        uint8_t value = ctx.result & 0xFF;
        return addr_read_result_t{.data = value, .mask = 0xFF};
    }

    if (address == ctx.sfr.ADRESH)
    {
        uint8_t value = ctx.result >> 8;
        return addr_read_result_t{.data = value, .mask = 0xFF};
    }

    return addr_read_result_none;
}

addr_bit_mask_t adc_bus_write(adc_ctx_t &ctx, uint16_t address, uint8_t value)
{
    if (address == ctx.sfr.ADCON0)
    {
        ctx.enabled = value & (1 << 0);
        if (!ctx.enabled)
            abort_conversion(ctx);

        bool running = value & (1 << 1);
        if (running)
            start_conversion(ctx);
        else
            abort_conversion(ctx);

        uint8_t channel = (value >> 2) & 0x1F;
        if (channel <= 10)
            ctx.positive_channel = static_cast<adc_chan_t>(channel + 1);
        else if (channel == 28)
            ctx.positive_channel = adc_chan_t::mux_disconnect;
        else if (channel == 29)
            ctx.positive_channel = adc_chan_t::temperature;
        else if (channel == 30)
            ctx.positive_channel = adc_chan_t::avddcore;
        else if (channel == 31)
            ctx.positive_channel = adc_chan_t::band_gap;
        else
            ctx.positive_channel = adc_chan_t::random;

        return 0x7F;
    }

    if (address == ctx.sfr.ADCON1)
    {
        ctx.trigger_select = static_cast<adc_trigger_t>(value >> 6);
        ctx.positive_vref = static_cast<adc_vref_p_config_t>((value >> 4) & 0x03);
        ctx.negative_vref = static_cast<adc_vref_n_config_t>((value >> 3) & 0x01);
        ctx.negative_channel = static_cast<adc_chan_t>(value & 0x07);
        return 0xFF;
    }

    if (address == ctx.sfr.ADCON2)
    {
        ctx.result_format = static_cast<adc_result_format_t>(value >> 7);
        ctx.acquisition_time = static_cast<adc_acquisition_time_t>((value >> 3) & 0x03);
        uint8_t clock_source = value & 0x03;
        if (clock_source == 0x7)
            ctx.clock_source = adc_clock_source_t::frc;
        else
            ctx.clock_source = static_cast<adc_clock_source_t>(clock_source);

        return 0xBF;
    }

    if (address == ctx.sfr.ADRESL)
    {
        ctx.result &= 0xFF00;
        ctx.result |= value;
        return 0xFF;
    }

    if (address == ctx.sfr.ADRESH)
    {
        ctx.result &= 0x00FF;
        ctx.result |= (value << 8);
        return 0xFF;
    }

    return 0x00;
}

void adc_initialize(adc_ctx_t &ctx, const adc_known_sfrs_t &sfr)
{
    ctx.enabled = false;
    ctx.running = false;
    ctx.result = 0;
    ctx.tad_counter = 0;
    ctx.sfr = sfr;
    ctx.result_format = adc_result_format_t::justify_left;
    ctx.clock_source = adc_clock_source_t::fosc_2;
    ctx.acquisition_time = adc_acquisition_time_t::tad_0;
    ctx.negative_channel = adc_chan_t::avss;
    ctx.positive_channel = adc_chan_t::an0;
    ctx.trigger_select = adc_trigger_t::ccp1;
    ctx.positive_vref = adc_vref_p_config_t::avdd;
    ctx.negative_vref = adc_vref_n_config_t::avss;
    ctx.acquisition_cycles = 0;
    ctx.done_interrupt = nullptr;
    ctx.read_channel = nullptr;
}

void adc_reset(adc_ctx_t &ctx)
{
    ctx.enabled = false;
    ctx.running = false;
    ctx.tad_counter = 0;
    ctx.result_format = adc_result_format_t::justify_left;
    ctx.clock_source = adc_clock_source_t::fosc_2;
    ctx.acquisition_time = adc_acquisition_time_t::tad_0;
    ctx.negative_channel = adc_chan_t::avss;
    ctx.positive_channel = adc_chan_t::an0;
    ctx.trigger_select = adc_trigger_t::ccp1;
    ctx.positive_vref = adc_vref_p_config_t::avdd;
    ctx.negative_vref = adc_vref_n_config_t::avss;
}

static void handle_tad_tick(adc_ctx_t &ctx)
{
    if (ctx.tad_counter == ctx.acquisition_cycles)
    {
        adc_voltage_t positive_voltage = ctx.read_channel(ctx.positive_channel);
        adc_voltage_t negative_voltage = ctx.read_channel(ctx.negative_channel);

        adc_voltage_t positive_vref;
        adc_voltage_t negative_vref;

        assert(ctx.negative_vref == adc_vref_n_config_t::avss); // External vref not implemented
        negative_vref = ctx.read_channel(adc_chan_t::avss);

        assert(ctx.positive_vref == adc_vref_p_config_t::avdd ||
               ctx.positive_vref == adc_vref_p_config_t::internal_2_0 ||
               ctx.positive_vref == adc_vref_p_config_t::internal_4_1); // External vref not implemented

        if (ctx.positive_vref == adc_vref_p_config_t::internal_2_0)
            positive_vref = 2.0;
        else if (ctx.positive_vref == adc_vref_p_config_t::internal_4_1)
            positive_vref = 4.1;
        else
            positive_vref = ctx.read_channel(adc_chan_t::avddcore);

        // *_voltage_ratio is a value between 0 and 1.
        adc_voltage_t positive_voltage_ratio = (positive_voltage - negative_vref) / (positive_vref - negative_vref);
        adc_voltage_t negative_voltage_ratio = (negative_voltage - negative_vref) / (positive_vref - negative_vref);

        ctx.adjusted_measurement = positive_voltage_ratio - negative_voltage_ratio;
    }

    if (ctx.tad_counter == ctx.acquisition_cycles + 12)
    {
        finish_conversion(ctx);
    }

    ctx.tad_counter++;
}

void adc_tick_fosc(adc_ctx_t &ctx)
{
    if (!ctx.enabled || !ctx.running)
        return;

    if (ctx.clock_source == adc_clock_source_t::frc)
        return;

    uint8_t div = 0;
    if (ctx.clock_source == adc_clock_source_t::fosc_2)
        div = 2;
    else if (ctx.clock_source == adc_clock_source_t::fosc_4)
        div = 4;
    else if (ctx.clock_source == adc_clock_source_t::fosc_8)
        div = 8;
    else if (ctx.clock_source == adc_clock_source_t::fosc_16)
        div = 16;
    else if (ctx.clock_source == adc_clock_source_t::fosc_32)
        div = 32;
    else if (ctx.clock_source == adc_clock_source_t::fosc_64)
        div = 64;

    if (div == 0)
    {
        assert(false);
        return;
    }

    handle_tad_tick(ctx);
}

void adc_tick_frc(adc_ctx_t &ctx)
{
    if (!ctx.enabled || !ctx.running)
        return;

    if (ctx.clock_source != adc_clock_source_t::frc)
        return;

    handle_tad_tick(ctx);
}

void adc_ccp2_event(adc_ctx_t &ctx)
{
    if (ctx.enabled && ctx.trigger_select == adc_trigger_t::ccp2)
        start_conversion(ctx);
}

void adc_timer1_event(adc_ctx_t &ctx)
{
    if (ctx.enabled && ctx.trigger_select == adc_trigger_t::timer1)
        start_conversion(ctx);
}

void adc_ctmu_event(adc_ctx_t &ctx)
{
    if (ctx.enabled && ctx.trigger_select == adc_trigger_t::ctmu)
        start_conversion(ctx);
}

void adc_eccp1_event(adc_ctx_t &ctx)
{
    if (ctx.enabled && ctx.trigger_select == adc_trigger_t::ccp1)
        start_conversion(ctx);
}
