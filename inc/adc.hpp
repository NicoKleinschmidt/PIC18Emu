#pragma once

#include "bus.hpp"
#include <cstdint>
#include <functional>

using adc_voltage_t = double;

enum class adc_chan_t
{
    avss,
    an0,
    an1,
    an2,
    an3,
    an4,
    an5,
    an6,
    an7,
    an8,
    an9,
    an10,
    mux_disconnect = 28,
    avddcore,
    temperature,
    band_gap,
    random = 0xFF,
};

enum class adc_result_format_t
{
    justify_left,
    justify_right,
};

enum class adc_acquisition_time_t
{
    tad_0,
    tad_2,
    tad_4,
    tad_6,
    tad_8,
    tad_12,
    tad_16,
    tad_20,
};

enum class adc_clock_source_t
{
    fosc_2,
    fosc_8,
    fosc_32,
    frc,
    fosc_4,
    fosc_16,
    fosc_64,
};

enum class adc_vref_n_config_t
{
    avss,
    external,
};

enum class adc_vref_p_config_t
{
    avdd,
    external,
    internal_2_0,
    internal_4_1,
};

enum class adc_trigger_t
{
    ccp1,
    ctmu,
    timer1,
    ccp2,
};

struct adc_known_sfrs_t
{
    uint16_t ANCON0;
    uint16_t ANCON1;
    uint16_t ADCON0;
    uint16_t ADCON1;
    uint16_t ADCON2;
    uint16_t ADRESH;
    uint16_t ADRESL;
};

struct adc_ctx_t
{
    bool enabled;
    bool running;
    uint16_t result;
    adc_result_format_t result_format;
    adc_acquisition_time_t acquisition_time;
    adc_clock_source_t clock_source;
    adc_chan_t positive_channel;
    adc_chan_t negative_channel;
    adc_vref_p_config_t positive_vref;
    adc_vref_n_config_t negative_vref;
    adc_trigger_t trigger_select;
    adc_known_sfrs_t sfr;

    uint8_t tad_counter;
    uint8_t acquisition_cycles;
    adc_voltage_t adjusted_measurement;
    std::function<adc_voltage_t(adc_chan_t channel)> read_channel;
    std::function<void(bool set)> done_interrupt;
};

addr_read_result_t adc_bus_read(adc_ctx_t &ctx, uint16_t address);
addr_bit_mask_t adc_bus_write(adc_ctx_t &ctx, uint16_t address, uint8_t value);
void adc_tick_fosc(adc_ctx_t &ctx);
void adc_tick_frc(adc_ctx_t &ctx);
void adc_initialize(adc_ctx_t &ctx, const adc_known_sfrs_t &sfr);
void adc_reset(adc_ctx_t &ctx);
void adc_ccp2_event(adc_ctx_t &ctx);
void adc_timer1_event(adc_ctx_t &ctx);
void adc_ctmu_event(adc_ctx_t &ctx);
void adc_eccp1_event(adc_ctx_t &ctx);
