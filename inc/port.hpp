#pragma once

#include "bus.hpp"
#include "io.hpp"
#include <cstdint>
#include <functional>

enum class port_data_direction_t
{
    input,
    output,
};

struct port_bus_config_t
{
    uint8_t bitmask;
    uint16_t TRISx;
    uint16_t PADCFGx;
    uint16_t ANCONx;
    uint16_t WPUx;
    uint8_t padcfg_bit;
    bool padcfg_enable;
    bool padcfg_invert;
    bool wpu_enable;
    uint8_t ansel_enable_mask;
};

struct port_ctx_t
{
    std::function<void(uint8_t pin, io_state_t state)> on_input_changed;
    std::function<void(uint8_t pin, io_state_t state)> on_output_changed;
    std::function<void(uint8_t pin, double value)> on_analog_input_changed;
    std::function<void(uint8_t pin, double value)> on_analog_output_changed;
    std::array<port_data_direction_t, 8> data_direction;
    std::array<bool, 8> analog_enable;
    std::array<bool, 8> pullup_enable;
    bool global_pullup_enable;

    // These are used to restore the previous state of input/output when the data direction or analog mode changes.
    std::array<double, 8> analog_in_value_buffer;
    std::array<double, 8> analog_out_value_buffer;
    std::array<io_state_t, 8> digital_in_value_buffer;
    std::array<io_state_t, 8> digital_out_value_buffer;
};

template <port_bus_config_t buscfg> addr_read_result_t port_bus_read(port_ctx_t &ctx, uint16_t address);

template <port_bus_config_t buscfg> addr_bit_mask_t port_bus_write(port_ctx_t &ctx, uint16_t address, uint8_t value);

void port_initialize(port_ctx_t &ctx);

void port_input_set_digital(port_ctx_t &ctx, uint8_t pin, io_state_t state);

void port_input_set_analog(port_ctx_t &ctx, uint8_t pin, double value);

void port_output_set_digital(port_ctx_t &ctx, uint8_t pin, io_state_t state);

void port_output_set_analog(port_ctx_t &ctx, uint8_t pin, double value);

#include "port.tcc"
