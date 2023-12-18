#include "port.hpp"

void port_initialize(port_ctx_t &ctx)
{
    ctx.data_direction.fill(port_data_direction_t::input);
    ctx.pullup_enable.fill(false);
    ctx.analog_enable.fill(false);
    ctx.digital_in_value_buffer.fill(io_state_t::high_z);
    ctx.digital_out_value_buffer.fill(io_state_t::high_z);
    ctx.analog_in_value_buffer.fill(0);
    ctx.analog_out_value_buffer.fill(0);
}

void port_input_set_digital(port_ctx_t &ctx, uint8_t pin, io_state_t state)
{
    ctx.digital_in_value_buffer[pin] = state;
    if (!ctx.analog_enable[pin] && ctx.data_direction[pin] == port_data_direction_t::input)
    {
        call_input_change_with_pu(ctx, pin, state);
    }
}

void port_input_set_analog(port_ctx_t &ctx, uint8_t pin, double value)
{
    ctx.analog_in_value_buffer[pin] = value;
    if (ctx.analog_enable[pin] && ctx.data_direction[pin] == port_data_direction_t::input)
    {
        ctx.on_analog_input_changed(pin, value);
    }
}

void port_output_set_digital(port_ctx_t &ctx, uint8_t pin, io_state_t state)
{
    ctx.digital_out_value_buffer[pin] = state;
    if (!ctx.analog_enable[pin] && ctx.data_direction[pin] == port_data_direction_t::output)
    {
        ctx.on_output_changed(pin, state);
    }
}

void port_output_set_analog(port_ctx_t &ctx, uint8_t pin, double value)
{
    ctx.analog_out_value_buffer[pin] = value;
    if (ctx.analog_enable[pin] && ctx.data_direction[pin] == port_data_direction_t::output)
    {
        ctx.on_analog_output_changed(pin, value);
    }
}
