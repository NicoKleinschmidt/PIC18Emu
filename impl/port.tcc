#include "port.hpp"
#include <cassert>

constexpr uint8_t pin_to_bit(uint8_t pin, uint8_t bitmask)
{
    uint8_t pin_count = 0;
    for (uint8_t bit = 0; bit < 8; bit++)
    {
        if ((bitmask & (1 << bit)) == 0)
            continue;

        if (pin_count++ == pin)
            return bit;
    }

    return 0xFF;
}

static_assert(pin_to_bit(0, 0b11111011) == 0);
static_assert(pin_to_bit(7, 0b11111011) == 0xFF);
static_assert(pin_to_bit(2, 0b11111111) == 2);
static_assert(pin_to_bit(5, 0b11111010) == 7);

constexpr uint8_t bit_to_pin(uint8_t bit, uint8_t bitmask)
{
    if ((bitmask & (1 << bit)) == 0)
        return 0xFF;

    uint8_t pin = 0;
    for (uint8_t i = 0; i < bit; i++)
    {
        if ((bitmask & (1 << i)) != 0)
            pin++;
    }

    return pin;
}

static_assert(bit_to_pin(0, 0b11111111) == 0);
static_assert(bit_to_pin(2, 0b11111111) == 2);
static_assert(bit_to_pin(7, 0b11111010) == 5);
static_assert(bit_to_pin(2, 0b11111010) == 0xFF);

static inline void call_input_change_with_pu(port_ctx_t &ctx, uint8_t pin, io_state_t state)
{
    if (state == io_state_t::high_z && (ctx.global_pullup_enable || ctx.pullup_enable[pin]))
        state = io_state_t::high;

    if (ctx.on_input_changed != nullptr)
        ctx.on_input_changed(pin, state);
}

static inline void set_pin_mode(port_ctx_t &ctx, uint8_t pin, port_data_direction_t dd, bool analog)
{
    double new_analog_out = 0;
    double new_analog_in = 0;
    io_state_t new_digital_out = io_state_t::high_z;
    io_state_t new_digital_in = io_state_t::high_z;

    if (analog && dd == port_data_direction_t::input)
        new_analog_in = ctx.analog_in_value_buffer[pin];
    else if (analog && dd == port_data_direction_t::output)
        new_analog_out = ctx.analog_out_value_buffer[pin];
    else if (!analog && dd == port_data_direction_t::input)
        new_digital_in = ctx.digital_in_value_buffer[pin];
    else if (!analog && dd == port_data_direction_t::output)
        new_digital_out = ctx.digital_out_value_buffer[pin];
    else
        assert(false);

    // TODO: Check the previous mode of the pin and call functions only when required.

    ctx.on_analog_input_changed(pin, new_analog_in);
    ctx.on_analog_output_changed(pin, new_analog_out);
    call_input_change_with_pu(ctx, pin, new_digital_in);
    ctx.on_output_changed(pin, new_digital_out);

    ctx.analog_enable[pin] = analog;
    ctx.data_direction[pin] = dd;
}

static inline void update_pin_pullup(port_ctx_t &ctx, uint8_t pin)
{
    if (ctx.data_direction[pin] != port_data_direction_t::input || ctx.analog_enable[pin])
        return;

    if (ctx.digital_in_value_buffer[pin] == io_state_t::high_z)
        call_input_change_with_pu(ctx, pin, ctx.digital_in_value_buffer[pin]);
}

template <port_bus_config_t buscfg> static void set_global_pullup(port_ctx_t &ctx, bool state)
{
    if (ctx.global_pullup_enable == state)
        return;

    ctx.global_pullup_enable = state;

    for (uint8_t pin = 0; pin < 8; pin++)
    {
        uint8_t bit = pin_to_bit(pin, buscfg.bitmask);
        if (bit == 0xFF)
            continue;

        update_pin_pullup(ctx, pin);
    }
}

static inline void set_pin_pullup(port_ctx_t &ctx, uint8_t pin, bool state)
{
    if (ctx.pullup_enable[pin] == state)
        return;

    ctx.pullup_enable[pin] = state;
    update_pin_pullup(ctx, pin);
}

template <port_bus_config_t buscfg> inline addr_read_result_t port_bus_read(port_ctx_t &ctx, uint16_t address)
{
    if (address == buscfg.TRISx)
    {
        uint8_t value = 0;
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            uint8_t pin = bit_to_pin(bit, buscfg.bitmask);
            if (pin == 0xFF)
                continue;

            if (ctx.data_direction[pin] == port_data_direction_t::input)
                value |= (1 << bit);
        }
        return addr_read_result_t{.data = value, .mask = buscfg.bitmask};
    }

    if (address == buscfg.ANCONx)
    {
        if (buscfg.ansel_enable_mask == 0)
            return addr_read_result_none;

        uint8_t value = 0;
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            uint8_t pin = bit_to_pin(bit, buscfg.bitmask);
            if (pin == 0xFF)
                continue;

            if ((buscfg.ansel_enable_mask & (1 << bit)) && ctx.analog_enable[pin])
                value |= (1 << bit);
        }
        return addr_read_result_t{.data = value, .mask = buscfg.ansel_enable_mask};
    }

    if (address == buscfg.PADCFGx)
    {
        if (!buscfg.padcfg_enable)
            return addr_read_result_none;

        uint8_t padcfg_state = ctx.global_pullup_enable != buscfg.padcfg_invert;
        uint8_t value = padcfg_state << buscfg.padcfg_bit;

        return addr_read_result_t{.data = value, .mask = (1 << buscfg.padcfg_bit)};
    }

    if (address == buscfg.WPUx)
    {
        if (!buscfg.wpu_enable)
            return addr_read_result_none;

        uint8_t value = 0;
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            uint8_t pin = bit_to_pin(bit, buscfg.bitmask);
            if (pin == 0xFF)
                continue;

            if (ctx.pullup_enable[pin])
                value |= (1 << bit);
        }
        return addr_read_result_t{.data = value, .mask = buscfg.bitmask};
    }

    return addr_read_result_none;
}

template <port_bus_config_t buscfg>
inline addr_bit_mask_t port_bus_write(port_ctx_t &ctx, uint16_t address, uint8_t value)
{
    if (address == buscfg.TRISx)
    {
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            uint8_t pin = bit_to_pin(bit, buscfg.bitmask);
            if (pin == 0xFF)
                continue;

            if ((value & (1 << bit)) != 0)
                set_pin_mode(ctx, pin, port_data_direction_t::input, ctx.analog_enable[pin]);
            else
                set_pin_mode(ctx, pin, port_data_direction_t::output, ctx.analog_enable[pin]);
        }
        return buscfg.bitmask;
    }

    if (address == buscfg.ANCONx)
    {
        if (buscfg.ansel_enable_mask == 0)
            return 0x00;

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            uint8_t pin = bit_to_pin(bit, buscfg.bitmask);
            if (pin == 0xFF || (buscfg.ansel_enable_mask & (1 << bit)) == 0)
                continue;

            bool analog = (value & (1 << bit)) != 0;
            set_pin_mode(ctx, pin, ctx.data_direction[pin], analog);
        }
        return buscfg.ansel_enable_mask;
    }

    if (address == buscfg.PADCFGx)
    {
        if (!buscfg.padcfg_enable)
            return 0x00;

        bool bit_set = (value & (1 << buscfg.padcfg_bit));
        bool padcfg_state = bit_set != buscfg.padcfg_invert;
        set_global_pullup<buscfg>(ctx, padcfg_state);

        return 1 << buscfg.padcfg_bit;
    }

    if (address == buscfg.WPUx)
    {
        if (!buscfg.wpu_enable)
            return 0x00;

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            uint8_t pin = bit_to_pin(bit, buscfg.bitmask);
            if (pin == 0xFF)
                continue;

            bool pullup_enable = (value & (1 << bit)) != 0;
            set_pin_pullup(ctx, pin, pullup_enable);
        }

        return buscfg.bitmask;
    }

    return 0x00;
}
