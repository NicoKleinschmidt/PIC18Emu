#pragma once

#include <cstdint>
#include <functional>

using gpio_on_change_interrupt_req = std::function<void(bool set)>;
using gpio_change_cb = std::function<void(uint8_t pin_num, io_state_t state)>;

/// @warning Don't modify directly, use the gpio_* functions.
template <size_t pin_count> struct gpio_ctx_t
{
    static_assert(pin_count <= 8);

    uint16_t portx;
    uint16_t latx;
    gpio_change_cb on_output_changed;
    std::array<gpio_on_change_interrupt_req, pin_count> on_change_interrupts;
    std::array<bool, pin_count> output_latch;

    std::array<io_state_t, pin_count> input_states;
};

template <size_t count> addr_read_result_t gpio_bus_read_port(gpio_ctx_t<count> &ctx)
{
    uint8_t value = 0;
    for (size_t pin = 0; pin < count; pin++)
    {
        if (ctx.input_states[pin] == io_state_t::high)
            value |= (1 << pin);
    }
    return addr_read_result_t{.data = value, .mask = 0xFF >> (8 - count)};
}

template <size_t count> addr_read_result_t gpio_bus_read_lat(gpio_ctx_t<count> &ctx)
{
    uint8_t value = 0;
    for (size_t pin = 0; pin < count; pin++)
    {
        if (ctx.output_latch[pin])
            value |= (1 << pin);
    }
    return addr_read_result_t{.data = value, .mask = 0xFF >> (8 - count)};
}

template <size_t count> addr_bit_mask_t gpio_bus_write_port(gpio_ctx_t<count> &ctx, uint8_t value)
{
    return gpio_bus_write_lat(ctx, value);
}

template <size_t count> addr_bit_mask_t gpio_bus_write_lat(gpio_ctx_t<count> &ctx, uint8_t value)
{
    for (size_t pin = 0; pin < count; pin++)
    {
        bool set = (value & (1 << pin));
        if (set == ctx.output_latch[pin])
            continue;

        ctx.output_latch[pin] = set;

        ctx.input_states[pin] = set ? io_state_t::high : io_state_t::low;
        if (ctx.on_output_changed != nullptr)
            ctx.on_output_changed(pin, ctx.input_states[pin]);
    }

    return 0xFF >> (8 - count);
}

template <size_t count> addr_read_result_t gpio_bus_read(gpio_ctx_t<count> &ctx, uint16_t addr)
{
    if (addr == ctx.latx)
        return gpio_bus_read_lat(ctx);
    if (addr == ctx.portx)
        return gpio_bus_read_port(ctx);

    return addr_read_result_none;
}

template <size_t count> addr_bit_mask_t gpio_bus_write(gpio_ctx_t<count> &ctx, uint16_t addr, uint8_t value)
{
    if (addr == ctx.latx)
        return gpio_bus_write_lat(ctx, value);
    if (addr == ctx.portx)
        return gpio_bus_write_port(ctx, value);

    return 0x00;
}

template <size_t count> void gpio_set_input_state(gpio_ctx_t<count> &ctx, uint8_t pin_num, io_state_t state)
{
    if (state == io_state_t::high_z)
        state = io_state_t::low;

    if (state == ctx.input_states[pin_num])
        return;

    ctx.input_states[pin_num] = state;

    gpio_on_change_interrupt_req on_change = ctx.on_change_interrupts[pin_num];
    if (on_change != nullptr)
        on_change(true);
}

template <size_t count> bool gpio_get_output_state(gpio_ctx_t<count> &ctx, uint8_t pin_num)
{
    return ctx.output_latch[pin_num];
}

/// @brief Registers an event handler, that gets called when the value of an output pin changes.
///        Only one handler can be registered at one time, registering another overwrites the last one.
/// @tparam count
/// @param ctx The context to use
/// @param cb The callback to register, can be nullptr to remove a callback.
template <size_t count> void gpio_on_output_changed(gpio_ctx_t<count> &ctx, gpio_change_cb cb)
{
    ctx.on_output_changed = cb;
}

template <size_t count>
void gpio_add_change_interrupt(gpio_ctx_t<count> &ctx, uint8_t pin_num, gpio_on_change_interrupt_req cb)
{
    ctx.on_change_interrupts[pin_num] = cb;
}

template <size_t count> void gpio_initialize(gpio_ctx_t<count> &ctx, uint16_t port_reg, uint16_t lat_reg)
{
    ctx.output_latch.fill(false);
    ctx.input_states.fill(io_state_t::high_z);
    ctx.portx = port_reg;
    ctx.latx = lat_reg;
}
