#include "ccp.hpp"
#include <cassert>

static bool is_capture_mode(ccp_mode_t mode)
{
    return mode == ccp_mode_t::capture_16th_rising || mode == ccp_mode_t::capture_4th_rising ||
           mode == ccp_mode_t::capture_falling || mode == ccp_mode_t::capture_rising;
}

static bool is_compare_mode(ccp_mode_t mode)
{
    return mode == ccp_mode_t::compare_force_high || mode == ccp_mode_t::compare_force_low ||
           mode == ccp_mode_t::compare_software || mode == ccp_mode_t::compare_special_event ||
           mode == ccp_mode_t::compare_toggle;
}

static bool is_pwm_mode(ccp_mode_t mode)
{
    return mode == ccp_mode_t::pwm_ac_high_bd_high || mode == ccp_mode_t::pwm_ac_high_bd_low ||
           mode == ccp_mode_t::pwm_ac_low_bd_high || mode == ccp_mode_t::pwm_ac_low_bd_low;
}

static io_state_t get_pin_state(const ccp_ctx_t &ctx)
{
    if (is_capture_mode(ctx.mode))
        return io_state_t::high_z;

    if (ctx.mode == ccp_mode_t::compare_special_event || ctx.mode == ccp_mode_t::compare_software)
        return io_state_t::high_z;

    if (ctx.output_latch == false)
        return io_state_t::low;
    if (ctx.output_latch == true && ctx.open_drain == false)
        return io_state_t::high;

    return io_state_t::high_z;
}

/// @brief Returns the timer number (0-4) that this CCP module is based on.
/// @param ctx The CCP module configuration
/// @return The timer number
static uint8_t get_timer_num(const ccp_ctx_t &ctx)
{
    bool pwm_mode = is_pwm_mode(ctx.mode);

    if (pwm_mode && ctx.timer == ccp_timer_selection_t::timer_1_2)
        return 1;
    else if (pwm_mode && ctx.timer == ccp_timer_selection_t::timer_3_4)
        return 3;
    else if (!pwm_mode && ctx.timer == ccp_timer_selection_t::timer_1_2)
        return 2;
    else if (!pwm_mode && ctx.timer == ccp_timer_selection_t::timer_3_4)
        return 4;

    assert(false);
    return 0;
}

/// @brief Reads the counter register of the specified timer
/// @return The counter value
static uint16_t read_timer_register(ccp_ctx_t &ctx, bus_reader_t<uint16_t, uint8_t> read_bus)
{
    uint8_t timer_num = get_timer_num(ctx);
    if (timer_num == 2)
        return read_bus(ctx.known_sfrs.common.TMR2);
    if (timer_num == 4)
        return read_bus(ctx.known_sfrs.common.TMR4);

    if (timer_num == 1)
    {
        uint8_t low = read_bus(ctx.known_sfrs.common.TMR1L);
        uint8_t high = read_bus(ctx.known_sfrs.common.TMR1H);
        return (static_cast<uint16_t>(high << 8) | static_cast<uint16_t>(low));
    }

    if (timer_num == 3)
    {
        uint8_t low = read_bus(ctx.known_sfrs.common.TMR3L);
        uint8_t high = read_bus(ctx.known_sfrs.common.TMR3H);
        return (static_cast<uint16_t>(high << 8) | static_cast<uint16_t>(low));
    }

    assert(false);
    return 0;
}

static void handle_pwm_tick(ccp_ctx_t &ctx, bus_reader_t<uint16_t, uint8_t> read_bus)
{
    assert(is_pwm_mode(ctx.mode));

    uint8_t timer_value = static_cast<uint8_t>(read_timer_register(ctx, read_bus));
    uint8_t dc_latch_high = static_cast<uint8_t>(ctx.duty_cycle_latch >> 2);

    if (timer_value == dc_latch_high)
    {
        ctx.output_latch = false;
        if (ctx.set_pin != nullptr)
            ctx.set_pin(get_pin_state(ctx));
    }
}

static void handle_compare_tick(ccp_ctx_t &ctx, bus_reader_t<uint16_t, uint8_t> read_bus)
{
    assert(is_compare_mode(ctx.mode));

    uint8_t timer_num = get_timer_num(ctx);
    uint16_t timer_value = read_timer_register(ctx, read_bus);
    if (timer_value != ctx.ccp_register)
        return;

    if (ctx.interrupt != nullptr)
        ctx.interrupt(true);

    else if (ctx.mode == ccp_mode_t::compare_special_event)
    {
        if (ctx.special_event_timer != nullptr)
            ctx.special_event_timer(timer_num);
    }
    else if (ctx.mode == ccp_mode_t::compare_toggle)
    {
        ctx.output_latch = !ctx.output_latch;
        if (ctx.set_pin != nullptr)
            ctx.set_pin(get_pin_state(ctx));
    }
    else if (ctx.mode == ccp_mode_t::compare_force_high)
    {
        ctx.output_latch = true;
        if (ctx.set_pin != nullptr)
            ctx.set_pin(get_pin_state(ctx));
    }
    else if (ctx.mode == ccp_mode_t::compare_force_low)
    {
        ctx.output_latch = false;
        if (ctx.set_pin != nullptr)
            ctx.set_pin(get_pin_state(ctx));
    }
}

void ccp_tick(ccp_ctx_t &ctx, bus_reader_t<uint16_t, uint8_t> read_bus)
{
    if (is_compare_mode(ctx.mode))
        handle_compare_tick(ctx, read_bus);
    else if (is_pwm_mode(ctx.mode))
        handle_pwm_tick(ctx, read_bus);
}

void ccp_pwm_match_input(ccp_ctx_t &ctx, uint8_t timer_num, bus_reader_t<uint16_t, uint8_t> read_bus)
{
    if (!is_pwm_mode(ctx.mode))
        return;
    if (timer_num != get_timer_num(ctx))
        return;

    uint16_t dc_high = ctx.ccp_register >> 8;
    uint8_t dc_low = ctx.duty_cycle_lower & 0x3;

    ctx.duty_cycle_latch = (dc_high << 2) | dc_low;
    if (ctx.duty_cycle_latch == 0)
        return;

    ctx.output_latch = true;
    if (ctx.set_pin != nullptr)
        ctx.set_pin(get_pin_state(ctx));
}

static void handle_capture(ccp_ctx_t &ctx, bus_reader_t<uint16_t, uint8_t> read_bus)
{
    assert(is_capture_mode(ctx.mode));

    ctx.prescaler_counter++;
    uint8_t div;
    if (ctx.mode == ccp_mode_t::capture_4th_rising)
        div = 4;
    else if (ctx.mode == ccp_mode_t::capture_16th_rising)
        div = 16;
    else
        div = 1;

    if (ctx.prescaler_counter % div != 0)
        return;

    ctx.ccp_register = read_timer_register(ctx, read_bus);
    if (ctx.interrupt != nullptr)
        ctx.interrupt(true);
}

void ccp_pin_input(ccp_ctx_t &ctx, bool high, bus_reader_t<uint16_t, uint8_t> read_bus)
{
    if (!is_capture_mode(ctx.mode))
        return;

    if (high && ctx.mode != ccp_mode_t::capture_falling)
        handle_capture(ctx, read_bus);
    else if (!high && ctx.mode == ccp_mode_t::capture_falling)
        handle_capture(ctx, read_bus);
}

void ccp_can_msg_received(ccp_ctx_t &ctx, bus_reader_t<uint16_t, uint8_t> read_bus)
{
    if (is_capture_mode(ctx.mode))
        handle_capture(ctx, read_bus);
}

addr_read_result_t ccp_bus_read(ccp_ctx_t &ctx, uint16_t address)
{
    if (address == ctx.known_sfrs.CCPRxH)
    {
        return addr_read_result_t{.data = static_cast<uint8_t>(ctx.ccp_register >> 8), .mask = 0xFF};
    }

    if (address == ctx.known_sfrs.CCPRxL)
    {
        return addr_read_result_t{.data = static_cast<uint8_t>(ctx.ccp_register), .mask = 0xFF};
    }

    if (address == ctx.known_sfrs.CCPxCON)
    {
        uint8_t value = 0;
        value |= (static_cast<uint8_t>(ctx.mode) & 0x0F);
        value |= (ctx.duty_cycle_lower & 0x3) << 4;
        return addr_read_result_t{.data = value, .mask = 0x3F};
    }

    if (address == ctx.known_sfrs.common.CCPTMRS)
    {
        uint8_t bit = ctx.ccp_num - 1;
        return addr_read_result_t{
            .data = static_cast<uint8_t>(static_cast<uint8_t>(ctx.timer) << bit),
            .mask = static_cast<uint8_t>(1 << bit),
        };
    }

    return addr_read_result_none;
}

addr_bit_mask_t ccp_bus_write(ccp_ctx_t &ctx, uint16_t address, uint8_t value)
{
    if (address == ctx.known_sfrs.CCPRxH)
    {
        if (is_pwm_mode(ctx.mode))
            return 0x00; // Read only register in PWM mode.

        ctx.ccp_register &= 0x00FF;
        ctx.ccp_register |= (static_cast<uint16_t>(value) << 8);
        return 0xFF;
    }

    if (address == ctx.known_sfrs.CCPRxL)
    {
        ctx.ccp_register &= 0xFF00;
        ctx.ccp_register |= static_cast<uint16_t>(value);
        return 0xFF;
    }

    if (address == ctx.known_sfrs.CCPxCON)
    {
        ctx.mode = static_cast<ccp_mode_t>(value & 0x0F);
        ctx.duty_cycle_lower = (value >> 4) & 0x3;
        return 0x3F;
    }

    if (address == ctx.known_sfrs.common.CCPTMRS)
    {
        uint8_t bit = ctx.ccp_num - 1;
        if (value & (1 << bit))
            ctx.timer = ccp_timer_selection_t::timer_3_4;
        else
            ctx.timer = ccp_timer_selection_t::timer_1_2;

        return 1 << bit;
    }

    return 0x00;
}

void ccp_initialize(ccp_ctx_t &ctx, uint8_t ccp_num, const ccp_known_sfrs_t &sfrs)
{
    ctx.ccp_num = ccp_num;
    ctx.mode = ccp_mode_t::disabled;
    ctx.timer = ccp_timer_selection_t::timer_1_2;
    ctx.output_latch = false;
    ctx.open_drain = false;
    ctx.ccp_register = 0;
    ctx.duty_cycle_latch = 0;
    ctx.duty_cycle_lower = 0;
    ctx.prescaler_counter = 0;
    ctx.interrupt = nullptr;
    ctx.set_pin = nullptr;
    ctx.special_event_timer = nullptr;
    ctx.known_sfrs = sfrs;
}

void ccp_reset(ccp_ctx_t &ctx)
{
}
