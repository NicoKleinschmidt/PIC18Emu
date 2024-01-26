#include "int.hpp"
#include "18f66k80.hpp"

constexpr uint8_t IPEN_bit = 7;
constexpr uint8_t GIEH_bit = 7;
constexpr uint8_t GIEL_bit = 6;

void interrupt_tick(int_ctx_t &ctx)
{
    uint8_t interrupt_prio_enable = ctx.IPEN;
    uint8_t interrupt_enable_high = ctx.GIEH;
    uint8_t interrupt_enable_low = ctx.GIEL;

    bool high_prio_requested = false;
    bool low_prio_requested = false;
    bool is_peripheral_interrupt = false;

    auto handle_interrupt_src = [&](bool flagged, bool enable, bool high_prio, bool peripheral) {
        if (high_prio_requested)
            return;

        if (!high_prio && low_prio_requested)
            return;

        if (!flagged || !enable)
            return;

        if (high_prio)
            high_prio_requested = true;
        else
            low_prio_requested = true;

        is_peripheral_interrupt = peripheral;
    };

    for (const auto &src : ctx.sources)
    {
        handle_interrupt_src(src.flagged, src.enabled, src.high_priority, src.is_peripheral);
    }

    if (!interrupt_prio_enable)
    {
        low_prio_requested = low_prio_requested || high_prio_requested;
    }

    bool should_wake = low_prio_requested || high_prio_requested;
    bool should_interrupt_high_prio = interrupt_prio_enable && interrupt_enable_high && high_prio_requested;
    bool should_interrupt_low_prio =
        (interrupt_prio_enable && interrupt_enable_low && interrupt_enable_high && low_prio_requested) ||
        (!interrupt_prio_enable && !is_peripheral_interrupt && interrupt_enable_high) ||
        (!interrupt_prio_enable && is_peripheral_interrupt && interrupt_enable_low && interrupt_enable_high);

    if (should_wake && ctx.wakeup != nullptr)
    {
        ctx.wakeup();
    }

    if (should_interrupt_high_prio)
    {
        ctx.GIEH = false;
        if (ctx.vector != nullptr)
        {
            ctx.vector(true);
        }
    }
    else if (should_interrupt_low_prio)
    {
        if (interrupt_prio_enable)
            ctx.GIEL = false;
        else
            ctx.GIEH = false;

        if (ctx.vector != nullptr)
        {
            ctx.vector(false);
        }
    }
}

static addr_read_result_t read_sources(const std::vector<int_source_t> &sources, uint16_t addr)
{
    if (addr == 0)
        return addr_read_result_none;

    addr_read_result_t result = {.data = 0, .mask = 0};

    for (const auto &src : sources)
    {
        if (src.enabled_reg == addr)
        {
            result.data |= (src.enabled << src.enabled_bit);
            result.mask |= (1 << src.enabled_bit);
        }
        if (src.priority_reg == addr)
        {
            result.data |= (src.high_priority << src.priority_bit);
            result.mask |= (1 << src.priority_bit);
        }
        if (src.flag_reg == addr)
        {
            result.data |= (src.flagged << src.flag_bit);
            result.mask |= (1 << src.flag_bit);
        }
    }

    return result;
}

static addr_bit_mask_t write_sources(std::vector<int_source_t> &sources, uint16_t addr, uint8_t value)
{
    if (addr == 0)
        return 0x00;

    addr_bit_mask_t mask = 0;

    for (auto &src : sources)
    {
        if (src.enabled_reg == addr)
        {
            src.enabled = value & (1 << src.enabled_bit);
            mask |= (1 << src.enabled_bit);
        }
        if (src.priority_reg == addr)
        {
            src.high_priority = value & (1 << src.priority_bit);
            mask |= (1 << src.priority_bit);
        }
        if (src.flag_reg == addr)
        {
            if (!src.flag_read_only)
            {
                src.flagged = value & (1 << src.flag_bit);
            }
            mask |= (1 << src.flag_bit);
        }
    }

    return mask;
}

addr_read_result_t interrupt_bus_read(int_ctx_t &ctx, uint16_t addr)
{
    addr_read_result_t result = {.data = 0, .mask = 0};

    if (addr == ctx.sfr.INTCON)
    {
        result.data |= (ctx.GIEH << GIEH_bit);
        result.data |= (ctx.GIEL << GIEL_bit);
        result.mask |= (1 << GIEH_bit) | (1 << GIEL_bit);
    }
    else if (addr == ctx.sfr.RCON)
    {
        result.data |= (ctx.IPEN << IPEN_bit);
        result.mask |= 1 << IPEN_bit;
    }

    addr_read_result_t src_result = read_sources(ctx.sources, addr);
    result.data |= src_result.data;
    result.mask |= src_result.mask;

    return result;
}

addr_bit_mask_t interrupt_bus_write(int_ctx_t &ctx, uint16_t addr, uint8_t value)
{
    uint8_t mask = 0;

    if (addr == ctx.sfr.INTCON)
    {
        ctx.GIEH = value & (1 << GIEH_bit);
        ctx.GIEL = value & (1 << GIEL_bit);

        mask |= (1 << GIEH_bit) | (1 << GIEL_bit);
    }
    else if (addr == ctx.sfr.RCON)
    {
        ctx.IPEN = value & (1 << IPEN_bit);
        mask |= 1 << IPEN_bit;
    }

    mask |= write_sources(ctx.sources, addr, value);

    return mask;
}

void interrupt_set_flag(int_source_t &src, bool b)
{
    src.flagged = b;
}

void interrupt_initialize(int_ctx_t &ctx)
{
    ctx.sources.clear();
}

void interrupt_reset(int_ctx_t &ctx)
{
    ctx.IPEN = false;
    ctx.GIEH = false;
    ctx.GIEL = false;
}
