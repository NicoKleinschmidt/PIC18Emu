#include "int.hpp"

template <size_t SRC_COUNT>
void interrupt_tick(bus_reader_t<uint32_t, uint8_t> read_prog_bus, int_state_t<SRC_COUNT> &state, vector_func_t vector,
                    wakeup_func_t wake)
{
    uint8_t interrupt_prio_enable = state.IPEN;
    uint8_t interrupt_enable_high = state.GIEH;
    uint8_t interrupt_enable_low = state.GIEL;

    bool high_prio_requested = false;
    bool low_prio_requested = false;
    bool is_peripheral_interrupt = false;

    auto handle_interrupt_src = [&](bool requested, bool enable, bool high_prio, bool peripheral) {
        if (high_prio_requested)
            return;

        if (!high_prio && low_prio_requested)
            return;

        if (!requested || !enable)
            return;

        if (high_prio)
            high_prio_requested = true;
        else
            low_prio_requested = true;

        is_peripheral_interrupt = peripheral;
    };

    for (size_t i = 0; i < SRC_COUNT; i++)
    {
        int_source_t src = state.sources[i];
        handle_interrupt_src(src.requested, src.enabled, src.high_priority, src.is_peripheral);
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

    if (should_wake)
    {
        wake();
    }

    if (should_interrupt_high_prio)
    {
        state.GIEH = false;
        vector(true);
    }
    else if (should_interrupt_low_prio)
    {
        if (interrupt_prio_enable)
            state.GIEL = false;
        else
            state.GIEH = false;
        vector(false);
    }
}
