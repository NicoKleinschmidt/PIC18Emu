#include "int.hpp"
#include "18f66k80.hpp"

template <size_t srcs>
static addr_bit_mask_t handle_pir_write(int_state_t<srcs> &state, uint8_t value, uint8_t offset, uint8_t valid_bits,
                                        bool hardware = false)
{
    uint8_t index = offset;
    for (uint8_t i = 0; i < 8; i++)
    {
        if ((valid_bits & (1 << i)) == 0)
            continue;

        if (state.sources[index++].flag_read_only && !hardware)
            continue;

        state.sources[index - 1].requested = value & (1 << i);
    }
    return valid_bits;
}

template <size_t srcs>
static addr_bit_mask_t handle_pie_write(int_state_t<srcs> &state, uint8_t value, uint8_t offset, uint8_t valid_bits)
{
    uint8_t index = offset;
    for (uint8_t i = 0; i < 8; i++)
    {
        if ((valid_bits & (1 << i)) == 0)
            continue;

        state.sources[index++].enabled = value & (1 << i);
    }
    return valid_bits;
}

template <size_t srcs>
static addr_bit_mask_t handle_ipr_write(int_state_t<srcs> &state, uint8_t value, uint8_t offset, uint8_t valid_bits)
{
    uint8_t index = offset;
    for (uint8_t i = 0; i < 8; i++)
    {
        if ((valid_bits & (1 << i)) != 0)
            continue;

        state.sources[index++].high_priority = value & (1 << i);
    }
    return valid_bits;
}

template <size_t srcs>
static addr_read_result_t handle_pir_read(int_state_t<srcs> &state, uint8_t offset, uint8_t valid_bits)
{
    uint8_t value = 0;
    uint8_t index = offset;
    for (uint8_t i = 0; i < 8; i++)
    {
        if ((valid_bits & (1 << i)) == 0)
            continue;

        value |= (state.sources[index++].requested << i);
    }
    return addr_read_result_t{.data = value, .mask = valid_bits};
}

template <size_t srcs>
static addr_read_result_t handle_pie_read(int_state_t<srcs> &state, uint8_t offset, uint8_t valid_bits)
{
    uint8_t value = 0;
    uint8_t index = offset;
    for (uint8_t i = 0; i < 8; i++)
    {
        if ((valid_bits & (1 << i)) == 0)
            continue;

        value |= (state.sources[index++].enabled << i);
    }
    return addr_read_result_t{.data = value, .mask = valid_bits};
}

template <size_t srcs>
static addr_read_result_t handle_ipr_read(int_state_t<srcs> &state, uint8_t offset, uint8_t valid_bits)
{
    uint8_t value = 0;
    uint8_t index = offset;
    for (uint8_t i = 0; i < 8; i++)
    {
        if ((valid_bits & (1 << i)) == 0)
            continue;

        value |= (state.sources[index++].high_priority << i);
    }
    return addr_read_result_t{.data = value, .mask = valid_bits};
}

constexpr uint8_t TX1IF_18F66K80 = 4;
constexpr uint8_t TX2IF_18F66K80 = 17;
constexpr uint8_t INT0_18F66K80 = 33;
constexpr uint8_t TMR0_18F66K80 = 34;
constexpr uint8_t RB_18F66K80 = 35;
constexpr uint8_t INT1_18F66K80 = 36;
constexpr uint8_t INT2_18F66K80 = 37;
constexpr uint8_t INT3_18F66K80 = 38;

addr_read_result_t int_addr_space_read_18f66k80(int_state_t<39> &state, const int_known_sfrs_18f66k80_t &sfr,
                                                uint16_t addr)
{
    if (addr == sfr.PIR1)
        return handle_pir_read(state, 0, 0xFF);
    else if (addr == sfr.PIE1)
        return handle_pie_read(state, 0, 0xFF);
    else if (addr == sfr.IPR1)
        return handle_ipr_read(state, 0, 0xFF);
    else if (addr == sfr.PIR2)
        return handle_pir_read(state, 8, 0x8F);
    else if (addr == sfr.PIE2)
        return handle_pie_read(state, 8, 0x8F);
    else if (addr == sfr.IPR2)
        return handle_ipr_read(state, 8, 0x8F);
    else if (addr == sfr.PIR3)
        return handle_pir_read(state, 13, 0x3E);
    else if (addr == sfr.PIE3)
        return handle_pie_read(state, 13, 0x3E);
    else if (addr == sfr.IPR3)
        return handle_ipr_read(state, 13, 0x3E);
    else if (addr == sfr.PIR4)
        return handle_pir_read(state, 18, 0xF7);
    else if (addr == sfr.PIE4)
        return handle_pie_read(state, 18, 0xF7);
    else if (addr == sfr.IPR4)
        return handle_ipr_read(state, 18, 0xF7);
    else if (addr == sfr.PIR5)
        return handle_pir_read(state, 25, 0xFF);
    else if (addr == sfr.PIE5)
        return handle_pie_read(state, 25, 0xFF);
    else if (addr == sfr.IPR5)
        return handle_ipr_read(state, 25, 0xFF);

    uint8_t result = 0;
    uint8_t mask = 0;

    if (addr == sfr.INTCON)
    {
        if (state.GIEH)
            result |= reg_intcon_mask_GIE_GIEH;
        if (state.GIEL)
            result |= reg_intcon_mask_PEIE_GIEL;
        if (state.sources[INT0_18F66K80].enabled)
            result |= reg_intcon_mask_INT0IE;
        if (state.sources[INT0_18F66K80].requested)
            result |= reg_intcon_mask_INT0IF;
        if (state.sources[TMR0_18F66K80].enabled)
            result |= reg_intcon_mask_TMR0IE;
        if (state.sources[TMR0_18F66K80].requested)
            result |= reg_intcon_mask_TMR0IF;
        if (state.sources[RB_18F66K80].enabled)
            result |= reg_intcon_mask_RBIE;
        if (state.sources[RB_18F66K80].requested)
            result |= reg_intcon_mask_RBIF;
        mask = 0xFF;
    }
    else if (addr == sfr.INTCON2)
    {
        if (state.sources[TMR0_18F66K80].high_priority)
            result |= reg_intcon2_mask_TMR0IP;
        if (state.sources[RB_18F66K80].high_priority)
            result |= reg_intcon2_mask_RBIP;
        if (state.sources[INT3_18F66K80].high_priority)
            result |= reg_intcon2_mask_INT3IP;
        mask = reg_intcon2_mask_TMR0IP | reg_intcon2_mask_RBIP | reg_intcon2_mask_INT3IP;
    }
    else if (addr == sfr.INTCON3)
    {
        if (state.sources[INT1_18F66K80].enabled)
            result |= reg_intcon3_mask_INT1IE;
        if (state.sources[INT1_18F66K80].requested)
            result |= reg_intcon3_mask_INT1IF;
        if (state.sources[INT1_18F66K80].high_priority)
            result |= reg_intcon3_mask_INT1IP;
        if (state.sources[INT2_18F66K80].requested)
            result |= reg_intcon3_mask_INT2IE;
        if (state.sources[INT2_18F66K80].enabled)
            result |= reg_intcon3_mask_INT2IF;
        if (state.sources[INT2_18F66K80].high_priority)
            result |= reg_intcon3_mask_INT2IP;
        if (state.sources[INT3_18F66K80].enabled)
            result |= reg_intcon3_mask_INT3IE;
        if (state.sources[INT3_18F66K80].requested)
            result |= reg_intcon3_mask_INT3IF;
        mask = 0xFF;
    }
    else if (addr == sfr.RCON)
    {
        if (state.IPEN)
            result |= reg_rcon_mask_IPEN;
        mask = reg_rcon_mask_IPEN;
    }

    return addr_read_result_t{
        .data = result,
        .mask = mask,
    };
}

addr_bit_mask_t int_addr_space_write_18f66k80(int_state_t<39> &state, const int_known_sfrs_18f66k80_t &sfr,
                                              uint16_t addr, uint8_t value)
{
    if (addr == sfr.PIR1)
        return handle_pir_write(state, value, 0, 0xFF);
    else if (addr == sfr.PIE1)
        return handle_pie_write(state, value, 0, 0xFF);
    else if (addr == sfr.IPR1)
        return handle_ipr_write(state, value, 0, 0xFF);
    else if (addr == sfr.PIR2)
        return handle_pir_write(state, value, 8, 0x8F);
    else if (addr == sfr.PIE2)
        return handle_pie_write(state, value, 8, 0x8F);
    else if (addr == sfr.IPR2)
        return handle_ipr_write(state, value, 8, 0x8F);
    else if (addr == sfr.PIR3)
        return handle_pir_write(state, value, 13, 0x3E);
    else if (addr == sfr.PIE3)
        return handle_pie_write(state, value, 13, 0x3E);
    else if (addr == sfr.IPR3)
        return handle_ipr_write(state, value, 13, 0x3E);
    else if (addr == sfr.PIR4)
        return handle_pir_write(state, value, 18, 0xF7);
    else if (addr == sfr.PIE4)
        return handle_pie_write(state, value, 18, 0xF7);
    else if (addr == sfr.IPR4)
        return handle_ipr_write(state, value, 18, 0xF7);
    else if (addr == sfr.PIR5)
        return handle_pir_write(state, value, 25, 0xFF);
    else if (addr == sfr.PIE5)
        return handle_pie_write(state, value, 25, 0xFF);
    else if (addr == sfr.IPR5)
        return handle_ipr_write(state, value, 25, 0xFF);

    if (addr == sfr.INTCON)
    {
        state.GIEH = value & reg_intcon_mask_GIE_GIEH;
        state.GIEL = value & reg_intcon_mask_PEIE_GIEL;
        state.sources[INT0_18F66K80].enabled = value & reg_intcon_mask_INT0IE;
        state.sources[INT0_18F66K80].requested = value & reg_intcon_mask_INT0IF;
        state.sources[TMR0_18F66K80].enabled = value & reg_intcon_mask_TMR0IE;
        state.sources[TMR0_18F66K80].requested = value & reg_intcon_mask_TMR0IF;
        state.sources[RB_18F66K80].enabled = value & reg_intcon_mask_RBIE;
        state.sources[RB_18F66K80].requested = value & reg_intcon_mask_RBIF;
        return 0xFF;
    }
    else if (addr == sfr.INTCON2)
    {
        state.sources[TMR0_18F66K80].high_priority = value & reg_intcon2_mask_TMR0IP;
        state.sources[RB_18F66K80].high_priority = value & reg_intcon2_mask_RBIP;
        state.sources[INT3_18F66K80].high_priority = value & reg_intcon2_mask_INT3IP;
        return reg_intcon2_mask_TMR0IP | reg_intcon2_mask_RBIP | reg_intcon2_mask_INT3IP;
    }
    else if (addr == sfr.INTCON3)
    {
        state.sources[INT1_18F66K80].enabled = value & reg_intcon3_mask_INT1IE;
        state.sources[INT1_18F66K80].requested = value & reg_intcon3_mask_INT1IF;
        state.sources[INT1_18F66K80].high_priority = value & reg_intcon3_mask_INT1IP;
        state.sources[INT2_18F66K80].requested = value & reg_intcon3_mask_INT2IE;
        state.sources[INT2_18F66K80].enabled = value & reg_intcon3_mask_INT2IF;
        state.sources[INT2_18F66K80].high_priority = value & reg_intcon3_mask_INT2IP;
        state.sources[INT3_18F66K80].enabled = value & reg_intcon3_mask_INT3IE;
        state.sources[INT3_18F66K80].requested = value & reg_intcon3_mask_INT3IF;
        return 0xFF;
    }
    else if (addr == sfr.RCON)
    {
        state.IPEN = value & reg_rcon_mask_IPEN;
        return reg_rcon_mask_IPEN;
    }

    return 0x00;
}

void int_set_flag(int_state_t<39> &state, const int_known_sfrs_18f66k80_t &sfr, uint16_t reg, uint8_t bit, bool b)
{
    uint8_t new_value = int_addr_space_read_18f66k80(state, sfr, reg).data;

    if (b)
    {
        new_value |= (1 << bit);
    }
    else
    {
        new_value &= ~(1 << bit);
    }

    if (reg == sfr.PIR1)
        handle_pir_write(state, new_value, 0, 0xFF, true);
    else if (reg == sfr.PIR2)
        handle_pir_write(state, new_value, 8, 0x8F, true);
    else if (reg == sfr.PIR3)
        handle_pir_write(state, new_value, 13, 0x3E, true);
    else if (reg == sfr.PIR4)
        handle_pir_write(state, new_value, 18, 0xF7, true);
    else if (reg == sfr.PIR5)
        handle_pir_write(state, new_value, 25, 0xFF, true);
    else
        int_addr_space_write_18f66k80(state, sfr, reg, new_value);
}

void int_state_initialize_18f66k80(int_state_t<39> &state)
{
    for (size_t i = 0; i < 39; i++)
    {
        state.sources[i].flag_read_only = false;

        if (i < 33)
        {
            state.sources[i].is_peripheral = true;
        }
    }

    state.sources[TX1IF_18F66K80].flag_read_only = true;
    state.sources[TX2IF_18F66K80].flag_read_only = true;
}

void int_state_reset_18f66k80(int_state_t<39> &state)
{
    for (size_t i = 0; i < 33; i++)
    {
        state.sources[i].high_priority = true;
        state.sources[i].enabled = false;
        state.sources[i].requested = false;
    }

    state.sources[INT0_18F66K80].requested = false;
    state.sources[TMR0_18F66K80].requested = false;
    // RBIF unknown because wakeup
    state.sources[INT1_18F66K80].requested = false;
    state.sources[INT2_18F66K80].requested = false;
    state.sources[INT3_18F66K80].requested = false;

    state.sources[INT0_18F66K80].enabled = false;
    state.sources[TMR0_18F66K80].enabled = false;
    state.sources[RB_18F66K80].enabled = false;
    state.sources[INT1_18F66K80].enabled = false;
    state.sources[INT2_18F66K80].enabled = false;
    state.sources[INT3_18F66K80].enabled = false;

    state.sources[INT0_18F66K80].high_priority = true;
    state.sources[TMR0_18F66K80].high_priority = true;
    state.sources[RB_18F66K80].high_priority = true;
    state.sources[INT1_18F66K80].high_priority = true;
    state.sources[INT2_18F66K80].high_priority = true;
    state.sources[INT3_18F66K80].high_priority = true;

    state.IPEN = false;
    state.GIEH = false;
    state.GIEL = false;
}
