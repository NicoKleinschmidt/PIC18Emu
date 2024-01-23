#include "bank.hpp"

enum class fsr_action
{
    none,
    postinc,
    preinc,
    postdec,
    plusw,
};

static uint16_t get_fsr_address(bank_ctx_t &ctx, uint8_t fsr_num, fsr_action action)
{
    uint16_t fsr_value;

    if (fsr_num == 0)
        fsr_value = ctx.fsr0;
    else if (fsr_num == 1)
        fsr_value = ctx.fsr1;
    else if (fsr_num == 2)
        fsr_value = ctx.fsr2;
    else
        return 0;

    if (action == fsr_action::none)
        return fsr_value;

    if (action == fsr_action::plusw)
    {
        uint8_t wreg = ctx.read_wreg();
        fsr_value += wreg;
        fsr_value &= 0x0FFF; // Register should only be 12 bits wide, mask to handle over/underflow correctly
        return fsr_value;
    }

    uint16_t retval;

    if (action == fsr_action::preinc)
    {
        fsr_value++;
        fsr_value &= 0x0FFF;
        retval = fsr_value;
    }

    if (action == fsr_action::postinc)
    {
        retval = fsr_value;
        fsr_value++;
        fsr_value &= 0x0FFF;
    }

    if (action == fsr_action::postdec)
    {
        retval = fsr_value;
        fsr_value--;
        fsr_value &= 0x0FFF;
    }

    if (fsr_num == 0)
        fsr_value = ctx.fsr0;
    else if (fsr_num == 1)
        fsr_value = ctx.fsr1;
    else if (fsr_num == 2)
        fsr_value = ctx.fsr2;

    return retval;
}

/// @brief Checks if a register is an INDF (INDF0/POSTINC0/PREINC1...) register.
/// @param address The register to check
/// @return True if the register is an INDF register.
static bool is_indf_register(const bank_known_sfrs_t &sfr, uint16_t reg)
{
    return reg == sfr.INDF0 || reg == sfr.INDF1 || reg == sfr.INDF2 || reg == sfr.PREINC0 || reg == sfr.PREINC1 ||
           reg == sfr.PREINC2 || reg == sfr.POSTINC0 || reg == sfr.POSTINC1 || reg == sfr.POSTINC2 ||
           reg == sfr.POSTDEC0 || reg == sfr.POSTDEC1 || reg == sfr.POSTDEC2 || reg == sfr.PLUSW0 ||
           reg == sfr.PLUSW1 || reg == sfr.PLUSW2;
}

void bank_initialize(bank_ctx_t &ctx)
{
    ctx.bsr = 0;
    ctx.fsr0 = 0;
    ctx.fsr1 = 0;
    ctx.fsr2 = 0;
}

addr_read_result_t bank_bus_read(bank_ctx_t &ctx, uint16_t address)
{
    auto indirect_read = [&ctx](uint8_t fsr_num, fsr_action action) -> uint8_t {
        uint16_t address = get_fsr_address(ctx, fsr_num, action);
        if (is_indf_register(ctx.sfr, address))
            return 0;

        return ctx.read_bus(address);
    };

    uint8_t result;

    // clang-format off
    if (address == ctx.sfr.INDF0) result = indirect_read(0, fsr_action::none);
    else if (address == ctx.sfr.INDF1) result = indirect_read(1, fsr_action::none);
    else if (address == ctx.sfr.INDF2) result = indirect_read(2, fsr_action::none);
    else if (address == ctx.sfr.PREINC0) result = indirect_read(0, fsr_action::preinc);
    else if (address == ctx.sfr.PREINC1) result = indirect_read(1, fsr_action::preinc);
    else if (address == ctx.sfr.PREINC2) result = indirect_read(2, fsr_action::preinc);
    else if (address == ctx.sfr.POSTINC0) result = indirect_read(0, fsr_action::postinc);
    else if (address == ctx.sfr.POSTINC1) result = indirect_read(1, fsr_action::postinc);
    else if (address == ctx.sfr.POSTINC2) result = indirect_read(2, fsr_action::postinc);
    else if (address == ctx.sfr.POSTDEC0) result = indirect_read(0, fsr_action::postdec);
    else if (address == ctx.sfr.POSTDEC1) result = indirect_read(1, fsr_action::postdec);
    else if (address == ctx.sfr.POSTDEC2) result = indirect_read(2, fsr_action::postdec);
    else if (address == ctx.sfr.PLUSW0) result = indirect_read(0, fsr_action::plusw);
    else if (address == ctx.sfr.PLUSW1) result = indirect_read(1, fsr_action::plusw);
    else if (address == ctx.sfr.PLUSW2) result = indirect_read(2, fsr_action::plusw);
    else if (address == ctx.sfr.BSR) result = ctx.bsr;
    else if (address == ctx.sfr.FSR0H) result = static_cast<uint8_t>((ctx.fsr0 >> 8) & 0x0F);
    else if (address == ctx.sfr.FSR0L) result = static_cast<uint8_t>(ctx.fsr0 & 0xFF);
    else if (address == ctx.sfr.FSR1H) result = static_cast<uint8_t>((ctx.fsr1 >> 8) & 0x0F);
    else if (address == ctx.sfr.FSR1L) result = static_cast<uint8_t>(ctx.fsr1 & 0xFF);
    else if (address == ctx.sfr.FSR2H) result = static_cast<uint8_t>((ctx.fsr2 >> 8) & 0x0F);
    else if (address == ctx.sfr.FSR2L) result = static_cast<uint8_t>(ctx.fsr2 & 0xFF);
    else return addr_read_result_none;
    // clang-format on

    return addr_read_result_t{.data = result, .mask = 0xFF};
}

addr_bit_mask_t bank_bus_write(bank_ctx_t &ctx, uint16_t address, uint8_t value)
{
    auto indirect_write = [&ctx](uint8_t fsr_num, fsr_action action, uint8_t val) {
        uint16_t address = get_fsr_address(ctx, fsr_num, action);
        if (is_indf_register(ctx.sfr, address))
            return;

        ctx.write_bus(address, val);
    };

    // clang-format off
    if (address == ctx.sfr.INDF0) indirect_write(0, fsr_action::none, value);
    else if (address == ctx.sfr.INDF1) indirect_write(1, fsr_action::none, value);
    else if (address == ctx.sfr.INDF2) indirect_write(2, fsr_action::none, value);
    else if (address == ctx.sfr.PREINC0) indirect_write(0, fsr_action::preinc, value);
    else if (address == ctx.sfr.PREINC1) indirect_write(1, fsr_action::preinc, value);
    else if (address == ctx.sfr.PREINC2) indirect_write(2, fsr_action::preinc, value);
    else if (address == ctx.sfr.POSTINC0) indirect_write(0, fsr_action::postinc, value);
    else if (address == ctx.sfr.POSTINC1) indirect_write(1, fsr_action::postinc, value);
    else if (address == ctx.sfr.POSTINC2) indirect_write(2, fsr_action::postinc, value);
    else if (address == ctx.sfr.POSTDEC0) indirect_write(0, fsr_action::postdec, value);
    else if (address == ctx.sfr.POSTDEC1) indirect_write(1, fsr_action::postdec, value);
    else if (address == ctx.sfr.POSTDEC2) indirect_write(2, fsr_action::postdec, value);
    else if (address == ctx.sfr.PLUSW0) indirect_write(0, fsr_action::plusw, value);
    else if (address == ctx.sfr.PLUSW1) indirect_write(1, fsr_action::plusw, value);
    else if (address == ctx.sfr.PLUSW2) indirect_write(2, fsr_action::plusw, value);
    else if (address == ctx.sfr.BSR) ctx.bsr = value;
    else if (address == ctx.sfr.FSR0H) ctx.fsr0 = (ctx.fsr0 & 0x00FF) | ((value & 0x0F) << 8);
    else if (address == ctx.sfr.FSR0L) ctx.fsr0 = (ctx.fsr0 & 0xFF00) | value;
    else if (address == ctx.sfr.FSR1H) ctx.fsr1 = (ctx.fsr1 & 0x00FF) | ((value & 0x0F) << 8);
    else if (address == ctx.sfr.FSR1L) ctx.fsr1 = (ctx.fsr1 & 0xFF00) | value;
    else if (address == ctx.sfr.FSR2H) ctx.fsr2 = (ctx.fsr2 & 0x00FF) | ((value & 0x0F) << 8);
    else if (address == ctx.sfr.FSR2L) ctx.fsr2 = (ctx.fsr2 & 0xFF00) | value;
    // clang-format on

    return 0xFF;
}