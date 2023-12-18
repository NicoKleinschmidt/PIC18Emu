#include "reg.hpp"

// clang-format off
reg_configuration_bit_info_t reg_configuration_bits[] = {
    { .file = CONFIG1L, .bit = 6, .invert = false,  },
    { .file = CONFIG1L, .bit = 4, .invert = false, },
    { .file = CONFIG1L, .bit = 3, .invert = false, },
    { .file = CONFIG1L, .bit = 2, .invert = false, },
    { .file = CONFIG2L, .bit = 0, .invert = true, },

    { .file = CONFIG1H, .bit = 5, .invert = true,  },
    { .file = CONFIG1H, .bit = 2, .invert = false, },
    { .file = CONFIG1H, .bit = 1, .invert = false, },
    { .file = CONFIG1H, .bit = 0, .invert = false, },
    { .file = CONFIG2L, .bit = 3, .invert = false, },
    { .file = CONFIG2L, .bit = 2, .invert = false, },
    { .file = CONFIG2L, .bit = 1, .invert = false, },
    { .file = CONFIG2L, .bit = 0, .invert = true,  },
    { .file = CONFIG2H, .bit = 3, .invert = false, },
    { .file = CONFIG2H, .bit = 2, .invert = false, },
    { .file = CONFIG2H, .bit = 1, .invert = false, },
    { .file = CONFIG2H, .bit = 0, .invert = false, },
    { .file = CONFIG3H, .bit = 0, .invert = false, },
    { .file = CONFIG4L, .bit = 7, .invert = false, },
    { .file = CONFIG4L, .bit = 2, .invert = false, },
    { .file = CONFIG4L, .bit = 1, .invert = false, },
    { .file = CONFIG5L, .bit = 3, .invert = false, },
    { .file = CONFIG5L, .bit = 2, .invert = false, },
    { .file = CONFIG5L, .bit = 1, .invert = false, },
    { .file = CONFIG5L, .bit = 0, .invert = false, },
    { .file = CONFIG5H, .bit = 7, .invert = false, },
    { .file = CONFIG5H, .bit = 6, .invert = false, },
    { .file = CONFIG6L, .bit = 3, .invert = false, },
    { .file = CONFIG6L, .bit = 2, .invert = false, },
    { .file = CONFIG6L, .bit = 1, .invert = false, },
    { .file = CONFIG6L, .bit = 0, .invert = false, },
    { .file = CONFIG6H, .bit = 7, .invert = false, },
    { .file = CONFIG6H, .bit = 6, .invert = false, },
    { .file = CONFIG6H, .bit = 5, .invert = false, },
    { .file = CONFIG7L, .bit = 3, .invert = false, },
    { .file = CONFIG7L, .bit = 2, .invert = false, },
    { .file = CONFIG7L, .bit = 1, .invert = false, },
    { .file = CONFIG7L, .bit = 0, .invert = false, },
    { .file = CONFIG7H, .bit = 6, .invert = false, },
};
// clang-format on

bool reg_check_configuration_bit(reg_configuration_bit_t bit, bus_reader_t<uint32_t, uint8_t> read_bus)
{
    reg_configuration_bit_info_t bit_info = reg_configuration_bits[static_cast<uint8_t>(bit)];

    uint8_t register_val = read_bus(bit_info.file);
    bool is_set = register_val & (1 << bit_info.bit);

    if (bit_info.invert)
        return !is_set;
    else
        return is_set;
}

uint8_t reg_sfr_read(bus_reader_t<uint16_t, uint8_t> read_bus, uint16_t reg, uint8_t reg_mask)
{
    // This check is important because of the while loop later on.
    if (reg_mask == 0)
        return 0;

    uint8_t value = read_bus(static_cast<uint16_t>(reg)) & reg_mask;
    while ((reg_mask & 0x1) == 0)
    {
        value >>= 1;
        reg_mask >>= 1;
    }

    return value;
}

void reg_sfr_write(bus_reader_t<uint16_t, uint8_t> read_bus, bus_writer_t<uint16_t, uint8_t> write_bus, uint16_t reg,
                   uint8_t reg_mask, uint8_t value)
{
    if (reg_mask == 0)
        return;

    uint8_t current_value = read_bus(static_cast<uint16_t>(reg)) & (~reg_mask);

    while ((reg_mask & 0x1) == 0)
    {
        value <<= 1;
        reg_mask >>= 1;
    }
    value |= (current_value);

    write_bus(static_cast<uint16_t>(reg), value);
}

enum class fsr_action
{
    none,
    postinc,
    preinc,
    postdec,
    plusw,
};

static uint16_t get_fsr_address(const indf_known_sfrs_t &sfr, uint8_t fsr_num, fsr_action action,
                                sfr_phy_reader_t read_reg, sfr_phy_writer_t write_reg)
{
    uint16_t fsrh_reg;
    uint16_t fsrl_reg;

    if (fsr_num == 0)
    {
        fsrh_reg = sfr.FSR0H;
        fsrl_reg = sfr.FSR0L;
    }
    else if (fsr_num == 1)
    {
        fsrh_reg = sfr.FSR1H;
        fsrl_reg = sfr.FSR1L;
    }
    else if (fsr_num == 2)
    {
        fsrh_reg = sfr.FSR2H;
        fsrl_reg = sfr.FSR2L;
    }
    else
    {
        return 0;
    }

    uint8_t fsrh = read_reg(fsrh_reg).data & 0x0F;
    uint8_t fsrl = read_reg(fsrl_reg).data & 0xFF;
    uint16_t fsr = ((static_cast<uint16_t>(fsrh) << 8) | static_cast<uint16_t>(fsrl));

    if (action == fsr_action::none)
        return fsr;

    if (action == fsr_action::plusw)
    {
        uint8_t wreg = read_reg(sfr.WREG).data;
        fsr += wreg;
        fsr &= 0x0FFF; // Make sure it doesn't over/underflow
        return fsr;
    }

    uint16_t retval;

    if (action == fsr_action::preinc)
    {
        fsr++;
        fsr &= 0x0FFF; // Make sure it doesn't over/underflow
        retval = fsr;
    }

    if (action == fsr_action::postinc)
    {
        retval = fsr;
        fsr++;
        fsr &= 0x0FFF; // Make sure it doesn't over/underflow
    }

    if (action == fsr_action::postdec)
    {
        retval = fsr;
        fsr--;
        fsr &= 0x0FFF; // Make sure it doesn't over/underflow
    }

    fsrh = (fsr >> 8) & 0xFF;
    fsrl = fsr & 0xFF;

    write_reg(fsrh_reg, fsrh);
    write_reg(fsrl_reg, fsrl);

    return retval;
}

/// @brief Checks if a register is an INDF (INDF0/POSTINC0/PREINC1...) register.
/// @param address The register to check
/// @return True if the register is an INDF register.
static bool is_indf_register(const indf_known_sfrs_t &sfr, uint16_t reg)
{
    return reg == sfr.INDF0 || reg == sfr.INDF1 || reg == sfr.INDF2 || reg == sfr.PREINC0 || reg == sfr.PREINC1 ||
           reg == sfr.PREINC2 || reg == sfr.POSTINC0 || reg == sfr.POSTINC1 || reg == sfr.POSTINC2 ||
           reg == sfr.POSTDEC0 || reg == sfr.POSTDEC1 || reg == sfr.POSTDEC2 || reg == sfr.PLUSW0 ||
           reg == sfr.PLUSW1 || reg == sfr.PLUSW2;
}

addr_read_result_t reg_sfr_bus_read(const sfr_ctx_t &ctx, uint16_t address)
{
    if (address < ctx.first_address || address > ctx.last_address)
        return addr_read_result_none;

    auto indirect_read = [&ctx](uint8_t fsr_num, fsr_action action) -> uint8_t {
        uint16_t address = get_fsr_address(ctx.sfr, fsr_num, action, ctx.read_sfr_phy, ctx.write_sfr_phy);
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
    else return ctx.read_sfr_phy(address);
    // clang-format on

    return addr_read_result_t{.data = result, .mask = 0xFF};
}

addr_bit_mask_t reg_sfr_bus_write(const sfr_ctx_t &ctx, uint16_t address, uint8_t value)
{
    if (address < ctx.first_address || address > ctx.last_address)
        return false;

    auto indirect_write = [&ctx](uint8_t fsr_num, fsr_action action, uint8_t val) {
        uint16_t address = get_fsr_address(ctx.sfr, fsr_num, action, ctx.read_sfr_phy, ctx.write_sfr_phy);
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
    else return ctx.write_sfr_phy(address, value);
    // clang-format on

    return 0xFF;
}
