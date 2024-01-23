#include "tbl.hpp"

void tbl_write(tbl_ctx_t &ctx, bus_writer_t<uint32_t, uint8_t> write_bus, tblptr_action_t action)
{
    if (action == tblptr_action_t::preinc)
        ctx.pointer = (ctx.pointer + 1) & 0x3FFFFF;

    write_bus(ctx.pointer, ctx.latch);

    if (action == tblptr_action_t::postinc)
        ctx.pointer = (ctx.pointer + 1) & 0x3FFFFF;
    else if (action == tblptr_action_t::postdec)
        ctx.pointer = (ctx.pointer - 1) & 0x3FFFFF;
}

void tbl_read(tbl_ctx_t &ctx, bus_reader_t<uint32_t, uint8_t> read_bus, tblptr_action_t action)
{
    if (action == tblptr_action_t::preinc)
        ctx.pointer = (ctx.pointer + 1) & 0x3FFFFF;

    ctx.latch = read_bus(ctx.pointer);

    if (action == tblptr_action_t::postinc)
        ctx.pointer = (ctx.pointer + 1) & 0x3FFFFF;
    else if (action == tblptr_action_t::postdec)
        ctx.pointer = (ctx.pointer - 1) & 0x3FFFFF;
}

void tbl_initialize(tbl_ctx_t &ctx)
{
    ctx.latch = 0;
    ctx.pointer = 0;
}

addr_read_result_t tbl_bus_read(const tbl_ctx_t &ctx, uint16_t address)
{
    if (address == ctx.TABLAT)
        return addr_read_result_t{.data = ctx.latch, .mask = 0xFF};
    else if (address == ctx.TBLPTRL)
        return addr_read_result_t{.data = static_cast<uint8_t>(ctx.pointer), .mask = 0xFF};
    else if (address == ctx.TBLPTRH)
        return addr_read_result_t{.data = static_cast<uint8_t>(ctx.pointer >> 8), .mask = 0xFF};
    else if (address == ctx.TBLPTRU)
        return addr_read_result_t{.data = static_cast<uint8_t>((ctx.pointer >> 16) & 0x3F), .mask = 0x3F};

    return addr_read_result_none;
}

addr_bit_mask_t tbl_bus_write(tbl_ctx_t &ctx, uint16_t address, uint8_t value)
{
    if (address == ctx.TABLAT)
    {
        ctx.latch = value;
        return 0xFF;
    }
    else if (address == ctx.TBLPTRL)
    {
        ctx.pointer = (ctx.pointer & 0x3FFF00) | value;
        return 0xFF;
    }
    else if (address == ctx.TBLPTRH)
    {
        ctx.pointer = (ctx.pointer & 0x3F00FF) | (value << 8);
        return 0xFF;
    }
    else if (address == ctx.TBLPTRU)
    {
        ctx.pointer = (ctx.pointer & 0x00FFFF) | ((value & 0x3F) << 16);
        return 0x3F;
    }

    return 0x00;
}