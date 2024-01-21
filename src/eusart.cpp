#include "eusart.hpp"

static void load_tsr(eusart_ctx_t &ctx)
{
    ctx.txreg_loaded = false;
    ctx.tsr_loaded = true;
    if (ctx.transmit != nullptr)
    {
        ctx.transmit(ctx.tx_reg, ctx.tx_bit9 && ctx.tx_9bit_en);
    }

    if (ctx.tx_interrupt != nullptr)
    {
        ctx.tx_interrupt(true);
    }
}

addr_read_result_t eusart_bus_read(eusart_ctx_t &ctx, uint16_t address)
{
    auto read_bits = [](uint8_t mask, bool b7, bool b6, bool b5, bool b4, bool b3, bool b2, bool b1,
                        bool b0) -> addr_read_result_t {
        uint8_t val = (b7 << 7) | (b6 << 6) | (b5 << 5) | (b4 << 4) | (b3 << 3) | (b2 << 2) | (b1 << 1) | (b0 << 0);
        return addr_read_result_t{.data = val, .mask = mask};
    };

    if (address == ctx.sfr.TXSTAx)
    {
        return read_bits(0xFF, ctx.sync_clk_master_mode, ctx.tx_9bit_en, ctx.tx_en, ctx.synchronous,
                         ctx.async_send_break, ctx.async_high_speed, !ctx.tsr_loaded, ctx.tx_bit9);
    }
    else if (address == ctx.sfr.RCSTAx)
    {
        return read_bits(0xFF, ctx.sp_en, ctx.rx_9bit_en, ctx.single_receive, ctx.continuous_receive,
                         ctx.address_detect, ctx.framing_err, ctx.overrun_err, ctx.rx_bit9);
    }
    else if (address == ctx.sfr.TXREGx)
    {
        return addr_read_result_t{.data = ctx.tx_reg, .mask = 0xFF};
    }
    else if (address == ctx.sfr.RCREGx)
    {
        return addr_read_result_t{.data = ctx.rx_reg, .mask = 0xFF};
    }
    else if (address == ctx.sfr.BAUDCONx)
    {
        return read_bits(0xFB, 0, !ctx.rx_active, ctx.async_invert_rx, ctx.clock_data_polarity, ctx.baud_rate_16bit_en,
                         0, ctx.wakeup_en, 0);
    }
    else if (address == ctx.sfr.SPBRGx)
    {
        return addr_read_result_t{.data = static_cast<uint8_t>(ctx.baud_rate & 0xFF), .mask = 0xFF};
    }
    else if (address == ctx.sfr.SPBRGHx)
    {
        return addr_read_result_t{.data = static_cast<uint8_t>((ctx.baud_rate >> 8) & 0xFF), .mask = 0xFF};
    }

    return addr_read_result_none;
}

addr_bit_mask_t eusart_bus_write(eusart_ctx_t &ctx, uint16_t address, uint8_t value)
{
    if (address == ctx.sfr.TXSTAx)
    {
        ctx.sync_clk_master_mode = value & (1 << 7);
        ctx.tx_9bit_en = value & (1 << 6);
        ctx.tx_en = value & (1 << 5);
        ctx.synchronous = value & (1 << 4);
        ctx.async_send_break = value & (1 << 3); // TODO: Implement send break.
        ctx.async_high_speed = value & (1 << 2);
        // Bit 1 is read only.
        ctx.tx_bit9 = value & (1 << 0);

        if (ctx.mode_change != nullptr)
        {
            ctx.mode_change();
        }

        if (ctx.tx_en && ctx.tx_interrupt != nullptr)
        {
            ctx.tx_interrupt(true);
        }

        return 0xFF;
    }
    else if (address == ctx.sfr.RCSTAx)
    {
        ctx.sp_en = value & (1 << 7);
        ctx.rx_9bit_en = value & (1 << 6);
        ctx.single_receive = value & (1 << 5);
        ctx.continuous_receive = value & (1 << 4);
        ctx.address_detect = value & (1 << 3);
        ctx.rx_bit9 = value & (1 << 0);

        if (!ctx.continuous_receive)
        {
            ctx.overrun_err = false;
        }

        if (ctx.mode_change != nullptr)
        {
            ctx.mode_change();
        }

        return 0xFF;
    }
    else if (address == ctx.sfr.TXREGx)
    {
        ctx.tx_reg = value;
        ctx.txreg_loaded = true;
        if (ctx.tx_interrupt != nullptr)
        {
            ctx.tx_interrupt(false);
        }
        if (!ctx.tsr_loaded)
        {
            load_tsr(ctx);
        }

        return 0xFF;
    }
    else if (address == ctx.sfr.RCREGx)
    {
        return 0xFF;
    }
    else if (address == ctx.sfr.BAUDCONx)
    {
        ctx.async_invert_rx = value & (1 << 5);
        ctx.clock_data_polarity = value & (1 << 4);
        ctx.baud_rate_16bit_en = value & (1 << 3);
        ctx.wakeup_en = value & (1 << 1);
        if (ctx.mode_change != nullptr)
        {
            ctx.mode_change();
        }
        return 0xFB;
    }
    else if (address == ctx.sfr.SPBRGx)
    {
        ctx.baud_rate = (ctx.baud_rate & 0xFF00) | value;
        if (ctx.mode_change != nullptr)
        {
            ctx.mode_change();
        }
        return 0xFF;
    }
    else if (address == ctx.sfr.SPBRGHx)
    {
        ctx.baud_rate = (ctx.baud_rate & 0x00FF) | (value << 8);
        if (ctx.mode_change != nullptr)
        {
            ctx.mode_change();
        }
        return 0xFF;
    }

    return 0x00;
}

void eusart_initialize(eusart_ctx_t &ctx)
{
    ctx = {};
}

void eusart_import_rx(eusart_ctx_t &ctx, uint8_t data, bool bit9)
{
    if (!ctx.sp_en)
        return;

    if (!ctx.continuous_receive && !ctx.single_receive)
        return;

    ctx.single_receive = false;
    ctx.rx_reg = data;

    if (ctx.rx_9bit_en)
    {
        ctx.rx_bit9 = bit9;
    }

    if (ctx.rx_interrupt != nullptr)
    {
        ctx.rx_interrupt(true);
    }
}

void eusart_import_tx_done(eusart_ctx_t &ctx)
{
    if (ctx.txreg_loaded)
    {
        load_tsr(ctx);
    }
    else
    {
        ctx.tsr_loaded = false;
    }
}
