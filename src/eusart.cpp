#include "eusart.hpp"

addr_read_result_t eusart_bus_read(eusart_ctx_t &ctx, uint16_t address)
{
    return addr_read_result_none;
}

addr_bit_mask_t eusart_bus_write(eusart_ctx_t &ctx, uint16_t address, uint8_t value)
{
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
