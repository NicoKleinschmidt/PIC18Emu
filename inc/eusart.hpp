#pragma once

#include "bus.hpp"
#include <cstdint>
#include <functional>

struct eusart_ctx_t
{
    bool synchronous;
    bool tx_en;
    bool tx_9bit_en;
    bool sync_clk_master_mode;
    bool async_send_break;
    bool async_high_speed;

    bool sp_en;
    bool rx_9bit_en;
    bool single_receive;
    bool continuous_receive;
    bool address_detect;

    bool framing_err;
    bool overrun_err;

    bool rx_idle;
    bool async_invert_rx;
    bool wakeup_en;
    bool clock_data_polarity;
    bool baud_rate_16bit_en;
    bool async_baud_detect_en;

    uint16_t baud_rate;

    uint8_t tx_reg;
    uint8_t rx_reg;
    bool tx_bit9;
    bool rx_bit9;

    std::function<void()> mode_change;
    std::function<void()> receive_ready;
    std::function<void(uint8_t data, bool bit9)> transmit;
    std::function<void(bool set)> rx_interrupt;
    std::function<void(bool set)> tx_interrupt;
};

addr_read_result_t eusart_bus_read(eusart_ctx_t &ctx, uint16_t address);
addr_bit_mask_t eusart_bus_write(eusart_ctx_t &ctx, uint16_t address, uint8_t value);
void eusart_initialize(eusart_ctx_t &ctx);
void eusart_import_rx(eusart_ctx_t &ctx, uint8_t data, bool bit9);
