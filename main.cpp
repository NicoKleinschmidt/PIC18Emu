#include <bitset>
#include <cassert>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>

#include "18f66k80.hpp"
#include "adc.hpp"
#include "ccp.hpp"
#include "cpu.hpp"
#include "environment.hpp"
#include "eusart.hpp"
#include "gpio.hpp"
#include "int.hpp"
#include "mem.hpp"
#include "port.hpp"
#include "reg.hpp"
#include "timer.hpp"

inline void sleep_us(uint32_t us)
{
    const auto delay = std::chrono::microseconds(us);
    const auto end = std::chrono::steady_clock::now() + delay;

    for (;;)
    {
        if (std::chrono::steady_clock::now() > end)
            break;
    }
}

static std::function<void(int)> signal_handler;
static void sig_handler(int s)
{
    if (signal_handler != nullptr)
        signal_handler(s);
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: pic18 program.bin";
        return 1;
    }
    std::string program_file_name = argv[1];

    cpu_t cpu;

    memory_t<0x600> sram = {
        .start_address = 0x00,
        .data = std::make_unique<std::array<uint8_t, 0x600>>(),
    };

    constexpr size_t sfr_count = 0xFFF - 0xE40;
    sfr_phy_map_t<sfr_count> registers;
    registers.first_address = 0xE41;
    std::vector<uint16_t> physical_regs{
        static_cast<uint16_t>(pic18f66k80_sfr_map::BSR),   static_cast<uint16_t>(pic18f66k80_sfr_map::FSR0L),
        static_cast<uint16_t>(pic18f66k80_sfr_map::FSR0H), static_cast<uint16_t>(pic18f66k80_sfr_map::FSR1L),
        static_cast<uint16_t>(pic18f66k80_sfr_map::FSR1H), static_cast<uint16_t>(pic18f66k80_sfr_map::FSR2L),
        static_cast<uint16_t>(pic18f66k80_sfr_map::FSR2H),
    };
    registers.implemented_registers = sfr_phy_implemented_regs<sfr_count>(0xE41, physical_regs);
    sfr_phy_reset(registers, reg_reset_type_t::power_on_reset);

    auto program_mem = std::make_unique<std::array<uint8_t, 32768>>();

    bus_reader_t<uint16_t, uint8_t> read_data_bus;
    bus_writer_t<uint16_t, uint8_t> write_data_bus;

    cpu_known_sfrs_t pic18fxx2_cpu_regs = {
        .WREG = static_cast<uint16_t>(pic18f66k80_sfr_map::WREG),
        .STATUS = static_cast<uint16_t>(pic18f66k80_sfr_map::STATUS),
        .BSR = static_cast<uint16_t>(pic18f66k80_sfr_map::BSR),
        .PCL = static_cast<uint16_t>(pic18f66k80_sfr_map::PCL),
        .PCLATH = static_cast<uint16_t>(pic18f66k80_sfr_map::PCLATH),
        .PCLATU = static_cast<uint16_t>(pic18f66k80_sfr_map::PCLATU),
        .STKPTR = static_cast<uint16_t>(pic18f66k80_sfr_map::STKPTR),
        .TOSL = static_cast<uint16_t>(pic18f66k80_sfr_map::TOSL),
        .TOSH = static_cast<uint16_t>(pic18f66k80_sfr_map::TOSH),
        .TOSU = static_cast<uint16_t>(pic18f66k80_sfr_map::TOSU),
        .TBLPTRL = static_cast<uint16_t>(pic18f66k80_sfr_map::TBLPTRL),
        .TBLPTRH = static_cast<uint16_t>(pic18f66k80_sfr_map::TBLPTRH),
        .TBLPTRU = static_cast<uint16_t>(pic18f66k80_sfr_map::TBLPTRU),
        .TABLAT = static_cast<uint16_t>(pic18f66k80_sfr_map::TABLAT),
        .RCON = static_cast<uint16_t>(pic18f66k80_sfr_map::RCON),
        .INTCON = static_cast<uint16_t>(pic18f66k80_sfr_map::INTCON),
        .PRODL = static_cast<uint16_t>(pic18f66k80_sfr_map::PRODL),
        .PRODH = static_cast<uint16_t>(pic18f66k80_sfr_map::PRODH),
        .FSR0L = static_cast<uint16_t>(pic18f66k80_sfr_map::FSR0L),
        .FSR1L = static_cast<uint16_t>(pic18f66k80_sfr_map::FSR1L),
        .FSR2L = static_cast<uint16_t>(pic18f66k80_sfr_map::FSR2L),
    };

    indf_known_sfrs_t pic18fxx2_indf_regs = {
        .WREG = static_cast<uint16_t>(pic18f66k80_sfr_map::WREG),
        .FSR0H = static_cast<uint16_t>(pic18f66k80_sfr_map::FSR0H),
        .FSR1H = static_cast<uint16_t>(pic18f66k80_sfr_map::FSR1H),
        .FSR2H = static_cast<uint16_t>(pic18f66k80_sfr_map::FSR2H),
        .FSR0L = static_cast<uint16_t>(pic18f66k80_sfr_map::FSR0L),
        .FSR1L = static_cast<uint16_t>(pic18f66k80_sfr_map::FSR1L),
        .FSR2L = static_cast<uint16_t>(pic18f66k80_sfr_map::FSR2L),
        .INDF0 = static_cast<uint16_t>(pic18f66k80_sfr_map::INDF0),
        .INDF1 = static_cast<uint16_t>(pic18f66k80_sfr_map::INDF1),
        .INDF2 = static_cast<uint16_t>(pic18f66k80_sfr_map::INDF2),
        .PREINC0 = static_cast<uint16_t>(pic18f66k80_sfr_map::PREINC0),
        .PREINC1 = static_cast<uint16_t>(pic18f66k80_sfr_map::PREINC1),
        .PREINC2 = static_cast<uint16_t>(pic18f66k80_sfr_map::PREINC2),
        .POSTINC0 = static_cast<uint16_t>(pic18f66k80_sfr_map::POSTINC0),
        .POSTINC1 = static_cast<uint16_t>(pic18f66k80_sfr_map::POSTINC1),
        .POSTINC2 = static_cast<uint16_t>(pic18f66k80_sfr_map::POSTINC2),
        .POSTDEC0 = static_cast<uint16_t>(pic18f66k80_sfr_map::POSTDEC0),
        .POSTDEC1 = static_cast<uint16_t>(pic18f66k80_sfr_map::POSTDEC1),
        .POSTDEC2 = static_cast<uint16_t>(pic18f66k80_sfr_map::POSTDEC2),
        .PLUSW0 = static_cast<uint16_t>(pic18f66k80_sfr_map::PLUSW0),
        .PLUSW1 = static_cast<uint16_t>(pic18f66k80_sfr_map::PLUSW1),
        .PLUSW2 = static_cast<uint16_t>(pic18f66k80_sfr_map::PLUSW2),
    };

    int_known_sfrs_18f66k80_t pic18f66k80_int_regs = {
        .INTCON = static_cast<uint16_t>(pic18f66k80_sfr_map::INTCON),
        .INTCON2 = static_cast<uint16_t>(pic18f66k80_sfr_map::INTCON2),
        .INTCON3 = static_cast<uint16_t>(pic18f66k80_sfr_map::INTCON3),
        .PIR1 = static_cast<uint16_t>(pic18f66k80_sfr_map::PIR1),
        .PIE1 = static_cast<uint16_t>(pic18f66k80_sfr_map::PIE1),
        .IPR1 = static_cast<uint16_t>(pic18f66k80_sfr_map::IPR1),
        .PIR2 = static_cast<uint16_t>(pic18f66k80_sfr_map::PIR2),
        .PIE2 = static_cast<uint16_t>(pic18f66k80_sfr_map::PIE2),
        .IPR2 = static_cast<uint16_t>(pic18f66k80_sfr_map::IPR2),
        .PIR3 = static_cast<uint16_t>(pic18f66k80_sfr_map::PIR3),
        .PIE3 = static_cast<uint16_t>(pic18f66k80_sfr_map::PIE3),
        .IPR3 = static_cast<uint16_t>(pic18f66k80_sfr_map::IPR3),
        .PIR4 = static_cast<uint16_t>(pic18f66k80_sfr_map::PIR4),
        .PIE4 = static_cast<uint16_t>(pic18f66k80_sfr_map::PIE4),
        .IPR4 = static_cast<uint16_t>(pic18f66k80_sfr_map::IPR4),
        .PIR5 = static_cast<uint16_t>(pic18f66k80_sfr_map::PIR5),
        .PIE5 = static_cast<uint16_t>(pic18f66k80_sfr_map::PIE5),
        .IPR5 = static_cast<uint16_t>(pic18f66k80_sfr_map::IPR5),
        .RCON = static_cast<uint16_t>(pic18f66k80_sfr_map::RCON),
    };

    constexpr timer0_known_sfrs_t pic18fxx2_timer0_regs = {
        .TMR0L = static_cast<uint16_t>(pic18f66k80_sfr_map::TMR0L),
        .TMR0H = static_cast<uint16_t>(pic18f66k80_sfr_map::TMR0H),
        .T0CON = static_cast<uint16_t>(pic18f66k80_sfr_map::T0CON),
    };

    constexpr timer1_known_sfrs_t pic18fxx2_timer1_regs = {
        .TMR1L = static_cast<uint16_t>(pic18f66k80_sfr_map::TMR1L),
        .TMR1H = static_cast<uint16_t>(pic18f66k80_sfr_map::TMR1H),
        .T1CON = static_cast<uint16_t>(pic18f66k80_sfr_map::T1CON),
    };

    constexpr timer2_4_known_sfrs_t pic18fxx2_timer2_regs = {
        .TMR = static_cast<uint16_t>(pic18f66k80_sfr_map::TMR2),
        .PR = static_cast<uint16_t>(pic18f66k80_sfr_map::PR2),
        .TCON = static_cast<uint16_t>(pic18f66k80_sfr_map::T2CON),
    };

    constexpr timer3_known_sfrs_t pic18fxx2_timer3_regs = {
        .TMR3L = static_cast<uint16_t>(pic18f66k80_sfr_map::TMR3L),
        .TMR3H = static_cast<uint16_t>(pic18f66k80_sfr_map::TMR3H),
        .T3CON = static_cast<uint16_t>(pic18f66k80_sfr_map::T3CON),
    };

    constexpr timer2_4_known_sfrs_t pic18fxx2_timer4_regs = {
        .TMR = static_cast<uint16_t>(pic18f66k80_sfr_map::TMR4),
        .PR = static_cast<uint16_t>(pic18f66k80_sfr_map::PR4),
        .TCON = static_cast<uint16_t>(pic18f66k80_sfr_map::T4CON),
    };

    constexpr adc_known_sfrs_t pic18f66k80_adc_regs = {
        .ANCON0 = static_cast<uint16_t>(pic18f66k80_sfr_map::ANCON0),
        .ANCON1 = static_cast<uint16_t>(pic18f66k80_sfr_map::ANCON1),
        .ADCON0 = static_cast<uint16_t>(pic18f66k80_sfr_map::ADCON0),
        .ADCON1 = static_cast<uint16_t>(pic18f66k80_sfr_map::ADCON1),
        .ADCON2 = static_cast<uint16_t>(pic18f66k80_sfr_map::ADCON2),
        .ADRESH = static_cast<uint16_t>(pic18f66k80_sfr_map::ADRESH),
        .ADRESL = static_cast<uint16_t>(pic18f66k80_sfr_map::ADRESL),
    };

    constexpr ccp_common_known_sfrs_t pic18f66k80_ccpx_regs = {
        .PMD0 = static_cast<uint16_t>(pic18f66k80_sfr_map::PMD0),
        .CCPTMRS = static_cast<uint16_t>(pic18f66k80_sfr_map::CCPTMRS),
        .TMR1L = static_cast<uint16_t>(pic18f66k80_sfr_map::TMR1L),
        .TMR1H = static_cast<uint16_t>(pic18f66k80_sfr_map::TMR1H),
        .TMR2 = static_cast<uint16_t>(pic18f66k80_sfr_map::TMR2),
        .TMR3L = static_cast<uint16_t>(pic18f66k80_sfr_map::TMR3L),
        .TMR3H = static_cast<uint16_t>(pic18f66k80_sfr_map::TMR3H),
        .TMR4 = static_cast<uint16_t>(pic18f66k80_sfr_map::TMR4),
    };

    constexpr ccp_known_sfrs_t pic18f66k80_ccp1_regs = {
        .CCPxCON = static_cast<uint16_t>(pic18f66k80_sfr_map::CCP1CON),
        .CCPRxL = static_cast<uint16_t>(pic18f66k80_sfr_map::CCPR1L),
        .CCPRxH = static_cast<uint16_t>(pic18f66k80_sfr_map::CCPR1H),
        .common = pic18f66k80_ccpx_regs,
    };

    constexpr ccp_known_sfrs_t pic18f66k80_ccp2_regs = {
        .CCPxCON = static_cast<uint16_t>(pic18f66k80_sfr_map::CCP2CON),
        .CCPRxL = static_cast<uint16_t>(pic18f66k80_sfr_map::CCPR2L),
        .CCPRxH = static_cast<uint16_t>(pic18f66k80_sfr_map::CCPR2H),
        .common = pic18f66k80_ccpx_regs,
    };

    constexpr ccp_known_sfrs_t pic18f66k80_ccp3_regs = {
        .CCPxCON = static_cast<uint16_t>(pic18f66k80_sfr_map::CCP3CON),
        .CCPRxL = static_cast<uint16_t>(pic18f66k80_sfr_map::CCPR3L),
        .CCPRxH = static_cast<uint16_t>(pic18f66k80_sfr_map::CCPR3H),
        .common = pic18f66k80_ccpx_regs,
    };

    constexpr ccp_known_sfrs_t pic18f66k80_ccp4_regs = {
        .CCPxCON = static_cast<uint16_t>(pic18f66k80_sfr_map::CCP4CON),
        .CCPRxL = static_cast<uint16_t>(pic18f66k80_sfr_map::CCPR4L),
        .CCPRxH = static_cast<uint16_t>(pic18f66k80_sfr_map::CCPR4H),
        .common = pic18f66k80_ccpx_regs,
    };

    constexpr ccp_known_sfrs_t pic18f66k80_ccp5_regs = {
        .CCPxCON = static_cast<uint16_t>(pic18f66k80_sfr_map::CCP5CON),
        .CCPRxL = static_cast<uint16_t>(pic18f66k80_sfr_map::CCPR5L),
        .CCPRxH = static_cast<uint16_t>(pic18f66k80_sfr_map::CCPR5H),
        .common = pic18f66k80_ccpx_regs,
    };

    constexpr eusart_known_sfrs_t pic18f66k80_eusart1_regs = {
        .TXSTAx = static_cast<uint16_t>(pic18f66k80_sfr_map::TXSTA1),
        .RCSTAx = static_cast<uint16_t>(pic18f66k80_sfr_map::RCSTA1),
        .TXREGx = static_cast<uint16_t>(pic18f66k80_sfr_map::TXREG1),
        .RCREGx = static_cast<uint16_t>(pic18f66k80_sfr_map::RCREG1),
        .BAUDCONx = static_cast<uint16_t>(pic18f66k80_sfr_map::BAUDCON1),
        .SPBRGx = static_cast<uint16_t>(pic18f66k80_sfr_map::SPBRG1),
        .SPBRGHx = static_cast<uint16_t>(pic18f66k80_sfr_map::SPBRGH1),
    };

    constexpr eusart_known_sfrs_t pic18f66k80_eusart2_regs = {
        .TXSTAx = static_cast<uint16_t>(pic18f66k80_sfr_map::TXSTA2),
        .RCSTAx = static_cast<uint16_t>(pic18f66k80_sfr_map::RCSTA2),
        .TXREGx = static_cast<uint16_t>(pic18f66k80_sfr_map::TXREG2),
        .RCREGx = static_cast<uint16_t>(pic18f66k80_sfr_map::RCREG2),
        .BAUDCONx = static_cast<uint16_t>(pic18f66k80_sfr_map::BAUDCON2),
        .SPBRGx = static_cast<uint16_t>(pic18f66k80_sfr_map::SPBRG2),
        .SPBRGHx = static_cast<uint16_t>(pic18f66k80_sfr_map::SPBRGH2),
    };

    constexpr port_bus_config_t porta_cfg = {
        .bitmask = 0b11101111,
        .TRISx = register_addr(pic18f66k80_sfr_map::TRISA),
        .PADCFGx = 0,
        .ANCONx = register_addr(pic18f66k80_sfr_map::ANCON0),
        .WPUx = 0,
        .padcfg_bit = 0,
        .padcfg_enable = false,
        .padcfg_invert = false,
        .wpu_enable = false,
        .ansel_enable_mask = 0b00011111,
    };

    constexpr port_bus_config_t portb_cfg = {
        .bitmask = 0b11111111,
        .TRISx = register_addr(pic18f66k80_sfr_map::TRISB),
        .PADCFGx = register_addr(pic18f66k80_sfr_map::INTCON2),
        .ANCONx = register_addr(pic18f66k80_sfr_map::ANCON1),
        .WPUx = register_addr(pic18f66k80_sfr_map::WPUB),
        .padcfg_bit = 7,
        .padcfg_enable = true,
        .padcfg_invert = true,
        .wpu_enable = true,
        .ansel_enable_mask = 0b00000111,
    };

    constexpr port_bus_config_t portc_cfg = {
        .bitmask = 0b11111111,
        .TRISx = register_addr(pic18f66k80_sfr_map::TRISC),
        .PADCFGx = 0,
        .ANCONx = 0,
        .WPUx = 0,
        .padcfg_bit = 0,
        .padcfg_enable = false,
        .padcfg_invert = false,
        .wpu_enable = false,
        .ansel_enable_mask = 0,
    };

    constexpr port_bus_config_t portd_cfg = {
        .bitmask = 0b11111111,
        .TRISx = register_addr(pic18f66k80_sfr_map::TRISD),
        .PADCFGx = register_addr(pic18f66k80_sfr_map::PADCFG1),
        .ANCONx = register_addr(pic18f66k80_sfr_map::ANCON1),
        .WPUx = 0,
        .padcfg_bit = 7,
        .padcfg_enable = true,
        .padcfg_invert = false,
        .wpu_enable = false,
        .ansel_enable_mask = 0b01111001,
    };

    constexpr port_bus_config_t porte_cfg = {
        .bitmask = 0b11111111,
        .TRISx = register_addr(pic18f66k80_sfr_map::TRISE),
        .PADCFGx = register_addr(pic18f66k80_sfr_map::PADCFG1),
        .ANCONx = register_addr(pic18f66k80_sfr_map::ANCON0),
        .WPUx = 0,
        .padcfg_bit = 6,
        .padcfg_enable = true,
        .padcfg_invert = false,
        .wpu_enable = false,
        .ansel_enable_mask = 0b11100000,
    };

    constexpr port_bus_config_t portf_cfg = {
        .bitmask = 0b11111111,
        .TRISx = register_addr(pic18f66k80_sfr_map::TRISF),
        .PADCFGx = register_addr(pic18f66k80_sfr_map::PADCFG1),
        .ANCONx = 0,
        .WPUx = 0,
        .padcfg_bit = 5,
        .padcfg_enable = true,
        .padcfg_invert = false,
        .wpu_enable = false,
        .ansel_enable_mask = 0,
    };

    constexpr port_bus_config_t portg_cfg = {
        .bitmask = 0b00011111,
        .TRISx = register_addr(pic18f66k80_sfr_map::TRISG),
        .PADCFGx = register_addr(pic18f66k80_sfr_map::PADCFG1),
        .ANCONx = 0,
        .WPUx = 0,
        .padcfg_bit = 4,
        .padcfg_enable = true,
        .padcfg_invert = false,
        .wpu_enable = false,
        .ansel_enable_mask = 0,
    };

    port_ctx_t porta;
    port_ctx_t portb;
    port_ctx_t portc;
    port_ctx_t portd;
    port_ctx_t porte;
    port_ctx_t portf;
    port_ctx_t portg;
    port_initialize(porta);
    port_initialize(portb);
    port_initialize(portc);
    port_initialize(portd);
    port_initialize(porte);
    port_initialize(portf);
    port_initialize(portg);

    gpio_ctx_t<8> gpioa;
    gpio_ctx_t<8> gpiob;
    gpio_ctx_t<8> gpioc;
    gpio_ctx_t<8> gpiod;
    gpio_ctx_t<8> gpioe;
    gpio_ctx_t<8> gpiof;
    gpio_ctx_t<8> gpiog;
    gpio_initialize(gpioa, register_addr(pic18f66k80_sfr_map::PORTA), register_addr(pic18f66k80_sfr_map::LATA));
    gpio_initialize(gpiob, register_addr(pic18f66k80_sfr_map::PORTB), register_addr(pic18f66k80_sfr_map::LATB));
    gpio_initialize(gpioc, register_addr(pic18f66k80_sfr_map::PORTC), register_addr(pic18f66k80_sfr_map::LATC));
    gpio_initialize(gpiod, register_addr(pic18f66k80_sfr_map::PORTD), register_addr(pic18f66k80_sfr_map::LATD));
    gpio_initialize(gpioe, register_addr(pic18f66k80_sfr_map::PORTE), register_addr(pic18f66k80_sfr_map::LATE));
    gpio_initialize(gpiof, register_addr(pic18f66k80_sfr_map::PORTF), register_addr(pic18f66k80_sfr_map::LATF));
    gpio_initialize(gpiog, register_addr(pic18f66k80_sfr_map::PORTG), register_addr(pic18f66k80_sfr_map::LATG));

    int_state_t<39> int_state;
    int_state_initialize_18f66k80(int_state);
    int_state_reset_18f66k80(int_state);

    auto on_portb_changed = [&](bool set) {
        reg_sfr_write(read_data_bus, write_data_bus, pic18fxx2_cpu_regs.INTCON, reg_intcon_mask_RBIF, set);
    };

    gpio_add_change_interrupt(gpiob, 7, on_portb_changed);
    gpio_add_change_interrupt(gpiob, 6, on_portb_changed);
    gpio_add_change_interrupt(gpiob, 5, on_portb_changed);
    gpio_add_change_interrupt(gpiob, 4, on_portb_changed);

    auto on_timer0_overflow = [&](bool set) {
        reg_sfr_write(read_data_bus, write_data_bus, pic18fxx2_cpu_regs.INTCON, reg_intcon_mask_TMR0IF, set);
    };

    auto on_timer1_overflow = [&](bool set) {
        reg_sfr_write(read_data_bus, write_data_bus, pic18f66k80_int_regs.PIR1, reg_pir1_mask_TMR1IF, set);
    };

    auto on_timer2_match = [&](bool set) {
        reg_sfr_write(read_data_bus, write_data_bus, pic18f66k80_int_regs.PIR1, reg_pir1_mask_TMR2IF, set);
    };

    auto on_timer3_overflow = [&](bool set) {
        reg_sfr_write(read_data_bus, write_data_bus, pic18f66k80_int_regs.PIR2, reg_pir2_mask_TMR3IF, set);
    };

    auto on_timer4_match = [&](bool set) {
        reg_sfr_write(read_data_bus, write_data_bus, pic18f66k80_int_regs.PIR4, reg_pir4_mask_TMR4IF, set);
    };

    timer0_t timer0;
    timer1_t timer1;
    timer2_t timer2;
    timer3_t timer3;
    timer2_t timer4;
    timer0_initialize(timer0);
    timer1_initialize(timer1);
    timer2_initialize(timer2);
    timer3_initialize(timer3);
    timer2_initialize(timer4);
    timer0.overflow_interrupt = on_timer0_overflow;
    timer1.overflow_interrupt = on_timer1_overflow;
    timer2.period_match_interrupt = on_timer2_match;
    timer3.overflow_interrupt = on_timer3_overflow;
    timer4.period_match_interrupt = on_timer4_match;

    adc_ctx_t adc;
    adc_initialize(adc, pic18f66k80_adc_regs);
    adc.done_interrupt = [&](bool set) {
        reg_sfr_write(read_data_bus, write_data_bus, pic18f66k80_int_regs.PIR1, reg_pir1_mask_ADIF, set);
    };

    eusart_ctx_t eusart1;
    eusart_ctx_t eusart2;
    eusart_initialize(eusart1);
    eusart_initialize(eusart2);

    eusart1.sfr = pic18f66k80_eusart1_regs;
    eusart2.sfr = pic18f66k80_eusart2_regs;

    eusart1.mode_change = [&]() { std::cout << "EUSART1: mode changed, baud=" << eusart1.baud_rate << "\n"; };
    eusart2.mode_change = [&]() { std::cout << "EUSART2: mode changed, baud=" << eusart2.baud_rate << "\n"; };
    eusart1.transmit = [](uint8_t data, bool) { std::cout << static_cast<char>(data); };
    eusart2.transmit = [](uint8_t data, bool) { std::cout << static_cast<char>(data); };
    eusart1.rx_interrupt = [&](bool set) {
        reg_sfr_write(read_data_bus, write_data_bus, pic18f66k80_int_regs.PIR1, reg_pir1_mask_RC1IF, set);
    };
    eusart2.rx_interrupt = [&](bool set) {
        reg_sfr_write(read_data_bus, write_data_bus, pic18f66k80_int_regs.PIR3, reg_pir3_mask_RC2IF, set);
    };
    eusart1.tx_interrupt = [&](bool set) {
        reg_sfr_write(read_data_bus, write_data_bus, pic18f66k80_int_regs.PIR1, reg_pir1_mask_TX1IF, set);
    };
    eusart2.tx_interrupt = [&](bool set) {
        reg_sfr_write(read_data_bus, write_data_bus, pic18f66k80_int_regs.PIR3, reg_pir3_mask_TX2IF, set);
    };

    auto ccpx_special_event_tigger = [&](uint8_t timer_num, uint8_t ccp_num) {
        if (timer_num == 1)
            timer1_special_event_trigger(timer1);
        if (timer_num == 3)
            timer3_special_event_trigger(timer3);

        if (ccp_num == 1 || ccp_num == 2)
            adc_ccp2_event(adc);
    };

    ccp_ctx_t ccp1;
    ccp_ctx_t ccp2;
    ccp_ctx_t ccp3;
    ccp_ctx_t ccp4;
    ccp_ctx_t ccp5;
    ccp_initialize(ccp1, 1, pic18f66k80_ccp1_regs);
    ccp_initialize(ccp2, 2, pic18f66k80_ccp2_regs);
    ccp_initialize(ccp3, 3, pic18f66k80_ccp3_regs);
    ccp_initialize(ccp4, 4, pic18f66k80_ccp4_regs);
    ccp_initialize(ccp5, 5, pic18f66k80_ccp5_regs);
    ccp1.special_event_timer = [&](uint8_t timer_num) { ccpx_special_event_tigger(timer_num, 1); };
    ccp2.special_event_timer = [&](uint8_t timer_num) { ccpx_special_event_tigger(timer_num, 2); };
    ccp3.special_event_timer = [&](uint8_t timer_num) { ccpx_special_event_tigger(timer_num, 3); };
    ccp4.special_event_timer = [&](uint8_t timer_num) { ccpx_special_event_tigger(timer_num, 4); };
    ccp5.special_event_timer = [&](uint8_t timer_num) { ccpx_special_event_tigger(timer_num, 5); };
    ccp1.interrupt = [&](bool set) {
        reg_sfr_write(read_data_bus, write_data_bus, pic18f66k80_int_regs.PIR3, reg_pir3_mask_CCP1IF, set);
    };
    ccp2.interrupt = [&](bool set) {
        reg_sfr_write(read_data_bus, write_data_bus, pic18f66k80_int_regs.PIR3, reg_pir3_mask_CCP2IF, set);
    };
    ccp3.interrupt = [&](bool set) {
        reg_sfr_write(read_data_bus, write_data_bus, pic18f66k80_int_regs.PIR4, reg_pir4_mask_CCP3IF, set);
    };
    ccp4.interrupt = [&](bool set) {
        reg_sfr_write(read_data_bus, write_data_bus, pic18f66k80_int_regs.PIR4, reg_pir4_mask_CCP4IF, set);
    };
    ccp5.interrupt = [&](bool set) {
        reg_sfr_write(read_data_bus, write_data_bus, pic18f66k80_int_regs.PIR4, reg_pir4_mask_CCP5IF, set);
    };

    timer2.output = [&]() {
        ccp_pwm_match_input(ccp1, 2, read_data_bus);
        ccp_pwm_match_input(ccp2, 2, read_data_bus);
        ccp_pwm_match_input(ccp3, 2, read_data_bus);
        ccp_pwm_match_input(ccp4, 2, read_data_bus);
        ccp_pwm_match_input(ccp5, 2, read_data_bus);
    };

    timer4.output = [&]() {
        ccp_pwm_match_input(ccp1, 4, read_data_bus);
        ccp_pwm_match_input(ccp2, 4, read_data_bus);
        ccp_pwm_match_input(ccp3, 4, read_data_bus);
        ccp_pwm_match_input(ccp4, 4, read_data_bus);
        ccp_pwm_match_input(ccp5, 4, read_data_bus);
    };

    read_data_bus = [&](uint16_t addr) -> uint8_t {
        sfr_ctx_t ctx{
            .read_bus = read_data_bus,
            .write_bus = write_data_bus,
            .read_sfr_phy = [&](uint16_t sfr) -> addr_read_result_t { return sfr_phy_read(registers, sfr); },
            .write_sfr_phy = [&](uint16_t sfr, uint8_t val) -> addr_bit_mask_t {
                return sfr_phy_write(registers, sfr, val);
            },
            .sfr = pic18fxx2_indf_regs,
            .first_address = 0xE41,
            .last_address = 0xFFF,
        };

        addr_read_result_t result;
        uint8_t value = 0;

        result = cpu_bus_read(cpu, pic18fxx2_cpu_regs, addr);
        value |= result.data & result.mask;
        result = reg_sfr_bus_read(ctx, addr);
        value |= result.data & result.mask;
        result = int_addr_space_read_18f66k80(int_state, pic18f66k80_int_regs, addr);
        value |= result.data & result.mask;
        result = gpio_bus_read(gpioa, addr);
        value |= result.data & result.mask;
        result = gpio_bus_read(gpiob, addr);
        value |= result.data & result.mask;
        result = gpio_bus_read(gpioc, addr);
        value |= result.data & result.mask;
        result = gpio_bus_read(gpiod, addr);
        value |= result.data & result.mask;
        result = gpio_bus_read(gpioe, addr);
        value |= result.data & result.mask;
        result = gpio_bus_read(gpiof, addr);
        value |= result.data & result.mask;
        result = gpio_bus_read(gpiog, addr);
        value |= result.data & result.mask;
        result = port_bus_read<porta_cfg>(porta, addr);
        value |= result.data & result.mask;
        result = port_bus_read<portb_cfg>(portb, addr);
        value |= result.data & result.mask;
        result = port_bus_read<portc_cfg>(portc, addr);
        value |= result.data & result.mask;
        result = port_bus_read<portd_cfg>(portd, addr);
        value |= result.data & result.mask;
        result = port_bus_read<porte_cfg>(porte, addr);
        value |= result.data & result.mask;
        result = port_bus_read<portf_cfg>(portf, addr);
        value |= result.data & result.mask;
        result = port_bus_read<portg_cfg>(portg, addr);
        value |= result.data & result.mask;
        result = timer0_bus_read(timer0, pic18fxx2_timer0_regs, addr);
        value |= result.data & result.mask;
        result = timer1_bus_read(timer1, pic18fxx2_timer1_regs, addr);
        value |= result.data & result.mask;
        result = timer2_bus_read(timer2, pic18fxx2_timer2_regs, addr);
        value |= result.data & result.mask;
        result = timer3_bus_read(timer3, pic18fxx2_timer3_regs, addr);
        value |= result.data & result.mask;
        result = timer2_bus_read(timer4, pic18fxx2_timer4_regs, addr);
        value |= result.data & result.mask;
        result = adc_bus_read(adc, addr);
        value |= result.data & result.mask;
        result = ccp_bus_read(ccp1, addr);
        value |= result.data & result.mask;
        result = ccp_bus_read(ccp2, addr);
        value |= result.data & result.mask;
        result = ccp_bus_read(ccp3, addr);
        value |= result.data & result.mask;
        result = ccp_bus_read(ccp4, addr);
        value |= result.data & result.mask;
        result = ccp_bus_read(ccp5, addr);
        value |= result.data & result.mask;
        result = eusart_bus_read(eusart1, addr);
        value |= result.data & result.mask;
        result = eusart_bus_read(eusart2, addr);
        value |= result.data & result.mask;
        result = mem_read(sram, addr);
        value |= result.data & result.mask;

        env_bus_read(addr, value);
        return value;
    };

    write_data_bus = [&](uint16_t addr, uint8_t val) {
        sfr_ctx_t ctx{
            .read_bus = read_data_bus,
            .write_bus = write_data_bus,
            .read_sfr_phy = [&](uint16_t sfr) -> addr_read_result_t { return sfr_phy_read(registers, sfr); },
            .write_sfr_phy = [&](uint16_t sfr, uint8_t val) -> addr_bit_mask_t {
                return sfr_phy_write(registers, sfr, val);
            },
            .sfr = pic18fxx2_indf_regs,
            .first_address = 0xE41,
            .last_address = 0xFFF,
        };

        cpu_bus_write(cpu, pic18fxx2_cpu_regs, addr, val);
        reg_sfr_bus_write(ctx, addr, val);
        int_addr_space_write_18f66k80(int_state, pic18f66k80_int_regs, addr, val);
        gpio_bus_write(gpioa, addr, val);
        gpio_bus_write(gpiob, addr, val);
        gpio_bus_write(gpioc, addr, val);
        gpio_bus_write(gpiod, addr, val);
        gpio_bus_write(gpioe, addr, val);
        gpio_bus_write(gpiof, addr, val);
        gpio_bus_write(gpiog, addr, val);
        port_bus_write<porta_cfg>(porta, addr, val);
        port_bus_write<portb_cfg>(portb, addr, val);
        port_bus_write<portc_cfg>(portc, addr, val);
        port_bus_write<portd_cfg>(portd, addr, val);
        port_bus_write<porte_cfg>(porte, addr, val);
        port_bus_write<portf_cfg>(portf, addr, val);
        port_bus_write<portg_cfg>(portg, addr, val);
        timer0_bus_write(timer0, pic18fxx2_timer0_regs, addr, val);
        timer1_bus_write(timer1, pic18fxx2_timer1_regs, addr, val);
        timer2_bus_write(timer2, pic18fxx2_timer2_regs, addr, val);
        timer3_bus_write(timer3, pic18fxx2_timer3_regs, addr, val);
        timer2_bus_write(timer4, pic18fxx2_timer4_regs, addr, val);
        adc_bus_write(adc, addr, val);
        ccp_bus_write(ccp1, addr, val);
        ccp_bus_write(ccp2, addr, val);
        ccp_bus_write(ccp3, addr, val);
        ccp_bus_write(ccp4, addr, val);
        ccp_bus_write(ccp5, addr, val);
        eusart_bus_write(eusart1, addr, val);
        eusart_bus_write(eusart2, addr, val);
        mem_write(sram, addr, val);

        env_bus_write(addr, val);
    };

    auto read_prog_bus = [&](uint32_t addr) -> uint8_t {
        if (addr >= program_mem->size())
            return 0x0000;

        return (*program_mem)[addr];
    };

    auto write_prog_bus = [&](uint32_t addr, uint8_t val) {
        if (addr >= program_mem->size())
            return;

        (*program_mem)[addr] = val;
    };

    auto interrupt_vector = [&](bool high_priority) {
        cpu_interrupt_vector(cpu, pic18fxx2_cpu_regs, read_data_bus, read_prog_bus, high_priority);
    };

    std::ifstream program_file;
    program_file.open(program_file_name, std::ios_base::binary);
    if (!program_file)
    {
        std::cout << "program file not found";
        return -1;
    }

    for (int i = 0; program_file.peek() != -1; i++)
    {
        (*program_mem)[i] = program_file.get();
    }

    cpu_reset_por(cpu);

    bool stop = false;
    bool sleep = false;

    auto events = [&](cpu_event_t e) {
        if (e == cpu_event_t::illegal_instruction)
        {
            std::cout << "Illegal Instruction\n";
            stop = true;
        }

        if (e == cpu_event_t::sleep)
        {
            std::cout << "Going to sleep, good night...\n";
            sleep = true;
        }
    };

    auto wakeup = [&]() {
        if (sleep)
        {
            std::cout << "Waking up...\n";
        }
        sleep = false;
    };

    auto pin_output = [](char port_id, uint8_t pin, io_state_t state) {
        std::string state_str = "Error";
        if (state == io_state_t::high)
            state_str = "HIGH";
        else if (state == io_state_t::low)
            state_str = "LOW";
        else if (state == io_state_t::high_z)
            state_str = "Disconnected";
        else
            assert(false);

        std::cout << "Pin output: Port=" << port_id << " pin=" << static_cast<int>(pin) << " state=" << state_str
                  << "\n";
    };

    porta.on_output_changed = [&](uint8_t pin, io_state_t state) { pin_output('A', pin, state); };
    portb.on_output_changed = [&](uint8_t pin, io_state_t state) { pin_output('B', pin, state); };
    portc.on_output_changed = [&](uint8_t pin, io_state_t state) { pin_output('C', pin, state); };
    portd.on_output_changed = [&](uint8_t pin, io_state_t state) { pin_output('D', pin, state); };
    porte.on_output_changed = [&](uint8_t pin, io_state_t state) { pin_output('E', pin, state); };
    portf.on_output_changed = [&](uint8_t pin, io_state_t state) { pin_output('F', pin, state); };
    portg.on_output_changed = [&](uint8_t pin, io_state_t state) { pin_output('G', pin, state); };

    porta.on_input_changed = [&](uint8_t pin, io_state_t state) { gpio_set_input_state(gpioa, pin, state); };
    portb.on_input_changed = [&](uint8_t pin, io_state_t state) { gpio_set_input_state(gpiob, pin, state); };
    portc.on_input_changed = [&](uint8_t pin, io_state_t state) { gpio_set_input_state(gpioc, pin, state); };
    portd.on_input_changed = [&](uint8_t pin, io_state_t state) { gpio_set_input_state(gpiod, pin, state); };
    porte.on_input_changed = [&](uint8_t pin, io_state_t state) { gpio_set_input_state(gpioe, pin, state); };
    portf.on_input_changed = [&](uint8_t pin, io_state_t state) { gpio_set_input_state(gpiof, pin, state); };
    portg.on_input_changed = [&](uint8_t pin, io_state_t state) { gpio_set_input_state(gpiog, pin, state); };

    std::array<double, 10> adc_channels;
    adc_channels.fill(0);

    auto nop_analog = [](uint8_t, double) {};

    portc.on_analog_input_changed = nop_analog;
    portd.on_analog_input_changed = nop_analog;
    portf.on_analog_input_changed = nop_analog;
    portg.on_analog_input_changed = nop_analog;
    porta.on_analog_output_changed = nop_analog;
    portb.on_analog_output_changed = nop_analog;
    portc.on_analog_output_changed = nop_analog;
    portd.on_analog_output_changed = nop_analog;
    porte.on_analog_output_changed = nop_analog;
    portf.on_analog_output_changed = nop_analog;
    portg.on_analog_output_changed = nop_analog;

    porta.on_analog_input_changed = [&](uint8_t pin, double value) {
        if (pin == 0)
            adc_channels[0] = value;
        if (pin == 1)
            adc_channels[1] = value;
        if (pin == 2)
            adc_channels[2] = value;
        if (pin == 3)
            adc_channels[3] = value;
        if (pin == 5)
            adc_channels[4] = value;
    };
    portb.on_analog_input_changed = [&](uint8_t pin, double value) {
        if (pin == 0)
            adc_channels[10] = value;
        if (pin == 1)
            adc_channels[8] = value;
        if (pin == 4)
            adc_channels[9] = value;
    };
    porte.on_analog_input_changed = [&](uint8_t pin, double value) {
        if (pin == 0)
            adc_channels[5] = value;
        if (pin == 1)
            adc_channels[6] = value;
        if (pin == 2)
            adc_channels[7] = value;
    };

    gpio_on_output_changed(gpioa, [&](auto pin, auto state) { port_output_set_digital(porta, pin, state); });
    gpio_on_output_changed(gpiob, [&](auto pin, auto state) { port_output_set_digital(portb, pin, state); });
    gpio_on_output_changed(gpioc, [&](auto pin, auto state) { port_output_set_digital(portc, pin, state); });
    gpio_on_output_changed(gpiod, [&](auto pin, auto state) { port_output_set_digital(portd, pin, state); });
    gpio_on_output_changed(gpioe, [&](auto pin, auto state) { port_output_set_digital(porte, pin, state); });
    gpio_on_output_changed(gpiof, [&](auto pin, auto state) { port_output_set_digital(portf, pin, state); });
    gpio_on_output_changed(gpiog, [&](auto pin, auto state) { port_output_set_digital(portg, pin, state); });

    adc.read_channel = [&](adc_chan_t channel) -> adc_voltage_t {
        if (channel == adc_chan_t::avss)
            return 0.0;
        if (channel == adc_chan_t::avddcore)
            return 5.0;

        if (channel == adc_chan_t::random)
            return (static_cast<adc_voltage_t>(rand()) / RAND_MAX) * 5.0;

        if (channel >= adc_chan_t::an0 || channel >= adc_chan_t::an10)
            return adc_channels[static_cast<uint8_t>(channel) - static_cast<uint8_t>(adc_chan_t::an0)];

        return 3.3;
    };

    auto por_reset = [&]() {
        cpu_reset_por(cpu);
        int_state_reset_18f66k80(int_state);
        timer0_reset(timer0);
        timer1_reset(timer1);
        timer2_reset(timer2);
        timer3_reset(timer3);
        timer2_reset(timer4);
        ccp_reset(ccp1);
        ccp_reset(ccp2);
        ccp_reset(ccp3);
        ccp_reset(ccp4);
        ccp_reset(ccp5);
        adc_reset(adc);
    };

    port_input_set_analog(porta, 0, 2.0);

    signal(SIGINT, sig_handler);
    signal_handler = [&](int) { stop = true; };

    env_init();

    std::cout << "Clock start\n";

    por_reset();
    auto start = std::chrono::steady_clock::now();
    int tick_counter = 0;

    for (;;)
    {
        tick_counter++;
        if (stop)
            break;

        interrupt_tick(read_prog_bus, int_state, interrupt_vector, wakeup);
        timer0_tick(timer0, read_data_bus);
        timer1_tick(timer1, read_data_bus);
        timer2_tick(timer2, read_data_bus);
        timer3_tick(timer3, read_data_bus);
        timer2_tick(timer4, read_data_bus);
        ccp_tick(ccp1, read_data_bus);
        ccp_tick(ccp2, read_data_bus);
        ccp_tick(ccp3, read_data_bus);
        ccp_tick(ccp4, read_data_bus);
        ccp_tick(ccp5, read_data_bus);
        adc_tick_fosc(adc);

        if (!sleep)
        {
            cpu_tick(cpu, pic18fxx2_cpu_regs, read_prog_bus, write_prog_bus, read_data_bus, write_data_bus, events);
        }

        sleep_us(1000);
    }

    auto end = std::chrono::steady_clock::now();

    auto format_duration = [](std::chrono::duration<double, std::micro> d) -> std::string {
        if (d.count() < 1'000)
            return std::to_string(d.count()) + "µs";
        if (d.count() < 1'000'000)
            return std::to_string(d.count() / 1'000) + "ms";

        return std::to_string(d.count() / 1'000'000) + "s";
    };

    std::cout << "\nTicks: " << tick_counter << ", Runtime: " << format_duration(end - start) << "\n";

    return 0;
}
