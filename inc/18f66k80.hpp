#pragma once

#include "int.hpp"

// clang-format off
enum class pic18f66k80_sfr_map
{
    TOSU        = 0xFFF, INDF2        = 0xFDF, ECCP1AS      = 0xFBF, IPR1        = 0xF9F, EECON1      = 0xF7F, CM1CON    = 0xF5F,
    TOSH        = 0xFFE, POSTINC2     = 0xFDE, ECCP1DEL     = 0xFBE, PIR1        = 0xF9E, EECON2      = 0xF7E, CM2CON    = 0xF5E,
    TOSL        = 0xFFD, POSTDEC2     = 0xFDD, CCPR1H       = 0xFBD, PIE1        = 0xF9D, SPBRGH1     = 0xF7D, ANCON0    = 0xF5D,
    STKPTR      = 0xFFC, PREINC2      = 0xFDC, CCPR1L       = 0xFBC, PSTR1CON    = 0xF9C, SPBRGH2     = 0xF7C, ANCON1    = 0xF5C,
    PCLATU      = 0xFFB, PLUSW2       = 0xFDB, CCP1CON      = 0xFBB, OSCTUNE     = 0xF9B, SPBRG2      = 0xF7B, WPUB      = 0xF5B,
    PCLATH      = 0xFFA, FSR2H        = 0xFDA, TXSTA2       = 0xFBA, REFOCON     = 0xF9A, RCREG2      = 0xF7A, IOCB      = 0xF5A,
    PCL         = 0xFF9, FSR2L        = 0xFD9, BAUDCON2     = 0xFB9, CCPTMRS     = 0xF99, TXREG2      = 0xF79, PMD0      = 0xF59,
    TBLPTRU     = 0xFF8, STATUS       = 0xFD8, IPR4         = 0xFB8, TRISG       = 0xF98, IPR5        = 0xF78, PMD1      = 0xF58,
    TBLPTRH     = 0xFF7, TMR0H        = 0xFD7, PIR4         = 0xFB7, TRISF       = 0xF97, PIR5        = 0xF77, PMD2      = 0xF57,
    TBLPTRL     = 0xFF6, TMR0L        = 0xFD6, PIE4         = 0xFB6, TRISE       = 0xF96, PIE5        = 0xF76, PADCFG1   = 0xF56,
    TABLAT      = 0xFF5, T0CON        = 0xFD5, CVRCON       = 0xFB5, TRISD       = 0xF95, EEADRH      = 0xF75, CTMUCONH  = 0xF55,
    PRODH       = 0xFF4, CMSTAT       = 0xFB4, TRISC        = 0xF94, EEADR       = 0xF74, CTMUCONL    = 0xF54,
    PRODL       = 0xFF3, OSCCON       = 0xFD3, TMR3H        = 0xFB3, TRISB       = 0xF93, EEDATA      = 0xF73, CTMUICONH = 0xF53,
    INTCON      = 0xFF2, OSCCON2      = 0xFD2, TMR3L        = 0xFB2, TRISA       = 0xF92, ECANCON     = 0xF72, CCPR2H    = 0xF52,
    INTCON2     = 0xFF1, WDTCON       = 0xFD1, T3CON        = 0xFB1, ODCON       = 0xF91, COMSTAT     = 0xF71, CCPR2L    = 0xF51,
    INTCON3     = 0xFF0, RCON         = 0xFD0, T3GCON       = 0xFB0, SLRCON      = 0xF90, CIOCON      = 0xF70, CCP2CON   = 0xF50,
    INDF0       = 0xFEF, TMR1H        = 0xFCF, SPBRG1       = 0xFAF, LATG        = 0xF8F, CANCON      = 0xF6F, CCPR3H    = 0xF4F,
    POSTINC0    = 0xFEE, TMR1L        = 0xFCE, RCREG1       = 0xFAE, LATF        = 0xF8E, CANSTAT     = 0xF6E, CCPR3L    = 0xF4E,
    POSTDEC0    = 0xFED, T1CON        = 0xFCD, TXREG1       = 0xFAD, LATE        = 0xF8D, RXB0D7      = 0xF6D, CCP3CON   = 0xF4D,
    PREINC0     = 0xFEC, TMR2         = 0xFCC, TXSTA1       = 0xFAC, LATD        = 0xF8C, RXB0D6      = 0xF6C, CCPR4H    = 0xF4C,
    PLUSW0      = 0xFEB, PR2          = 0xFCB, RCSTA1       = 0xFAB, LATC        = 0xF8B, RXB0D5      = 0xF6B, CCPR4L    = 0xF4B,
    FSR0H       = 0xFEA, T2CON        = 0xFCA, T1GCON       = 0xFAA, LATB        = 0xF8A, RXB0D4      = 0xF6A, CCP4CON   = 0xF4A,
    FSR0L       = 0xFE9, SSPBUF       = 0xFC9, PR4          = 0xFA9, LATA        = 0xF89, RXB0D3      = 0xF69, CCPR5H    = 0xF49,
    WREG        = 0xFE8, SSPADD       = 0xFC8, HLVDCON      = 0xFA8, T4CON       = 0xF88, RXB0D2      = 0xF68, CCPR5L    = 0xF48,
    INDF1       = 0xFE7, SSPMSK       = 0xFC8, BAUDCON1     = 0xFA7, TMR4        = 0xF87, RXB0D1      = 0xF67, CCP5CON   = 0xF47,
    POSTINC1    = 0xFE6, SSPSTAT      = 0xFC7, RCSTA2       = 0xFA6, PORTG       = 0xF86, RXB0D0      = 0xF66, PSPCON    = 0xF46,
    POSTDEC1    = 0xFE5, SSPCON1      = 0xFC6, IPR3         = 0xFA5, PORTF       = 0xF85, RXB0DLC     = 0xF65, MDCON     = 0xF45,
    PREINC1     = 0xFE4, SSPCON2      = 0xFC5, PIR3         = 0xFA4, PORTE       = 0xF84, RXB0EIDL    = 0xF64, MDSRC     = 0xF44,
    PLUSW1      = 0xFE3, ADRESH       = 0xFC4, PIE3         = 0xFA3, PORTD       = 0xF83, RXB0EIDH    = 0xF63, MDCARH    = 0xF43,
    FSR1H       = 0xFE2, ADRESL       = 0xFC3, IPR2         = 0xFA2, PORTC       = 0xF82, RXB0SIDL    = 0xF62, MDCARL    = 0xF42,
    FSR1L       = 0xFE1, ADCON0       = 0xFC2, PIR2         = 0xFA1, PORTB       = 0xF81, RXB0SIDH    = 0xF61, 
    BSR         = 0xFE0, ADCON1       = 0xFC1, PIE2         = 0xFA0, PORTA       = 0xF80, RXB0CON     = 0xF60, RXF7EIDL  = 0xE4F,
    ADCON2      = 0xFC0, CANCON_RO0   = 0xF3F, CANCON_RO3   = 0xF0F, CANCON_RO4  = 0xEDF, CANCON_RO7  = 0xEAF, TXBIE     = 0xE7F, 
    CANSTAT_RO0 = 0xF3E, CANSTAT_RO3  = 0xF0E, CANSTAT_RO4  = 0xEDE, CANSTAT_RO7 = 0xEAE, BIE0        = 0xE7E, RXF7EIDH  = 0xE4E,
    RXB1D7      = 0xF3D, TXB2D7       = 0xF0D, B5D7         = 0xEDD, B2D7        = 0xEAD, BSEL0       = 0xE7D, RXF7SIDL  = 0xE4D,
    RXB1D6      = 0xF3C, TXB2D6       = 0xF0C, B5D6         = 0xEDC, B2D6        = 0xEAC, MSEL3       = 0xE7C, RXF7SIDH  = 0xE4C,
    RXB1D5      = 0xF3B, TXB2D5       = 0xF0B, B5D5         = 0xEDB, B2D5        = 0xEAB, MSEL2       = 0xE7B, RXF6EIDL  = 0xE4B,
    RXB1D4      = 0xF3A, TXB2D4       = 0xF0A, B5D4         = 0xEDA, B2D4        = 0xEAA, MSEL1       = 0xE7A, RXF6EIDH  = 0xE4A,
    RXB1D3      = 0xF39, TXB2D3       = 0xF09, B5D3         = 0xED9, B2D3        = 0xEA9, MSEL0       = 0xE79, RXF6SIDL  = 0xE49,
    RXB1D2      = 0xF38, TXB2D2       = 0xF08, B5D2         = 0xED8, B2D2        = 0xEA8, RXFBCON7    = 0xE78, RXF6SIDH  = 0xE48,
    RXB1D1      = 0xF37, TXB2D1       = 0xF07, B5D1         = 0xED7, B2D1        = 0xEA7, RXFBCON6    = 0xE77, RXFCON1   = 0xE47,
    RXB1D0      = 0xF36, TXB2D0       = 0xF06, B5D0         = 0xED6, B2D0        = 0xEA6, RXFBCON5    = 0xE76, RXFCON0   = 0xE46,
    RXB1DLC     = 0xF35, TXB2DLC      = 0xF05, B5DLC        = 0xED5, B2DLC       = 0xEA5, RXFBCON4    = 0xE75, BRGCON3   = 0xE45,
    RXB1EIDL    = 0xF34, TXB2EIDL     = 0xF04, B5EIDL       = 0xED4, B2EIDL      = 0xEA4, RXFBCON3    = 0xE74, BRGCON2   = 0xE44,
    RXB1EIDH    = 0xF33, TXB2EIDH     = 0xF03, B5EIDH       = 0xED3, B2EIDH      = 0xEA3, RXFBCON2    = 0xE73, BRGCON1   = 0xE43,
    RXB1SIDL    = 0xF32, TXB2SIDL     = 0xF02, B5SIDL       = 0xED2, B2SIDL      = 0xEA2, RXFBCON1    = 0xE72, TXERRCNT  = 0xE42,
    RXB1SIDH    = 0xF31, TXB2SIDH     = 0xF01, B5SIDH       = 0xED1, B2SIDH      = 0xEA1, RXFBCON0    = 0xE71, RXERRCNT  = 0xE41,
    RXB1CON     = 0xF30, TXB2CON      = 0xF00, B5CON        = 0xED0, B2CON       = 0xEA0, SDFLC       = 0xE70,
    RXM1EIDL     = 0xEFF, CANCON_RO5   = 0xECF, CANCON_RO8  = 0xE9F, RXF15EIDL   = 0xE6F,
    CANCON_RO1  = 0xF2F, RXM1EIDH     = 0xEFE, CANSTAT_RO5  = 0xECE, CANSTAT_RO8 = 0xE9E, RXF15EIDH   = 0xE6E,
    CANSTAT_RO1 = 0xF2E, RXM1SIDL     = 0xEFD, B4D7         = 0xECD, B1D7        = 0xE9D, RXF15SIDL   = 0xE6D,
    TXB0D7      = 0xF2D, RXM1SIDH     = 0xEFC, B4D6         = 0xECC, B1D6        = 0xE9C, RXF15SIDH   = 0xE6C,
    TXB0D6      = 0xF2C, RXM0EIDL     = 0xEFB, B4D5         = 0xECB, B1D5        = 0xE9B, RXF14EIDL   = 0xE6B,
    TXB0D5      = 0xF2B, RXM0EIDH     = 0xEFA, B4D4         = 0xECA, B1D4        = 0xE9A, RXF14EIDH   = 0xE6A,
    TXB0D4      = 0xF2A, RXM0SIDL     = 0xEF9, B4D3         = 0xEC9, B1D3        = 0xE99, RXF14SIDL   = 0xE69,
    TXB0D3      = 0xF29, RXM0SIDH     = 0xEF8, B4D2         = 0xEC8, B1D2        = 0xE98, RXF14SIDH   = 0xE68,
    TXB0D2      = 0xF28, RXF5EIDL     = 0xEF7, B4D1         = 0xEC7, B1D1        = 0xE97, RXF13EIDL   = 0xE67,
    TXB0D1      = 0xF27, RXF5EIDH     = 0xEF6, B4D0         = 0xEC6, B1D0        = 0xE96, RXF13EIDH   = 0xE66,
    TXB0D0      = 0xF26, RXF5SIDL     = 0xEF5, B4DLC        = 0xEC5, B1DLC       = 0xE95, RXF13SIDL   = 0xE65,
    TXB0DLC     = 0xF25, RXF5SIDH     = 0xEF4, B4EIDL       = 0xEC4, B1EIDL      = 0xE94, RXF13SIDH   = 0xE64,
    TXB0EIDL    = 0xF24, RXF4EIDL     = 0xEF3, B4EIDH       = 0xEC3, B1EIDH      = 0xE93, RXF12EIDL   = 0xE63,
    TXB0EIDH    = 0xF23, RXF4EIDH     = 0xEF2, B4SIDL       = 0xEC2, B1SIDL      = 0xE92, RXF12EIDH   = 0xE62,
    TXB0SIDL    = 0xF22, RXF4SIDL     = 0xEF1, B4SIDH       = 0xEC1, B1SIDH      = 0xE91, RXF12SIDL   = 0xE61,
    TXB0SIDH    = 0xF21, RXF4SIDH     = 0xEF0, B4CON        = 0xEC0, B1CON       = 0xE90, RXF12SIDH   = 0xE60,
    TXB0CON     = 0xF20, RXF3EIDL     = 0xEEF, CANCON_RO6   = 0xEBF, RXF11EIDL   = 0xE5F,
    CANCON_RO2  = 0xF1F, RXF3EIDH     = 0xEEE, CANSTAT_RO6  = 0xEBE, CANCON_RO9  = 0xE8F, RXF11EIDH   = 0xE5E,
    CANSTAT_RO2 = 0xF1E, RXF3SIDL     = 0xEED, B3D7         = 0xEBD, CANSTAT_RO9 = 0xE8E, RXF11SIDL   = 0xE5D,
    TXB1D7      = 0xF1D, RXF3SIDH     = 0xEEC, B3D6         = 0xEBC, B0D7        = 0xE8D, RXF11SIDH   = 0xE5C,
    TXB1D6      = 0xF1C, RXF2EIDL     = 0xEEB, B3D5         = 0xEBB, B0D6        = 0xE8C, RXF10EIDL   = 0xE5B,
    TXB1D5      = 0xF1B, RXF2EIDH     = 0xEEA, B3D4         = 0xEBA, B0D5        = 0xE8B, RXF10EIDH   = 0xE5A,
    TXB1D4      = 0xF1A, RXF2SIDL     = 0xEE9, B3D3         = 0xEB9, B0D4        = 0xE8A, RXF10SIDL   = 0xE59,
    TXB1D3      = 0xF19, RXF2SIDH     = 0xEE8, B3D2         = 0xEB8, B0D3        = 0xE89, RXF10SIDH   = 0xE58,
    TXB1D2      = 0xF18, RXF1EIDL     = 0xEE7, B3D1         = 0xEB7, B0D2        = 0xE88, RXF9EIDL    = 0xE57,
    TXB1D1      = 0xF17, RXF1EIDH     = 0xEE6, B3D0         = 0xEB6, B0D1        = 0xE87, RXF9EIDH    = 0xE56,
    TXB1D0      = 0xF16, RXF1SIDL     = 0xEE5, B3DLC        = 0xEB5, B0D0        = 0xE86, RXF9SIDL    = 0xE55,
    TXB1DLC     = 0xF15, RXF1SIDH     = 0xEE4, B3EIDL       = 0xEB4, B0DLC       = 0xE85, RXF9SIDH    = 0xE54,
    TXB1EIDL    = 0xF14, RXF0EIDL     = 0xEE3, B3EIDH       = 0xEB3, B0EIDL      = 0xE84, RXF8EIDL    = 0xE53,
    TXB1EIDH    = 0xF13, RXF0EIDH     = 0xEE2, B3SIDL       = 0xEB2, B0EIDH      = 0xE83, RXF8EIDH    = 0xE52,
    TXB1SIDL    = 0xF12, RXF0SIDL     = 0xEE1, B3SIDH       = 0xEB1, B0SIDL      = 0xE82, RXF8SIDL    = 0xE51,
    TXB1SIDH    = 0xF11, RXF0SIDH     = 0xEE0, B3CON        = 0xEB0, B0SIDH      = 0xE81, RXF8SIDH    = 0xE50,
    TXB1CON     = 0xF10, B0CON        = 0xE80,
};
// clang-format on

inline constexpr uint16_t register_addr(pic18f66k80_sfr_map sfr)
{
    return static_cast<uint16_t>(sfr);
}

enum reg_status_mask_t
{
    reg_status_mask_C = 1 << 0,
    reg_status_mask_DC = 1 << 1,
    reg_status_mask_Z = 1 << 2,
    reg_status_mask_OV = 1 << 3,
    reg_status_mask_N = 1 << 4,
};

enum reg_rcon_mask_t
{
    reg_rcon_mask_BOR = 1 << 0,
    reg_rcon_mask_POR = 1 << 1,
    reg_rcon_mask_PD = 1 << 2,
    reg_rcon_mask_TO = 1 << 3,
    reg_rcon_mask_RI = 1 << 4,
    reg_rcon_mask_CM = 1 << 5,
    reg_rcon_mask_SBOREN = 1 << 6,
    reg_rcon_mask_IPEN = 1 << 7,
};

enum reg_stkptr_mask_t
{
    reg_stkptr_mask_SP = 0b00011111,
    reg_stkptr_mask_STKUNF = 1 << 6,
    reg_stkptr_mask_STKOVF = 1 << 7,
};

namespace interrupt
{
/// @brief Cast a register name to its address.
constexpr uint16_t a(pic18f66k80_sfr_map sfr)
{
    return static_cast<uint16_t>(sfr);
}

using REG = pic18f66k80_sfr_map;
using enum int_make_source_flags_t;

struct pic18f66k80_interrupt_map
{
    int_source_t INT0 = int_make_source(a(REG::INTCON), 4, a(REG::INTCON), 1, 0, 0);
    int_source_t INT1 = int_make_source(a(REG::INTCON3), 3, a(REG::INTCON3), 0, a(REG::INTCON3), 6);
    int_source_t INT2 = int_make_source(a(REG::INTCON3), 4, a(REG::INTCON3), 1, a(REG::INTCON3), 7);
    int_source_t INT3 = int_make_source(a(REG::INTCON3), 5, a(REG::INTCON3), 2, a(REG::INTCON2), 1);
    int_source_t RB = int_make_source(a(REG::INTCON), 3, a(REG::INTCON), 0, a(REG::INTCON2), 0);
    int_source_t TMR0 = int_make_source(a(REG::INTCON), 7, a(REG::INTCON), 2, a(REG::INTCON2), 2);
    int_source_t TMR1 = int_make_source(a(REG::PIE1), 0, a(REG::PIR1), 0, a(REG::IPR1), 0, peripheral);
    int_source_t TMR1G = int_make_source(a(REG::PIE1), 2, a(REG::PIR1), 2, a(REG::IPR1), 2, peripheral);
    int_source_t TMR2 = int_make_source(a(REG::PIE1), 1, a(REG::PIR1), 1, a(REG::IPR1), 1, peripheral);
    int_source_t TMR3 = int_make_source(a(REG::PIE2), 1, a(REG::PIR2), 1, a(REG::IPR2), 1, peripheral);
    int_source_t TMR3G = int_make_source(a(REG::PIE2), 0, a(REG::PIR2), 0, a(REG::IPR2), 0, peripheral);
    int_source_t TMR4 = int_make_source(a(REG::PIE4), 7, a(REG::PIR4), 7, a(REG::IPR4), 7, peripheral);
    int_source_t CCP1 = int_make_source(a(REG::PIE3), 1, a(REG::PIR3), 1, a(REG::IPR3), 1, peripheral);
    int_source_t CCP2 = int_make_source(a(REG::PIE3), 2, a(REG::PIR3), 2, a(REG::IPR3), 2, peripheral);
    int_source_t CCP3 = int_make_source(a(REG::PIE4), 0, a(REG::PIR4), 0, a(REG::IPR4), 0, peripheral);
    int_source_t CCP4 = int_make_source(a(REG::PIE4), 1, a(REG::PIR4), 1, a(REG::IPR4), 1, peripheral);
    int_source_t CCP5 = int_make_source(a(REG::PIE4), 2, a(REG::PIR4), 2, a(REG::IPR4), 2, peripheral);
    int_source_t EE = int_make_source(a(REG::PIE4), 6, a(REG::PIR4), 6, a(REG::IPR4), 6, peripheral);
    int_source_t CMP1 = int_make_source(a(REG::PIE4), 4, a(REG::PIR4), 4, a(REG::IPR4), 4, peripheral);
    int_source_t CMP2 = int_make_source(a(REG::PIE4), 5, a(REG::PIR4), 5, a(REG::IPR4), 5, peripheral);
    int_source_t CTMU = int_make_source(a(REG::PIE3), 3, a(REG::PIR3), 3, a(REG::IPR3), 3, peripheral);
    int_source_t PSP = int_make_source(a(REG::PIE1), 7, a(REG::PIR1), 7, a(REG::IPR1), 7, peripheral);
    int_source_t AD = int_make_source(a(REG::PIE1), 6, a(REG::PIR1), 6, a(REG::IPR1), 6, peripheral);
    int_source_t SSP = int_make_source(a(REG::PIE1), 3, a(REG::PIR1), 3, a(REG::IPR1), 3, peripheral);
    int_source_t OSC = int_make_source(a(REG::PIE2), 7, a(REG::PIR2), 7, a(REG::IPR2), 7, peripheral);
    int_source_t TX1 = int_make_source(a(REG::PIE1), 4, a(REG::PIR1), 4, a(REG::IPR1), 4, peripheral | flag_ro);
    int_source_t TX2 = int_make_source(a(REG::PIE3), 4, a(REG::PIR3), 4, a(REG::IPR3), 4, peripheral | flag_ro);
    int_source_t RC1 = int_make_source(a(REG::PIE1), 5, a(REG::PIR1), 5, a(REG::IPR1), 5, peripheral);
    int_source_t RC2 = int_make_source(a(REG::PIE3), 5, a(REG::PIR3), 5, a(REG::IPR3), 5, peripheral);
    int_source_t BCL = int_make_source(a(REG::PIE2), 3, a(REG::PIR2), 3, a(REG::IPR2), 3, peripheral);
    int_source_t HLVD = int_make_source(a(REG::PIE2), 2, a(REG::PIR2), 2, a(REG::IPR2), 2, peripheral);
    int_source_t IRX = int_make_source(a(REG::PIE5), 7, a(REG::PIR5), 7, a(REG::IPR5), 7, peripheral);
    int_source_t WAK = int_make_source(a(REG::PIE5), 6, a(REG::PIR5), 6, a(REG::IPR5), 6, peripheral);
    int_source_t ERR = int_make_source(a(REG::PIE5), 5, a(REG::PIR5), 5, a(REG::IPR5), 5, peripheral);
    int_source_t TX2B = int_make_source(a(REG::PIE5), 4, a(REG::PIR5), 4, a(REG::IPR5), 4, peripheral);
    int_source_t TXB1 = int_make_source(a(REG::PIE5), 3, a(REG::PIR5), 3, a(REG::IPR5), 3, peripheral);
    int_source_t TXB0 = int_make_source(a(REG::PIE5), 2, a(REG::PIR5), 2, a(REG::IPR5), 2, peripheral);
    int_source_t RXB1 = int_make_source(a(REG::PIE5), 1, a(REG::PIR5), 1, a(REG::IPR5), 1, peripheral);
    int_source_t RXB0 = int_make_source(a(REG::PIE5), 0, a(REG::PIR5), 0, a(REG::IPR5), 0, peripheral);
};

} // namespace interrupt

using interrupt::pic18f66k80_interrupt_map;

std::vector<int_source_t> pic18f66k80_make_interrupt_sources(pic18f66k80_interrupt_map m)
{
    return std::vector<int_source_t>{
        m.INT0, m.INT1, m.INT2, m.INT3, m.RB,   m.TMR0, m.TMR1, m.TMR1G, m.TMR2, m.TMR3, m.TMR3G, m.TMR4, m.CCP1,
        m.CCP2, m.CCP3, m.CCP4, m.CCP5, m.EE,   m.CMP1, m.CMP2, m.CTMU,  m.PSP,  m.AD,   m.SSP,   m.OSC,  m.TX1,
        m.TX2,  m.RC1,  m.RC2,  m.BCL,  m.HLVD, m.IRX,  m.WAK,  m.ERR,   m.TX2B, m.TXB1, m.TXB0,  m.RXB1, m.RXB0,
    };
}