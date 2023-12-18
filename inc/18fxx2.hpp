#pragma once

enum class pic18fxx2_sfr_map
{
    PORTA = 0xF80,
    PORTB = 0xF81,
    PORTC = 0xF82,
    PORTD = 0xF83,
    PORTE = 0xF84,
    LATA = 0xF89,
    LATB = 0xF8A,
    LATC = 0xF8B,
    LATD = 0xF8C,
    LATE = 0xF8D,
    TRISA = 0xF92,
    TRISB = 0xF93,
    TRISC = 0xF94,
    TRISD = 0xF95,
    TRISE = 0xF96,
    PIE1 = 0xF9D,
    PIR1 = 0xF9E,
    IPR1 = 0xF9F,
    PIE2 = 0xFA0,
    PIR2 = 0xFA1,
    IPR2 = 0xFA2,
    EECON1 = 0xFA6,
    EECON2 = 0xFA7,
    EEDATA = 0xFA8,
    EEADR = 0xFA9,
    RCSTA = 0xFAB,
    TXSTA = 0xFAC,
    TXREG = 0xFAD,
    RCREG = 0xFAE,
    SPBRG = 0xFAF,
    T3CON = 0xFB1,
    TMR3L = 0xFB2,
    TMR3H = 0xFB3,
    CCP2CON = 0xFBA,
    CCPR2L = 0xFBB,
    CCPR2H = 0xFBC,
    CCP1CON = 0xFBD,
    CCPR1L = 0xFBE,
    CCPR1H = 0xFBF,
    ADCON1 = 0xFC1,
    ADCON0 = 0xFC2,
    ADRESL = 0xFC3,
    ADRESH = 0xFC4,
    SSPCON2 = 0xFC5,
    SSPCON1 = 0xFC6,
    SSPSTAT = 0xFC7,
    SSPADD = 0xFC8,
    SSPBUF = 0xFC9,
    T2CON = 0xFCA,
    PR2 = 0xFCB,
    TMR2 = 0xFCC,
    T1CON = 0xFCD,
    TMR1L = 0xFCE,
    TMR1H = 0xFCF,
    RCON = 0xFD0,
    WDTCON = 0xFD1,
    LVDCON = 0xFD2,
    OSCCON = 0xFD3,
    T0CON = 0xFD5,
    TMR0L = 0xFD6,
    TMR0H = 0xFD7,
    STATUS = 0xFD8,
    FSR2L = 0xFD9,
    FSR2H = 0xFDA,
    PLUSW2 = 0xFDB,
    PREINC2 = 0xFDC,
    POSTDEC2 = 0xFDD,
    POSTINC2 = 0xFDE,
    INDF2 = 0xFDF,
    BSR = 0xFE0,
    FSR1L = 0xFE1,
    FSR1H = 0xFE2,
    PLUSW1 = 0xFE3,
    PREINC1 = 0xFE4,
    POSTDEC1 = 0xFE5,
    POSTINC1 = 0xFE6,
    INDF1 = 0xFE7,
    WREG = 0xFE8,
    FSR0L = 0xFE9,
    FSR0H = 0xFEA,
    PLUSW0 = 0xFEB,
    PREINC0 = 0xFEC,
    POSTDEC0 = 0xFED,
    POSTINC0 = 0xFEE,
    INDF0 = 0xFEF,
    INTCON3 = 0xFF0,
    INTCON2 = 0xFF1,
    INTCON = 0xFF2,
    PRODL = 0xFF3,
    PRODH = 0xFF4,
    TABLAT = 0xFF5,
    TBLPTRL = 0xFF6,
    TBLPTRH = 0xFF7,
    TBLPTRU = 0xFF8,
    PCL = 0xFF9,
    PCLATH = 0xFFA,
    PCLATU = 0xFFB,
    STKPTR = 0xFFC,
    TOSL = 0xFFD,
    TOSH = 0xFFE,
    TOSU = 0xFFF,
};

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
    reg_rcon_mask_IPEN = 1 << 7,
};

enum reg_stkptr_mask_t
{
    reg_stkptr_mask_SP = 0b00011111,
    reg_stkptr_mask_STKUNF = 1 << 6,
    reg_stkptr_mask_STKOVF = 1 << 7,
};

enum reg_intcon_mask_t
{
    reg_intcon_mask_RBIF = 1 << 0,
    reg_intcon_mask_INT0IF = 1 << 1,
    reg_intcon_mask_TMR0IF = 1 << 2,
    reg_intcon_mask_RBIE = 1 << 3,
    reg_intcon_mask_INT0IE = 1 << 4,
    reg_intcon_mask_TMR0IE = 1 << 5,
    reg_intcon_mask_PEIE_GIEL = 1 << 6,
    reg_intcon_mask_GIE_GIEH = 1 << 7,
};

enum reg_intcon2_mask_t
{
    reg_intcon2_mask_RBIP = 1 << 0,
    reg_intcon2_mask_TMR0IP = 1 << 2,
    reg_intcon2_mask_INTEDG2 = 1 << 4,
    reg_intcon2_mask_INTEDG1 = 1 << 5,
    reg_intcon2_mask_INTEDG0 = 1 << 6,
    reg_intcon2_mask_RBPU = 1 << 7,
};

enum reg_intcon3_mask_t
{
    reg_intcon3_mask_INT1IF = 1 << 0,
    reg_intcon3_mask_INT2IF = 1 << 1,
    reg_intcon3_mask_INT1IE = 1 << 3,
    reg_intcon3_mask_INT2IE = 1 << 4,
    reg_intcon3_mask_INT1IP = 1 << 6,
    reg_intcon3_mask_INT2IP = 1 << 7,
};

enum reg_pir1_mask_t
{
    reg_pir1_mask_TMR1IF = 1 << 0,
    reg_pir1_mask_TMR2IF = 1 << 1,
    reg_pir1_mask_CCP1IF = 1 << 2,
    reg_pir1_mask_SSPIF = 1 << 3,
    reg_pir1_mask_TXIF = 1 << 4,
    reg_pir1_mask_RCIF = 1 << 5,
    reg_pir1_mask_ADIF = 1 << 6,
    reg_pir1_mask_PSPIF = 1 << 7,
};

enum reg_pir2_mask_t
{
    reg_pir2_mask_CCP2IF = 1 << 0,
    reg_pir2_mask_TMR3IF = 1 << 1,
    reg_pir2_mask_LVDIF = 1 << 2,
    reg_pir2_mask_BCLIF = 1 << 3,
    reg_pir2_mask_EEIF = 1 << 4,
};

enum reg_pie1_mask_t
{
    reg_pie1_mask_TMR1IE = 1 << 0,
    reg_pie1_mask_TMR2IE = 1 << 1,
    reg_pie1_mask_CCP1IE = 1 << 2,
    reg_pie1_mask_SSPIE = 1 << 3,
    reg_pie1_mask_TXIE = 1 << 4,
    reg_pie1_mask_RCIE = 1 << 5,
    reg_pie1_mask_ADIE = 1 << 6,
    reg_pie1_mask_PSPIE = 1 << 7,
};

enum reg_pie2_mask_t
{
    reg_pie2_mask_CCP2IE = 1 << 0,
    reg_pie2_mask_TMR3IE = 1 << 1,
    reg_pie2_mask_LVDIE = 1 << 2,
    reg_pie2_mask_BCLIE = 1 << 3,
    reg_pie2_mask_EEIE = 1 << 4,
};

enum reg_ipr1_mask_t
{
    reg_ipr1_mask_TMR1IP = 1 << 0,
    reg_ipr1_mask_TMR2IP = 1 << 1,
    reg_ipr1_mask_CCP1IP = 1 << 2,
    reg_ipr1_mask_SSPIP = 1 << 3,
    reg_ipr1_mask_TXIP = 1 << 4,
    reg_ipr1_mask_RCIP = 1 << 5,
    reg_ipr1_mask_ADIP = 1 << 6,
    reg_ipr1_mask_PSPIP = 1 << 7,
};

enum reg_ipr2_mask_t
{
    reg_ipr2_mask_CCP2IP = 1 << 0,
    reg_ipr2_mask_TMR3IP = 1 << 1,
    reg_ipr2_mask_LVDIP = 1 << 2,
    reg_ipr2_mask_BCLIP = 1 << 3,
    reg_ipr2_mask_EEIP = 1 << 4,
};
