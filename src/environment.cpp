#include "environment.hpp"

#include <iomanip>
#include <iostream>

const char *instruction_names[] = {
    "ILLEGAL", "ADDWF",  "ADDWFC",  "ANDWF",  "CLRF",   "COMF",  "CPFSEQ", "CPFSGT", "CPFSLT", "DECF",
    "DECFSZ",  "DCFSNZ", "INCF",    "INCFSZ", "INFSNZ", "IORWF", "MOVF",   "MOVFF",  "MOVWF",  "MULWF",
    "NEGF",    "RLCF",   "RLNCF",   "RRCF",   "RRNCF",  "SETF",  "SUBFWB", "SUBWF",  "SUBWFB", "SWAPF",
    "TSTFSZ",  "XORWF",  "BCF",     "BSF",    "BTFSC",  "BTFSS", "BTG",    "BC",     "BN",     "BNC",
    "BNN",     "BNOV",   "BNZ",     "BOV",    "BRA",    "BZ",    "CALL",   "CLRWDT", "DAW",    "GOTO",
    "NOP",     "NOP1",   "POP",     "PUSH",   "RCALL",  "RESET", "RETFIE", "RETLW",  "RETURN", "SLEEP",
    "ADDLW",   "ANDLW",  "IORLW",   "LFSR",   "MOVLB",  "MOVLW", "MULLW",  "SUBLW",  "XORLW",  "TBLRD",
    "TBLWT",   "ADDFSR", "ADDULNK", "CALLW",  "MOVSF",  "MOVSS", "PUSHL",  "SUBFSR", "SUBULNK"};

const char *register_names[] = {
    "TOSU",       "TOSH",        "TOSL",      "STKPTR",    "PCLATU",    "PCLATH",    "PCL",       "TBLPTRU",
    "TBLPTRH",    "TBLPTRL",     "TABLAT",    "PRODH",     "PRODL",     "INTCON",    "INTCON2",   "INTCON3",
    "INDF0",      "POSTINC0",    "POSTDEC0",  "PREINC0",   "PLUSW0",    "FSR0H",     "FSR0L",     "WREG",
    "INDF1",      "POSTINC1",    "POSTDEC1",  "PREINC1",   "PLUSW1",    "FSR1H",     "FSR1L",     "BSR",
    "INDF2",      "POSTINC2",    "POSTDEC2",  "PREINC2",   "PLUSW2",    "FSR2H",     "FSR2L",     "STATUS",
    "TMR0H",      "TMR0L",       "T0CON",     "OSCCON",    "OSCCON2",   "WDTCON",    "RCON",      "TMR1H",
    "TMR1L",      "T1CON",       "TMR2",      "PR2",       "T2CON",     "SSPBUF",    "SSPMSK",    "SSPADD",
    "SSPSTAT",    "SSPCON1",     "SSPCON2",   "ADRESH",    "ADRESL",    "ADCON0",    "ADCON1",    "ADCON2",
    "ECCP1AS",    "ECCP1DEL",    "CCPR1H",    "CCPR1L",    "CCP1CON",   "TXSTA2",    "BAUDCON2",  "IPR4",
    "PIR4",       "PIE4",        "CVRCON",    "CMSTAT",    "TMR3H",     "TMR3L",     "T3CON",     "T3GCON",
    "SPBRG1",     "RCREG1",      "TXREG1",    "TXSTA1",    "RCSTA1",    "T1GCON",    "PR4",       "HLVDCON",
    "BAUDCON1",   "RCSTA2",      "IPR3",      "PIR3",      "PIE3",      "IPR2",      "PIR2",      "PIE2",
    "IPR1",       "PIR1",        "PIE1",      "PSTR1CON",  "OSCTUNE",   "REFOCON",   "CCPTMRS",   "TRISG",
    "TRISF",      "TRISE",       "TRISD",     "TRISC",     "TRISB",     "TRISA",     "ODCON",     "SLRCON",
    "LATG",       "LATF",        "LATE",      "LATD",      "LATC",      "LATB",      "LATA",      "T4CON",
    "TMR4",       "PORTG",       "PORTF",     "PORTE",     "PORTD",     "PORTC",     "PORTB",     "PORTA",
    "EECON1",     "EECON2",      "SPBRGH1",   "SPBRGH2",   "SPBRG2",    "RCREG2",    "TXREG2",    "IPR5",
    "PIR5",       "PIE5",        "EEADRH",    "EEADR",     "EEDATA",    "ECANCON",   "COMSTAT",   "CIOCON",
    "CANCON",     "CANSTAT",     "RXB0D7",    "RXB0D6",    "RXB0D5",    "RXB0D4",    "RXB0D3",    "RXB0D2",
    "RXB0D1",     "RXB0D0",      "RXB0DLC",   "RXB0EIDL",  "RXB0EIDH",  "RXB0SIDL",  "RXB0SIDH",  "RXB0CON",
    "CM1CON",     "CM2CON",      "ANCON0",    "ANCON1",    "WPUB",      "IOCB",      "PMD0",      "PMD1",
    "PMD2",       "PADCFG1",     "CTMUCONH",  "CTMUCONL",  "CTMUICONH", "CCPR2H",    "CCPR2L",    "CCP2CON",
    "CCPR3H",     "CCPR3L",      "CCP3CON",   "CCPR4H",    "CCPR4L",    "CCP4CON",   "CCPR5H",    "CCPR5L",
    "CCP5CON",    "PSPCON",      "MDCON",     "MDSRC",     "MDCARH",    "MDCARL",    "XXXX",      "XXXX",
    "CANCON_RO0", "CANSTAT_RO0", "RXB1D7",    "RXB1D6",    "RXB1D5",    "RXB1D4",    "RXB1D3",    "RXB1D2",
    "RXB1D1",     "RXB1D0",      "RXB1DLC",   "RXB1EIDL",  "RXB1EIDH",  "RXB1SIDL",  "RXB1SIDH",  "RXB1CON",
    "CANCON_RO1", "CANSTAT_RO1", "TXB0D7",    "TXB0D6",    "TXB0D5",    "TXB0D4",    "TXB0D3",    "TXB0D2",
    "TXB0D1",     "TXB0D0",      "TXB0DLC",   "TXB0EIDL",  "TXB0EIDH",  "TXB0SIDL",  "TXB0SIDH",  "TXB0CON",
    "CANCON_RO2", "CANSTAT_RO2", "TXB1D7",    "TXB1D6",    "TXB1D5",    "TXB1D4",    "TXB1D3",    "TXB1D2",
    "TXB1D1",     "TXB1D0",      "TXB1DLC",   "TXB1EIDL",  "TXB1EIDH",  "TXB1SIDL",  "TXB1SIDH",  "TXB1CON",
    "CANCON_RO3", "CANSTAT_RO3", "TXB2D7",    "TXB2D6",    "TXB2D5",    "TXB2D4",    "TXB2D3",    "TXB2D2",
    "TXB2D1",     "TXB2D0",      "TXB2DLC",   "TXB2EIDL",  "TXB2EIDH",  "TXB2SIDL",  "TXB2SIDH",  "TXB2CON",
    "RXM1EIDL",   "RXM1EIDH",    "RXM1SIDL",  "RXM1SIDH",  "RXM0EIDL",  "RXM0EIDH",  "RXM0SIDL",  "RXM0SIDH",
    "RXF5EIDL",   "RXF5EIDH",    "RXF5SIDL",  "RXF5SIDH",  "RXF4EIDL",  "RXF4EIDH",  "RXF4SIDL",  "RXF4SIDH",
    "RXF3EIDL",   "RXF3EIDH",    "RXF3SIDL",  "RXF3SIDH",  "RXF2EIDL",  "RXF2EIDH",  "RXF2SIDL",  "RXF2SIDH",
    "RXF1EIDL",   "RXF1EIDH",    "RXF1SIDL",  "RXF1SIDH",  "RXF0EIDL",  "RXF0EIDH",  "RXF0SIDL",  "RXF0SIDH",
    "CANCON_RO4", "CANSTAT_RO4", "B5D7",      "B5D6",      "B5D5",      "B5D4",      "B5D3",      "B5D2",
    "B5D1",       "B5D0",        "B5DLC",     "B5EIDL",    "B5EIDH",    "B5SIDL",    "B5SIDH",    "B5CON",
    "CANCON_RO5", "CANSTAT_RO5", "B4D7",      "B4D6",      "B4D5",      "B4D4",      "B4D3",      "B4D2",
    "B4D1",       "B4D0",        "B4DLC",     "B4EIDL",    "B4EIDH",    "B4SIDL",    "B4SIDH",    "B4CON",
    "CANCON_RO6", "CANSTAT_RO6", "B3D7",      "B3D6",      "B3D5",      "B3D4",      "B3D3",      "B3D2",
    "B3D1",       "B3D0",        "B3DLC",     "B3EIDL",    "B3EIDH",    "B3SIDL",    "B3SIDH",    "B3CON",
    "CANCON_RO7", "CANSTAT_RO7", "B2D7",      "B2D6",      "B2D5",      "B2D4",      "B2D3",      "B2D2",
    "B2D1",       "B2D0",        "B2DLC",     "B2EIDL",    "B2EIDH",    "B2SIDL",    "B2SIDH",    "B2CON",
    "CANCON_RO8", "CANSTAT_RO8", "B1D7",      "B1D6",      "B1D5",      "B1D4",      "B1D3",      "B1D2",
    "B1D1",       "B1D0",        "B1DLC",     "B1EIDL",    "B1EIDH",    "B1SIDL",    "B1SIDH",    "B1CON",
    "CANCON_RO9", "CANSTAT_RO9", "B0D7",      "B0D6",      "B0D5",      "B0D4",      "B0D3",      "B0D2",
    "B0D1",       "B0D0",        "B0DLC",     "B0EIDL",    "B0EIDH",    "B0SIDL",    "B0SIDH",    "B0CON",
    "TXBIE",      "BIE0",        "BSEL0",     "MSEL3",     "MSEL2",     "MSEL1",     "MSEL0",     "RXFBCON7",
    "RXFBCON6",   "RXFBCON5",    "RXFBCON4",  "RXFBCON3",  "RXFBCON2",  "RXFBCON1",  "RXFBCON0",  "SDFLC",
    "RXF15EIDL",  "RXF15EIDH",   "RXF15SIDL", "RXF15SIDH", "RXF14EIDL", "RXF14EIDH", "RXF14SIDL", "RXF14SIDH",
    "RXF13EIDL",  "RXF13EIDH",   "RXF13SIDL", "RXF13SIDH", "RXF12EIDL", "RXF12EIDH", "RXF12SIDL", "RXF12SIDH",
    "RXF11EIDL",  "RXF11EIDH",   "RXF11SIDL", "RXF11SIDH", "RXF10EIDL", "RXF10EIDH", "RXF10SIDL", "RXF10SIDH",
    "RXF9EIDL",   "RXF9EIDH",    "RXF9SIDL",  "RXF9SIDH",  "RXF8EIDL",  "RXF8EIDH",  "RXF8SIDL",  "RXF8SIDH",
    "RXF7EIDL",   "RXF7EIDH",    "RXF7SIDL",  "RXF7SIDH",  "RXF6EIDL",  "RXF6EIDH",  "RXF6SIDL",  "RXF6SIDH",
    "RXFCON1",    "RXFCON0",     "BRGCON3",   "BRGCON2",   "BRGCON1",   "TXERRCNT",  "RXERRCNT",
};

const int register_names_max = 0xFFF;
const int register_names_size = sizeof(register_names) / sizeof(*register_names);

void env_init()
{
}

void env_cpu_current_instruction(decode_result_t instruction, uint32_t pc)
{
    std::cerr << "CPU: 0x" << std::setfill('0') << std::setw(8) << std::hex << pc << "  "
              << instruction_names[static_cast<int>(instruction.opcode)] << "\n";
}

void env_bus_write(uint16_t address, uint8_t val)
{
    if (address <= register_names_max && address > register_names_max - register_names_size)
    {
        std::cerr << "W: " << register_names[register_names_max - address] << " <- 0x" << std::setw(2) << std::hex
                  << static_cast<int>(val) << "\n";
    }
    else
    {
        std::cerr << "W: 0x" << std::setfill('0') << std::setw(4) << std::hex << address << " <- 0x" << std::setw(2)
                  << std::hex << static_cast<int>(val) << "\n";
    }
}

void env_bus_read(uint16_t address, uint8_t val)
{
    if (address <= register_names_max && address > register_names_max - register_names_size)
    {
        std::cerr << "R: " << register_names[register_names_max - address] << " -> 0x" << std::setw(2) << std::hex
                  << static_cast<int>(val) << "\n";
    }
    else
    {
        std::cerr << "R: 0x" << std::setfill('0') << std::setw(4) << std::hex << address << " -> 0x" << std::setw(2)
                  << std::hex << static_cast<int>(val) << "\n";
    }
}
