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
    std::cerr << "_W: 0x" << std::setfill('0') << std::setw(4) << std::hex << address << " <- 0x" << std::setw(2)
              << std::hex << static_cast<int>(val) << "\n";
}

void env_bus_read(uint16_t address, uint8_t val)
{
    std::cerr << "R_: 0x" << std::setfill('0') << std::setw(4) << std::hex << address << " -> 0x" << std::setw(2)
              << std::hex << static_cast<int>(val) << "\n";
}
