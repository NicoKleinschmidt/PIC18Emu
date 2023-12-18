#pragma once

#include <bus.hpp>
#include <cstdint>
#include <functional>

enum class opcode_t : uint8_t
{
    ILLEGAL = 0,
    ADDWF,
    ADDWFC,
    ANDWF,
    CLRF,
    COMF,
    CPFSEQ,
    CPFSGT,
    CPFSLT,
    DECF,
    DECFSZ,
    DCFSNZ,
    INCF,
    INCFSZ,
    INFSNZ,
    IORWF,
    MOVF,
    MOVFF,
    MOVWF,
    MULWF,
    NEGF,
    RLCF,
    RLNCF,
    RRCF,
    RRNCF,
    SETF,
    SUBFWB,
    SUBWF,
    SUBWFB,
    SWAPF,
    TSTFSZ,
    XORWF,
    BCF,
    BSF,
    BTFSC,
    BTFSS,
    BTG,
    BC,
    BN,
    BNC,
    BNN,
    BNOV,
    BNZ,
    BOV,
    BRA,
    BZ,
    CALL,
    CLRWDT,
    DAW,
    GOTO,
    NOP,
    // NOP1 is executed the same as NOP.
    // This instruction is a result of executing the second word in a 2 word instruction.
    NOP1,
    POP,
    PUSH,
    RCALL,
    RESET,
    RETFIE,
    RETLW,
    RETURN,
    SLEEP,
    ADDLW,
    ANDLW,
    IORLW,
    LFSR,
    MOVLB,
    MOVLW,
    MULLW,
    SUBLW,
    XORLW,
    TBLRD,
    TBLWT,
    ADDFSR,
    ADDULNK,
    CALLW,
    MOVSF,
    MOVSS,
    PUSHL,
    SUBFSR,
    SUBULNK,
};

struct instruction_t
{
    union {
        struct
        {
            uint16_t f : 8;
            uint16_t a : 1;
            uint16_t d : 1;
            uint16_t opcode : 6;
        } byte_oriented;
        struct
        {
            uint16_t f_source : 12;
            uint16_t opcode : 4;
        } byte_to_byte_high;
        struct
        {
            uint16_t f_dest : 12;
            uint16_t : 4;
        } byte_to_byte_low;
        struct
        {
            uint16_t f : 8;
            uint16_t a : 1;
            uint16_t bit : 3;
            uint16_t opcode : 4;
        } bit_oriented;
        struct
        {
            uint16_t k : 8;
            uint16_t opcode : 8;
        } literal;
        struct
        {
            uint16_t literal : 8;
            uint16_t opcode : 8;
        } control_goto_high;
        struct
        {
            uint16_t literal : 12;
            uint16_t : 4;
        } control_goto_low;
        struct
        {
            uint16_t literal : 8;
            uint16_t s : 1;
            uint16_t opcode : 7;
        } control_call_high;
        struct
        {
            uint16_t literal : 12;
            uint16_t : 4;
        } control_call_low;
        struct
        {
            uint16_t literal : 11;
            uint16_t opcode : 5;
        } control_branch;
        struct
        {
            uint16_t literal : 8;
            uint16_t opcode : 8;
        } control_branch_status;
        struct
        {
            uint16_t s : 1;
            uint16_t opcode : 15;
        } control_return;
        struct
        {
            uint16_t k : 4;
            uint16_t f : 2;
            uint16_t opcode : 12;
        } load_fsr_high;
        struct
        {
            uint16_t k : 8;
            uint16_t opcode : 8;
        } load_fsr_low;

        struct
        {
            uint16_t n : 2;
            uint16_t opcode : 16;
        } table_op;

        struct
        {
            uint16_t k : 6;
            uint16_t f : 2;
            uint16_t opcode : 8;
        } xinst_fsr;

        struct
        {
            uint16_t z : 7;
            uint16_t opcode : 9;
        } xinst_movsf_high_movss;

        struct
        {
            uint16_t f : 12;
            uint16_t : 4;
        } xinst_movsf_low;
    };
};

struct cpu_t
{
    uint16_t fetched_instruction;

    /// If an operation requires multiple clock cycles, it sets the next_action function.
    /// This will cause the function to be executed instead of the next instruction.
    /// This value is automatically cleared.
    /// The called function may set next_action function again.
    std::function<void(cpu_t &cpu, instruction_t &instruction)> next_action;

    std::array<uint32_t, 31> hw_stack;
    uint8_t ws;
    uint8_t statuss;
    uint8_t bsrs;

    /// @brief When returning from an ISR, we need to remember if it was high or low priority,
    ///        so we can reenable the correct global interrupt enable bit.
    ///        This is either GIEH (high prio) or GIEL (low prio).
    ///        If interrupt priority isn't enabled, all interrupts are treated as high priority.
    ///
    /// @see PIC18FXX2 Datasheet 8.0 Interrupts
    bool is_high_priority_interrupt;
};

struct cpu_known_sfrs_t
{
    uint16_t WREG;
    uint16_t STATUS;
    uint16_t BSR;
    uint16_t PCL;
    uint16_t PCLATH;
    uint16_t PCLATU;
    uint16_t STKPTR;
    uint16_t TOSL;
    uint16_t TOSH;
    uint16_t TOSU;
    uint16_t TBLPTRL;
    uint16_t TBLPTRH;
    uint16_t TBLPTRU;
    uint16_t TABLAT;
    uint16_t RCON;
    uint16_t INTCON;
    uint16_t PRODL;
    uint16_t PRODH;
    uint16_t FSR0L;
    uint16_t FSR1L;
    uint16_t FSR2L;
};

struct decode_result_t
{
    instruction_t instruction;
    opcode_t opcode;
};

enum class cpu_event_t
{
    illegal_instruction,
    sleep,
    reset_wdt,
};

void cpu_tick(cpu_t &cpu, const cpu_known_sfrs_t &regs, bus_reader_t<uint32_t, uint8_t> read_prog_bus,
              bus_writer_t<uint32_t, uint8_t> write_prog_bus, bus_reader_t<uint16_t, uint8_t> read_data_bus,
              bus_writer_t<uint16_t, uint8_t> write_data_bus, std::function<void(cpu_event_t e)> event_handler);

void cpu_reset_por(cpu_t &cpu, bus_reader_t<uint16_t, uint8_t> read_data_bus,
                   bus_writer_t<uint16_t, uint8_t> write_data_bus);

void cpu_reset_mclr(cpu_t &cpu, bus_reader_t<uint16_t, uint8_t> read_data_bus,
                    bus_writer_t<uint16_t, uint8_t> write_data_bus);

decode_result_t cpu_decode(uint16_t instruction);

bool cpu_stack_push(cpu_t &cpu, const cpu_known_sfrs_t &regs, uint32_t value,
                    bus_reader_t<uint32_t, uint8_t> read_prog_bus, bus_reader_t<uint16_t, uint8_t> read_data_bus,
                    bus_writer_t<uint16_t, uint8_t> write_data_bus);

bool cpu_stack_pop(cpu_t &cpu, const cpu_known_sfrs_t &regs, bus_reader_t<uint32_t, uint8_t> read_prog_bus,
                   bus_reader_t<uint16_t, uint8_t> read_data_bus, bus_writer_t<uint16_t, uint8_t> write_data_bus);

uint32_t cpu_stack_top(cpu_t &cpu, const cpu_known_sfrs_t &regs, bus_reader_t<uint16_t, uint8_t> read_data_bus);

/// @brief Causes the cpu to vector to the specifed address.
///        The return address is pushed onto the stack
///        and registers are pushed to the fast register stack.
///        No interrupt specific registers are checked or updated.
/// @param cpu
/// @param read_prog_bus
/// @param read_data_bus
/// @param write_data_bus
/// @param high_priority Wether or not a high priority interrupt was requested.
void cpu_interrupt_vector(cpu_t &cpu, const cpu_known_sfrs_t &regs, bus_reader_t<uint32_t, uint8_t> read_prog_bus,
                          bus_reader_t<uint16_t, uint8_t> read_data_bus, bus_writer_t<uint16_t, uint8_t> write_data_bus,
                          bool high_priority);

uint32_t cpu_read_pc(const cpu_known_sfrs_t &regs, bus_reader_t<uint16_t, uint8_t> read_data_bus);
void cpu_write_pc(const cpu_known_sfrs_t &regs, bus_writer_t<uint16_t, uint8_t> write_data_bus, uint32_t pc);

void test_2s_complement();
