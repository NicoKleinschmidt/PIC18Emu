#include "cpu.hpp"
#include "18f66k80.hpp"
#include "alu.hpp"
#include "reg.hpp"
#include <environment.hpp>
#include <iostream>

/// @brief Converts an 8bit 2's complement encoded signed number to the host platforms native signed 8 bit
static int8_t from_2s_complement_8(uint8_t val)
{
    return *reinterpret_cast<int8_t *>(&val);
}

/// @brief Converts an 11bit 2's complement encoded signed number to the host platforms native signed 16 bit
static int16_t from_2s_complement_11(uint16_t val)
{
    return static_cast<int16_t>(val) - ((val & (1 << 10)) << 1);
}

void test_2s_complement()
{
    uint16_t minus_one = 0b11111111111;
    int16_t result = from_2s_complement_11(minus_one);
    std::cout << "-1 = " << result << "\n";

    uint16_t one = 0b1;
    result = from_2s_complement_11(one);
    std::cout << "1 = " << result << "\n";

    uint16_t minus_two = 0b11111111110;
    result = from_2s_complement_11(minus_two);
    std::cout << "-2 = " << result << "\n";

    uint16_t two = 0b10;
    result = from_2s_complement_11(two);
    std::cout << "2 = " << result << "\n";
}

/// @brief Reads a value from the bank specified by the BSR register or the access bank.
/// @param location The 8 bit address of the value in the bank.
/// @param use_bsr If false, the value will be read from the access bank instead of the one specified by BSR
static uint8_t cpu_read_from_bank(const cpu_known_sfrs_t &sfr, bus_reader_t<uint16_t, uint8_t> read_bus,
                                  uint8_t location, bool use_bsr)
{
    if (!use_bsr)
    {
        if (location < 0x80)
            return read_bus(location);
        else
            return read_bus(0xF00 + location);
    }

    uint8_t bank = read_bus(static_cast<uint16_t>(sfr.BSR));
    uint16_t addr = (static_cast<uint16_t>(bank) << 8) | static_cast<uint16_t>(location);
    return read_bus(addr);
}

/// @brief Writes a value to the bank specified by the BSR register or the access bank.
/// @param location The 8 bit address of the value in the bank.
/// @param use_bsr If false, the value will be written to the access bank instead of the one specified by BSR
static void cpu_write_to_bank(const cpu_known_sfrs_t &sfr, bus_writer_t<uint16_t, uint8_t> write_bus,
                              bus_reader_t<uint16_t, uint8_t> read_bus, uint8_t location, uint8_t val, bool use_bsr)
{
    if (!use_bsr)
    {
        if (location < 0x80)
            return write_bus(location, val);
        else
            return write_bus(0xF00 + location, val);
    }

    uint8_t bank = read_bus(static_cast<uint16_t>(sfr.BSR));
    uint16_t addr = (static_cast<uint16_t>(bank) << 8) | static_cast<uint16_t>(location);
    return write_bus(addr, val);
}

void cpu_tick(cpu_t &cpu, const cpu_known_sfrs_t &sfr, bus_reader_t<uint32_t, uint8_t> read_prog_bus,
              bus_writer_t<uint32_t, uint8_t> write_prog_bus, bus_reader_t<uint16_t, uint8_t> read_data_bus,
              bus_writer_t<uint16_t, uint8_t> write_data_bus, std::function<void(cpu_event_t e)> event_handler)
{
    // Get the current value of common registers, will be written back
    // once the instruction has been executed.
    uint8_t status = read_data_bus(static_cast<uint16_t>(sfr.STATUS));
    uint8_t w = read_data_bus(static_cast<uint16_t>(sfr.WREG));
    uint32_t pc = cpu_read_pc(sfr, read_data_bus);

    decode_result_t decoded = cpu_decode(cpu.fetched_instruction);
    instruction_t instruction = decoded.instruction;

    uint8_t fetched_low = read_prog_bus(pc++);
    uint8_t fetched_high = read_prog_bus(pc++);
    cpu.fetched_instruction = (static_cast<uint16_t>(fetched_high) << 8) | fetched_low;

    // Decode the next instruction for debugging.
    decode_result_t debug_next_instruction = cpu_decode(cpu.fetched_instruction);

    bool xinst_enabled = reg_check_configuration_bit(reg_configuration_bit_t::XINST, read_prog_bus);

    if (cpu.next_action != nullptr)
    {
        // We need to clear the next action function before calling it,
        // because the function could set it again internally.
        auto tmp = cpu.next_action;
        cpu.next_action = nullptr;
        tmp(cpu, instruction);

        return;
    }

    env_cpu_current_instruction(decoded, pc);

    if (decoded.opcode == opcode_t::ILLEGAL)
    {
        if (event_handler != nullptr)
            event_handler(cpu_event_t::illegal_instruction);
        return;
    }

    switch (decoded.opcode)
    {
    case opcode_t::ADDFSR: {
        if (!xinst_enabled)
            break;

        uint16_t fsrl_offset;
        if (instruction.xinst_fsr.f == 0)
            fsrl_offset = static_cast<uint16_t>(sfr.FSR0L);
        else if (instruction.xinst_fsr.f == 1)
            fsrl_offset = static_cast<uint16_t>(sfr.FSR1L);
        else
            fsrl_offset = static_cast<uint16_t>(sfr.FSR2L);

        uint8_t low = read_data_bus(fsrl_offset);
        uint8_t high = read_data_bus(fsrl_offset + 1);
        uint16_t value = (static_cast<uint16_t>(high) << 8) | low;
        value += instruction.xinst_fsr.k;
        write_data_bus(fsrl_offset, value & 0xFF);
        write_data_bus(fsrl_offset + 1, (value >> 8) & 0xFF);

        if (instruction.xinst_fsr.f == 3)
        {
            w = instruction.literal.k;
            pc = cpu_stack_top(cpu, sfr, read_data_bus);
            if (!cpu_stack_pop(cpu, sfr, read_prog_bus, read_data_bus, write_data_bus))
                return;

            cpu.fetched_instruction = 0x0000; // NOP
        }

        break;
    }

    case opcode_t::ADDLW: {
        w = alu_add(w, instruction.literal.k, status);
        break;
    }

    case opcode_t::ADDWF: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        uint8_t result = alu_add(w, val, status);

        if (instruction.byte_oriented.d)
            cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                              instruction.byte_oriented.a);
        else
            w = result;
        break;
    }

    case opcode_t::ADDWFC: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        uint8_t result = alu_add(w, val, (status & alu_status_C), status);

        if (instruction.byte_oriented.d)
            cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                              instruction.byte_oriented.a);
        else
            w = result;
        break;
    }

    case opcode_t::ANDLW: {
        w = alu_and(w, instruction.literal.k, status);
        break;
    }

    case opcode_t::ANDWF: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        uint8_t result = alu_and(w, val, status);

        if (instruction.byte_oriented.d)
            cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                              instruction.byte_oriented.a);
        else
            w = result;
        break;
    }

    case opcode_t::BC: {
        if (status & alu_status_C)
        {
            pc += from_2s_complement_8(instruction.control_branch_status.literal) * 2;
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::BCF: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.bit_oriented.f, instruction.bit_oriented.a);
        val &= ~(1 << instruction.bit_oriented.bit);
        cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.bit_oriented.f, val,
                          instruction.bit_oriented.a);
        break;
    }

    case opcode_t::BN: {
        if (status & alu_status_N)
        {
            pc += from_2s_complement_8(instruction.control_branch_status.literal) * 2;
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::BNC: {
        if (!(status & alu_status_C))
        {
            pc += from_2s_complement_8(instruction.control_branch_status.literal) * 2;
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::BNN: {
        if (!(status & alu_status_N))
        {
            pc += from_2s_complement_8(instruction.control_branch_status.literal) * 2;
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::BNOV: {
        if (!(status & alu_status_OV))
        {
            pc += from_2s_complement_8(instruction.control_branch_status.literal) * 2;
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::BNZ: {
        if (!(status & alu_status_Z))
        {
            pc += from_2s_complement_8(instruction.control_branch_status.literal) * 2;
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::BRA: {
        int16_t offset = from_2s_complement_11(instruction.control_branch.literal) * 2;
        pc += offset - 2;
        cpu.fetched_instruction = 0x0000; // NOP
        break;
    }

    case opcode_t::BSF: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.bit_oriented.f, instruction.bit_oriented.a);
        val |= (1 << instruction.bit_oriented.bit);
        cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.bit_oriented.f, val,
                          instruction.bit_oriented.a);
        break;
    }

    case opcode_t::BTFSC: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.bit_oriented.f, instruction.bit_oriented.a);
        bool is_set = val & (1 << instruction.bit_oriented.bit);
        if (!is_set)
        {
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::BTFSS: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.bit_oriented.f, instruction.bit_oriented.a);
        bool is_set = val & (1 << instruction.bit_oriented.bit);
        if (is_set)
        {
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::BTG: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.bit_oriented.f, instruction.bit_oriented.a);
        val ^= (1 << instruction.bit_oriented.bit);
        cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.bit_oriented.f, val,
                          instruction.bit_oriented.a);
        break;
    }

    case opcode_t::BOV: {
        if (status & alu_status_OV)
        {
            pc += from_2s_complement_8(instruction.control_branch_status.literal * 2);
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::BZ: {
        if (status & alu_status_Z)
        {
            pc += from_2s_complement_8(instruction.control_branch_status.literal * 2);
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::CALL: {
        if (!cpu_stack_push(cpu, sfr, pc, read_prog_bus, read_data_bus, write_data_bus))
        {
            return;
        }

        instruction_t next_instruction = *reinterpret_cast<instruction_t *>(&cpu.fetched_instruction);
        pc = (static_cast<uint32_t>(next_instruction.control_call_low.literal << 8) |
              static_cast<uint32_t>(instruction.control_call_high.literal))
             << 1;

        if (instruction.control_call_high.s)
        {
            cpu.ws = w;
            cpu.statuss = status;
            cpu.bsrs = read_data_bus(static_cast<uint16_t>(sfr.BSR));
        }

        cpu.fetched_instruction = 0x0000; // NOP
        break;
    }

    case opcode_t::CALLW: {
        if (!xinst_enabled)
            break;

        if (!cpu_stack_push(cpu, sfr, pc, read_prog_bus, read_data_bus, write_data_bus))
            return;

        pc &= ~0xFF;
        pc |= w;

        cpu.fetched_instruction = 0x0000; // NOP
        break;
    }

    case opcode_t::CLRF: {
        (void)cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, 0,
                          instruction.byte_oriented.a);
        status |= alu_status_Z;
        break;
    }

    case opcode_t::CLRWDT: {
        reg_sfr_write(read_data_bus, write_data_bus, sfr.RCON, reg_rcon_mask_TO, 1);
        reg_sfr_write(read_data_bus, write_data_bus, sfr.RCON, reg_rcon_mask_PD, 1);
        event_handler(cpu_event_t::reset_wdt);
        break;
    }

    case opcode_t::COMF: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        uint8_t result = alu_complement(val, status);

        if (instruction.byte_oriented.d)
            cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                              instruction.byte_oriented.a);
        else
            w = result;
        break;
    }

    case opcode_t::CPFSEQ: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        if (val == w)
        {
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::CPFSGT: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        if (val > w)
        {
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::CPFSLT: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        if (val < w)
        {
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::DAW: {
        uint8_t result = alu_decimal_adjust(w, status);
        w = result;
        break;
    }

    case opcode_t::DECF: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        uint8_t result = alu_sub(val, 1, status);

        if (instruction.byte_oriented.d)
            cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                              instruction.byte_oriented.a);
        else
            w = result;
        break;
    }

    case opcode_t::DECFSZ: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        uint8_t result = val - 1;

        if (instruction.byte_oriented.d)
            cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                              instruction.byte_oriented.a);
        else
            w = result;

        if (result == 0)
        {
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::DCFSNZ: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        uint8_t result = val - 1;

        if (instruction.byte_oriented.d)
            cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                              instruction.byte_oriented.a);
        else
            w = result;

        if (result != 0)
        {
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::GOTO: {
        instruction_t next_instruction = *reinterpret_cast<instruction_t *>(&cpu.fetched_instruction);
        cpu.fetched_instruction = 0x0000; // NOP
        uint32_t address = ((static_cast<uint32_t>(next_instruction.control_goto_low.literal) << 8) |
                            instruction.control_goto_high.literal)
                           << 1;
        pc = address;
        break;
    }

    case opcode_t::INCF: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        uint8_t result = alu_add(val, 1, status);

        if (instruction.byte_oriented.d)
            cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                              instruction.byte_oriented.a);
        else
            w = result;
        break;
    }

    case opcode_t::INCFSZ: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        uint8_t result = val + 1;

        if (instruction.byte_oriented.d)
            cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                              instruction.byte_oriented.a);
        else
            w = result;

        if (result == 0)
        {
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::INFSNZ: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        uint8_t result = val + 1;

        if (instruction.byte_oriented.d)
            cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                              instruction.byte_oriented.a);
        else
            w = result;

        if (result != 0)
        {
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::IORLW: {
        w = alu_or(w, instruction.literal.k, status);
        break;
    }

    case opcode_t::IORWF: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        uint8_t result = alu_or(w, val, status);

        if (instruction.byte_oriented.d)
            cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                              instruction.byte_oriented.a);
        else
            w = result;
        break;
    }

    case opcode_t::LFSR: {
        uint16_t fsrl_offset;
        if (instruction.load_fsr_high.f == 0)
            fsrl_offset = static_cast<uint16_t>(sfr.FSR0L);
        else if (instruction.load_fsr_high.f == 1)
            fsrl_offset = static_cast<uint16_t>(sfr.FSR1L);
        else
            fsrl_offset = static_cast<uint16_t>(sfr.FSR2L);

        write_data_bus(fsrl_offset + 1, instruction.load_fsr_high.k);
        cpu.next_action = [fsrl_offset, write_data_bus](cpu_t &cpu, instruction_t next_instruction) {
            write_data_bus(fsrl_offset, next_instruction.load_fsr_low.k);
        };
        break;
    }

    case opcode_t::MOVF: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        status &= ~alu_status_N;
        status &= ~alu_status_Z;

        if (val == 0)
            status |= alu_status_Z;
        if (val & 0b10000000)
            status |= alu_status_N;

        if (instruction.byte_oriented.d)
            cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, val,
                              instruction.byte_oriented.a);
        else
            w = val;
        break;
    }

    case opcode_t::MOVFF: {
        uint8_t val = read_data_bus(instruction.byte_to_byte_high.f_source);
        pc -= 2;
        cpu.next_action = [val, write_data_bus, sfr](cpu_t &cpu, instruction_t &instruction) {
            uint16_t dest = instruction.byte_to_byte_low.f_dest;
            if (dest == static_cast<uint16_t>(sfr.PCL) || dest == static_cast<uint16_t>(sfr.TOSU) ||
                dest == static_cast<uint16_t>(sfr.TOSH) || dest == static_cast<uint16_t>(sfr.TOSL))
            {
                return;
            }

            write_data_bus(dest, val);
        };
        break;
    }

    case opcode_t::MOVLB: {
        write_data_bus(static_cast<uint16_t>(sfr.BSR), instruction.literal.k & 0x0F);
        break;
    }

    case opcode_t::MOVLW: {
        w = instruction.literal.k;
        break;
    }

    case opcode_t::MOVSF: {
        if (!xinst_enabled)
            break;

        uint8_t fsr2l = read_data_bus(sfr.FSR2L);
        uint8_t fsr2h = read_data_bus(sfr.FSR2L + 1);
        uint16_t fsr2_val = (static_cast<uint16_t>(fsr2h) << 8) | static_cast<uint16_t>(fsr2l);
        uint8_t src_value = read_data_bus(fsr2_val + instruction.xinst_movsf_high_movss.z);
        cpu.next_action = [src_value, write_data_bus](cpu_t &cpu, instruction_t &instruction) {
            write_data_bus(instruction.xinst_movsf_low.f, src_value);
        };

        break;
    }

    case opcode_t::MOVSS: {
        if (!xinst_enabled)
            break;

        uint8_t fsr2l = read_data_bus(sfr.FSR2L);
        uint8_t fsr2h = read_data_bus(sfr.FSR2L + 1);
        uint16_t fsr2_val = (static_cast<uint16_t>(fsr2h) << 8) | static_cast<uint16_t>(fsr2l);
        uint8_t src_value = read_data_bus(fsr2_val + instruction.xinst_movsf_high_movss.z);
        cpu.next_action = [src_value, read_data_bus, write_data_bus, sfr](cpu_t &cpu, instruction_t &instruction) {
            uint8_t fsr2l = read_data_bus(sfr.FSR2L);
            uint8_t fsr2h = read_data_bus(sfr.FSR2L + 1);
            uint16_t fsr2_val = (static_cast<uint16_t>(fsr2h) << 8) | static_cast<uint16_t>(fsr2l);
            write_data_bus(fsr2_val + instruction.xinst_movsf_high_movss.z, src_value);
        };

        break;
    }

    case opcode_t::MOVWF: {
        cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, w,
                          instruction.byte_oriented.a);
        break;
    }

    case opcode_t::MULLW: {
        uint16_t result = static_cast<uint16_t>(w) * static_cast<uint16_t>(instruction.literal.k);
        write_data_bus(static_cast<uint16_t>(sfr.PRODH), (result >> 8) & 0xFF);
        write_data_bus(static_cast<uint16_t>(sfr.PRODL), result & 0xFF);
        break;
    }

    case opcode_t::MULWF: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        uint16_t result = static_cast<uint16_t>(w) * static_cast<uint16_t>(val);
        write_data_bus(static_cast<uint16_t>(sfr.PRODH), (result >> 8) & 0xFF);
        write_data_bus(static_cast<uint16_t>(sfr.PRODL), result & 0xFF);
        break;
    }

    case opcode_t::NEGF: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        uint8_t result = alu_negate(val, status);

        cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                          instruction.byte_oriented.a);
        break;
    }

    case opcode_t::NOP:
    case opcode_t::NOP1: {
        break;
    }

    case opcode_t::POP: {
        cpu_stack_pop(cpu, sfr, read_prog_bus, read_data_bus, write_data_bus);
        break;
    }

    case opcode_t::PUSH: {
        if (!cpu_stack_push(cpu, sfr, pc, read_prog_bus, read_data_bus, write_data_bus))
        {
            return;
        }
        break;
    }

    case opcode_t::PUSHL: {
        if (!xinst_enabled)
            break;

        uint8_t fsr2l = read_data_bus(sfr.FSR2L);
        uint8_t fsr2h = read_data_bus(sfr.FSR2L + 1);
        uint16_t fsr2_val = (static_cast<uint16_t>(fsr2h) << 8) | static_cast<uint16_t>(fsr2l);
        write_data_bus(fsr2_val, instruction.literal.k);
        fsr2_val--;
        write_data_bus(sfr.FSR2L, fsr2_val & 0xFF);
        write_data_bus(sfr.FSR2L + 1, (fsr2_val >> 8) & 0xFF);

        break;
    }

    case opcode_t::RCALL: {
        if (!cpu_stack_push(cpu, sfr, pc, read_prog_bus, read_data_bus, write_data_bus))
        {
            return;
        }

        pc += 2 + from_2s_complement_11(instruction.control_branch.literal) * 2;
        cpu.fetched_instruction = 0xFFFF; // NOP
        break;
    }

    case opcode_t::RESET: {
        cpu_reset_mclr(cpu, read_data_bus, write_data_bus);
        break;
    }

    case opcode_t::RETFIE: {
        pc = cpu_stack_top(cpu, sfr, read_data_bus);
        if (!cpu_stack_pop(cpu, sfr, read_prog_bus, read_data_bus, write_data_bus))
            return;

        if (cpu.is_high_priority_interrupt)
            reg_sfr_write(read_data_bus, write_data_bus, sfr.INTCON, reg_intcon_mask_GIE_GIEH, 1);
        else
            reg_sfr_write(read_data_bus, write_data_bus, sfr.INTCON, reg_intcon_mask_PEIE_GIEL, 1);

        if (instruction.control_return.s)
        {
            w = cpu.ws;
            status = cpu.statuss;
            write_data_bus(static_cast<uint16_t>(sfr.BSR), cpu.bsrs);
        }

        cpu.fetched_instruction = 0x0000; // NOP
        break;
    }

    case opcode_t::RETLW: {
        w = instruction.literal.k;
        pc = cpu_stack_top(cpu, sfr, read_data_bus);
        if (!cpu_stack_pop(cpu, sfr, read_prog_bus, read_data_bus, write_data_bus))
            return;

        cpu.fetched_instruction = 0x0000; // NOP
        break;
    }

    case opcode_t::RETURN: {
        pc = cpu_stack_top(cpu, sfr, read_data_bus);
        if (!cpu_stack_pop(cpu, sfr, read_prog_bus, read_data_bus, write_data_bus))
            return;

        if (instruction.control_return.s)
        {
            w = cpu.ws;
            status = cpu.statuss;
            write_data_bus(static_cast<uint16_t>(sfr.BSR), cpu.bsrs);
        }

        cpu.fetched_instruction = 0x0000; // NOP
        break;
    }

    case opcode_t::RLCF: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        uint8_t result = alu_rotate_left_carry(val, status);

        if (instruction.byte_oriented.d)
            cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                              instruction.byte_oriented.a);
        else
            w = result;
        break;
    }

    case opcode_t::RLNCF: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        uint8_t result = alu_rotate_left(val, status);

        if (instruction.byte_oriented.d)
            cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                              instruction.byte_oriented.a);
        else
            w = result;
        break;
    }

    case opcode_t::RRCF: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        uint8_t result = alu_rotate_right_carry(val, status);

        if (instruction.byte_oriented.d)
            cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                              instruction.byte_oriented.a);
        else
            w = result;
        break;
    }

    case opcode_t::RRNCF: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        uint8_t result = alu_rotate_right(val, status);

        if (instruction.byte_oriented.d)
            cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                              instruction.byte_oriented.a);
        else
            w = result;
        break;
    }

    case opcode_t::SETF: {
        (void)cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, 0xFF,
                          instruction.byte_oriented.a);
        break;
    }

    case opcode_t::SLEEP: {
        reg_sfr_write(read_data_bus, write_data_bus, sfr.RCON, reg_rcon_mask_TO, 1);
        reg_sfr_write(read_data_bus, write_data_bus, sfr.RCON, reg_rcon_mask_PD, 0);
        event_handler(cpu_event_t::reset_wdt);
        event_handler(cpu_event_t::sleep);
        break;
    }

    case opcode_t::SUBFSR: {
        if (!xinst_enabled)
            break;

        uint16_t fsrl_offset;
        if (instruction.xinst_fsr.k == 0)
            fsrl_offset = static_cast<uint16_t>(sfr.FSR0L);
        else if (instruction.xinst_fsr.k == 1)
            fsrl_offset = static_cast<uint16_t>(sfr.FSR1L);
        else
            fsrl_offset = static_cast<uint16_t>(sfr.FSR2L);

        uint8_t low = read_data_bus(fsrl_offset);
        uint8_t high = read_data_bus(fsrl_offset + 1);
        uint16_t value = (static_cast<uint16_t>(high) << 8) | low;
        value -= instruction.xinst_fsr.k;
        write_data_bus(fsrl_offset, value & 0xFF);
        write_data_bus(fsrl_offset + 1, (value >> 8) & 0xFF);

        if (instruction.xinst_fsr.f == 3)
        {
            w = instruction.literal.k;
            pc = cpu_stack_top(cpu, sfr, read_data_bus);
            if (!cpu_stack_pop(cpu, sfr, read_prog_bus, read_data_bus, write_data_bus))
                return;

            cpu.fetched_instruction = 0x0000; // NOP
        }

        break;
    }

    case opcode_t::SUBFWB: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        uint8_t result = alu_sub(w, val, !(status & alu_status_C), status);

        if (instruction.byte_oriented.d)
            cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                              instruction.byte_oriented.a);
        else
            w = result;
        break;
    }

    case opcode_t::SUBLW: {
        w = alu_sub(instruction.literal.k, w, status);
        break;
    }

    case opcode_t::SUBWF: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        uint8_t result = alu_sub(val, w, status);

        if (instruction.byte_oriented.d)
            cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                              instruction.byte_oriented.a);
        else
            w = result;
        break;
    }

    case opcode_t::SUBWFB: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        uint8_t result = alu_sub(val, w, !(status & alu_status_C), status);

        if (instruction.byte_oriented.d)
            cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                              instruction.byte_oriented.a);
        else
            w = result;
        break;
    }

    case opcode_t::SWAPF: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        uint8_t result = (val << 4) | (val >> 4);

        if (instruction.byte_oriented.d)
            cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                              instruction.byte_oriented.a);
        else
            w = result;
        break;
    }

    case opcode_t::TBLRD: {
        uint8_t tblptru = read_data_bus(static_cast<uint16_t>(sfr.TBLPTRU));
        uint8_t tblptrh = read_data_bus(static_cast<uint16_t>(sfr.TBLPTRH));
        uint8_t tblptrl = read_data_bus(static_cast<uint16_t>(sfr.TBLPTRL));
        uint32_t tblptr = (static_cast<uint32_t>(tblptru) << 16) | (static_cast<uint32_t>(tblptrh) << 8) |
                          static_cast<uint32_t>(tblptrl);

        if (instruction.table_op.n == 3)
            tblptr++;

        uint8_t val = read_prog_bus(tblptr);
        write_data_bus(static_cast<uint16_t>(sfr.TABLAT), val);

        if (instruction.table_op.n == 1)
            tblptr++;
        else if (instruction.table_op.n == 2)
            tblptr--;

        if (instruction.table_op.n != 0)
        {
            tblptru = static_cast<uint8_t>(tblptr >> 16);
            tblptrh = static_cast<uint8_t>(tblptr >> 8);
            tblptrl = static_cast<uint8_t>(tblptr);
            write_data_bus(static_cast<uint16_t>(sfr.TBLPTRU), tblptru);
            write_data_bus(static_cast<uint16_t>(sfr.TBLPTRH), tblptrh);
            write_data_bus(static_cast<uint16_t>(sfr.TBLPTRL), tblptrl);
        }
        // Wait one tick
        pc -= 2;
        cpu.next_action = [](cpu_t &, instruction_t &) {};
        break;
    }

    case opcode_t::TBLWT: {
        uint8_t tblptru = read_data_bus(static_cast<uint16_t>(sfr.TBLPTRU));
        uint8_t tblptrh = read_data_bus(static_cast<uint16_t>(sfr.TBLPTRH));
        uint8_t tblptrl = read_data_bus(static_cast<uint16_t>(sfr.TBLPTRL));
        uint32_t tblptr = (static_cast<uint32_t>(tblptru) << 16) | (static_cast<uint32_t>(tblptrh) << 8) |
                          static_cast<uint32_t>(tblptrl);

        if (instruction.table_op.n == 3)
            tblptr++;

        uint8_t val = read_data_bus(static_cast<uint16_t>(sfr.TABLAT));
        write_prog_bus(tblptr, val);

        if (instruction.table_op.n == 1)
            tblptr++;
        else if (instruction.table_op.n == 2)
            tblptr--;

        if (instruction.table_op.n != 0)
        {
            tblptru = static_cast<uint8_t>(tblptr >> 16);
            tblptrh = static_cast<uint8_t>(tblptr >> 8);
            tblptrl = static_cast<uint8_t>(tblptr);
            write_data_bus(static_cast<uint16_t>(sfr.TBLPTRU), tblptru);
            write_data_bus(static_cast<uint16_t>(sfr.TBLPTRH), tblptrh);
            write_data_bus(static_cast<uint16_t>(sfr.TBLPTRL), tblptrl);
        }
        // Wait one tick
        pc -= 2;
        cpu.next_action = [](cpu_t &, instruction_t &) {};
        break;
    }

    case opcode_t::TSTFSZ: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        if (val == 0)
        {
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::XORLW: {
        w = alu_xor(instruction.literal.k, w, status);
        break;
    }

    case opcode_t::XORWF: {
        uint8_t val = cpu_read_from_bank(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a);
        uint8_t result = alu_xor(w, val, status);

        if (instruction.byte_oriented.d)
            cpu_write_to_bank(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                              instruction.byte_oriented.a);
        else
            w = result;
        break;
    }

    default: break;
    }

    write_data_bus(static_cast<uint16_t>(sfr.STATUS), status);
    write_data_bus(static_cast<uint16_t>(sfr.WREG), w);
    cpu_write_pc(sfr, write_data_bus, pc);
}

void cpu_reset_por(cpu_t &cpu, bus_reader_t<uint16_t, uint8_t> read_data_bus,
                   bus_writer_t<uint16_t, uint8_t> write_data_bus)
{
    cpu.fetched_instruction = 0;
}

void cpu_reset_mclr(cpu_t &cpu, bus_reader_t<uint16_t, uint8_t> read_data_bus,
                    bus_writer_t<uint16_t, uint8_t> write_data_bus)
{
}

decode_result_t cpu_decode(uint16_t instruction)
{
    opcode_t op = opcode_t::ILLEGAL;

    if (0b0000000000000100 == (instruction & 0b1111111111111111))
        op = opcode_t::CLRWDT;
    else if (0b0000000000000111 == (instruction & 0b1111111111111111))
        op = opcode_t::DAW;
    else if (0b0000000000000000 == (instruction & 0b1111111111111111))
        op = opcode_t::NOP;
    else if (0b0000000000000110 == (instruction & 0b1111111111111111))
        op = opcode_t::POP;
    else if (0b0000000000000101 == (instruction & 0b1111111111111111))
        op = opcode_t::PUSH;
    else if (0b0000000011111111 == (instruction & 0b1111111111111111))
        op = opcode_t::RESET;
    else if (0b0000000000000011 == (instruction & 0b1111111111111111))
        op = opcode_t::SLEEP;
    else if (0b0000000000010100 == (instruction & 0b1111111111111111))
        op = opcode_t::CALLW;
    else if (0b0000000000010000 == (instruction & 0b1111111111111110))
        op = opcode_t::RETFIE;
    else if (0b0000000000010010 == (instruction & 0b1111111111111110))
        op = opcode_t::RETURN;
    else if (0b0000000000001000 == (instruction & 0b1111111111111100))
        op = opcode_t::TBLRD;
    else if (0b0000000000001100 == (instruction & 0b1111111111111100))
        op = opcode_t::TBLWT;
    else if (0b0000000100000000 == (instruction & 0b1111111111110000))
        op = opcode_t::MOVLB;
    else if (0b1110111000000000 == (instruction & 0b1111111111000000))
        op = opcode_t::LFSR;
    else if (0b1110101100000000 == (instruction & 0b1111111110000000))
        op = opcode_t::MOVSF;
    else if (0b1110101110000000 == (instruction & 0b1111111110000000))
        op = opcode_t::MOVSS;
    else if (0b1110001000000000 == (instruction & 0b1111111100000000))
        op = opcode_t::BC;
    else if (0b1110011000000000 == (instruction & 0b1111111100000000))
        op = opcode_t::BN;
    else if (0b1110001100000000 == (instruction & 0b1111111100000000))
        op = opcode_t::BNC;
    else if (0b1110011100000000 == (instruction & 0b1111111100000000))
        op = opcode_t::BNN;
    else if (0b1110010100000000 == (instruction & 0b1111111100000000))
        op = opcode_t::BNOV;
    else if (0b1110000100000000 == (instruction & 0b1111111100000000))
        op = opcode_t::BNZ;
    else if (0b1110010000000000 == (instruction & 0b1111111100000000))
        op = opcode_t::BOV;
    else if (0b1110000000000000 == (instruction & 0b1111111100000000))
        op = opcode_t::BZ;
    else if (0b1110111100000000 == (instruction & 0b1111111100000000))
        op = opcode_t::GOTO;
    else if (0b0000110000000000 == (instruction & 0b1111111100000000))
        op = opcode_t::RETLW;
    else if (0b0000111100000000 == (instruction & 0b1111111100000000))
        op = opcode_t::ADDLW;
    else if (0b0000101100000000 == (instruction & 0b1111111100000000))
        op = opcode_t::ANDLW;
    else if (0b0000100100000000 == (instruction & 0b1111111100000000))
        op = opcode_t::IORLW;
    else if (0b0000111000000000 == (instruction & 0b1111111100000000))
        op = opcode_t::MOVLW;
    else if (0b0000110100000000 == (instruction & 0b1111111100000000))
        op = opcode_t::MULLW;
    else if (0b0000110000000000 == (instruction & 0b1111111100000000))
        op = opcode_t::RETLW;
    else if (0b0000100000000000 == (instruction & 0b1111111100000000))
        op = opcode_t::SUBLW;
    else if (0b0000101000000000 == (instruction & 0b1111111100000000))
        op = opcode_t::XORLW;
    else if (0b1110100000000000 == (instruction & 0b1111111100000000))
        op = opcode_t::ADDFSR;
    else if (0b1110101000000000 == (instruction & 0b1111111100000000))
        op = opcode_t::PUSHL;
    else if (0b1110100100000000 == (instruction & 0b1111111100000000))
        op = opcode_t::SUBFSR;
    else if (0b0110001000000000 == (instruction & 0b1111111000000000))
        op = opcode_t::CPFSEQ;
    else if (0b0110010000000000 == (instruction & 0b1111111000000000))
        op = opcode_t::CPFSGT;
    else if (0b0110000000000000 == (instruction & 0b1111111000000000))
        op = opcode_t::CPFSLT;
    else if (0b0110101000000000 == (instruction & 0b1111111000000000))
        op = opcode_t::CLRF;
    else if (0b0110111000000000 == (instruction & 0b1111111000000000))
        op = opcode_t::MOVWF;
    else if (0b0000001000000000 == (instruction & 0b1111111000000000))
        op = opcode_t::MULWF;
    else if (0b0110110000000000 == (instruction & 0b1111111000000000))
        op = opcode_t::NEGF;
    else if (0b0110100000000000 == (instruction & 0b1111111000000000))
        op = opcode_t::SETF;
    else if (0b0110011000000000 == (instruction & 0b1111111000000000))
        op = opcode_t::TSTFSZ;
    else if (0b1110110000000000 == (instruction & 0b1111111000000000))
        op = opcode_t::CALL;
    else if (0b0010010000000000 == (instruction & 0b1111110000000000))
        op = opcode_t::ADDWF;
    else if (0b0010000000000000 == (instruction & 0b1111110000000000))
        op = opcode_t::ADDWFC;
    else if (0b0001010000000000 == (instruction & 0b1111110000000000))
        op = opcode_t::ANDWF;
    else if (0b0001110000000000 == (instruction & 0b1111110000000000))
        op = opcode_t::COMF;
    else if (0b0000010000000000 == (instruction & 0b1111110000000000))
        op = opcode_t::DECF;
    else if (0b0010110000000000 == (instruction & 0b1111110000000000))
        op = opcode_t::DECFSZ;
    else if (0b0100110000000000 == (instruction & 0b1111110000000000))
        op = opcode_t::DCFSNZ;
    else if (0b0010100000000000 == (instruction & 0b1111110000000000))
        op = opcode_t::INCF;
    else if (0b0011110000000000 == (instruction & 0b1111110000000000))
        op = opcode_t::INCFSZ;
    else if (0b0100100000000000 == (instruction & 0b1111110000000000))
        op = opcode_t::INFSNZ;
    else if (0b0001000000000000 == (instruction & 0b1111110000000000))
        op = opcode_t::IORWF;
    else if (0b0101000000000000 == (instruction & 0b1111110000000000))
        op = opcode_t::MOVF;
    else if (0b0011010000000000 == (instruction & 0b1111110000000000))
        op = opcode_t::RLCF;
    else if (0b0100010000000000 == (instruction & 0b1111110000000000))
        op = opcode_t::RLNCF;
    else if (0b0011000000000000 == (instruction & 0b1111110000000000))
        op = opcode_t::RRCF;
    else if (0b0100000000000000 == (instruction & 0b1111110000000000))
        op = opcode_t::RRNCF;
    else if (0b0101010000000000 == (instruction & 0b1111110000000000))
        op = opcode_t::SUBFWB;
    else if (0b0101110000000000 == (instruction & 0b1111110000000000))
        op = opcode_t::SUBWF;
    else if (0b0101100000000000 == (instruction & 0b1111110000000000))
        op = opcode_t::SUBWFB;
    else if (0b0011100000000000 == (instruction & 0b1111110000000000))
        op = opcode_t::SWAPF;
    else if (0b0001100000000000 == (instruction & 0b1111110000000000))
        op = opcode_t::XORWF;
    else if (0b1101000000000000 == (instruction & 0b1111100000000000))
        op = opcode_t::BRA;
    else if (0b1101100000000000 == (instruction & 0b1111100000000000))
        op = opcode_t::RCALL;
    else if (0b1100000000000000 == (instruction & 0b1111000000000000))
        op = opcode_t::MOVFF;
    else if (0b1001000000000000 == (instruction & 0b1111000000000000))
        op = opcode_t::BCF;
    else if (0b1000000000000000 == (instruction & 0b1111000000000000))
        op = opcode_t::BSF;
    else if (0b1011000000000000 == (instruction & 0b1111000000000000))
        op = opcode_t::BTFSC;
    else if (0b1010000000000000 == (instruction & 0b1111000000000000))
        op = opcode_t::BTFSS;
    else if (0b0111000000000000 == (instruction & 0b1111000000000000))
        op = opcode_t::BTG;
    else if (0b1111000000000000 == (instruction & 0b1111000000000000))
        op = opcode_t::NOP1;

    return decode_result_t{
        .instruction = *reinterpret_cast<instruction_t *>(&instruction),
        .opcode = op,
    };
}

bool cpu_stack_push(cpu_t &cpu, const cpu_known_sfrs_t &sfr, uint32_t value,
                    bus_reader_t<uint32_t, uint8_t> read_prog_bus, bus_reader_t<uint16_t, uint8_t> read_data_bus,
                    bus_writer_t<uint16_t, uint8_t> write_data_bus)
{
    uint8_t stkptr_reg = read_data_bus(static_cast<uint16_t>(sfr.STKPTR));
    uint8_t stkptr = stkptr_reg & reg_stkptr_mask_SP;

    if (stkptr == cpu.hw_stack.size())
    {
        // Stack is already full, set STKOVF and ignore push.
        stkptr_reg |= reg_stkptr_mask_STKOVF;
        write_data_bus(static_cast<uint16_t>(sfr.STKPTR), stkptr_reg);
        return false;
    }

    stkptr++;
    stkptr_reg &= ~reg_stkptr_mask_SP;
    stkptr_reg |= stkptr;

    if (stkptr == cpu.hw_stack.size())
    {
        // Stack became full
        stkptr_reg |= reg_stkptr_mask_STKOVF;
    }

    cpu.hw_stack[stkptr - 1] = value;
    write_data_bus(static_cast<uint16_t>(sfr.TOSU), (value >> 16) & 0xFF);
    write_data_bus(static_cast<uint16_t>(sfr.TOSH), (value >> 8) & 0xFF);
    write_data_bus(static_cast<uint16_t>(sfr.TOSL), value & 0xFF);
    write_data_bus(static_cast<uint16_t>(sfr.STKPTR), stkptr_reg);

    if (stkptr == cpu.hw_stack.size())
    {
        // Check if the cpu should be reset because stack became full.
        bool reset_on_full = reg_check_configuration_bit(reg_configuration_bit_t::STVREN, read_prog_bus);

        if (reset_on_full)
        {
            cpu_reset_mclr(cpu, read_data_bus, write_data_bus);
            return false;
        }
    }

    return true;
}

bool cpu_stack_pop(cpu_t &cpu, const cpu_known_sfrs_t &sfr, bus_reader_t<uint32_t, uint8_t> read_prog_bus,
                   bus_reader_t<uint16_t, uint8_t> read_data_bus, bus_writer_t<uint16_t, uint8_t> write_data_bus)
{
    uint8_t stkptr_reg = read_data_bus(static_cast<uint16_t>(sfr.STKPTR));
    uint8_t stkptr = stkptr_reg & reg_stkptr_mask_SP;

    if (stkptr == 0)
    {
        stkptr_reg |= reg_stkptr_mask_STKUNF;
        write_data_bus(static_cast<uint16_t>(sfr.STKPTR), stkptr_reg);

        bool reset_on_underflow = reg_check_configuration_bit(reg_configuration_bit_t::STVREN, read_prog_bus);

        if (reset_on_underflow)
        {
            cpu_reset_mclr(cpu, read_data_bus, write_data_bus);
            return false;
        }
        else
        {
            return true;
        }
    }

    uint32_t tos;
    stkptr--;

    if (stkptr == 0)
        tos = 0;
    else
        tos = cpu.hw_stack[stkptr - 1];

    write_data_bus(static_cast<uint16_t>(sfr.TOSU), (tos >> 16) & 0xFF);
    write_data_bus(static_cast<uint16_t>(sfr.TOSH), (tos >> 8) & 0xFF);
    write_data_bus(static_cast<uint16_t>(sfr.TOSL), tos & 0xFF);

    stkptr_reg &= ~reg_stkptr_mask_SP;
    stkptr_reg |= stkptr;

    write_data_bus(static_cast<uint16_t>(sfr.STKPTR), stkptr_reg);

    return true;
}

uint32_t cpu_stack_top(cpu_t &cpu, const cpu_known_sfrs_t &sfr, bus_reader_t<uint16_t, uint8_t> read_data_bus)
{
    uint8_t stkptr = read_data_bus(static_cast<uint16_t>(sfr.STKPTR)) & reg_stkptr_mask_SP;

    if (stkptr == 0)
        return 0;

    return cpu.hw_stack[stkptr - 1];
}

void cpu_interrupt_vector(cpu_t &cpu, const cpu_known_sfrs_t &sfr, bus_reader_t<uint32_t, uint8_t> read_prog_bus,
                          bus_reader_t<uint16_t, uint8_t> read_data_bus, bus_writer_t<uint16_t, uint8_t> write_data_bus,
                          bool high_priority)
{
    cpu.ws = read_data_bus(static_cast<uint16_t>(sfr.WREG));
    cpu.statuss = read_data_bus(static_cast<uint16_t>(sfr.STATUS));
    cpu.bsrs = read_data_bus(static_cast<uint16_t>(sfr.BSR));
    cpu.is_high_priority_interrupt = high_priority;

    uint32_t pc = cpu_read_pc(sfr, read_data_bus);
    if (!cpu_stack_push(cpu, sfr, pc, read_prog_bus, read_data_bus, write_data_bus))
        return;

    uint32_t int_vector = high_priority ? 0x8 : 0x18;

    cpu.fetched_instruction = 0x0000; // NOP
    cpu_write_pc(sfr, write_data_bus, int_vector);
}

uint32_t cpu_read_pc(const cpu_known_sfrs_t &sfr, bus_reader_t<uint16_t, uint8_t> read_data_bus)
{
    uint8_t pclatu = read_data_bus(static_cast<uint16_t>(sfr.PCLATU));
    uint8_t pclath = read_data_bus(static_cast<uint16_t>(sfr.PCLATH));
    uint8_t pcl = read_data_bus(static_cast<uint16_t>(sfr.PCL));

    return (static_cast<uint32_t>(pclatu) << 16) | (static_cast<uint32_t>(pclath) << 8) | pcl;
}

void cpu_write_pc(const cpu_known_sfrs_t &sfr, bus_writer_t<uint16_t, uint8_t> write_data_bus, uint32_t pc)
{
    write_data_bus(static_cast<uint16_t>(sfr.PCL), pc & 0xFF);
    write_data_bus(static_cast<uint16_t>(sfr.PCLATH), (pc >> 8) & 0xFF);
    write_data_bus(static_cast<uint16_t>(sfr.PCLATU), (pc >> 16) & 0xFF);
}
