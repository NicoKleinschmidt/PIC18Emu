#include "cpu.hpp"
#include "18f66k80.hpp"
#include "alu.hpp"
#include "config.hpp"
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

static uint16_t get_fsr2(const cpu_known_sfrs_t &sfr, bus_reader_t<uint16_t, uint8_t> read_bus)
{
    uint8_t low = read_bus(sfr.FSR2L);
    uint8_t high = read_bus(sfr.FSR2L + 1);
    return (high << 8) | low;
}

/// @brief Reads a value from the bank specified by the BSR register or the access bank.
/// @param location The 8 bit address of the value in the bank.
/// @param use_bsr If false, the value will be read from the access bank instead of the one specified by BSR
/// @param x_inst Wether or not the extended instruction set is enabled
static uint8_t read_bus(const cpu_known_sfrs_t &sfr, bus_reader_t<uint16_t, uint8_t> read_bus, uint8_t location,
                        bool use_bsr, bool x_inst)
{
    if (!use_bsr)
    {
        if (x_inst && location < 0x60)
            return read_bus(get_fsr2(sfr, read_bus) + location);

        if (location >= 0x80)
            return read_bus(0xF00 + location);

        return read_bus(location);
    }

    uint8_t bank = read_bus(static_cast<uint16_t>(sfr.BSR));
    uint16_t addr = (static_cast<uint16_t>(bank) << 8) | static_cast<uint16_t>(location);
    return read_bus(addr);
}

/// @brief Writes a value to the bank specified by the BSR register or the access bank.
/// @param location The 8 bit address of the value in the bank.
/// @param use_bsr If false, the value will be written to the access bank instead of the one specified by BSR
/// @param x_inst Wether or not the extended instruction set is enabled
static void write_bus(const cpu_known_sfrs_t &sfr, bus_writer_t<uint16_t, uint8_t> write_bus,
                      bus_reader_t<uint16_t, uint8_t> read_bus, uint8_t location, uint8_t val, bool use_bsr,
                      bool x_inst)
{
    if (!use_bsr)
    {
        if (x_inst && location < 0x60)
            return write_bus(get_fsr2(sfr, read_bus) + location, val);

        if (location >= 0x80)
            return write_bus(0xF00 + location, val);

        return write_bus(location, val);
    }

    uint8_t bank = read_bus(static_cast<uint16_t>(sfr.BSR));
    uint16_t addr = (static_cast<uint16_t>(bank) << 8) | static_cast<uint16_t>(location);
    return write_bus(addr, val);
}

void cpu_tick(cpu_t &cpu, const cpu_known_sfrs_t &sfr, bus_reader_t<uint32_t, uint8_t> read_prog_bus,
              bus_writer_t<uint32_t, uint8_t> write_prog_bus, bus_reader_t<uint16_t, uint8_t> read_data_bus,
              bus_writer_t<uint16_t, uint8_t> write_data_bus, std::function<void(cpu_event_t e)> event_handler)
{
    decode_result_t decoded = cpu_decode(cpu.fetched_instruction);
    uint32_t decoded_addr = cpu.fetched_instruction_addr;
    instruction_t instruction = decoded.instruction;

    cpu.fetched_instruction_addr = cpu.pc;
    uint8_t fetched_low = read_prog_bus(cpu.pc++);
    uint8_t fetched_high = read_prog_bus(cpu.pc++);
    cpu.fetched_instruction = (static_cast<uint16_t>(fetched_high) << 8) | fetched_low;

    if (cpu.next_action != nullptr)
    {
        // We need to clear the next action function before calling it,
        // because the function could set it again internally.
        auto tmp = cpu.next_action;
        cpu.next_action = nullptr;
        tmp(cpu, instruction);

        return;
    }

    env_cpu_current_instruction(decoded, decoded_addr);

    bool xinst = configuration_check_bit(CONFIG::XINST, read_prog_bus);

    if (decoded.opcode == opcode_t::ILLEGAL)
    {
        if (event_handler != nullptr)
            event_handler(cpu_event_t::illegal_instruction);
        return;
    }

    switch (decoded.opcode)
    {
    case opcode_t::ADDFSR: {
        if (!xinst)
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
            cpu.wreg = instruction.literal.k;
            cpu.pc = cpu_stack_top(cpu);
            if (!cpu_stack_pop(cpu, read_prog_bus))
                return;

            cpu.fetched_instruction = 0x0000; // NOP
        }

        break;
    }

    case opcode_t::ADDLW: {
        cpu.wreg = alu_add(cpu.wreg, instruction.literal.k, cpu.status);
        break;
    }

    case opcode_t::ADDWF: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        uint8_t result = alu_add(cpu.wreg, val, cpu.status);

        if (instruction.byte_oriented.d)
            write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                      instruction.byte_oriented.a, xinst);
        else
            cpu.wreg = result;
        break;
    }

    case opcode_t::ADDWFC: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        uint8_t result = alu_add(cpu.wreg, val, (cpu.status & alu_status_C), cpu.status);

        if (instruction.byte_oriented.d)
            write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                      instruction.byte_oriented.a, xinst);
        else
            cpu.wreg = result;
        break;
    }

    case opcode_t::ANDLW: {
        cpu.wreg = alu_and(cpu.wreg, instruction.literal.k, cpu.status);
        break;
    }

    case opcode_t::ANDWF: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        uint8_t result = alu_and(cpu.wreg, val, cpu.status);

        if (instruction.byte_oriented.d)
            write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                      instruction.byte_oriented.a, xinst);
        else
            cpu.wreg = result;
        break;
    }

    case opcode_t::BC: {
        if (cpu.status & alu_status_C)
        {
            cpu.pc += from_2s_complement_8(instruction.control_branch_status.literal) * 2;
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::BCF: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.bit_oriented.f, instruction.bit_oriented.a, xinst);
        val &= ~(1 << instruction.bit_oriented.bit);
        write_bus(sfr, write_data_bus, read_data_bus, instruction.bit_oriented.f, val, instruction.bit_oriented.a,
                  xinst);
        break;
    }

    case opcode_t::BN: {
        if (cpu.status & alu_status_N)
        {
            cpu.pc += from_2s_complement_8(instruction.control_branch_status.literal) * 2;
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::BNC: {
        if (!(cpu.status & alu_status_C))
        {
            cpu.pc += from_2s_complement_8(instruction.control_branch_status.literal) * 2;
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::BNN: {
        if (!(cpu.status & alu_status_N))
        {
            cpu.pc += from_2s_complement_8(instruction.control_branch_status.literal) * 2;
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::BNOV: {
        if (!(cpu.status & alu_status_OV))
        {
            cpu.pc += from_2s_complement_8(instruction.control_branch_status.literal) * 2;
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::BNZ: {
        if (!(cpu.status & alu_status_Z))
        {
            cpu.pc += from_2s_complement_8(instruction.control_branch_status.literal) * 2;
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::BRA: {
        int16_t offset = from_2s_complement_11(instruction.control_branch.literal) * 2;
        cpu.pc += offset - 2;
        cpu.fetched_instruction = 0x0000; // NOP
        break;
    }

    case opcode_t::BSF: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.bit_oriented.f, instruction.bit_oriented.a, xinst);
        val |= (1 << instruction.bit_oriented.bit);
        write_bus(sfr, write_data_bus, read_data_bus, instruction.bit_oriented.f, val, instruction.bit_oriented.a,
                  xinst);
        break;
    }

    case opcode_t::BTFSC: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.bit_oriented.f, instruction.bit_oriented.a, xinst);
        bool is_set = val & (1 << instruction.bit_oriented.bit);
        if (!is_set)
        {
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::BTFSS: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.bit_oriented.f, instruction.bit_oriented.a, xinst);
        bool is_set = val & (1 << instruction.bit_oriented.bit);
        if (is_set)
        {
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::BTG: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.bit_oriented.f, instruction.bit_oriented.a, xinst);
        val ^= (1 << instruction.bit_oriented.bit);
        write_bus(sfr, write_data_bus, read_data_bus, instruction.bit_oriented.f, val, instruction.bit_oriented.a,
                  xinst);
        break;
    }

    case opcode_t::BOV: {
        if (cpu.status & alu_status_OV)
        {
            cpu.pc += from_2s_complement_8(instruction.control_branch_status.literal * 2);
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::BZ: {
        if (cpu.status & alu_status_Z)
        {
            cpu.pc += from_2s_complement_8(instruction.control_branch_status.literal * 2);
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::CALL: {
        if (!cpu_stack_push(cpu, cpu.pc, read_prog_bus))
        {
            return;
        }

        instruction_t next_instruction = *reinterpret_cast<instruction_t *>(&cpu.fetched_instruction);
        cpu.pc = (static_cast<uint32_t>(next_instruction.control_call_low.literal << 8) |
                  static_cast<uint32_t>(instruction.control_call_high.literal))
                 << 1;

        if (instruction.control_call_high.s)
        {
            cpu.ws = cpu.wreg;
            cpu.status = cpu.status;
            cpu.bsrs = read_data_bus(static_cast<uint16_t>(sfr.BSR));
        }

        cpu.fetched_instruction = 0x0000; // NOP
        break;
    }

    case opcode_t::CALLW: {
        if (!xinst)
            break;

        if (!cpu_stack_push(cpu, cpu.pc, read_prog_bus))
            return;

        cpu.pc &= ~0xFF;
        cpu.pc |= cpu.wreg;

        cpu.fetched_instruction = 0x0000; // NOP
        break;
    }

    case opcode_t::CLRF: {
        (void)read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, 0, instruction.byte_oriented.a,
                  xinst);
        cpu.status |= alu_status_Z;
        break;
    }

    case opcode_t::CLRWDT: {
        // TODO:
        // reg_sfr_write(read_data_bus, write_data_bus, sfr.RCON, reg_rcon_mask_TO, 1);
        // reg_sfr_write(read_data_bus, write_data_bus, sfr.RCON, reg_rcon_mask_PD, 1);
        event_handler(cpu_event_t::reset_wdt);
        break;
    }

    case opcode_t::COMF: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        uint8_t result = alu_complement(val, cpu.status);

        if (instruction.byte_oriented.d)
            write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                      instruction.byte_oriented.a, xinst);
        else
            cpu.wreg = result;
        break;
    }

    case opcode_t::CPFSEQ: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        if (val == cpu.wreg)
        {
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::CPFSGT: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        if (val > cpu.wreg)
        {
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::CPFSLT: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        if (val < cpu.wreg)
        {
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::DAW: {
        uint8_t result = alu_decimal_adjust(cpu.wreg, cpu.status);
        cpu.wreg = result;
        break;
    }

    case opcode_t::DECF: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        uint8_t result = alu_sub(val, 1, cpu.status);

        if (instruction.byte_oriented.d)
            write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                      instruction.byte_oriented.a, xinst);
        else
            cpu.wreg = result;
        break;
    }

    case opcode_t::DECFSZ: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        uint8_t result = val - 1;

        if (instruction.byte_oriented.d)
            write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                      instruction.byte_oriented.a, xinst);
        else
            cpu.wreg = result;

        if (result == 0)
        {
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::DCFSNZ: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        uint8_t result = val - 1;

        if (instruction.byte_oriented.d)
            write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                      instruction.byte_oriented.a, xinst);
        else
            cpu.wreg = result;

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
        cpu.pc = address;
        break;
    }

    case opcode_t::INCF: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        uint8_t result = alu_add(val, 1, cpu.status);

        if (instruction.byte_oriented.d)
            write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                      instruction.byte_oriented.a, xinst);
        else
            cpu.wreg = result;
        break;
    }

    case opcode_t::INCFSZ: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        uint8_t result = val + 1;

        if (instruction.byte_oriented.d)
            write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                      instruction.byte_oriented.a, xinst);
        else
            cpu.wreg = result;

        if (result == 0)
        {
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::INFSNZ: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        uint8_t result = val + 1;

        if (instruction.byte_oriented.d)
            write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                      instruction.byte_oriented.a, xinst);
        else
            cpu.wreg = result;

        if (result != 0)
        {
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::IORLW: {
        cpu.wreg = alu_or(cpu.wreg, instruction.literal.k, cpu.status);
        break;
    }

    case opcode_t::IORWF: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        uint8_t result = alu_or(cpu.wreg, val, cpu.status);

        if (instruction.byte_oriented.d)
            write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                      instruction.byte_oriented.a, xinst);
        else
            cpu.wreg = result;
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
        cpu.next_action = [fsrl_offset, write_data_bus](cpu_t &, instruction_t next_instruction) {
            write_data_bus(fsrl_offset, next_instruction.load_fsr_low.k);
        };
        break;
    }

    case opcode_t::MOVF: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        cpu.status &= ~alu_status_N;
        cpu.status &= ~alu_status_Z;

        if (val == 0)
            cpu.status |= alu_status_Z;
        if (val & 0b10000000)
            cpu.status |= alu_status_N;

        if (instruction.byte_oriented.d)
            write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, val, instruction.byte_oriented.a,
                      xinst);
        else
            cpu.wreg = val;
        break;
    }

    case opcode_t::MOVFF: {
        uint8_t val = read_data_bus(instruction.byte_to_byte_high.f_source);
        cpu.pc -= 2;
        cpu.next_action = [val, write_data_bus, sfr](cpu_t &, instruction_t &instruction) {
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
        cpu.wreg = instruction.literal.k;
        break;
    }

    case opcode_t::MOVSF: {
        if (!xinst)
            break;

        uint8_t fsr2l = read_data_bus(sfr.FSR2L);
        uint8_t fsr2h = read_data_bus(sfr.FSR2L + 1);
        uint16_t fsr2_val = (static_cast<uint16_t>(fsr2h) << 8) | static_cast<uint16_t>(fsr2l);
        uint8_t src_value = read_data_bus(fsr2_val + instruction.xinst_movsf_high_movss.z);
        cpu.next_action = [src_value, write_data_bus](cpu_t &, instruction_t &instruction) {
            write_data_bus(instruction.xinst_movsf_low.f, src_value);
        };

        break;
    }

    case opcode_t::MOVSS: {
        if (!xinst)
            break;

        uint8_t fsr2l = read_data_bus(sfr.FSR2L);
        uint8_t fsr2h = read_data_bus(sfr.FSR2L + 1);
        uint16_t fsr2_val = (static_cast<uint16_t>(fsr2h) << 8) | static_cast<uint16_t>(fsr2l);
        uint8_t src_value = read_data_bus(fsr2_val + instruction.xinst_movsf_high_movss.z);
        cpu.next_action = [src_value, read_data_bus, write_data_bus, sfr](cpu_t &, instruction_t &instruction) {
            uint8_t fsr2l = read_data_bus(sfr.FSR2L);
            uint8_t fsr2h = read_data_bus(sfr.FSR2L + 1);
            uint16_t fsr2_val = (static_cast<uint16_t>(fsr2h) << 8) | static_cast<uint16_t>(fsr2l);
            write_data_bus(fsr2_val + instruction.xinst_movsf_high_movss.z, src_value);
        };

        break;
    }

    case opcode_t::MOVWF: {
        write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, cpu.wreg,
                  instruction.byte_oriented.a, xinst);
        break;
    }

    case opcode_t::MULLW: {
        uint16_t result = static_cast<uint16_t>(cpu.wreg) * static_cast<uint16_t>(instruction.literal.k);
        write_data_bus(static_cast<uint16_t>(sfr.PRODH), (result >> 8) & 0xFF);
        write_data_bus(static_cast<uint16_t>(sfr.PRODL), result & 0xFF);
        break;
    }

    case opcode_t::MULWF: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        uint16_t result = static_cast<uint16_t>(cpu.wreg) * static_cast<uint16_t>(val);
        write_data_bus(static_cast<uint16_t>(sfr.PRODH), (result >> 8) & 0xFF);
        write_data_bus(static_cast<uint16_t>(sfr.PRODL), result & 0xFF);
        break;
    }

    case opcode_t::NEGF: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        uint8_t result = alu_negate(val, cpu.status);

        write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result, instruction.byte_oriented.a,
                  xinst);
        break;
    }

    case opcode_t::NOP:
    case opcode_t::NOP1: {
        break;
    }

    case opcode_t::POP: {
        cpu_stack_pop(cpu, read_prog_bus);
        break;
    }

    case opcode_t::PUSH: {
        if (!cpu_stack_push(cpu, cpu.pc, read_prog_bus))
        {
            return;
        }
        break;
    }

    case opcode_t::PUSHL: {
        if (!xinst)
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
        if (!cpu_stack_push(cpu, cpu.pc, read_prog_bus))
        {
            return;
        }

        cpu.pc += 2 + from_2s_complement_11(instruction.control_branch.literal) * 2;
        cpu.fetched_instruction = 0xFFFF; // NOP
        break;
    }

    case opcode_t::RESET: {
        cpu_reset_mclr(cpu);
        break;
    }

    case opcode_t::RETFIE: {
        cpu.pc = cpu_stack_top(cpu);
        if (!cpu_stack_pop(cpu, read_prog_bus))
            return;

        cpu.global_interrupt_enable(cpu.is_high_priority_interrupt, true);

        if (instruction.control_return.s)
        {
            cpu.wreg = cpu.ws;
            cpu.status = cpu.statuss;
            write_data_bus(static_cast<uint16_t>(sfr.BSR), cpu.bsrs);
        }

        cpu.fetched_instruction = 0x0000; // NOP
        break;
    }

    case opcode_t::RETLW: {
        cpu.wreg = instruction.literal.k;
        cpu.pc = cpu_stack_top(cpu);
        if (!cpu_stack_pop(cpu, read_prog_bus))
            return;

        cpu.fetched_instruction = 0x0000; // NOP
        break;
    }

    case opcode_t::RETURN: {
        cpu.pc = cpu_stack_top(cpu);
        if (!cpu_stack_pop(cpu, read_prog_bus))
            return;

        if (instruction.control_return.s)
        {
            cpu.wreg = cpu.ws;
            cpu.status = cpu.statuss;
            write_data_bus(static_cast<uint16_t>(sfr.BSR), cpu.bsrs);
        }

        cpu.fetched_instruction = 0x0000; // NOP
        break;
    }

    case opcode_t::RLCF: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        uint8_t result = alu_rotate_left_carry(val, cpu.status);

        if (instruction.byte_oriented.d)
            write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                      instruction.byte_oriented.a, xinst);
        else
            cpu.wreg = result;
        break;
    }

    case opcode_t::RLNCF: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        uint8_t result = alu_rotate_left(val, cpu.status);

        if (instruction.byte_oriented.d)
            write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                      instruction.byte_oriented.a, xinst);
        else
            cpu.wreg = result;
        break;
    }

    case opcode_t::RRCF: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        uint8_t result = alu_rotate_right_carry(val, cpu.status);

        if (instruction.byte_oriented.d)
            write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                      instruction.byte_oriented.a, xinst);
        else
            cpu.wreg = result;
        break;
    }

    case opcode_t::RRNCF: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        uint8_t result = alu_rotate_right(val, cpu.status);

        if (instruction.byte_oriented.d)
            write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                      instruction.byte_oriented.a, xinst);
        else
            cpu.wreg = result;
        break;
    }

    case opcode_t::SETF: {
        (void)read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, 0xFF, instruction.byte_oriented.a,
                  xinst);
        break;
    }

    case opcode_t::SLEEP: {
        // TODO:
        // reg_sfr_write(read_data_bus, write_data_bus, sfr.RCON, reg_rcon_mask_TO, 1);
        // reg_sfr_write(read_data_bus, write_data_bus, sfr.RCON, reg_rcon_mask_PD, 0);
        event_handler(cpu_event_t::reset_wdt);
        event_handler(cpu_event_t::sleep);
        break;
    }

    case opcode_t::SUBFSR: {
        if (!xinst)
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
            cpu.wreg = instruction.literal.k;
            cpu.pc = cpu_stack_top(cpu);
            if (!cpu_stack_pop(cpu, read_prog_bus))
                return;

            cpu.fetched_instruction = 0x0000; // NOP
        }

        break;
    }

    case opcode_t::SUBFWB: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        uint8_t result = alu_sub(cpu.wreg, val, !(cpu.status & alu_status_C), cpu.status);

        if (instruction.byte_oriented.d)
            write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                      instruction.byte_oriented.a, xinst);
        else
            cpu.wreg = result;
        break;
    }

    case opcode_t::SUBLW: {
        cpu.wreg = alu_sub(instruction.literal.k, cpu.wreg, cpu.status);
        break;
    }

    case opcode_t::SUBWF: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        uint8_t result = alu_sub(val, cpu.wreg, cpu.status);

        if (instruction.byte_oriented.d)
            write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                      instruction.byte_oriented.a, xinst);
        else
            cpu.wreg = result;
        break;
    }

    case opcode_t::SUBWFB: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        uint8_t result = alu_sub(val, cpu.wreg, !(cpu.status & alu_status_C), cpu.status);

        if (instruction.byte_oriented.d)
            write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                      instruction.byte_oriented.a, xinst);
        else
            cpu.wreg = result;
        break;
    }

    case opcode_t::SWAPF: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        uint8_t result = (val << 4) | (val >> 4);

        if (instruction.byte_oriented.d)
            write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                      instruction.byte_oriented.a, xinst);
        else
            cpu.wreg = result;
        break;
    }

    case opcode_t::TBLRD: {
        tblptr_action_t action = tblptr_action_t::none;
        if (instruction.table_op.n == 1)
            action = tblptr_action_t::postinc;
        else if (instruction.table_op.n == 2)
            action = tblptr_action_t::postdec;
        else if (instruction.table_op.n == 3)
            action = tblptr_action_t::preinc;

        cpu.table_read(action);

        // Wait one tick
        cpu.pc -= 2;
        cpu.next_action = [](cpu_t &, instruction_t &) {};
        break;
    }

    case opcode_t::TBLWT: {
        tblptr_action_t action = tblptr_action_t::none;
        if (instruction.table_op.n == 1)
            action = tblptr_action_t::postinc;
        else if (instruction.table_op.n == 2)
            action = tblptr_action_t::postdec;
        else if (instruction.table_op.n == 3)
            action = tblptr_action_t::preinc;

        cpu.table_write(action);

        // Wait one tick
        cpu.pc -= 2;
        cpu.next_action = [](cpu_t &, instruction_t &) {};
        break;
    }

    case opcode_t::TSTFSZ: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        if (val == 0)
        {
            cpu.fetched_instruction = 0x0000; // NOP
        }
        break;
    }

    case opcode_t::XORLW: {
        cpu.wreg = alu_xor(instruction.literal.k, cpu.wreg, cpu.status);
        break;
    }

    case opcode_t::XORWF: {
        uint8_t val = read_bus(sfr, read_data_bus, instruction.byte_oriented.f, instruction.byte_oriented.a, xinst);
        uint8_t result = alu_xor(cpu.wreg, val, cpu.status);

        if (instruction.byte_oriented.d)
            write_bus(sfr, write_data_bus, read_data_bus, instruction.byte_oriented.f, result,
                      instruction.byte_oriented.a, xinst);
        else
            cpu.wreg = result;
        break;
    }

    default: break;
    }
}

void cpu_reset_por(cpu_t &cpu)
{
    cpu.fetched_instruction = 0;
    cpu.fetched_instruction_addr = 0xFFFFFFFF;
    cpu.pc = 0;
    cpu.pclath = 0;
    cpu.pclatu = 0;
    cpu.stkptr = 0;
    cpu.stkful = false;
    cpu.stkunf = false;
}

void cpu_reset_mclr(cpu_t &cpu)
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

bool cpu_stack_push(cpu_t &cpu, uint32_t value, bus_reader_t<uint32_t, uint8_t> read_prog_bus)
{
    if (cpu.stkptr == cpu.hw_stack.size())
    {
        // Stack is already full, set STKFUL and ignore push.
        cpu.stkful = true;
        return false;
    }

    cpu.stkptr++;
    if (cpu.stkptr == cpu.hw_stack.size())
    {
        // Stack became full
        cpu.stkful = true;
    }

    cpu.hw_stack[cpu.stkptr - 1] = value;

    if (cpu.stkful)
    {
        // Check if the cpu should be reset because stack became full.
        bool reset_on_full = configuration_check_bit(CONFIG::STVREN, read_prog_bus);

        if (reset_on_full)
        {
            cpu_reset_mclr(cpu);
            return false;
        }
    }

    return true;
}

bool cpu_stack_pop(cpu_t &cpu, bus_reader_t<uint32_t, uint8_t> read_prog_bus)
{
    if (cpu.stkptr == 0)
    {
        cpu.stkunf = true;

        bool reset_on_underflow = configuration_check_bit(CONFIG::STVREN, read_prog_bus);

        if (reset_on_underflow)
        {
            cpu_reset_mclr(cpu);
            return false;
        }
        else
        {
            return true;
        }
    }

    cpu.stkptr--;
    return true;
}

uint32_t cpu_stack_top(cpu_t &cpu)
{
    if (cpu.stkptr == 0)
        return 0;

    return cpu.hw_stack[cpu.stkptr - 1];
}

void cpu_stack_top_set(cpu_t &cpu, uint32_t val)
{
    if (cpu.stkptr == 0)
        return;

    cpu.hw_stack[cpu.stkptr - 1] = val;
}

void cpu_interrupt_vector(cpu_t &cpu, const cpu_known_sfrs_t &sfr, bus_reader_t<uint16_t, uint8_t> read_data_bus,
                          bus_reader_t<uint32_t, uint8_t> read_prog_bus, bool high_priority)
{
    cpu.ws = read_data_bus(static_cast<uint16_t>(sfr.WREG));
    cpu.statuss = read_data_bus(static_cast<uint16_t>(sfr.STATUS));
    cpu.bsrs = read_data_bus(static_cast<uint16_t>(sfr.BSR));
    cpu.is_high_priority_interrupt = high_priority;

    if (!cpu_stack_push(cpu, cpu.pc, read_prog_bus))
        return;

    uint32_t int_vector = high_priority ? 0x8 : 0x18;

    cpu.fetched_instruction = 0x0000; // NOP
    cpu.pc = int_vector;
}

addr_read_result_t cpu_bus_read(cpu_t &cpu, const cpu_known_sfrs_t &regs, uint16_t address)
{
    if (address == regs.PCL)
    {
        cpu.pclatu = (cpu.pc >> 16) & 0xFF;
        cpu.pclath = (cpu.pc >> 8) & 0xFF;
        uint8_t pcl = static_cast<uint8_t>(cpu.pc & 0xFF);

        return addr_read_result_t{.data = pcl, .mask = 0xFF};
    }
    else if (address == regs.PCLATH)
    {
        return addr_read_result_t{.data = cpu.pclath, .mask = 0xFF};
    }
    else if (address == regs.PCLATU)
    {
        return addr_read_result_t{.data = cpu.pclatu, .mask = 0xFF};
    }
    else if (address == regs.STKPTR)
    {
        return addr_read_result_t{.data = static_cast<uint8_t>((static_cast<uint8_t>(cpu.stkful) << 7) |
                                                               (static_cast<uint8_t>(cpu.stkunf) << 6) |
                                                               (cpu.stkptr & 0x1F)),
                                  .mask = 0xFF};
    }
    else if (address == regs.TOSL)
    {
        return addr_read_result_t{.data = static_cast<uint8_t>(cpu_stack_top(cpu) & 0xFF), .mask = 0xFF};
    }
    else if (address == regs.TOSH)
    {
        return addr_read_result_t{.data = static_cast<uint8_t>((cpu_stack_top(cpu) >> 8) & 0xFF), .mask = 0xFF};
    }
    else if (address == regs.TOSU)
    {
        return addr_read_result_t{.data = static_cast<uint8_t>((cpu_stack_top(cpu) >> 16) & 0xFF), .mask = 0xFF};
    }
    else if (address == regs.WREG)
    {
        return addr_read_result_t{.data = cpu.wreg, .mask = 0xFF};
    }
    else if (address == regs.STATUS)
    {
        return addr_read_result_t{.data = cpu.status, .mask = 0x1F};
    }

    return addr_read_result_none;
}

addr_bit_mask_t cpu_bus_write(cpu_t &cpu, const cpu_known_sfrs_t &regs, uint16_t address, uint8_t value)
{
    if (address == regs.PCL)
    {
        cpu.pc = (static_cast<uint32_t>(cpu.pclatu) << 16) | (static_cast<uint32_t>(cpu.pclath) << 8) |
                 static_cast<uint32_t>(value);
    }
    else if (address == regs.PCLATH)
    {
        cpu.pclath = value;
        return 0xFF;
    }
    else if (address == regs.PCLATU)
    {
        cpu.pclatu = value;
        return 0xFF;
    }
    else if (address == regs.STKPTR)
    {
        cpu.stkful = value & (1 << 7);
        cpu.stkunf = value & (1 << 6);
        cpu.stkptr = value & 0x1F;
        return 0xDF;
    }
    else if (address == regs.TOSL)
    {
        cpu_stack_top_set(cpu, (cpu_stack_top(cpu) & 0xFFFF00) | value);
        return 0xFF;
    }
    else if (address == regs.TOSH)
    {
        cpu_stack_top_set(cpu, (cpu_stack_top(cpu) & 0xFF00FF) | (value << 8));
        return 0xFF;
    }
    else if (address == regs.TOSU)
    {
        cpu_stack_top_set(cpu, (cpu_stack_top(cpu) & 0x00FFFF) | (value << 16));
        return 0xFF;
    }
    else if (address == regs.WREG)
    {
        cpu.wreg = value;
        return 0xFF;
    }
    else if (address == regs.STATUS)
    {
        cpu.status = value & 0x1F;
        return 0x1F;
    }

    return 0x00;
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
