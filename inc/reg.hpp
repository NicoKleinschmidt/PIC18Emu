#pragma once

#include "bus.hpp"
#include <cstdint>

using sfr_phy_reader_t = std::function<addr_read_result_t(uint16_t sfr)>;
using sfr_phy_writer_t = std::function<addr_bit_mask_t(uint16_t sfr, uint8_t val)>;

struct indf_known_sfrs_t
{
    uint16_t WREG;
    uint16_t FSR0H;
    uint16_t FSR1H;
    uint16_t FSR2H;
    uint16_t FSR0L;
    uint16_t FSR1L;
    uint16_t FSR2L;
    uint16_t INDF0;
    uint16_t INDF1;
    uint16_t INDF2;
    uint16_t PREINC0;
    uint16_t PREINC1;
    uint16_t PREINC2;
    uint16_t POSTINC0;
    uint16_t POSTINC1;
    uint16_t POSTINC2;
    uint16_t POSTDEC0;
    uint16_t POSTDEC1;
    uint16_t POSTDEC2;
    uint16_t PLUSW0;
    uint16_t PLUSW1;
    uint16_t PLUSW2;
};

struct sfr_ctx_t
{
    /// @brief Required for indirect memory access.
    /// @note Can be nullptr for write only functions.
    ///        This should be the bus_reader that calls the sfr_buf_read function (Recursion).
    bus_reader_t<uint16_t, uint8_t> read_bus;
    /// @brief Required for indirect memory access.
    /// @note Can be nullptr for read only functions.
    ///        This should be the bus_writer that calls the sfr_buf_write function (Recursion).
    bus_writer_t<uint16_t, uint8_t> write_bus;

    sfr_phy_reader_t read_sfr_phy;
    sfr_phy_writer_t write_sfr_phy;

    const indf_known_sfrs_t &sfr;
    uint16_t first_address;
    uint16_t last_address;
};

enum reg_config_register_t
{
    CONFIG1L = 0x300000,
    CONFIG1H = 0x300001,
    CONFIG2L = 0x300002,
    CONFIG2H = 0x300003,
    CONFIG3H = 0x300005,
    CONFIG4L = 0x300006,
    CONFIG5L = 0x300008,
    CONFIG5H = 0x300009,
    CONFIG6L = 0x30000A,
    CONFIG6H = 0x30000B,
    CONFIG7L = 0x30000C,
    CONFIG7H = 0x30000D,
    DEVID1 = 0x3FFFFE,
    DEVID2 = 0x3FFFFF,
};

// clang-format off
enum class reg_configuration_bit_t
{
    XINST,  SOSCSEL1, SOSCSEL0, INTOSCSEL, RETEN,
    OSCSEN, FOSC2,    FOSC1,    FOSC0,
    BORV1,  BORV0,    BOREN,    PWRTEN,
    WDTPS2, WDTPS1,   WDTPS0,   WDTEN,
    CCP2MX, DEBUG,    LVP,      STVREN,
    CP3,    CP2,      CP1,      CP0,
    CPD,    CPB,    
    WRT3,   WRT2,     WRT1,     WRT0,
    WRTD,   WRTB,     WRTC,
    EBTR3,  EBTR2,    EBTR1,    EBTR0,
    EBTRB,
};
// clang-format on

struct reg_configuration_bit_info_t
{
    reg_config_register_t file;
    uint8_t bit;
    bool invert;
};

enum class reg_reset_type_t
{
    none,
    power_on_reset,
    brown_out_reset,
};

bool reg_check_configuration_bit(reg_configuration_bit_t bit, bus_reader_t<uint32_t, uint8_t> read_bus);

/// @brief This is a helper function for reading the value of an SFR.
/// @param read_bus
/// @param reg The address of the SFR to read from
/// @param reg_mask A bitmask specifying which bits to read.
/// @return The value at the specified location.
///
/// Register value: 0111 0000
/// reg_mask:       1111 0000
///
/// result: 0000 0111
uint8_t reg_sfr_read(bus_reader_t<uint16_t, uint8_t> read_bus, uint16_t reg_addr, uint8_t reg_mask);

/// @brief This is a helper function for writing bits of an SFR.
/// @param read_bus The read bus function is required to keep bits, which are not part of reg_mask, unchanged.
/// @param write_bus
/// @param reg The address of the SFR to write to.
/// @param reg_mask A bitmask specifying which bits to write.
///                 Unmasked bits in the register are unaffected by this function.
/// @param value The value to write to the specified bits.
void reg_sfr_write(bus_reader_t<uint16_t, uint8_t> read_bus, bus_writer_t<uint16_t, uint8_t> write_bus,
                   uint16_t reg_addr, uint8_t reg_mask, uint8_t value);

/// @brief Reads a value from an SFR
/// @param regs The register context to read from
/// @param address The absolute address of the register
/// @return Value at the specified address. 0 if the address is outside the SFR address range.
addr_read_result_t reg_sfr_bus_read(const sfr_ctx_t &ctx, uint16_t address);

/// @brief Writes a value to an SFR
/// @param regs The register context to write to
/// @param address The absolute address of the register
/// @param value The value to write
/// @note NOP if the address is outside the SFR address range.
addr_bit_mask_t reg_sfr_bus_write(const sfr_ctx_t &ctx, uint16_t address, uint8_t value);

/// @brief Contains the state of physical SFRs.
template <size_t size> struct sfr_phy_map_t
{
    uint16_t first_address;

    std::array<uint8_t, size> implemented_registers;
    std::array<uint8_t, size> registers;
};

/// @brief Reads the value of the physical SFR pointed to by sfr_addr.
///        Returns 0 if the register at sfr_addr is not implemented (at least not in sfr_phy_map).
/// @param map
/// @param sfr
/// @return
template <size_t size> addr_read_result_t sfr_phy_read(const sfr_phy_map_t<size> &map, uint16_t sfr_addr)
{
    uint16_t index = sfr_addr - map.first_address;
    if (index >= size)
        return addr_read_result_none;

    uint8_t value = map.registers[index] & map.implemented_registers[index];

    return addr_read_result_t{.data = value, .mask = map.implemented_registers[index]};
}

/// @brief Write a value to the physical SFR pointed to by sfr_addr.
///        NOP if the register at sfr_addr is not implemented (at least not in sfr_phy_map).
/// @param map
/// @param sfr_addr
/// @param val
template <size_t size> addr_bit_mask_t sfr_phy_write(sfr_phy_map_t<size> &map, uint16_t sfr_addr, uint8_t val)
{
    uint16_t index = sfr_addr - map.first_address;
    if (index >= size)
        return 0x00;

    map.registers[index] = val & map.implemented_registers[index];
    return map.implemented_registers[index];
}

/// @brief Resets all physical SFRs to their initial values according to the datasheet.
/// @param map
/// @param reset_type
template <size_t size> void sfr_phy_reset(sfr_phy_map_t<size> &map, reg_reset_type_t reset_type)
{
    // TODO: Reset correctly
    map.registers.fill(0);
}

template <size_t size>
constexpr std::array<uint8_t, size> sfr_phy_implemented_regs(uint16_t first_address, std::vector<uint16_t> valid_regs)
{
    std::array<uint8_t, size> implemented_registers;
    implemented_registers.fill(0);

    for (auto const &reg : valid_regs)
    {
        uint16_t index = reg - first_address;
        implemented_registers[index] = 0xFF;
    }

    return implemented_registers;
}
