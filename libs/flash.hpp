/*******************************************************************************
 * @file                    flash.hpp
 * @author                  Rafael Andrioli Bauer
 * @date                    22.06.2023
 * @matriculation number    5163344
 * @e-mail contact          abauer.rafael@gmail.com
 *
 * @brief   Header file with the prototypes of the Flash C functions as well as the
 *          declaration of the Flash class.
 *          Its implementation must be in the header because it is templated.
 ******************************************************************************/

#ifndef LIBS_FLASH_H_
#define LIBS_FLASH_H_

#include "spi.hpp"
namespace AdvancedMicrotech {

template<typename CS, typename HOLD, typename WP, typename SPI>
class FLASH_T {
  static constexpr uint8_t M25P16_MANUFACTURER = 0x20;
  static constexpr uint8_t M25P16_MEMORY_TYPE = 0x20;

public:
  /**
   * Initialize the flash chip
   */
  static constexpr void init() {
    CS::init();
    HOLD::init();
    WP::init();
    SPI::init();
    uint8_t ID[3]{0};

    sendInstruction(READ_ID, false);
    SPI::read(sizeof(ID), &ID[0]);
    CS::set_high();

    if (ID[0] != M25P16_MANUFACTURER || ID[1] != M25P16_MEMORY_TYPE) {
      while (1) {
        __no_operation();
      }
    }
    return;
  }

  /**
   * Read <length> bytes into <data> starting from address <address>.
   * The address will be incremented by the number of bytes written.
   * @param address The initial address of the write operation
   * @param length The number of bytes to write
   * @param data Pointer to the initial data to write. The data in the next addresses starting at this pointer
   *             will be written
   *
   * (1 pt.)
   */
  static constexpr void read(uint32_t address, uint16_t length, uint8_t* data) {
    waitUntilNotBusy();
    sendInstructionToAddress(address, READ_DATA, false);
    SPI::read(length, data);
    CS::set_high();
  }

  /**
   * Write <length> bytes from <data>, starting at address <address>.
   * The address will be incremented by the number of bytes written.
   * @param address The initial address of the write operation
   * @param length The number of bytes to write
   * @param data Pointer to the initial data to write. The data in the next addresses starting at this pointer
   *             will be written
   *
   *  (1 pt.)
   */
  static constexpr void write(uint32_t address, uint16_t length, uint8_t* data) {
    sectorErase(address);
    waitUntilNotBusy();
    sendInstruction(WRITE_ENABLE, true);
    waitUntilNotBusy();
    sendInstructionToAddress(address, PAGE_PROGRAM, false);
    SPI::write(length, data);
    CS::set_high();
  }

  /**
   * Method to erase a sector of the flash chip.
   * @param address Address within the sector to erase. Any address within the sector will trigger the whole
   *                sector erase.
   */
  static constexpr void sectorErase(uint32_t address) {
    sendInstruction(WRITE_ENABLE, true);
    waitUntilNotBusy();
    sendInstructionToAddress(address, SECTOR_ERASE, true);
  }

private:
  /**
   * Struct that defines the information in the Status Register.
   */
  struct StatusRegister {
    static constexpr uint8_t BUSY_MASK = 0x01;
    static constexpr uint8_t WRITE_ENABLED_MASK = 0x02;
    StatusRegister(bool newBusy, bool newWriteEnabled) : busy(newBusy), writeEnabled(newWriteEnabled) {}
    bool busy;          ///< Indicates if the flash is busy or not
    bool writeEnabled;  ///< Indicates if the flash is in write mode or not
  };
  /**
   * Method to read the Status Register.
   * @return the current values of the Status Register
   */
  static constexpr StatusRegister readStatusRegister() {
    uint8_t status = 0;
    sendInstruction(READ_STATUS_REGISTER, false);
    SPI::read(1, &status);
    CS::set_high();
    return StatusRegister(status & StatusRegister::BUSY_MASK, status & StatusRegister::WRITE_ENABLED_MASK);
  }

  /**
   * Method to send an instruction to the flash chip.
   * @param instruction The instruction to send to the flash chip
   * @param finalMessage Whether at the end of sending the instruction the CS must be set to high or not.
   */
  static constexpr void sendInstruction(uint8_t instruction, bool finalMessage) {
    CS::set_low();
    SPI::write(1, &instruction);
    if (finalMessage) {
      CS::set_high();
    }
  }

  /**
   * Method to send an instruction to an specific address in the flash chip.
   * Usually when sending an instruction to a specific address, the address must be sending right after the
   * instruction.
   * @param address Address to which the instruction will be sent
   * @param instruction Instruction to send to the flash chip
   * @param finalMessage Whether at the end of sending the instruction the CS must be set to high or not.
   */
  static constexpr void sendInstructionToAddress(uint32_t address, uint8_t instruction, bool finalMessage) {
    uint8_t instructionAndAddress[4] = {instruction, static_cast<uint8_t>((address & 0xFF0000) >> 16),
                                        static_cast<uint8_t>((address & 0x00FF00) >> 8),
                                        static_cast<uint8_t>((address & 0xFF))};
    CS::set_low();
    SPI::write(sizeof(instructionAndAddress), &instructionAndAddress[0]);
    if (finalMessage) {
      CS::set_high();
    }
  }

  /**
   * Method is a blocking call that waits until the flash is not busy.
   */
  static constexpr void waitUntilNotBusy() {
    StatusRegister statusRegister = readStatusRegister();
    while (statusRegister.busy) {
      statusRegister = readStatusRegister();
    }
  }

  static constexpr uint8_t WRITE_ENABLE{0x06};           ///< Write enable instruction
  static constexpr uint8_t WRITE_DISABLE{0x04};          ///< Write disable instruction
  static constexpr uint8_t READ_ID{0x9F};                ///< Read ID instruction
  static constexpr uint8_t READ_STATUS_REGISTER{0x05};   ///< Read Status Register instruction
  static constexpr uint8_t WRITE_STATUS_REGISTER{0x01};  ///< Write Status Register instruction
  static constexpr uint8_t READ_DATA{0x03};              ///< Read data instruction
  static constexpr uint8_t READ_DATA_FAST{0x0B};         ///< Read data fast instruction
  static constexpr uint8_t PAGE_PROGRAM{0x02};           ///< Page program instruction
  static constexpr uint8_t SECTOR_ERASE{0xD8};           ///< Sector erase instruction
  static constexpr uint8_t BULK_ERASE{0xC7};             ///< Bulk erase instruction
};

}  // namespace AdvancedMicrotech

#endif /* LIBS_FLASH_H_ */
