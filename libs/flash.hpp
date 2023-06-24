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

  static constexpr void read(uint32_t address, uint16_t length, uint8_t* data) {
    waitUntilNotBusy();
    sendInstructionToAddress(address, READ_DATA, false);
    SPI::read(length, data);
    CS::set_high();
  }

  static constexpr void write(uint32_t address, uint16_t length, uint8_t* data) {
    waitUntilNotBusy();
    sendInstructionToAddress(address, SECTOR_ERASE, true);
    waitUntilNotBusy();
    sendInstruction(WRITE_ENABLE, true);
    waitUntilNotBusy();
    sendInstructionToAddress(address, PAGE_PROGRAM, false);
    SPI::write(length, data);
    CS::set_high();
  }

private:
  struct StatusRegister {
    uint8_t busy;
    uint8_t writeEnabled;
  };
  static constexpr StatusRegister readStatusRegister() {
    constexpr uint8_t BUSY_MASK = 0x01;
    constexpr uint8_t WRITE_ENABLED_MASK = 0x02;
    uint8_t status = 0;
    sendInstruction(READ_STATUS_REGISTER, false);
    SPI::read(sizeof(status), &status);
    CS::set_high();
    // StatusRegister retVal;
    // retVal.busy = status & BUSY_MASK;
    // retVal.writeEnabled = status & WRITE_ENABLED_MASK;
    return {status & BUSY_MASK, status & WRITE_ENABLED_MASK};
  }

  static constexpr void sendInstruction(uint8_t instruction, bool finalMessage) {
    CS::set_low();
    SPI::write(1, &instruction);
    if (finalMessage) {
      CS::set_high();
    }
  }

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

  static constexpr void waitUntilNotBusy() {
    StatusRegister statusRegister = readStatusRegister();
    while (statusRegister.busy) {
      statusRegister = readStatusRegister();
    }
  }
  static constexpr uint8_t WRITE_ENABLE{0x06};
  static constexpr uint8_t WRITE_DISABLE{0x04};
  static constexpr uint8_t READ_ID{0x9F};
  static constexpr uint8_t READ_STATUS_REGISTER{0x05};
  static constexpr uint8_t WRITE_STATUS_REGISTER{0x01};
  static constexpr uint8_t READ_DATA{0x03};
  static constexpr uint8_t READ_DATA_FAST{0x0B};
  static constexpr uint8_t PAGE_PROGRAM{0x02};
  static constexpr uint8_t SECTOR_ERASE{0xD8};
  static constexpr uint8_t BULK_ERASE{0xC7};
};

}  // namespace AdvancedMicrotech
// Read <length> bytes into <rxData> starting from address <address> (1 pt.)
void flash_read(long int address, unsigned char length, unsigned char* rxData);

// Write <length> bytes from <txData>, starting at address <address> (1 pt.)
void flash_write(long int address, unsigned char length, unsigned char* txData);

// Returns 1 if the FLASH is busy or 0 if not.
// Note: this is optional. You will probably need this, but you don't have to
// implement this if you solve it differently.
unsigned char flash_busy(void);

#endif /* LIBS_FLASH_H_ */
