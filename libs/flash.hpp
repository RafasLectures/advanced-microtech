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

template<typename CS, typename HOLD, typename WP, typename SPI>
class FLASH_T {
public:

  /**
   * Initialise the flash chip
   */
  static constexpr void init() {
    CS::init();
    HOLD::init();
    WP::init();
    SPI::init();
    uint8_t ID[3]{0};

    CS::set_low();
    uint8_t instruction = 0x9F;
    SPI::write(1, &instruction);
    SPI::read(3, &ID[0]);
    CS::set_high();

    if(ID[0] != 0x20) {
      while(1) {
        __no_operation();
      }
    }
    return;
  }

  static constexpr void read(uint32_t address, uint16_t length, uint8_t* data) {
    CS::set_low();
    uint8_t addressBuffer[3] = {static_cast<uint8_t>(address << 16), static_cast<uint8_t>(address << 8), static_cast<uint8_t>(address)};
    SPI::write(3,&addressBuffer[0]);
    SPI::read(length, data);
    CS::set_high();
  }

  static constexpr void write(uint16_t address, uint16_t length, uint8_t* data) {
    CS::set_low();
    uint8_t addressBuffer[3] = {static_cast<uint8_t>(address << 16), static_cast<uint8_t>(address << 8), static_cast<uint8_t>(address)};
    SPI::write(3,&addressBuffer[0]);
    SPI::write(length, data);
    CS::set_high();
  }
private:

};

// Read <length> bytes into <rxData> starting from address <address> (1 pt.)
void flash_read(long int address, unsigned char length, unsigned char * rxData);

// Write <length> bytes from <txData>, starting at address <address> (1 pt.)
void flash_write(long int address, unsigned char length, unsigned char * txData);

// Returns 1 if the FLASH is busy or 0 if not.
// Note: this is optional. You will probably need this, but you don't have to
// implement this if you solve it differently.
unsigned char flash_busy(void);

#endif /* LIBS_FLASH_H_ */
