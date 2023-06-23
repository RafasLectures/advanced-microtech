/*******************************************************************************
 * @file                    spi.hpp
 * @author                  Rafael Andrioli Bauer
 * @date                    21.06.2023
 * @matriculation number    5163344
 * @e-mail contact          abauer.rafael@gmail.com
 *
 * @brief   Header file with the prototypes of the SPI C functions as well as the
 *          declaration of the SPI class.
 *          Its implementation must be in the header because it is templated.
 ******************************************************************************/

#ifndef LIBS_SPI_H_
#define LIBS_SPI_H_

#include "common/usci.hpp"
#include "libs/common/clocks.hpp"
#include <msp430g2553.h>

namespace AdvancedMicrotech {

template< typename MOSI, typename MISO, typename SCLK, typename CLOCK, typename BUS_SELECTION,
         const uint32_t BAUDRATE = 1000000, const bool MASTER = true, const bool MSB = true>
class SPI_T {
  using USCI = USCI_T<USCI_MODULE::USCI_B, 0>;  // Alias to get the USCIB0 registers and enable interruptions.
public:
  /**
   * Sets the USCI-machine to SPI mode switch the 74HCT4066
   *
   *  (1 pt.)
   */
  static constexpr void init() {
    MOSI::init();
    MISO::init();
    SCLK::init();
    BUS_SELECTION::init();
    BUS_SELECTION::set_low();
    delay_ms(1);        // Added delay to make sure IO levels have settled

    *USCI::CTL1 |= UCSWRST;

    // As can be seen in USCI_T, if we use USCI_A, we have MCTL, otherwise not, therefore the check.
    // But this is done in compile time, therefore, there is no overhead.
    if(USCI::MCTL != nullptr) {
      *USCI::MCTL = 0;
    }

    // CKPH = 0, CKPL = 1
    *USCI::CTL0 = UCCKPL | (MSB ? UCMSB : 0) | (MASTER ? UCMST : 0) | UCMODE_0 | UCSYNC;

    // Calculate in compile time the value from the register based on the frequency of the clock and the
    // desired baud rate.
    constexpr uint16_t BITCLOCK = (CLOCK::frequency / BAUDRATE);
    static_assert(BITCLOCK <= CLOCK::frequency,
                  "According to page 469 of TIs SLAU144K, BITCLOCK can be up to BRCLK");
    *USCI::BR0 = BITCLOCK & 0xff;
    *USCI::BR1 = BITCLOCK >> 8;

    // Set clock source
    if (CLOCK::type == CLOCK_TYPE::CLOCK_TYPE_ACLK) {
      *USCI::CTL1 = UCSSEL_1 | UCSWRST;
    } else {
      // Sets to SMCLK. UCLKI is not an option, since it is only used for SPI.
      *USCI::CTL1 = UCSSEL_2 | UCSWRST;
    }

    // Make sure loopback is disabled
    *USCI::STAT = 0x00;

    *USCI::CTL1 &= ~UCSWRST;  // Release SW reset so USCI is operational

    USCI::spiTxISRFunction = &SPI_T<MOSI, MISO, SCLK, CLOCK, BUS_SELECTION, BAUDRATE, MASTER, MSB>::handleTxIsr;
    USCI::spiRxISRFunction = &SPI_T<MOSI, MISO, SCLK, CLOCK, BUS_SELECTION, BAUDRATE, MASTER, MSB>::handleRxIsr;
    USCI::set_spi_active(true);
  }

  /**
   * Reads n bytes into the buffer.
   * @param length The number of bytes to read.
   * @param data Buffer to store the read data. The first byte is the address to be read from.
   *
   * (1 pt.)
   */
  static constexpr void read(uint8_t length, uint8_t *data) {
    while(busy()) {
    }
    transferCount = length;
    transferBuffer = data;

    USCI::clear_rx_irq();
    USCI::enable_rx_irq();
    *USCI::TXBUF = 0xFF; // Write dummy data to generate clock output
    while (transferCount > 0) {
    }
    USCI::disable_rx_tx_irq();
    transferBuffer = nullptr;
    while(busy()) {
    }
  }

  /**
   * Writes n bytes from the buffer.
   * @param length The number of bytes to write.
   * @param data Buffer containing the data to write.
   *
   * (1 pt.)
   */
  static constexpr void write(uint8_t length, uint8_t *data) {
    while(busy()) {
    }
    transferCount = length;
    transferBuffer = data;

    USCI::enable_tx_irq();
    while (transferCount > 0) {
    }
    USCI::disable_rx_tx_irq();
    transferBuffer = nullptr;
    while(busy()) {
    }
  }

private:
  /**
   *
   * @return
   */
  static constexpr uint8_t busy() {
    return *USCI::STAT & UCBUSY;
  }
  static void handleTxIsr() {
    if (transferCount == 0) {
      // If the transfer count is 0, it means we already the transfer has been completed,
      // so we just disable the interruption and do an early return.
      USCI::disable_tx_irq();
      return;
    }

    // The UCx0TXIFG is set when TXBUF is empty, so there has been a TX transfer
    while(!USCI::tx_irq_pending()) {
    }
    // Put current value of transferBuffer and point to the next transferBuffer position.
    *USCI::TXBUF = *transferBuffer++;
    transferCount--;  // Decrease transferCount, since one more transfer has been done.
  }

  static void handleRxIsr() {
    if (transferCount == 0) {

      return;
    }
    while(!USCI::rx_irq_pending()) {
    }
    // The UCx0RXIFG is set when RXBUF has received a complete character, so there has been a RX transfer
    // Put current value from RXBUF to transferBuffer and point to the next transferBuffer position.
    *transferBuffer++ = *USCI::RXBUF;
    transferCount--;  // Decrease transferCount, since one more transfer has been done.
    if(transferCount == 0) {
        // If the transfer count is 0, it means we already the transfer has been completed,
        // so we just disable the interruption and do an early return.
        USCI::disable_rx_irq();
    } else {
        // Writes dummy data to get more clock
        while(!USCI::tx_irq_pending()) {
        }
        *USCI::TXBUF = 0xFF; // Write dummy data to generate clock output
    }
    USCI::clear_rx_irq();
  }

  static uint8_t transferCount;    // Variable used to know how many bytes are left to be transferred.
  static uint8_t* transferBuffer;  // Pointer to where to write/get the received/send data
};

template<typename MOSI, typename MISO, typename SCLK, typename CLOCK, typename BUS_SELECTION,
  const uint32_t BAUDRATE, const bool MASTER, const bool MSB>
uint8_t SPI_T<MOSI, MISO, SCLK, CLOCK, BUS_SELECTION, BAUDRATE, MASTER, MSB>::transferCount = 0;
template<typename MOSI, typename MISO, typename SCLK, typename CLOCK, typename BUS_SELECTION,
         const uint32_t BAUDRATE, const bool MASTER, const bool MSB>
uint8_t* SPI_T<MOSI, MISO, SCLK, CLOCK, BUS_SELECTION, BAUDRATE, MASTER, MSB>::transferBuffer = 0;
}
// Interrupt service routines in your spi.c (1 pt.)

// Returns 1 if the SPI is still busy or 0 if not.
// Note: this is optional. You will most likely need this, but you don't have
// to implement or use this.
unsigned char spi_busy(void);

#endif /* LIBS_SPI_H_ */
