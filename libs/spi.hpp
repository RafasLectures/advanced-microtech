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

/**
 * Class implements an abstraction for the SPI peripheral of the MSP430. It provides read and write methods
 * to make easier to use this peripheral.
 *
 * @tparam MOSI The MOSI pin
 * @tparam MISO The MISO pin
 * @tparam SCLK The SCLK pin
 * @tparam CLOCK The clock source
 * @tparam BUS_SELECTION The pin that allows to select the bus (I2C or SPI)
 * @tparam BAUDRATE The baudrate of the SPI bus
 * @tparam MASTER Whether this is master or slave
 * @tparam MSB Whether the bus is MSB or LSB
 */
template<typename MOSI, typename MISO, typename SCLK, typename CLOCK, typename BUS_SELECTION,
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
    delay_us(200);
      // Make sure the bus is not being used by another instance before change it to SPI mode.
    while(busy()) {}
    MOSI::init();
    MISO::init();
    SCLK::init();

    BUS_SELECTION::init();
    BUS_SELECTION::set_low();


    
    // Set the interruption function pointers to current class interruption handle functions
    USCI::spiTxISRFunction = &SPI_T<MOSI, MISO, SCLK, CLOCK, BUS_SELECTION, BAUDRATE, MASTER, MSB>::handleTxIsr;
    USCI::spiRxISRFunction = &SPI_T<MOSI, MISO, SCLK, CLOCK, BUS_SELECTION, BAUDRATE, MASTER, MSB>::handleRxIsr;

    *USCI::CTL1 |= UCSWRST;

    // As can be seen in USCI_T, if we use USCI_A, we have MCTL, otherwise not, therefore the check.
    // But this is done in compile time, therefore, there is no overhead.
    if (USCI::MCTL != nullptr) {
      *USCI::MCTL = 0;
    }

    // CKPH = 0, CKPL = 1
    *USCI::CTL0 = UCCKPL | (MSB ? UCMSB : 0) | (MASTER ? UCMST : 0) | UCMODE_0 | UCSYNC;

    // Calculate in compile time the value from the register based on the frequency of the clock and the
    // desired baud rate.
    constexpr uint16_t BITCLOCK = (CLOCK::frequency / BAUDRATE);
    static_assert(BITCLOCK <= CLOCK::frequency, "According to page 469 of TIs SLAU144K, BITCLOCK can be up to BRCLK");
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

    USCI::set_spi_active(true);
    transferCount = 0;
    transferBuffer = nullptr;

    USCI::clear_rx_irq();
    USCI::clear_tx_irq();
    USCI::disable_rx_tx_irq();

    *USCI::CTL1 &= ~UCSWRST;  // Release SW reset so USCI is operational
  }

  /**
   * Reads n bytes into the buffer.
   * @param length The number of bytes to read.
   * @param data Buffer to store the read data. The first byte is the address to be read from.
   *
   * (1 pt.)
   */
  static constexpr void read(uint8_t length, uint8_t* data) {
    // Make sure the bus is not busy before manipulating the buffer variables.
    while (busy()) {
    }
    transferCount = length;
    transferBuffer = data;

    USCI::clear_rx_irq();
    USCI::enable_rx_irq();

    // Make sure the TX buffer is ready to accept new data.
    while (!USCI::tx_irq_pending()) {
    }
    *USCI::TXBUF = 0xFF;  // Write dummy data to generate clock output

    // Wait until all data has been written from the RX buffer into the pointer
    while (transferCount > 0) {
    }

    while (busy()) {
    }
    USCI::disable_rx_tx_irq();
    transferBuffer = nullptr;
  }

  /**
   * Writes n bytes from the buffer.
   * @param length The number of bytes to write.
   * @param data Buffer containing the data to write.
   *
   * (1 pt.)
   */
  static constexpr void write(uint8_t length, uint8_t* data) {
    // Make sure the bus is not busy before manipulating the buffer variables.
    while (busy()) {
    }
    transferCount = length;
    transferBuffer = data;

    USCI::enable_tx_irq();
    // Wait until all data has been written from the pointer into the TX buffer
    while (transferCount > 0) {
    }
    while (busy()) {
    }
    USCI::disable_rx_tx_irq();
    transferBuffer = nullptr;
  }

private:
  /**
   * Verify if the bus is busy.
   * @return if the bus is busy (1) or not (0)
   */
  static constexpr uint8_t busy() {
    return *USCI::STAT & UCBUSY;
  }

  /**
   * Method to handle the TX interruption of the SPI.
   */
  static void handleTxIsr() {
    if (transferCount == 0) {
      // If the transfer count is 0, it means we already the transfer has been completed,
      // so we just disable the interruption and do an early return.
      USCI::disable_tx_irq();
      return;
    }
    // The UCx0TXIFG is set when TXBUF is ready to send the next data, so if the interruption happened, it means
    // we can already write the next data
    // Put current value of transferBuffer and point to the next transferBuffer position.
    *USCI::TXBUF = *transferBuffer++;
    transferCount--;  // Decrease transferCount, since one more transfer has been done.
  }

  static void handleRxIsr() {
    if (transferCount == 0) {
      // If the transfer count is 0, it means we already the transfer has been completed,
      // so we just disable the interruption and do an early return.
      USCI::clear_rx_irq();
      USCI::disable_rx_irq();
      return;
    }
    // The UCx0RXIFG is set when RXBUF has received a complete character, so there has been a RX transfer
    // Put current value from RXBUF to transferBuffer and point to the next transferBuffer position.
    *transferBuffer++ = *USCI::RXBUF;
    transferCount--;  // Decrease transferCount, since one more transfer has been done.
    if (transferCount == 0) {
      // If the transfer count is 0, it means we already the transfer has been completed,
      // so we just disable the interruption.
      USCI::disable_rx_irq();
    } else {
      // Writes dummy data to get more clock
      while (!USCI::tx_irq_pending()) {
      }
      *USCI::TXBUF = 0xFF;  // Write dummy data to generate clock output
    }
    USCI::clear_rx_irq();
  }

  static volatile uint8_t transferCount;    // Variable used to know how many bytes are left to be transferred.
  static volatile uint8_t* transferBuffer;  // Pointer to where to write/get the received/send data
};

template<typename MOSI, typename MISO, typename SCLK, typename CLOCK, typename BUS_SELECTION, const uint32_t BAUDRATE,
         const bool MASTER, const bool MSB>
volatile uint8_t SPI_T<MOSI, MISO, SCLK, CLOCK, BUS_SELECTION, BAUDRATE, MASTER, MSB>::transferCount = 0;
template<typename MOSI, typename MISO, typename SCLK, typename CLOCK, typename BUS_SELECTION, const uint32_t BAUDRATE,
         const bool MASTER, const bool MSB>
volatile uint8_t* SPI_T<MOSI, MISO, SCLK, CLOCK, BUS_SELECTION, BAUDRATE, MASTER, MSB>::transferBuffer = 0;
}  // namespace AdvancedMicrotech

#endif /* LIBS_SPI_H_ */
