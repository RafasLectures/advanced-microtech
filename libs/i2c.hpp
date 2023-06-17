/******************************************************************************
 * @file                    i2c.hpp
 * @author                  Rafael Andrioli Bauer
 * @date                    22.05.2023
 * @matriculation number    5163344
 * @e-mail contact          abauer.rafael@gmail.com
 * @brief                  Header file with the I2C class
 *
 *  Its implementation must be in the header because it is templated.
 ******************************************************************************/

#ifndef EXERCISE_LIBS_I2C_H_
#define EXERCISE_LIBS_I2C_H_

#include <msp430g2553.h>

#include "common/usci.hpp"
#include <cassert>
#include <cstdint>

namespace AdvancedMicrotech {

// Function pointers that are called by the interruptions.
// They are set in the initialize methods of the I2C.
extern void (*handleUSCIB0TxIsrFunc)(void);
extern void (*handleUSCIB0RxIsrFunc)(void);

/**
 * Class implements an abstraction for the I2C peripheral of the MSP430. It provides read and write methods
 * to make easier to use this peripheral
 *
 * @tparam SDA SDA pin to be used by the I2C
 * @tparam SCL SCL pin to be used by the I2C
 * @tparam CLOCK Source clock to be used the peripheral
 * @tparam BAUDRATE The desired baud rate of the I2C
 * @tparam IS_MASTER Boolean to define weather the I2C is a master or a slave. True = master.
 */
template<typename SDA, typename SCL, typename CLOCK, uint32_t BAUDRATE = 100000, const bool IS_MASTER = true>
class I2C_T {
public:
  using USCI = USCI_T<USCI_MODULE::USCI_B, 0>;  // Alias to get the USCIB0 registers and enable interruptions.

  /**
   * Initialize the I2C state machine. The speed is 100 kBit/s.
   * @param slaveAddress The 7-bit address of the slave (MSB shall always be 0, i.e. "right alignment").
   *
   * (2 pts.)
   */
  static constexpr void initialize(const uint8_t slaveAddress) noexcept {
    // The address must be a 7-bit size, so first check if input size is correct.
    if (slaveAddress & 0x80) {
      assert(false);
      return;
    }
    // Initialize the IOs
    SDA::init();
    SCL::init();

    // Set the interruption function pointers to current class interruption handle functions
    handleUSCIB0TxIsrFunc = &I2C_T<SDA, SCL, CLOCK, BAUDRATE, IS_MASTER>::handleTxIsr;
    handleUSCIB0RxIsrFunc = &I2C_T<SDA, SCL, CLOCK, BAUDRATE, IS_MASTER>::handleRxIsr;

    // Enable SW reset to prevent the operation of USCI.
    // According to the datasheet:
    // "Configuring and reconfiguring the USCI module should be done when
    // UCSWRST is set to avoid unpredictable behavior."
    *USCI::CTL1 |= UCSWRST;

    // Sets as I2C master or not, USCI as I2C mode and synchronous mode
    *USCI::CTL0 = (IS_MASTER ? UCMST : 0) | UCMODE_3 | UCSYNC;

    // Calculate in compile time the value from the register based on the frequency of the clock and the
    // desired baud rate.
    constexpr uint16_t BITCLOCK = (CLOCK::frequency / BAUDRATE);
    static_assert(BITCLOCK <= CLOCK::frequency / 4,
                  "According to page 498 of TIs SLAU144K, BITCLOCK can be up to is BRCLK/4");
    *USCI::BR0 = BITCLOCK & 0xff;
    *USCI::BR1 = BITCLOCK >> 8;

    *USCI::I2CSA = slaveAddress;  // Set slave address.

    // Sets USCI clock source according to template argument
    if (CLOCK::type == CLOCK_TYPE::CLOCK_TYPE_ACLK) {
      *USCI::CTL1 = UCSSEL_1 | UCSWRST;
    } else {
      // Sets to SMCLK. UCLKI is not an option, since it is only used for SPI.
      *USCI::CTL1 = UCSSEL_2 | UCSWRST;
    }

    *USCI::CTL1 &= ~UCSWRST;  // Release SW reset so USCI is operational

    transferCount = 0;
    transferBuffer = nullptr;
    nackReceived = false;

    USCI::clear_rx_irq();
    USCI::clear_tx_irq();
    USCI::disable_rx_tx_irq();
  }

  /**
   * Write a sequence of characters and a stop condition. It stops transmitting further bytes upon a missing
   * acknowledge.
   * @param length Amount of characters to be transferred
   * @param txData Pointer to the data to be transmitted.
   * @param stop The stop condition. Only sent if it is not 0.
   * @return Return 0 if the sequence was acknowledged, 1 if not.
   *
   * (2 pts.)
   */
  static bool write(const uint8_t length, uint8_t* txData, const bool stop) noexcept {
    // Before writing, you should always check if the last STOP-condition has already been sent.
    while (*USCI::CTL1 & UCTXSTP) {
    }
    // Make sure the bus is not busy
    //while (*USCI::STAT & UCBBUSY) {
    //}

    transferBuffer = txData;  // Assign transfer buffer to the txData pointer
    transferCount = length;   // Get length
    nackReceived = false;     // Initialize NACK variable

    USCI::clear_tx_irq();     // Make sure there is no TX IRQ pending
    // Enable TX and RX interruptions, so we can also get the NACKs.
    // The interruption is responsible to disable it again.
    USCI::enable_rx_tx_irq();

    *USCI::CTL1 |= UCTR + UCTXSTT;  // I2C as TX and send start condition

    // This makes the function a blocking function.
    // Wait until we have finished to transfer all data.
    while (transferCount > 0) {
    }

    if (stop) {
      *USCI::CTL1 |= UCTXSTP;  // I2C stop condition
    }

    // Reset internal variables and return if there were any NACKs.
    transferBuffer = nullptr;
    transferCount = 0;
    return nackReceived;
  }

  /**
   * Performs a read of the I2C bus. It listens for the specific amount of characters defined by the input argument
   * and writes them to the input pointer.
   * The allocation of the memory must be guaranteed by the user.
   * @param length The amount of data to be transferred
   * @param rxData [output] Pointer to the container that will be written.
   *
   * (2 pts.)
   */
  static void read(const uint8_t length, uint8_t* rxData) {
    // Before reading, you should always check if the last STOP-condition has already been sent.
    while (*USCI::CTL1 & UCTXSTP) {
    }

    transferBuffer = rxData;  // Assign transfer buffer to the rxData pointer
    transferCount = length;   // Get length
    nackReceived = false;     // Initialize NACK variable

    // Make sure there are no interruptions pending.
    USCI::clear_tx_irq();
    USCI::clear_rx_irq();

    // Enable TX and RX interruptions, so we can also get the NACKs.
    // The interruption is responsible to disable it again.
    USCI::enable_rx_tx_irq();

    *USCI::CTL1 &= ~UCTR;    // I2C RX
    *USCI::CTL1 |= UCTXSTT;  // I2C start condition

    // This makes the function a blocking function.
    // Wait until all data has been transfered.
    while (transferCount > 0) {
    }

    // Reset internal variables.
    transferBuffer = nullptr;
    transferCount = 0;
  }

  static void handleTxIsr() {
    if (transferCount == 0) {
      // If the transfer count is 0, it means we already the transfer has been completed,
      // so we just disable the interruption and do an early return.
      USCI::disable_rx_tx_irq();
      return;
    }

    // The UCx0TXIFG is set when TXBUF is empty, so there has been a TX transfer
    if (USCI::tx_irq_pending()) {
      // Put current value of transferBuffer and point to the next transferBuffer position.
      *USCI::TXBUF = *transferBuffer++;
      transferCount--;  // Decrease transferCount, since one more transfer has been done.
    }

    // The UCx0RXIFG is set when RXBUF has received a complete character, so there has been a RX transfer
    if (USCI::rx_irq_pending()) {
      // Put current value from RXBUF to transferBuffer and point to the next transferBuffer position.
      *transferBuffer++ = *USCI::RXBUF;
      transferCount--;  // Decrease transferCount, since one more transfer has been done.

      // If you only want to receive one byte, you instantly have to write a STOP-condition
      // after the START-condition got sent.
      if (transferCount == 1) {
        *USCI::CTL1 |= UCTXSTP;  // I2C stop condition
      }
    }
  }

  static void handleRxIsr() {
    // If there is a NACK, set internal variable to true and clear flag
    if (*USCI::STAT & UCNACKIFG) {
      nackReceived = true;
      *USCI::STAT &= ~UCNACKIFG;
    }
  }

private:
  static uint8_t transferCount;    // Variable used to know how many bytes are left to be transferred.
  static uint8_t* transferBuffer;  // Pointer to where to write/get the received/send data
  static bool nackReceived;        // Variable used to know if a NACK was received.
};

template<typename SDA, typename SCL, typename CLOCK, uint32_t BAUDRATE, const bool IS_MASTER>
uint8_t I2C_T<SDA, SCL, CLOCK, BAUDRATE, IS_MASTER>::transferCount;

template<typename SDA, typename SCL, typename CLOCK, uint32_t BAUDRATE, const bool IS_MASTER>
uint8_t* I2C_T<SDA, SCL, CLOCK, BAUDRATE, IS_MASTER>::transferBuffer;

template<typename SDA, typename SCL, typename CLOCK, uint32_t BAUDRATE, const bool IS_MASTER>
bool I2C_T<SDA, SCL, CLOCK, BAUDRATE, IS_MASTER>::nackReceived;

/******************************************************************************
 * FUNCTION PROTOTYPES
 *****************************************************************************/

#if 0
// Initialize the I2C state machine. The speed should be 100 kBit/s.
// <addr> is the 7-bit address of the slave (MSB shall always be 0, i.e. "right alignment"). (2 pts.)
void i2c_init (unsigned char addr);

// Write a sequence of <length> characters from the pointer <txData>.
// Return 0 if the sequence was acknowledged, 1 if not. Also stop transmitting further bytes upon a missing acknowledge.
// Only send a stop condition if <stop> is not 0. (2 pts.)
unsigned char i2c_write(unsigned char length, unsigned char * txData, unsigned char stop);

// Returns the next <length> characters from the I2C interface. (2 pts.)
void i2c_read(unsigned char length, unsigned char * rxData);
#endif

}  // namespace AdvancedMicrotech
#endif /* EXERCISE_LIBS_I2C_H_ */
