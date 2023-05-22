/******************************************************************************
* @file                    i2c.hpp
* @author                  Rafael Andrioli Bauer
* @date                    14.05.2023
* @matriculation number    5163344
* @e-mail contact          abauer.rafael@gmail.com
* @brief                   <brief description>
*
* Here goes a detailed description if required.
******************************************************************************/

#ifndef EXERCISE_LIBS_I2C_H_
#define EXERCISE_LIBS_I2C_H_

#include <msp430g2553.h>

#include <cstdint>
#include <cassert>

template<typename SDA,
         typename SCL,
         typename SMCLK>
class I2C {
public:

  /**
   * Initialize the I2C state machine. The speed is 100 kBit/s.
   * @param slaveAddress The 7-bit address of the slave (MSB shall always be 0, i.e. "right alignment").
   *
   * (2 pts.)
   */
  constexpr void initialize(const uint8_t slaveAddress) noexcept {
    static constexpr uint8_t BIT_8_MASK = 0x80;
    // The address must be a 7-bit size, so first check if input size is correct.
    if(slaveAddress & BIT_8_MASK) {
      assert(false);
      return;
    }

    SDA::init();
    SCL::init();

    // Enable SW reset to prevent the operation of USCI.
    // According to the datasheet:
    // "Configuring and reconfiguring the USCI module should be done when
    // UCSWRST is set to avoid unpredictable behavior."
    UCB0CTL1 |= UCSWRST;
    // Sets USCI as I2C mode, I2C Master, synchronous mode
    UCB0CTL0 = UCMST+UCMODE_3+UCSYNC;

    UCB0CTL1 = UCSSEL_2+UCSWRST;              // Use SMCLK, keep SW reset

    // TODO Do constexpr equation to calculate this.
    UCB0BR0 = 12;                             // fSCL = SMCLK/12 = ~100kHz
    UCB0BR1 = 0;

    UCB0I2CSA = slaveAddress;                 // Set slave address.
    UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
    IE2 |= UCB0RXIE;                          // Enable RX interrupt
  }

  /**
   * Write a sequence of characters and a stop condition. It stops transmitting further bytes upon a missing acknowledge.
   * @param length Amount of characters to be transferred
   * @param txData Pointer to the data to be transmitted.
   * @param stop The stop condition. Only sent if it is not 0.
   * @return Return 0 if the sequence was acknowledged, 1 if not.
   *
   * (2 pts.)
   */
  uint8_t write(const uint8_t length, uint8_t* txData, const uint8_t stop) noexcept {
    // Before writing, you should always check if the last STOP-condition has already been sent.
    while (UCB0CTL1 & UCTXSTP) {}
    // read UCBBUSY somewhere??

    // TODO check that
    UCB0CTL1 |= UCTR + UCTXSTT;             // I2C TX, start condition
    __bis_SR_register(CPUOFF + GIE);        // CPU off, interrupts enabled
    while (UCB0CTL1 & UCTXSTT);             // Loop until I2C STT is sent
    UCB0CTL1 |= UCTXSTP;                    // I2C stop condition after 1st TX
  }

  /**
   * Performs a read of the I2C bus. It listens for the specific amount of characters defined by the input argument
   * and writes them to the input pointer.
   * The allocation of the memory must be guaranteed by the user.
   * @param length The amu
   * @param rxData [output] Pointer to the container that will be written.
   *
   * (2 pts.)
   */
  void read(const uint8_t length, uint8_t* rxData) {
    // TODO check that
    UCB0CTL1 &= ~UCTR;                      // I2C RX
    UCB0CTL1 |= UCTXSTT;                    // I2C start condition
    while (UCB0CTL1 & UCTXSTT);             // Loop until I2C STT is sent
  }
};

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

#endif /* EXERCISE_LIBS_I2C_H_ */
