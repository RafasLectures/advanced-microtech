/*******************************************************************************
 * @file                    parallelbus.hpp
 * @author                  Rafael Andrioli Bauer
 * @date                    10.05.2023
 * @matriculation number    5163344
 * @e-mail contact          abauer.rafael@gmail.com
 *
 * @brief   Header file that implements a parallel data bus.
 ******************************************************************************/

#ifndef LIBS_COMMON_PARALLELBUS_HPP_
#define LIBS_COMMON_PARALLELBUS_HPP_

#include "gpio.hpp"

/**
 * Object that defines a parallel port at an specific port.
 *
 * It basically uses the functionality of the GPIOs but the user sees it
 * as one data bus.
 * @tparam PORT Port to be used as parallel bus
 * @tparam MASK Mask of pins to be used. So from pin 0 to 3 it would be mask Ox0F.
 * @tparam SHIFT Allows the user to have the value to be reported shifted. For example:
 *               If the mask is 0xF0, but the data is a 4 bit value, when setting the shift to
 *               4, when performing a read, the data reported is 0x0F. Its default value is 0.
 */
template<const int PORT, const uint8_t MASK, const uint8_t SHIFT = 0>
struct PARALLEL_BUS_T {
  // Just a static const definition of the ports. The arrays are defined in gpio.hpp
  static constexpr volatile uint8_t *PxIN = ports[PORT - 1][0];
  static constexpr volatile uint8_t *PxOUT = ports[PORT - 1][1];
  static constexpr volatile uint8_t *PxDIR = ports[PORT - 1][2];
  static constexpr volatile uint8_t *PxIFG = ports[PORT - 1][3];
  static constexpr volatile uint8_t *PxIES = ports[PORT - 1][4];
  static constexpr volatile uint8_t *PxIE = ports[PORT - 1][5];
  static constexpr volatile uint8_t *PxSEL = ports[PORT - 1][6];
  static constexpr volatile uint8_t *PxSEL2 = ports[PORT - 1][7];
  static constexpr volatile uint8_t *PxREN = ports[PORT - 1][8];
  static constexpr volatile uint8_t *PxIFGS = ports[PORT - 1][9];

  /**
   * Method to initialize the bus. It sets the initial values of the gpio registers for this port.
   * The default value is that all the pins are output.s
   */
  static constexpr void init() {
    *PxDIR |= MASK;  // Sets pins as output
    // Set pins as IOs
    *PxSEL &= ~MASK;
    *PxSEL2 &= ~MASK;

    *PxREN &= ~MASK;  // Disable pull up/down resistor
    *PxIE &= ~MASK;   // Interruptions are disabled
    *PxIES &= ~MASK;
    *PxIFG &= ~MASK;  // make sure interruption flags are 0

    *PxOUT &= ~MASK;  // Set initial output value
  }

  /**
   * Method that reads the value in the bus.
   * It sets the pins as inputs and performs a read of the bus.
   * @return The current value in the bus.
   */
  static constexpr uint8_t read() noexcept {
    // Set pins as input
    *PxDIR &= ~MASK;
    return (*PxIN & MASK) >> SHIFT;
  }

  /**
   * Writes a value to the bus.
   * @param newValue Value to be written.
   */
  static constexpr void write(const uint8_t newValue) noexcept {
    // Set pins as output
    *PxDIR |= MASK;
    // Write value to output
    *PxOUT &= ~MASK;
    *PxOUT |= newValue;
  }
};
#endif /* LIBS_COMMON_PARALLELBUS_HPP_ */
