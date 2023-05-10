#ifndef LIBS_COMMON_PARALLELBUS_HPP_
#define LIBS_COMMON_PARALLELBUS_HPP_

#include "gpio.hpp"


template<const int PORT,
    const uint8_t MASK,
    const uint8_t SHIFT = 0>
struct PARALLEL_BUS_T {
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

    static constexpr void init() {
        *PxDIR |= MASK;       // Sets pins as output
        // Set pins as IOs
        *PxSEL &= ~MASK;
        *PxSEL2 &= ~MASK;

        *PxREN &= ~MASK;      // Disable pull up/down resistor
        *PxIE &= ~MASK;       // Interruptions are disabled
        *PxIES &= ~MASK;
        *PxIFG &= ~MASK;      // make sure interruption flags are 0

        *PxOUT &= ~MASK;      // Set initial output value
    }

    static constexpr uint8_t read() noexcept {
        // Set pins as input
        *PxDIR &= ~MASK;
        return (*PxIN & MASK) >> SHIFT;
    }

    static constexpr void write(const uint8_t newValue) noexcept {
        // set pins as output
        *PxDIR |= MASK;
        // Write value to output
        *PxOUT |= (MASK & newValue);
        *PxOUT &= ~(MASK & ~newValue);
        return;
    }

};
#endif /* LIBS_COMMON_PARALLELBUS_HPP_ */
