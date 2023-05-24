/***************************************************************************//**
 * @file    i2c.cpp
 * @author  <your name>
 * @date    <date of creation>
 *
 * @brief   <brief description>
 *
 * Here goes a detailed description if required.
 ******************************************************************************/

#include "./i2c.hpp"

namespace AdvancedMicrotech {

// Function pointers to be called by the interruptions
void (*handleUSCIB0TxIsrFunc)(void);
void (*handleUSCIB0RxIsrFunc)(void);


#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void) {
    handleUSCIB0TxIsrFunc();
}

#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void) {
    handleUSCIB0RxIsrFunc();
}
}
