/******************************************************************************
 * @file    i2c.cpp
 * @author                  Rafael Andrioli Bauer
 * @date                    22.05.2023
 * @matriculation number    5163344
 * @e-mail contact          abauer.rafael@gmail.com
 *
 * @brief   File with the declaration of the USCI interruptions
 ******************************************************************************/

#include "./i2c.hpp"

namespace AdvancedMicrotech {

// Function pointers to be called by the interruptions
void (*handleUSCIB0TxIsrFunc)(void);
void (*handleUSCIB0RxIsrFunc)(void);

}  // namespace AdvancedMicrotech

#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void) {
  AdvancedMicrotech::handleUSCIB0TxIsrFunc();
}

#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void) {
  AdvancedMicrotech::handleUSCIB0RxIsrFunc();
}
