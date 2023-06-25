/******************************************************************************
* @file                    usci.cpp
* @author                  Rafael Andrioli Bauer
* @date                    24.05.2023
* @matriculation number    5163344
* @e-mail contact          abauer.rafael@gmail.com
* @brief                   Implementation with the ISRs from USCI
*
******************************************************************************/

#include "usci.hpp"
// Function pointers to be called by the interruptions
void (*handleUSCIB0TxIsrFunc)(void);
void (*handleUSCIB0RxIsrFunc)(void);

#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void) {
  if (handleUSCIB0TxIsrFunc != nullptr) {
    handleUSCIB0TxIsrFunc();
  }
}

#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void) {
  if (handleUSCIB0RxIsrFunc != nullptr) {
    handleUSCIB0RxIsrFunc();
  }
}
