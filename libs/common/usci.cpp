
#include "usci.hpp"
// Function pointers to be called by the interruptions
void (*handleUSCIB0TxIsrFunc)(void);
void (*handleUSCIB0RxIsrFunc)(void);

#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void) {
  if(handleUSCIB0TxIsrFunc != nullptr) {
    handleUSCIB0TxIsrFunc();
  }
}

#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void) {
  if(handleUSCIB0RxIsrFunc != nullptr) {
    handleUSCIB0RxIsrFunc();
  }
}
