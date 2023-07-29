#include "adc.hpp"

namespace Microtech {

void (*handleADC10IsrFunc)(void) = nullptr;

// ADC10 Interruption
 #pragma vector = ADC10_VECTOR
__interrupt void ADC10_ISR(void) {
    if(handleADC10IsrFunc != nullptr) {
      handleADC10IsrFunc();
    }
}
}
