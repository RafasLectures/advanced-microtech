#include "Timer.hpp"

namespace Microtech {

void (*handleTimerA0IsrFunc)(void);
void (*handleTimerA1IsrFunc)(void);

}

// Timer0 Interruption
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A_CCR0_ISR(void) {
  Microtech::handleTimerA0IsrFunc();
}

// Timer1 Interruption
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1_A_CCR0_ISR(void) {
  Microtech::handleTimerA1IsrFunc();
}
