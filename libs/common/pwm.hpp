/******************************************************************************
* @file                    Pwm.hpp
* @author                  Rafael Andrioli Bauer
* @date                    14.12.2022
* @matriculation number    5163344
* @e-mail contact          abauer.rafael@gmail.com
*
* @brief   Header contains abstraction of PWM
*
* Description: In order to abstract the manipulation of registers and
*              ease up maintainability of the application code, this header
*              provides an easy interface to use the PWM
******************************************************************************/
#ifndef MICROTECH_PWM_HPP
#define MICROTECH_PWM_HPP

#include "Timer.hpp"
#include "GPIOs.hpp"

#include "IQmathLib.h"
#include <msp430g2553.h>
#include <chrono>
#include <memory>

namespace Microtech {
/**
* Class to abstract the PWM.
*/
template <typename TIMER, typename PIN>
class PWM_T {
public:
  using IsrCallback = void(*)();
  static constexpr _iq MAX_DUTY_CYCLE = _IQ(100.0);

  static constexpr void initialize() {
    TIMER::initialize();
    PIN::init();
  }

 /**
  * Class to set the period of the PWM
  * At the moment we are using the timer 0 as the timer for the PWM.
  * So this method basically registers a task in the timer, and
  * since the init configured TA0CTTL2, the compar value of the timer will
  * be in the pwmOutput.
  */
 template<uint64_t periodValue, typename Duration = std::chrono::microseconds>
 static void setPwmPeriod() {
   // Re-Register the task
   TIMER::template registerTask<periodValue, Duration>(&currentTask);

   // Update dutycycle, since comparator value changed.
   updateDutyCycleRegister();


 }

  static constexpr void setIsrCallback(IsrCallback newCallback) {
    isrCallback = newCallback;
  }
 /**
  * Method to set the PWM duty cycle.
  * The duty cycle can be between 0 and 100.
  */
 static constexpr bool setNextDutyCycle(const _iq newDutyCycle) {
   if (newDutyCycle > MAX_DUTY_CYCLE) {
     return false;
   }
   dutyCycle = newDutyCycle;
   return true;
 }

 /**
  * Method to set the PWM duty cycle.
  * The duty cycle can be between 0 and 100.
  */
 static constexpr void setNextCCR2Val(const uint16_t newNextCCR2) {
   nextCCR2 = newNextCCR2;
 }

 static constexpr uint16_t getCCR0() {
   return *TIMER::REG_CCR0;
 }
 static constexpr void stop() {
  TIMER::stop();
 }

 static constexpr void start() {
  TIMER::start();
 }


private:
 /**
  * Method to configure the register responsible by the duty cycle.
  */
 static constexpr void updateDutyCycleRegister() {
   volatile const _iq valueCCR0 = _IQ(*TIMER::REG_CCR0);  // Reads current "period" register
   //  Calculates the value of the CCR2 based on the value of the current CCR0 value.
   volatile const _iq valueCCR1 = _IQdiv(_IQmpy(valueCCR0, dutyCycle), MAX_DUTY_CYCLE);
   *(TIMER::REG_CCR2) = _IQint(valueCCR1);
 }

 static constexpr void updateCCR2() {
   *(TIMER::REG_CCR2) = nextCCR2;
 }

  static constexpr void handleIsr() {
   *(TIMER::REG_CCR2) = nextCCR2;
//   updateCCR2();
   isrCallback();
  }

 static TaskHandler currentTask;
 static volatile _iq dutyCycle;
 static IsrCallback isrCallback;
 static uint16_t nextCCR2;
};

template <typename TIMER, typename PIN>
TaskHandler PWM_T<TIMER, PIN>::currentTask = TaskHandler(&PWM_T<TIMER, PIN>::handleIsr, true);

template <typename TIMER, typename PIN>
volatile _iq PWM_T<TIMER, PIN>::dutyCycle = 0;
template <typename TIMER, typename PIN>
uint16_t PWM_T<TIMER, PIN>::nextCCR2 = 0;

template <typename TIMER, typename PIN>
typename PWM_T<TIMER, PIN>::IsrCallback PWM_T<TIMER, PIN>::isrCallback = nullptr;
}  // namespace Microtech

#endif  // MICROTECH_PWM_HPP
