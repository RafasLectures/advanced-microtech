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
class Pwm {
public:
  Pwm() = delete;
  explicit Pwm(const OutputHandle& outputPin) : pwmOutput(outputPin), TIMER_CONFIG(TimerClockSource::Option::SMCLK){}

  void init() const {
    Timer<0>::getTimer().init(TIMER_CONFIG);
    pwmOutput.init();
    pwmOutput.disablePinResistor();
    pwmOutput.setIoFunctionality(IOFunctionality::TA0_COMPARE_OUT2);
    TA0CCTL2 = OUTMOD_3;
  }

  /**
   * Class to set the period of the PWM
   * At the moment we are using the timer 0 as the timer for the PWM.
   * So this method basically registers a task in the timer, and
   * since the init configured TA0CTTL2, the compar value of the timer will
   * be in the pwmOutput.
   */
  template<uint64_t periodValue, typename Duration = std::chrono::microseconds>
  void setPwmPeriod() {
    // Pointer of a newTask
    std::unique_ptr<TaskHandler<periodValue, Duration>> newTask =
      std::make_unique<TaskHandler<periodValue, Duration>>(nullptr, true);

    if (currentTask) {  // if there is a task playing, deregister the task
      Timer<0>::getTimer().deregisterTask(*currentTask);
    }
    // Register the newTask
    Timer<0>::getTimer().registerTask(TIMER_CONFIG, *newTask);

    // Update dutycycle, since comparator value changed.
    updateDutyCycleRegister();

    currentTask = std::move(newTask);  // Stores new task pointer.
  }

  /**
   * Method to set the PWM duty cycle.
   * The duty cycle can be between 0 and 100.
   */
  bool setDutyCycle(const _iq15 newDutyCycle) {
    if (newDutyCycle > MAX_DUTY_CYCLE) {
      return false;
    }
    dutyCycle = newDutyCycle;
    updateDutyCycleRegister();
    return true;
  }

  void stop() {
    Timer<0>::getTimer().stop();
  }

private:
  /**
   * Method to configure the register responsible by the duty cycle.
   */
  void updateDutyCycleRegister() {
    const _iq15 valueCCR0 = _IQ15(TACCR0);  // Reads current "period" register
    //  Calculates the value of the CCR2 based on the value of the current CCR0 value.
    const _iq15 valueCCR1 = _IQ15div(_IQ15mpy(valueCCR0, dutyCycle), MAX_DUTY_CYCLE);
    TA0CCR2 = _IQ15int(valueCCR1);
  }

  static constexpr _iq15 MAX_DUTY_CYCLE = _IQ15(100.0);

  const OutputHandle pwmOutput;

  const TimerConfigBase<8, 1> TIMER_CONFIG;
  std::unique_ptr<TaskHandlerBase> currentTask;
  _iq15 dutyCycle = 0;
};

}  // namespace Microtech

#endif  // MICROTECH_PWM_HPP
