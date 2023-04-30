/******************************************************************************
 * @file                    Timer.hpp
 * @author                  Rafael Andrioli Bauer
 * @date                    09.11.2022
 * @matriculation number    5163344
 * @e-mail contact          abauer.rafael@gmail.com
 *
 * @brief   Header contains abstraction of GPIOs
 *
 * Description: This file is still work in progress, but its goal is described below.
 *              In order to abstract the manipulation of registers and
 *              ease up maintainability of the application code, this header
 *              provides an easy interface to setup a timer and tasks.
 *              It allows the user to set callback functions for periodic
 *              wakeu-ps or a one time wake-up
 *
 *              The interface was developed using template classes as well as
 *              constexpr so the program memory usage would be optimized.
 ******************************************************************************/

#ifndef COMMON_TIMER_HPP_
#define COMMON_TIMER_HPP_

#include "helpers.hpp"

#include <msp430g2553.h>
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>

namespace Microtech {
/**
 * Class that serves as a base for a TaskHanlder. The intention of it
 * is since a timer would store different task handlers, it cannot be tamplated
 * but for registering tasks the idea is that it is templated, because
 * there are some static calculations that can be done.
 *
 * So in order to have a type that can be used to put in an array
 * the TaskHandlerBase was created. It is the parent class of a TaskHandler.
 *
 */
class TaskHandlerBase {
public:
  // using CallbackFunction = std::function<void()>;
  typedef void (*CallbackFunction)();  ///< Type definition of callback

  /**
   * Class constructor.
   * @param callback function pointer to task callback
   * @param isPeriodic To inform weather this is a one-time callback (non periodic) or it is periodic.
   */
  TaskHandlerBase(CallbackFunction callback, bool isPeriodic) : taskCallback(callback), isPeriodic(isPeriodic) {}

  /**
   * Function intended to be called by the timer interrupt and it basically calls the callback.
   */
  inline void callCallback() {
    // Make sure the function pointer is not null
    // So we don't call an invalid address
    if (taskCallback != nullptr) {
      taskCallback();
    }
  }

private:
  const CallbackFunction taskCallback = nullptr;  ///< Callback pointer. Initially null, but can be set on constructor
  const bool isPeriodic;                          ///< Stores if the task is periodic or not
};

/**
 * TaskHandler is intended for the user to be able to handle different tasks.
 * At the moment the software supports only one task in the timer, but the advantage
 * of having such class is that later when this is expanded to allow multiple tasks
 * there is no change on the task concept. For example, in continuous mode the MSP supports
 * up to three timer comparators, so one could say that there are three tasks.
 *
 * Software-wise one can eve multiplex the timer and make multiple tasks with one timer
 * interrupt. Like a scheduler.
 *
 * @tparam periodValue Period of the task
 * @tparam Duration Time scale of the period (milliseconds, microseconds...)
 */
template<uint64_t periodValue, typename Duration = std::chrono::microseconds>
class TaskHandler : public TaskHandlerBase {
public:
  /**
   * Constructor of TaskHandler.
   *
   * @param callback function pointer to task callback
   * @param isPeriodic To inform weather this is a one-time callback (non periodic) or it is periodic.
   */
  TaskHandler(CallbackFunction callback, bool isPeriodic) : TaskHandlerBase(callback, isPeriodic) {}
};

class TimerClockSource {
public:
    enum class Option {
        TACLK,
        ACLK,
        SMCLK,
        INCLK
    };

    static uint16_t getTASSELValue(const Option option) noexcept {
      switch (option) {
          case Option::TACLK: return TASSEL_0;
          case Option::ACLK: return TASSEL_1;
          case Option::SMCLK: return TASSEL_2;
          case Option::INCLK: return TASSEL_3;
        }
      return TASSEL_2;  // It will actually never get here. But it is needed due to the compiler warning
    }
};

template<int64_t CLK_DIV, int64_t SOURCE_CLK_PERIOD_US>
class TimerConfigBase {
  template<uint8_t TIMER_NUMBER>
  friend class Timer;
public:
  constexpr TimerConfigBase(TimerClockSource::Option clkSource) : clkSource(clkSource) {}

protected:
  const TimerClockSource::Option clkSource;
};
/**
 * Timer class is responsible for managing the timer of MSP430.
 * @tparam TIMER_NUMBER The number of the timer.
 */
template<uint8_t TIMER_NUMBER>
class Timer {
  friend class Pwm;
  using RegisterRef = volatile uint16_t&;

public:
  Timer() : TAxCTL(getTAxCTL()), TAxCCR0(getTAxCCR0()), TAxCCTL0(getTAxCCTL0()) {}
  ~Timer() = default;

  /**
   * Method that guarantees that there is only one instance of the Timer<TIMER_NUM> class in the software
   * @return A reference to the instance
   */
  static Timer<TIMER_NUMBER>& getTimer() {
    static Timer<TIMER_NUMBER> timer;
    return timer;
  }
  /**
   * Method to initialize the timer. The CLK_DIV is at the moment a template parameter
   * the timer class.
   * Apart from that, this method also sets the
   * the count type, which IS hard coded. It can also be an argument
   * in the near future.
   */
  template<int64_t CLK_DIV, int64_t SOURCE_CLK_PERIOD_US>
  constexpr void init(TimerConfigBase<CLK_DIV, SOURCE_CLK_PERIOD_US> config) {
    constexpr uint16_t TIMER_INPUT_DIVIDER = getTimerInputDivider<CLK_DIV>();
    // Choose SMCLK as clock source
    // Counting in Up Mode
    TAxCTL = TimerClockSource::getTASSELValue(config.clkSource) + TIMER_INPUT_DIVIDER + MC_0;
  }
  /**
   * Method to register a task to the timer. It will enable the interrupt of timer 0,
   * but one must call _enable_interrupt() at some other time
   * for the interruption start to happen.
   * It is an easy interface to setup the timer to a desired interrupt period. E.g.:
   *
   * @code
   *  Timer timer0;
   *
   *  // Sets periodic task that is called every 500ms
   *  TaskHandler<500, std::chrono::milliseconds> task1(&task1Callback, true);
   *  timer0.registerTask(task1);
   *
   *  // Sets periodic task that is called every 1s
   *  TaskHandler<1, std::chrono::seconds> task2(&task2Callback, true);
   *  timer0.registerTask(task2);
   *
   *  // Sets non-periodic task that is once 300us from now
   *  TaskHandler<500, std::chrono::milliseconds> event1(&event1Callback, false);
   *  timer0.registerTask(event1);
   * @endcode
   *
   * @tparam periodValue The period value in that specific magnitude.
   * @tparam Duration std::chrono duration type.
   *
   * The whole logic ad calculation is done in compile time. The only thing that goes to the binary is the setting of
   * the registers.
   *
   * At the moment it is only supported adding one periodic task and no non-periodic
   * Usually when calling the registerTask, the template parameters don't have to be completed,
   * since the compiler can deduce them from the TaskHandler type.
   */
  template<int64_t CLK_DIV, int64_t SOURCE_CLK_PERIOD_US, uint64_t periodValue, typename Duration = std::chrono::microseconds>
  constexpr void registerTask(const TimerConfigBase<CLK_DIV, SOURCE_CLK_PERIOD_US>& /*config*/, TaskHandler<periodValue, Duration>& task) {
    // Gets the timer0 compare value
    constexpr uint16_t COMPARE_VALUE = calculateCompareValue<CLK_DIV, SOURCE_CLK_PERIOD_US, periodValue, Duration>();

    // For now only one task. This is work in progress.
    // Task is added to taskHandlers array
    taskHandlers[0] = &task;

    /* Set Timer 0 compare value.
     * One can see that it is the correct value when looking at the disassembly on the call:
     * TaskHandler<500, std::chrono::milliseconds> task1(&task1Callback, true);
     * registerTask(task1); the assembly command is:
     * MOV.W   #0xf423,&Timer0_A3_TA0CCR0. Where #0xf423 is the value coming from compareValue evaluated in compile time
     * which is correct: (500000/8)-1 = 62499 = 0xF423.
     */
    TAxCCR0 = COMPARE_VALUE;
    // Enable interrupt for CCR0.
    setRegisterBits(TAxCCTL0, static_cast<uint16_t>(CCIE));
    setRegisterBits(TAxCTL, static_cast<uint16_t>(MC_1));
  }

  constexpr void stop() {
    resetRegisterBits(TAxCTL, static_cast<uint16_t>(MC_3));
    resetRegisterBits(TAxCCTL0, static_cast<uint16_t>(CCIE));
  }
  /**
   * Method to deregister a task. But not yet implemented
   * @param taskHandler Takes the reference of the taskHandler.
   */
  bool deregisterTask(TaskHandlerBase& /*taskHandler*/) {
    // static_assert(false, "Function not yet implemented");
    return false;
  }

  /**
   * Function must be called by the timer interrupt.
   *
   * @note There must be a better way of doing this. Maybe encapsulating
   * this function or setting durin runtime the interrupt function
   * to a private method of the Timer instance. Need to figure this out.
   */
  inline void interruptionHappened() {
    taskHandlers[0]->callCallback();
  }

protected:
  /**
   * Function to calculate the TACCRx value.
   * Since it is static constexpr and it has all of its values in compile time,
   * this doesn't result in a function call during runtime and it is
   * evaluated in compile time resulting in just a number in the program binary.
   */
  template<int64_t CLK_DIV, int64_t SOURCE_CLK_PERIOD_US, uint64_t periodValue, typename Duration = std::chrono::microseconds>
  static constexpr uint16_t calculateCompareValue() {
    // Declares the duration given by the user and then converts it to microseconds
    // !! Duration is a type and periodValue is a value !!
    constexpr Duration PERIOD(periodValue);
    constexpr std::chrono::microseconds PERIOD_IN_US = PERIOD;

    /*
     * Internal clock is 1 MHz, according to templateEMP.h
     *
     * So the compare value is basically the (period[us] / (1us * clockDiv)) - 1. The -1 because the counter starts in 0
     * !!!! Since all the values are constants, the division is evaluated in compile time. !!!!!
     */
    constexpr int64_t COMPARE_VALUE = (PERIOD_IN_US.count() / (CLK_DIV*SOURCE_CLK_PERIOD_US)) - 1;

    // Static assert so if the compareValue is bigger than 0xFFFF the compiler gives an error. This is not added as
    // instructions in the binary. One could verify that it works by calling "setupTimer0<5, std::chrono::seconds>();".
    static_assert((COMPARE_VALUE <= 0xFFFF), "Cannot set desired timer period. It exceeds the counter maximum value");
    return static_cast<uint16_t>(COMPARE_VALUE);
  }

private:
  /**
   * Method to get the correct value of the timer interrupt
   * Divider given in the template parameter of this class.
   * There is also a static_assert that is triggered in case the
   * CLK_DIV is invalid.
   * An enum was not added since this value is also used for the
   * timerCompareValue calculation.
   */
  template<int64_t CLK_DIV>
  static constexpr uint16_t getTimerInputDivider() {
    switch (CLK_DIV) {
      case 1: return ID_0;
      case 2: return ID_1;
      case 4: return ID_2;
      case 8: return ID_3;
      default:
        static_assert(CLK_DIV == 1 || CLK_DIV == 2 || CLK_DIV == 4 || CLK_DIV == 8,
                      "Timer input divider is invalid. Must be either 1, 2, 4 or 8");
    }
    return ID_0;  // It will actually never get here. But it is needed due to the compiler warning
  }

  static RegisterRef getTAxCTL() {
    switch (TIMER_NUMBER) {
      case 0: return TA0CTL;
      case 1: return TA1CTL;
    };
    return TA0CTL;
  }

  static RegisterRef getTAxCCR0() {
    switch (TIMER_NUMBER) {
      case 0: return TA0CCR0;
      case 1: return TA1CCR0;
    };
    return TA0CCR0;
  }

  static RegisterRef getTAxCCTL0() {
    switch (TIMER_NUMBER) {
      case 0: return TA0CCTL0;
      case 1: return TA1CCTL0;
    };
    return TA0CCTL0;
  }

  /**
   * List of the task handlers registered to the timer.
   *
   * For now it is only one, but the container is there already so later it is easier
   * to update the code. And also doesn't add much extra overhead or memory.
   */
  std::array<TaskHandlerBase*, 1> taskHandlers;
  RegisterRef TAxCTL;
  RegisterRef TAxCCR0;
  RegisterRef TAxCCTL0;
};

} /* namespace Microtech */

// Timer0 Interruption
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A_CCR0_ISR(void) {
  Microtech::Timer<0>::getTimer().interruptionHappened();
}

// Timer1 Interruption
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1_A_CCR0_ISR(void) {
  Microtech::Timer<1>::getTimer().interruptionHappened();
}

#endif /* COMMON_TIMER_HPP_ */
