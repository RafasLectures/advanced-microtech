/*******************************************************************************
 * @file    tasks.hpp
 * @author  ekoeppen
 * @date    25.12.2014
 *
 * @brief   Header file that contains abstractions for the tasks. Taken from:
 *          https://github.com/ekoeppen/msp430-template-library/blob/master/tasks.h
 ******************************************************************************/

#ifndef LIBS_COMMON_TASKS_HPP_
#define LIBS_COMMON_TASKS_HPP_

#include "clocks.hpp"

#include <cstdint>

template<typename CLOCK>
struct TIMEOUT_T {
  static volatile uint32_t timeout;

  static void set(const uint32_t milliseconds) {
    __bic_SR_register(GIE);
    timeout = ((uint32_t)milliseconds * (uint32_t)CLOCK::frequency / (uint32_t)1000);
    __bis_SR_register(GIE);
    CLOCK::claim();
  };

  static inline bool count_down(void) {
    return !timeout || (--timeout == 0);
  };

  static inline bool triggered(void) {
    return timeout == 0;
  };

  static inline void disable(void) {
    CLOCK::release();
  }

  static inline uint32_t get() {
    return timeout;
  };

  static void set_and_wait(const uint32_t milliseconds) {
    set(milliseconds);
    while (!triggered()) {
      enter_idle();
    }
    disable();
  }
};

struct TIMEOUT_NEVER {
  static void set(const uint32_t milliseconds) {}
  static inline bool count_down(void) {
    return false;
  }
  static inline bool triggered(void) {
    return false;
  }
  static inline uint32_t get() {
    return 1;
  }
  static void disable(void) {}
};

struct TIMEOUT_IMMEDIATELY {
  static void set(const uint32_t milliseconds) {}
  static inline bool count_down(void) {
    return true;
  }
  static inline bool triggered(void) {
    return true;
  }
  static inline uint32_t get() {
    return 0;
  }
  static void disable(void) {}
};

template<typename CLOCK>
volatile uint32_t TIMEOUT_T<CLOCK>::timeout;

#endif /* LIBS_COMMON_TASKS_HPP_ */
