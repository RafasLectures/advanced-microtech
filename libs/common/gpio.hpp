/*******************************************************************************
 * @file    gpio.hpp
 * @author  ekoeppen
 * @date    30.10.2015
 *
 * @brief   Header file that abstracts GPIO of MSP430. Taken from:
 *          https://github.com/ekoeppen/msp430-template-library/blob/master/gpio.h
 ******************************************************************************/

#ifndef LIBS_COMMON_GPIO_HPP_
#define LIBS_COMMON_GPIO_HPP_

#include "tasks.hpp"

#include <msp430g2553.h>
#include <stdint.h>

#define P2IN_ 1
#define P3IN_ 1

enum DIRECTION { INPUT = 0, OUTPUT = 1 };

enum TRIGGER_EDGE { TRIGGER_RISING = 0, TRIGGER_FALLING = 1 };

enum LEVEL { LOW = false, PULL_DOWN = false, HIGH = true, PULL_UP = true };

enum INTERRUPT_ENABLE { INTERRUPT_DISABLED = false, INTERRUPT_ENABLED = true };

enum RESISTOR_ENABLE { RESISTOR_DISABLED = false, RESISTOR_ENABLED = true };

enum PIN_FUNCTION {
  TIMER_OUTPUT,
  TIMER_COMPARE,
  ACLK_OUTPUT,
  CAPACITIVE_SENSE,
  ADCCLK_OUTPUT,
  COMPARATOR_OUTPUT,
  SMCLK_OUTUT,
  XIN,
  XOUT,
  USCI
};

uint8_t P1IFGS;
uint8_t P2IFGS;

static constexpr volatile uint8_t *ports[][10] = {
  {const_cast<uint8_t *>(&P1IN), &P1OUT, &P1DIR, &P1IFG, &P1IES, &P1IE, &P1SEL, &P1SEL2, &P1REN, &P1IFGS},
#ifdef P2IN_
  {const_cast<uint8_t *>(&P2IN), &P2OUT, &P2DIR, &P2IFG, &P2IES, &P2IE, &P2SEL, &P2SEL2, &P2REN, &P2IFGS},
#ifdef P3IN_
  {const_cast<uint8_t *>(&P3IN), &P3OUT, &P3DIR, 0, 0, 0, &P3SEL, &P3SEL2, &P3REN, 0},
#ifdef P4IN_
  {const_cast<uint8_t *>(&P4IN), &P4OUT, &P4DIR, 0, 0, 0, &P4SEL, &P4SEL2, &P4REN, 0},
#ifdef P5IN_
  {const_cast<uint8_t *>(&P5IN), &P5OUT, &P5DIR, 0, 0, 0, &P5SEL, &P5SEL2, &P5REN, 0},
#ifdef P6IN_
  {const_cast<uint8_t *>(&P6IN), &P6OUT, &P6DIR, 0, 0, 0, &P6SEL, &P6SEL2, &P6REN, 0},
#ifdef P7IN_
  {const_cast<uint8_t *>(&P7IN), &P7OUT, &P7DIR, 0, 0, 0, &P7SEL, &P7SEL2, &P7REN, 0},
#ifdef P8IN_
  {const_cast<uint8_t *>(&P8IN), &P8OUT, &P8DIR, 0, 0, 0, &P8SEL, &P8SEL2, &P8REN, 0},
#endif
#endif
#endif
#endif
#endif
#endif
#endif
};

template<const char PORT, const char PIN, const DIRECTION PIN_DIRECTION = OUTPUT, const LEVEL INITIAL_LEVEL = LOW,
         const INTERRUPT_ENABLE INTERRUPT = INTERRUPT_DISABLED, const TRIGGER_EDGE EDGE = TRIGGER_RISING,
         const char FUNCTION_SELECT = 0, const RESISTOR_ENABLE RESISTOR = RESISTOR_DISABLED>
struct GPIO_PIN_T {
  static_assert(PORT > 0 && PORT < 4, "Port range must be between 1 and 3");
  static constexpr volatile uint8_t *PxIN = ports[PORT - 1][0];
  static constexpr volatile uint8_t *PxOUT = ports[PORT - 1][1];
  static constexpr volatile uint8_t *PxDIR = ports[PORT - 1][2];
  static constexpr volatile uint8_t *PxIFG = ports[PORT - 1][3];
  static constexpr volatile uint8_t *PxIES = ports[PORT - 1][4];
  static constexpr volatile uint8_t *PxIE = ports[PORT - 1][5];
  static constexpr volatile uint8_t *PxSEL = ports[PORT - 1][6];
  static constexpr volatile uint8_t *PxSEL2 = ports[PORT - 1][7];
  static constexpr volatile uint8_t *PxREN = ports[PORT - 1][8];
  static constexpr volatile uint8_t *PxIFGS = ports[PORT - 1][9];

  static constexpr uint8_t pin = PIN;
  static constexpr uint8_t bit_value = 1 << PIN;

  static constexpr uint8_t direction = (PIN_DIRECTION == OUTPUT ? bit_value : 0);
  static constexpr uint8_t interrupt_enable = (INTERRUPT ? bit_value : 0);
  static constexpr uint8_t interrupt_edge = (EDGE == TRIGGER_FALLING ? bit_value : 0);
  static constexpr uint8_t function_select = (FUNCTION_SELECT & 0b01 ? bit_value : 0);
  static constexpr uint8_t function_select2 = (FUNCTION_SELECT & 0b10 ? bit_value : 0);
  static constexpr uint8_t resistor_enable = (RESISTOR ? bit_value : 0);
  static constexpr uint8_t initial_level = (INITIAL_LEVEL ? bit_value : 0);
  static constexpr uint16_t adc_input = (PORT == 1 ? PIN << 12 : 0);
  static constexpr bool is_unused(void) {
    return false;
  }

  static void init(void) {
    if (INITIAL_LEVEL)
      *PxOUT |= bit_value;
    if (PIN_DIRECTION == OUTPUT)
      *PxDIR |= bit_value;
    if (INTERRUPT) {
      // static_assert(PxIE != 0, "Port not interrupt capable");
      *PxIE |= bit_value;
    }
    if (EDGE == TRIGGER_FALLING) {
      // static_assert(PxIES != 0, "Port not interrupt capable");
      *PxIES |= bit_value;
    }
    if (FUNCTION_SELECT & 0b01) {
      static_assert(PxSEL != 0, "Port does not have alternate functions");
      *PxSEL |= bit_value;
    }
    if (FUNCTION_SELECT & 0b10) {
      static_assert(PxSEL2 != 0, "Port does not have alternate functions");
      *PxSEL2 |= bit_value;
    }
    if (RESISTOR)
      *PxREN |= bit_value;
  };

  static void set_high(void) {
    *PxOUT |= bit_value;
  };

  static void set_low(void) {
    *PxOUT &= ~bit_value;
  }

  static void set(bool value) {
    if (value)
      set_high();
    else
      set_low();
  }

  static bool get(void) {
    return *PxIN & bit_value;
  }

  static bool is_high(void) {
    return *PxIN & bit_value;
  }

  static bool is_low(void) {
    return !is_high();
  }

  static void toggle(void) {
    *PxOUT ^= bit_value;
  }

  static void clear_irq(void) {
    *PxIFG &= ~bit_value;
    *PxIFGS &= ~bit_value;
  }

  static bool irq_raised(void) {
    return *PxIFGS & bit_value;
  }

  static void disable_irq(void) {
    *PxIE &= ~bit_value;
  }

  static void enable_irq(void) {
    *PxIE |= bit_value;
  }

  template<typename TIMEOUT = TIMEOUT_NEVER>
  static bool wait_for_irq(void) {
    while (!TIMEOUT::triggered() && !(*PxIFGS & bit_value)) {
      enter_idle();
    }
    return irq_raised();
  }

  static void set_output(void) {
    *PxDIR |= bit_value;
  }

  static void set_input(void) {
    *PxDIR &= ~bit_value;
  }

  static void enable_resistor(void) {
    *PxREN |= bit_value;
  }

  static void disable_resistor(void) {
    *PxREN &= ~bit_value;
  }

  static void pull_up(void) {
    set_high();
  }

  static void pull_down(void) {
    set_low();
  }
};

template<const char PORT, const char PIN, const RESISTOR_ENABLE RESISTOR = RESISTOR_DISABLED,
         const LEVEL PULL = PULL_DOWN, const INTERRUPT_ENABLE INTERRUPT = INTERRUPT_DISABLED,
         const TRIGGER_EDGE EDGE = TRIGGER_RISING>
struct GPIO_INPUT_T : public GPIO_PIN_T<PORT, PIN, INPUT, PULL, INTERRUPT, EDGE, 0, RESISTOR> {};

template<const char PORT, const char PIN, const LEVEL INITIAL_LEVEL = LOW>
struct GPIO_OUTPUT_T
  : public GPIO_PIN_T<PORT, PIN, OUTPUT, INITIAL_LEVEL, INTERRUPT_DISABLED, TRIGGER_RISING, 0, RESISTOR_DISABLED> {};

template<const char PORT, const char PIN, const char FUNCTION_SELECT = 0, const DIRECTION PIN_DIRECTION = OUTPUT>
struct GPIO_MODULE_T : public GPIO_PIN_T<PORT, PIN, PIN_DIRECTION, LOW, INTERRUPT_DISABLED, TRIGGER_RISING,
                                         FUNCTION_SELECT, RESISTOR_DISABLED> {};

template<const char PORT, const char PIN>
struct GPIO_ANALOG_T {
  static constexpr uint8_t direction = 0;
  static constexpr uint8_t interrupt_enable = 0;
  static constexpr uint8_t interrupt_edge = 0;
  static constexpr uint8_t function_select = 0;
  static constexpr uint8_t function_select2 = 0;
  static constexpr uint8_t resistor_enable = 0;
  static constexpr uint8_t initial_level = 0;
  static constexpr uint8_t pin = PIN;
  static constexpr uint8_t bit_value = 1 << PIN;
  static constexpr uint16_t adc_input = (PORT == 1 ? PIN << 12 : 0);
};

struct PIN_UNUSED {
  static constexpr uint8_t direction = 0;
  static constexpr uint8_t interrupt_enable = 0;
  static constexpr uint8_t interrupt_edge = 0;
  static constexpr uint8_t function_select = 0;
  static constexpr uint8_t function_select2 = 0;
  static constexpr uint8_t resistor_enable = 0;
  static constexpr uint8_t initial_level = 0;
  static constexpr uint16_t adc_input = 0;
  static constexpr uint8_t pin = 0;
  static constexpr uint8_t bit_value = 0;
  static constexpr bool is_unused(void) {
    return true;
  }
  static constexpr bool is_high(void) {
    return false;
  }
  static constexpr bool is_low(void) {
    return true;
  }
  static void toggle(void) {}
  static void set_high(void) {}
  static void set_low(void) {}
  static void clear_irq(void) {}
  static bool wait_for_irq(void) {
    return true;
  }
  static constexpr bool irq_raised(void) {
    return false;
  }
};

template<const int PORT, typename PIN0 = PIN_UNUSED, typename PIN1 = PIN_UNUSED, typename PIN2 = PIN_UNUSED,
         typename PIN3 = PIN_UNUSED, typename PIN4 = PIN_UNUSED, typename PIN5 = PIN_UNUSED, typename PIN6 = PIN_UNUSED,
         typename PIN7 = PIN_UNUSED>
struct GPIO_PORT_T {
  static constexpr volatile uint8_t *PxIN = ports[PORT - 1][0];
  static constexpr volatile uint8_t *PxOUT = ports[PORT - 1][1];
  static constexpr volatile uint8_t *PxDIR = ports[PORT - 1][2];
  static constexpr volatile uint8_t *PxIFG = ports[PORT - 1][3];
  static constexpr volatile uint8_t *PxIES = ports[PORT - 1][4];
  static constexpr volatile uint8_t *PxIE = ports[PORT - 1][5];
  static constexpr volatile uint8_t *PxSEL = ports[PORT - 1][6];
  static constexpr volatile uint8_t *PxSEL2 = ports[PORT - 1][7];
  static constexpr volatile uint8_t *PxREN = ports[PORT - 1][8];
  static constexpr volatile uint8_t *PxIFGS = ports[PORT - 1][9];

  static void init(void) {
    uint8_t reg;

    reg = PIN0::direction | PIN1::direction | PIN2::direction | PIN3::direction | PIN4::direction | PIN5::direction
          | PIN6::direction | PIN7::direction;
    if (reg)
      *PxDIR = reg;

    reg = PIN0::interrupt_enable | PIN1::interrupt_enable | PIN2::interrupt_enable | PIN3::interrupt_enable
          | PIN4::interrupt_enable | PIN5::interrupt_enable | PIN6::interrupt_enable | PIN7::interrupt_enable;
    if (reg)
      *PxIE = reg;

    reg = PIN0::interrupt_edge | PIN1::interrupt_edge | PIN2::interrupt_edge | PIN3::interrupt_edge
          | PIN4::interrupt_edge | PIN5::interrupt_edge | PIN6::interrupt_edge | PIN7::interrupt_edge;
    if (reg)
      *PxIES = reg;

    reg = PIN0::function_select | PIN1::function_select | PIN2::function_select | PIN3::function_select
          | PIN4::function_select | PIN5::function_select | PIN6::function_select | PIN7::function_select;
    if (reg)
      *PxSEL = reg;

    reg = PIN0::function_select2 | PIN1::function_select2 | PIN2::function_select2 | PIN3::function_select2
          | PIN4::function_select2 | PIN5::function_select2 | PIN6::function_select2 | PIN7::function_select2;
    if (reg)
      *PxSEL2 = reg;

    reg = PIN0::resistor_enable | PIN1::resistor_enable | PIN2::resistor_enable | PIN3::resistor_enable
          | PIN4::resistor_enable | PIN5::resistor_enable | PIN6::resistor_enable | PIN7::resistor_enable;
    if (reg)
      *PxREN = reg;

    *PxOUT = PIN0::initial_level | PIN1::initial_level | PIN2::initial_level | PIN3::initial_level | PIN4::initial_level
             | PIN5::initial_level | PIN6::initial_level | PIN7::initial_level;
  }

  static void clear_irq(void) {
    *PxIFG = 0;
  };

  static bool handle_irq(void) {
    bool resume = false;

    if (*PxIFG) {
      *PxIFGS = *PxIFG;
      clear_irq();
      resume = true;
    }
    return resume;
  }
};

#endif /* LIBS_COMMON_GPIO_HPP_ */
