/******************************************************************************
 * @file                    GPIOs.hpp
 * @author                  Rafael Andrioli Bauer
 * @date                    09.11.2022
 * @matriculation number    5163344
 * @e-mail contact          abauer.rafael@gmail.com
 *
 * @brief   Header contains abstraction of GPIOs
 *
 * Description: In order to abstract the manipulation of registers and
 *              ease up maintainability of the application code, this header
 *              provides an easy interface to set specific GPIOs as inputs or
 *              outputs. It allows the user to hold a handle of an specific pin
 *              which makes easier to manipulate and even listen to the pin
 *              events.
 *
 *              The interface was developed using template classes as well as
 *              constexpr so the program memory usage would be optimized.
 ******************************************************************************/
#ifndef COMMON_GPIOS_HPP_
#define COMMON_GPIOS_HPP_

#include "helpers.hpp"

#include <msp430g2553.h>

#include <cstdint>

namespace Microtech {

/**
 * Enum to differentiate between the different IO Ports available
 * in the Microcontroller. This type is used in some template arguments
 */
enum class IOPort {
  PORT_1 = 1,  ///< Represents the port 1
  PORT_2,      ///< Represents the port 2
  PORT_3,      ///< Represents the port 3
};

/**
 * Enum representing the different IO states
 */
enum class IOState {
  LOW = 0,  ///< The IO is low.
  HIGH,     ///< The IO is high.
};

enum class IOResistor {
  PULL_DOWN = 0,
  PULL_UP
};

enum class IOFunctionality {
  GPIO = 0,
  TA0_COMPARE_OUT1,
  TA0_COMPARE_OUT2,
};

enum class IoDirection {
  INPUT = 0,
  OUTPUT,
};

/**
 * Helper class to return the references of the
 * GPIO registers.
 * Since the name always follows a pattern, such as
 * "PxDIR" where "x" would be replaced by the port number, this was
 * created, so one can retrieve the correct port
 * register references only by calling the respective
 * method.
 *
 * This class also helps to prevent bugs from being introduced, since
 * one simply declares the constexpr object and then uses the "Px" register
 * methods. This way, one doesn't forget to swap one value when doing
 * a change.
 *
 * Since it is declared in the private section within the GPIOs class,
 * it cannot be "seen" from the outside.
 *
 */
class GPIORegisters {
public:
  constexpr GPIORegisters() {}

  /**
   * Returns the PxDir register reference.
   * Since this is a static constexpr function,
   * the compiler already resolves the internal logic.
   * There is no extra allocation in program memory
   *
   * @returns The reference to the respective PxDir register
   */
  static constexpr volatile uint8_t& getPxDir(IOPort port) noexcept {
    switch (port) {
      case IOPort::PORT_1: return P1DIR;
      case IOPort::PORT_2: return P2DIR;
      default:
      case IOPort::PORT_3: return P3DIR;
    }
  }
  /**
   * Returns the PxSel register reference.
   * Since this is a static constexpr function,
   * the compiler already resolves the internal logic.
   * There is no extra allocation in program memory
   *
   * @returns The reference to the respective PxSel register
   */
  static constexpr volatile uint8_t& getPxSel(IOPort port) noexcept {
    switch (port) {
      case IOPort::PORT_1: return P1SEL;
      case IOPort::PORT_2: return P2SEL;
      default:
      case IOPort::PORT_3: return P3SEL;
    }
  }
  /**
   * Returns the PxSel2 register reference.
   * Since this is a static constexpr function,
   * the compiler already resolves the internal logic.
   * There is no extra allocation in program memory
   *
   * @returns The reference to the respective PxSel2 register
   */
  static constexpr volatile uint8_t& getPxSel2(IOPort port) noexcept {
    switch (port) {
      case IOPort::PORT_1: return P1SEL2;
      case IOPort::PORT_2: return P2SEL2;
      default:
      case IOPort::PORT_3: return P3SEL2;
    }
  }
  /**
   * Returns the PxRen register reference.
   * Since this is a static constexpr function,
   * the compiler already resolves the internal logic.
   * There is no extra allocation in program memory.
   *
   * @returns The reference to the respective PxRen register
   */
  static constexpr volatile uint8_t& getPxRen(IOPort port) noexcept {
    switch (port) {
      case IOPort::PORT_1: return P1REN;
      case IOPort::PORT_2: return P2REN;
      default:
      case IOPort::PORT_3: return P3REN;
    }
  }
  /**
   * Returns the PxIn register reference.
   * Since this is a static constexpr function,
   * the compiler already resolves the internal logic.
   * There is no extra allocation in program memory.
   *
   * @returns The reference to the respective PxIn register
   */
  static constexpr volatile uint8_t& getPxIn(IOPort port) noexcept {
    switch (port) {
      case IOPort::PORT_1: return P1IN;
      case IOPort::PORT_2: return P2IN;
      default:
      case IOPort::PORT_3: return P3IN;
    }
  }
  /**
   * Returns the PxOut register reference.
   * Since this is a static constexpr function,
   * the compiler already resolves the internal logic.
   * There is no extra allocation in program memory.
   *
   * @returns The reference to the respective PxOut register
   */
  static constexpr volatile uint8_t& getPxOut(IOPort port) noexcept {
    switch (port) {
      case IOPort::PORT_1: return P1OUT;
      case IOPort::PORT_2: return P2OUT;
      default:
      case IOPort::PORT_3: return P3OUT;
    }
  }
  /**
   * Returns the PxIe register reference.
   * Since this is a static constexpr function,
   * the compiler already resolves the internal logic.
   * There is no extra allocation in program memory.
   *
   * @returns The reference to the respective PxIe register
   */
  static constexpr volatile uint8_t& getPxIe(IOPort port) noexcept {
    switch (port) {
      case IOPort::PORT_1: return P1IE;
      case IOPort::PORT_2: return P2IE;
      default:
      case IOPort::PORT_3:
        return P1IE;  // It will never get here, but there was a warning.
        // static_assert(port == IOPort::PORT_1 || port == IOPort::PORT_2, "IOPort 3 does not support interruptions");
    }
  }
  /**
   * Returns the PxIes register reference.
   * Since this is a static constexpr function,
   * the compiler already resolves the internal logic.
   * There is no extra allocation in program memory.
   *
   * @returns The reference to the respective PxIes register
   */
  static constexpr volatile uint8_t& getPxIes(IOPort port) noexcept {
    switch (port) {
      case IOPort::PORT_1: return P1IES;
      case IOPort::PORT_2: return P2IES;
      default:
      case IOPort::PORT_3:
        return P1IES;  // It will never get here, but there was a warning
        // static_assert(port == IOPort::PORT_1 || port == IOPort::PORT_2, "IOPort 3 does not support interruptions");
    }
  }
  /**
   * Returns the PxIfg register reference.
   * Since this is a static constexpr function,
   * the compiler already resolves the internal logic.
   * There is no extra allocation in program memory.
   *
   * @returns The reference to the respective PxIfg register
   */
  static constexpr volatile uint8_t& getPxIfg(IOPort port) noexcept {
    switch (port) {
      case IOPort::PORT_1: return P1IFG;
      case IOPort::PORT_2: return P2IFG;
      default:
      case IOPort::PORT_3:
        return P1IFG;  // It will never get here, but there was a warning
        // static_assert(port == IOPort::PORT_1 || port == IOPort::PORT_2, "IOPort 3 does not support interruptions");
    }
  }
};

/**
 * Class serves as a base class and holds the common attributes between
 * OutputHandle and InputHandle.
 *
 * It cannot be constructed by anyone else other then its childs, since the constructor
 * is protected.
 *
 * Since it is declared in the private section within the GPIOs class,
 * it cannot be "seen" from the outside.
 *
 */
class IoHandleBase {
  friend class Pwm;

public:
  IoHandleBase() = delete;
  /**
   * Gets the state of the Pin.
   *
   * The compiler tries to already resolve this in compile time.
   *
   * @returns The state of the pin. IOState::HIGH or IOState::LOW
   */
  constexpr IOState getState() const noexcept {
    // According to MSP430 Manual, PxIn will always be updated with the pins state
    // regardless if it is configured as an input or output.
    if (getRegisterBits(GPIORegisters::getPxIn(port), mBitMask, mPin)) {
      return IOState::HIGH;
    }
    return IOState::LOW;
  }

  /**
   * Method to enable an interrupt at the pin.
   * At the moment hard-coded to be High/Low edge
   *
   * @note it can be improved in the future to take the edge as a parameter.
   */
  void enableInterrupt() const noexcept {
    // somehow I need to find a way to wrap the pin interrupt here and
    // add a callback. But for now we just enable the pin interrupt and
    // the rest has to be handled from the outside
    setRegisterBits(GPIORegisters::getPxIes(port), mBitMask);    // High /Low - Edge
    resetRegisterBits(GPIORegisters::getPxIfg(port), mBitMask);  // Clear interrupt flag
    setRegisterBits(GPIORegisters::getPxIe(port), mBitMask);     // Enable interrupt

  }

  void disableInterrupt() const noexcept {
    resetRegisterBits(GPIORegisters::getPxIe(port), mBitMask);  // Disable interrupt
  }

  constexpr bool enablePinResistor(IOResistor resistorType) const {
      setRegisterBits(GPIORegisters::getPxRen(port), mBitMask);
      switch (resistorType) {
      case IOResistor::PULL_DOWN:
        resetRegisterBits(GPIORegisters::getPxOut(port), mBitMask);
        break;
      case IOResistor::PULL_UP:
        setRegisterBits(GPIORegisters::getPxOut(port), mBitMask);
        break;
      default:
        return false;
      }
      return true;
  }

  constexpr void disablePinResistor() const {
    resetRegisterBits(GPIORegisters::getPxRen(port), mBitMask);
  }

  constexpr bool setIoFunctionality(IOFunctionality functionality) const {
    switch (functionality) {
      case IOFunctionality::GPIO:
        resetRegisterBits(GPIORegisters::getPxSel(port), mBitMask);
        resetRegisterBits(GPIORegisters::getPxSel2(port), mBitMask);
        return true;
      case IOFunctionality::TA0_COMPARE_OUT1:
        if (port == IOPort::PORT_3 && mPin == 5) {
          setRegisterBits(GPIORegisters::getPxSel(port), mBitMask);
          resetRegisterBits(GPIORegisters::getPxSel2(port), mBitMask);
          return true;
        }
        return false;
      case IOFunctionality::TA0_COMPARE_OUT2:
        if (port == IOPort::PORT_3 && mPin == 6) {
          setRegisterBits(GPIORegisters::getPxSel(port), mBitMask);
          resetRegisterBits(GPIORegisters::getPxSel2(port), mBitMask);
          return true;
        }
        return false;
    };

    return false;
  }
protected:

  // Constructor is protected, so it cannot be constructed by anyone else other than its
  // child
  explicit constexpr IoHandleBase(IOPort desiredPort, uint8_t desiredPin)
    : mPin(desiredPin),
      port(desiredPort),
      mBitMask(static_cast<uint8_t>(0x01) << desiredPin) {}

  const uint8_t mPin;      ///< Pin of the IO
  const uint8_t mBitMask;  ///< Mask of the IO used to manipulate the registers
  const IOPort port;
};
/**
 * Class is a public interface to use an IO pin.
 * It serves as a handle to an output pin.
 * Whenever someone creates the handle, the GPIO pin gets set as an output.
 * So to use a pin as an output, is very straight forward. One only has to call:
 *  @code
 *      OutputHandle p1_0 = GPIOs:getOutputHandle<IOPort::PORT_1, 0>;
 *      p1_0.init();
 *      p1_0.setState(IOState::HIGH);
 *      p1_0.setState(IOState::LOW);
 *  @endcode
 */
class OutputHandle : public IoHandleBase {
public:
  OutputHandle() = delete;
  /**
   * Class constructor.
   *
   * Since the constructor is constexpr the compiler tries to resolved in compile time
   */
  explicit constexpr OutputHandle(IOPort port, uint8_t desiredPin) : IoHandleBase(port, desiredPin) {}

  /**
   * Method initializes the pin. It sets the pin as an output.
   */
  constexpr void init() const {
    setRegisterBits(GPIORegisters::getPxDir(port), mBitMask);
    setIoFunctionality(IOFunctionality::GPIO);
  }

  /**
   * Class constructor with the possibility of setting an initial state.
   *
   * Since the constructor is constexpr the compiler tries to resolved in compile time
   *
   * @param initialState initial state of the output
   */
  constexpr OutputHandle(IOPort port, uint8_t desiredPin, IOState initialState) : OutputHandle(port, desiredPin) {
    setState(initialState);
  }

  /**
   * Overloaded method (see next method) so one can set the pin state using a boolean.
   * true = IOState::HIGH
   * false == IOState::LOW
   *
   * The compiler tries to already resolve this in compile time.
   *
   * @param state Desired pin state.
   */
  constexpr void setState(const bool state) const noexcept {
    if (state) {
      setRegisterBits(GPIORegisters::getPxOut(port), mBitMask);
    } else {
      resetRegisterBits(GPIORegisters::getPxOut(port), mBitMask);
    }
  }
  /**
   * Sets the Pin to an specific state. IOState::HIGH or IOState::LOW
   *
   * The compiler tries to already resolve this in compile time.
   *
   * @param state Desired pin state.
   */
  constexpr void setState(const IOState state) const noexcept {
    // Calls the overloaded method with boolean arguments
    setState(state == IOState::HIGH);
  }

  /**
   * Toggles the Pin. If the pin state is IOState::HIGH it will toggle to IOState::LOW or vice-versa
   *
   * The compiler tries to already resolve this in compile time.
   *
   */
  constexpr void toggle() const noexcept {
    toggleRegisterBits(GPIORegisters::getPxOut(port), mBitMask);
  }
};

/**
 * Class is a public interface to use an IO pin.
 * It serves as a handle to an input pin.
 * Whenever someone creates the handle, the GPIO pin gets set as an input.
 * So to use a pin as an input, is very straight forward. One only has to call:
 *  @code
 *      InputHandle GPIOs::GPIOs::getOutputHandle<IOPort::PORT_1,0> p1_0;
 *      p1_0.init();
 *
 *      IOState stateP1_0 = p1_0.getState();
 *  @endcode
 *
 */
class InputHandle : public IoHandleBase {
public:
  using State = IOState;
  /**
   * Class constructor.
   *
   * Since the constructor is constexpr the compiler tries to resolved in compile time
   *
   */
  constexpr InputHandle(IOPort port, uint8_t desiredPin) : IoHandleBase(port, desiredPin) {}

  /**
   * Method initializes the pin. It sets the pin as an input.
   * @note At the moment the input is always disabling the internal resistor.
   */
  constexpr void init() const {
    resetRegisterBits(GPIORegisters::getPxDir(port), mBitMask);
    resetRegisterBits(GPIORegisters::getPxIfg(port), mBitMask);
    setIoFunctionality(IOFunctionality::GPIO);
  }
  // getState method is implemented in the IoHandleBase, since this class inherits from it
  // also has that functionality
};

template<IOPort port, uint8_t bitMask>
class IoBus {
public:
    constexpr void initialize() const noexcept {
        // Sets pins as output
        setRegisterBits(GPIORegisters::getPxDir(port), bitMask);
        // Set pins as IOs
        resetRegisterBits(GPIORegisters::getPxSel(port), bitMask);
        resetRegisterBits(GPIORegisters::getPxSel2(port), bitMask);
        // Disable pull up/down resistor
        resetRegisterBits(GPIORegisters::getPxRen(port), bitMask);
        // make sure interruption flags are 0
        resetRegisterBits(GPIORegisters::getPxIfg(port), bitMask);
        // Interruptions are disabled
        resetRegisterBits(GPIORegisters::getPxIe(port), bitMask);
    }

    constexpr uint8_t read() const noexcept {
        // Set pins as input
        resetRegisterBits(GPIORegisters::getPxDir(port), bitMask);
        return getRegisterBits(GPIORegisters::getPxIn(port), bitMask, static_cast<uint8_t>(0));
    }

    constexpr void write(const uint8_t newValue) noexcept {
        // set pins as output
        setRegisterBits(GPIORegisters::getPxDir(port), bitMask);
        return writeValueToRegister(GPIORegisters::getPxOut(port), bitMask, newValue);
    }
};

/**
 * This is a actually serves as a namespace to keep some GPIO classes together
 * and hides away some non-public types.
 *
 * @tparam port IOPort of that is handled by the GPIO instance
 */
class GPIOs {
public:
  /**
   * Constructor of the GPIOs class
   */
  constexpr GPIOs() {}

  /**
   * Method to get an OutputHandle. To see the purpose of an OutputHandle, please check its documentation
   * above.
   */
  template<IOPort port, uint8_t desiredPin>
  static constexpr OutputHandle getOutputHandle() noexcept {
    constexpr OutputHandle handle = OutputHandle(port, desiredPin);
    return handle;
  }

  /**
   * Method to get an InputHandle. To see the purpose of an InputHandle, please check its documentation
   * above.
   */
  template<IOPort port, uint8_t desiredPin>
  static constexpr InputHandle getInputHandle() noexcept {
    constexpr InputHandle handle(port, desiredPin);
    return handle;
  }
};
} /* namespace Microtech */
#endif /* COMMON_GPIOS_HPP_ */
