/******************************************************************************
 * @file                    ShiftRegister.hpp
 * @author                  Rafael Andrioli Bauer
 * @date                    18.11.2022
 * @matriculation number    5163344
 * @e-mail contact          abauer.rafael@gmail.com
 *
 * @brief   Header contains abstraction of a shift register
 *
 * Description: In order to abstract the manipulation of the shift register and
 *              ease up maintainability of the application code, this header
 *              provides an easy interface to manipulate the shift register
 *
 *              The interface was developed using template classes as well as
 *              constexpr so the program memory usage would be optimized.
 ******************************************************************************/

#ifndef COMMON_SHIFTREGISTER_HPP_
#define COMMON_SHIFTREGISTER_HPP_

#include "GPIOs.hpp"
namespace Microtech {

/**
 * Shift register base class is the base of every shift register.
 * It implements the basic supported functionality of the shift registers.
 */
class ShiftRegisterBase {
public:
  enum class Mode {
    PAUSE = 0,        ///< The shift register stops to change the output regardless of the clock
    SHIFT_RIGHT,      ///< Shifts the every output to the right. The value of QA is defined the serial input right (SR)
    SHIFT_LEFT,       ///< Shifts the every output to the left. The value of QD is defined the serial input left (SL)
    MIRROR_PARALLEL,  ///< The outputs QA~D will output the steady state input A~D
  };

  constexpr ShiftRegisterBase(const OutputHandle& clockHandle, const OutputHandle& clearHandle,
                              const OutputHandle& s0Handle, const OutputHandle& s1Handle)
    : clock(clockHandle), clear(clearHandle), s0(s0Handle), s1(s1Handle) {}

  /**
   * Initializes the shift registers.
   * It makes sure that the CLK pin is LOW and the shift registers are with CLR LOW,
   * so no matter if there is a clock sign, it won't start to change the outputs Qx until
   * the start() method has been called.
   */
  void init() const noexcept;

  /**
   * Method that allows user to set the mode of the shift register.
   */
  void setMode(const Mode mode) const noexcept;

  /**
   * Enables the shift register to shift the outputs.
   * It sets the CLR to HIGH.
   */
  void start() const noexcept;

  /**
   * It resets the outputs and stops to shift the shit register.
   * It sets the CLR to LOW.
   */
  void stopAndReset() const noexcept;

  /**
   * Only resets the output.
   * It changes the CLR to LOW and the HIGH again.
   */
  void reset() const noexcept;

  /**
   * Method makes a one clock cycle of the shift register
   */
  void clockOneCycle() const noexcept;

private:
  const OutputHandle s0;     ///< Pin that controls the S0 of the shift register
  const OutputHandle s1;     ///< Pin that controls the S1 of the shift register
  const OutputHandle clock;  ///< Output pin connected to the CLK input of the shift register
  const OutputHandle clear;  ///< Output pin connected to the CLR input of the shift register
};

/**
 * Class that represents the Shift register connected to the the LED
 */
class ShiftRegisterLED : public ShiftRegisterBase {
public:
  constexpr ShiftRegisterLED(const OutputHandle& clockHandle, const OutputHandle& clearHandle,
                             const OutputHandle& s0Handle, const OutputHandle& s1Handle,
                             const OutputHandle& shiftRightHandle)
    : ShiftRegisterBase(clockHandle, clearHandle, s0Handle, s1Handle), shiftRight(shiftRightHandle) {}

  /**
   * Calls the base initializer and set the the state to of QA to LOW when shifting right
   */
  void init() const noexcept {
    ShiftRegisterBase::init();
    shiftRight.init();
    shiftRight.setState(IOState::LOW);
  }

  void writeValue(uint8_t value) {
    // Cannot print a value more than 0xF, since it is more than 4 bits.
    constexpr uint8_t MAX_PRINT_VAL = 0xF;
    constexpr uint8_t NUM_SHIFTS_TO_PRINT = 5;  // Since its 4 bits, we need to have 4 shifts + 1.
    if (value > MAX_PRINT_VAL) {
      return;
    }
    // If the value was already printed, there is no need to print again.
    if (value == currentValue) {
      return;
    }

    currentValue = value;
    reset();  // clears the register
    setMode(Mode::SHIFT_RIGHT);
    for (uint8_t i = NUM_SHIFTS_TO_PRINT; i > 0; i--) {
      if (value & (0x01 << (i - 1))) {
        setQAStateOnRightShift(IOState::HIGH);
      } else {
        setQAStateOnRightShift(IOState::LOW);
      }
      ShiftRegisterBase::clockOneCycle();
    }
    setMode(Mode::PAUSE);
  }
  /**
   * Sets the QA state when a right shift happens
   */
  constexpr void setQAStateOnRightShift(const IOState state) const noexcept {
    shiftRight.setState(state);
  }

private:
  uint8_t currentValue = 0;
  const OutputHandle shiftRight;  ///< Pin that is connected to SR pin on the shift register
};

/**
 * Class that represents the Shift register connected to the the PB
 */
class ShiftRegisterPB : public ShiftRegisterBase {
public:
  constexpr ShiftRegisterPB(const OutputHandle& clockHandle, const OutputHandle& clearHandle,
                            const OutputHandle& s0Handle, const OutputHandle& s1Handle,
                            const InputHandle& inputQDtHandle)
    : ShiftRegisterBase(clockHandle, clearHandle, s0Handle, s1Handle), inputQD(inputQDtHandle) {}

  /**
   * Calls the base initializer initializes the input pin
   */
  void init() const noexcept {
    ShiftRegisterBase::init();
    inputQD.init();
  }

  uint8_t getPBValues() const noexcept {
      constexpr uint8_t NUM_SHIFTS_TO_READ = 5;  // Since its 4 bits, we need to have 4 shifts + 1.

      uint8_t returnValue = 0;
      reset();  // clears the register
      setMode(Mode::MIRROR_PARALLEL);
      for (uint8_t i = 0; i < NUM_SHIFTS_TO_READ; i++) {
        const IOState currentState = inputQD.getState();
        if (currentState == IOState::HIGH) {
            returnValue |= 0x1;
        }
        returnValue <<= 0x01;
        ShiftRegisterBase::clockOneCycle();
        setMode(ShiftRegisterBase::Mode::SHIFT_RIGHT);
      }
      returnValue >>= 0x01;
      setMode(Mode::PAUSE);
      return returnValue;
  }

  /**
   * Returns the state of the QD output of the shift register
   */
  constexpr IOState getInputQDState() const noexcept {
    return inputQD.getState();
  }

private:
  const InputHandle inputQD;  ///< Input pin connected to the QD output of the shift register
};
} /* namespace Microtech */

#endif /* COMMON_SHIFTREGISTER_HPP_ */
