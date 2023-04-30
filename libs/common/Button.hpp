/*******************************************************************************
 * @file                    Button.hpp
 * @author                  Rafael Andrioli Bauer
 * @date                    09.11.2022
 * @matriculation number    5163344
 * @e-mail contact          abauer.rafael@gmail.com
 *
 * @brief   Header contains an abstraction for a Button
 *
 * @note    The project was exported using CCS 12.1.0.00007
 ******************************************************************************/

#ifndef COMMON_BUTTON_HPP_
#define COMMON_BUTTON_HPP_

#include "GPIOs.hpp"
#include "Debouncer.hpp"

namespace Microtech {

/**
 * Enum that contains the possible states of a Button
 */
enum class ButtonState {
  RELEASED = 0,  ///< The button is not being pressed.
  PRESSED        ///< The button is being pressed.
};

/**
 * Class represents a button
 * One can define to which port and Pin this button is connected to
 * and whether the input logic is inverted or not.
 *
 * Apart from that, the its debounce is performed by the button and one can
 * also subscribe to the buttons state changes instead of polling the button state
 * all the time.
 *
 * @note it could be nice to find a way to remove these template parameters from
 * the button and let someone set the pin with a function.
 *
 */
class Button {
public:
  typedef void (*StateCallback)(ButtonState);

  /**
   * Class constructor.
   * One can say if it is inverted logic:
   * 0 = ButtonState::PRESSED
   * 1 = ButtonState::RELEASED
   */
  constexpr Button(const InputHandle& newInputHandle, bool invertedLogic) : inputHandle(newInputHandle), invertedLogic(invertedLogic), debouncer(*this, &Button::pinStateChanged) {}

  /**
   * Initialize the button. Gets state of the input pin and sets it as its state.
   */
  void init() noexcept {
    inputHandle.init();
    inputHandle.enablePinResistor(IOResistor::PULL_UP);
    setState(evaluateButtonState(inputHandle.getState()));
    inputHandle.enableInterrupt();
  }

  /**
   * Method to return the buttons state
   * @returns The button state
   */
  ButtonState getState() noexcept {
    return state;
  }

  /**
   * Method to perform the button debounce
   * Usually this is will be a function called by another periodic function
   * to constantly perform the pulling of the button state and then perform
   * debounce.
   */
  void evaluateDebounce() noexcept {
      debouncer.evaluateDebounce();
  }

  /**
   * Method to register a callback to the the button state, so whenever the button
   * changes state, the callback gets called.
   *
   * @param callbackPtr pointer to the callback function
   */
  void registerPressedStateChangeCallback(StateCallback callbackPtr) noexcept {
    stateCallback = callbackPtr;
  }

  Debouncer<Button>& getDebouncer() {
      return debouncer;
  }

private:

  void pinStateChanged() {
      setState(evaluateButtonState(IOState::LOW));
  }

  /**
   * Method to set the state of the button. Whenever a new state is set
   * it is also responsible for calling the button's callback
   *
   * @param newState The new state of the button
   *
   * @note In theory this method should be private, but for now
   *       I could not find a way to have the interrupts encapsulated,
   *       therefore, this is public.
   */
  void setState(ButtonState newState) noexcept {
    // If the newState is the same, there is no need to do
    // anything.
    if (state == newState) {
      return;
    }
    state = newState;

    // Make sure the callback pointer is not null before
    // calling it so there is no invalid memory access
    if (stateCallback != nullptr && newState == ButtonState::PRESSED) {
      stateCallback(state);  // calls the state callback
    }
    state = ButtonState::RELEASED;
  }
  /**
   * Method to set the state of the button. Whenever a new state is set
   * it is also responsible for calling the button's callback
   *
   * @param newState The new state of the button
   * @return the button state based on the input state
   *
   * @note In theory this method should be private, but for now
   *       I could not find a way to have the interrupts encapsulated,
   *       therefore, this is public.
   */
  constexpr ButtonState evaluateButtonState(const IOState pinState) noexcept {
    if (invertedLogic) {
      switch (pinState) {
        case IOState::LOW: return ButtonState::PRESSED;
        default:
        case IOState::HIGH: return ButtonState::RELEASED;
      };
    } else {
      switch (pinState) {
        case IOState::LOW: return ButtonState::RELEASED;
        default:
        case IOState::HIGH: return ButtonState::PRESSED;
      };
    }
  }

  const InputHandle inputHandle;  ///< Input pin of the button
  /**
   * If the logic of the button is inverted. Meaning that:
   * isInverted true:
   *      IOState::LOW = ButtonState::PRESSED and IOState::HIGH = ButtonState::RELEASED
   */
  const bool invertedLogic;
  ButtonState state = ButtonState::RELEASED;     ///< Current state of the button
  StateCallback stateCallback = nullptr;         ///< Function pointer to the state callback


  Debouncer<Button> debouncer;
};

} /* namespace Microtech */

#endif /* COMMON_BUTTON_HPP_ */
