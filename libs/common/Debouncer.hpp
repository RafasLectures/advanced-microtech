#ifndef MICROTECH_DEBOUNCER_HPP
#define MICROTECH_DEBOUNCER_HPP

#include <cstdint>

namespace Microtech {
/**
* Classed used to filter the debounce of the input. It can generate multiple interrupts due to debouncing.
* This can be done with template to be easier, but since I don't have much time, it will stay like this.
 */
template<class CLASS_TYPE>
class Debouncer {
public:
  typedef void (CLASS_TYPE::*FuncPointer)();  ///< Definition of type of a function pointer from the chosen class

  Debouncer() = delete;
  constexpr explicit Debouncer(CLASS_TYPE& classRef, FuncPointer classFuncPtr)
    : objRef(classRef), funcPtr(classFuncPtr) {}

  /**
  * Function called by the timer and make sure that debouncing doesn't happen.
   */
  void evaluateDebounce() {
    if (buttonPushed == true) {  // If the button was pushed
      debounceCounter++;         // Increase debounce counter

      if (debounceCounter > 10) {  // If the counter is bigger than 10
        allowTrigger = true;      // Allows trigger when the button is pushed
        buttonPushed = false;
        debounceCounter = 0;
      }
    }
  }

  /**
  * Method called by the interrupt when the button is pushed
   */
  void pinStateChanged() {
    buttonPushed = true;
    if (allowTrigger == true) {                        // If the trigger is allowed
      if (funcPtr != nullptr) {         // Make sure function pointer is not null
        (objRef.*(funcPtr))();  // Calls the function pointer
      }
      allowTrigger = false;  // disallow trigger from the button.
    }
  }

private:
  uint8_t debounceCounter = 0;
  bool allowTrigger = true;
  bool buttonPushed = false;
  CLASS_TYPE& objRef;
  FuncPointer funcPtr = nullptr;
};
}

#endif  // MICROTECH_DEBOUNCER_HPP
