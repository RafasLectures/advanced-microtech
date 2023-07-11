#include "Joystick.hpp"

namespace AdvancedMicrotech {

void Joystick::registerUpEventCallback(Callback callback) noexcept {
  upEventCallback = callback;
}
void Joystick::registerDownEventCallback(Callback callback) noexcept {
  downEventCallback = callback;
}
void Joystick::registerLeftEventCallback(Callback callback) noexcept {
  leftEventCallback = callback;
}
void Joystick::registerRightEventCallback(Callback callback) noexcept {
  rightEventCallback = callback;
}
void Joystick::registerPressEventCallback(Callback callback) noexcept {
  pressEventCallback = callback;
}

void Joystick::evaluateJoystick(const uint8_t (*adcValues)[4]) noexcept {
  constexpr uint8_t X_AD_CHANNEL = 1;      // Variable just to make easier to extract the ADC value
  constexpr uint8_t Y_AD_CHANNEL = 2;      // Variable just to make easier to extract the ADC value
  constexpr uint8_t PRESS_AD_CHANNEL = 3;  // Variable just to make easier to extract the ADC value

  constexpr uint8_t LEFT_THRESHOLD = 253;
  constexpr uint8_t RIGHT_THRESHOLD = 2;

  constexpr uint8_t UP_THRESHOLD = 2;
  constexpr uint8_t DOWN_THRESHOLD = 253;

  static constexpr uint8_t PRESSED_THRESHOLD = 254;

  static bool xActive = false;
  static bool yActive = false;
  static bool pressActive = false;

  // I think I can make this more elegant, but for now I will just leave it as is
  if ((*adcValues)[X_AD_CHANNEL] > LEFT_THRESHOLD) {
    if(!xActive && leftEventCallback != nullptr) {
      leftEventCallback();
      xActive = true;
    }
  } else if ((*adcValues)[X_AD_CHANNEL] < RIGHT_THRESHOLD) {
    if(!xActive && rightEventCallback != nullptr) {
      rightEventCallback();
      xActive = true;
    }
  } else {
    xActive = false;
  }

  if ((*adcValues)[Y_AD_CHANNEL] < UP_THRESHOLD) {
    if(!yActive && upEventCallback != nullptr) {
      upEventCallback();
      yActive = true;
    }
  } else if ((*adcValues)[Y_AD_CHANNEL] > DOWN_THRESHOLD) {
    if(!yActive && downEventCallback != nullptr) {
      downEventCallback();
      yActive = true;
    }
  } else {
    yActive = false;
  }

  if ((*adcValues)[PRESS_AD_CHANNEL] > PRESSED_THRESHOLD) {
    if(!pressActive && pressEventCallback != nullptr) {
      pressEventCallback();
      pressActive = true;
    }
  } else {
    pressActive = false;
  }
}
}  // namespace AdvancedMicrotech
