#include "Menu.hpp"

namespace AdvancedMicrotech {
AudioRecorder* MenuItem::audioRecorder = nullptr;
Joystick* MenuItem::joystick = nullptr;
MenuItem::ClearDisplayFunctionPtr MenuItem::clearDisplayFunction = nullptr;
MenuItem::BlinkDisplayCursorFunctionPtr MenuItem::blinkDisplayCursorFunction = nullptr;
MenuItem::WriteStringToDisplayFunctionPtr MenuItem::writeStringToDisplayFunction = nullptr;
MenuItem::WriteCharToDisplayFunctionPtr MenuItem::writeCharToDisplayFunction = nullptr;
MenuItem::SetCursorPositionFunctionPtr MenuItem::setCursorPositionFunction = nullptr;

void MenuItem::clearDisplay() {
  clearDisplayFunction();
}
void MenuItem::blinkDisplayCursor(const uint8_t blink) {
  blinkDisplayCursorFunction(blink);
}
void MenuItem::writeStringToDisplay(const char* textToWrite) {
  while (*textToWrite != 0x00) {
    if (*textToWrite == *"\n") {
      MenuItem::setDisplayCursorPosition(0, 1);
    } else {
      writeCharToDisplay(*textToWrite);
    }
    textToWrite++;
  }
}
void MenuItem::writeCharToDisplay(const char charToWrite) {
  writeCharToDisplayFunction(charToWrite);
}
void MenuItem::setDisplayCursorPosition(const uint8_t x, const uint8_t y) {
  setCursorPositionFunction(x, y);
}
AudioRecorder* MenuItem::getAudioRecorderPtr() {
  return audioRecorder;
}
Joystick* MenuItem::getJoystickPtr() {
  return joystick;
}
}  // namespace AdvancedMicrotech