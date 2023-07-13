#ifndef ADVANVED_MICROTECH_MENU_HPP
#define ADVANVED_MICROTECH_MENU_HPP

#include "exercise5/AudioRecorder.hpp"
#include "libs/common/Joystick.hpp"
#include <cstdint>

namespace AdvancedMicrotech {

class MenuItem {
  using ClearDisplayFunctionPtr = void (*)();
  using BlinkDisplayCursorFunctionPtr = void (*)(const uint8_t);
  using WriteStringToDisplayFunctionPtr = void (*)(const char*);
  using WriteCharToDisplayFunctionPtr = void (*)(const char);
  using SetCursorPositionFunctionPtr = void (*)(const uint8_t, const uint8_t);

public:
  using DisplayTitleFunctionPtr = void (*)();

  static constexpr void setDependencies(AudioRecorder* const audioRecorderPtr, Joystick* const joystickPtr) {
    audioRecorder = audioRecorderPtr;
    joystick = joystickPtr;
  }

  template<typename DISPLAY>
  static constexpr void setDisplay() {
    clearDisplayFunction = &DISPLAY::clearDisplay;
    blinkDisplayCursorFunction = &DISPLAY::blinkCursor;
    writeStringToDisplayFunction = &DISPLAY::writeString;
    writeCharToDisplayFunction = &DISPLAY::writeChar;
    setCursorPositionFunction = &DISPLAY::setCursorPosition;
  }

protected:
  static void clearDisplay();
  static void blinkDisplayCursor(const uint8_t blink);
  static void writeStringToDisplay(const char* textToWrite);
  static void writeCharToDisplay(const char charToWrite);
  static void setDisplayCursorPosition(const uint8_t x, const uint8_t y);

  static AudioRecorder* getAudioRecorderPtr();
  static Joystick* getJoystickPtr();

private:
  static AudioRecorder* audioRecorder;
  static Joystick* joystick;
  static ClearDisplayFunctionPtr clearDisplayFunction;
  static BlinkDisplayCursorFunctionPtr blinkDisplayCursorFunction;
  static WriteStringToDisplayFunctionPtr writeStringToDisplayFunction;
  static WriteCharToDisplayFunctionPtr writeCharToDisplayFunction;
  static SetCursorPositionFunctionPtr setCursorPositionFunction;
};

template<typename CHILD>
class Menu : public MenuItem {
public:
  static constexpr void select() {
    displayMenu();
    CHILD::showItems();
    getJoystickPtr()->registerUpEventCallback(&upAction);
    getJoystickPtr()->registerDownEventCallback(&downAction);
    getJoystickPtr()->registerLeftEventCallback(&leftAction);
    getJoystickPtr()->registerRightEventCallback(&rightAction);
    getJoystickPtr()->registerPressEventCallback(&enterAction);
  }

  static void displayTitle() {
    MenuItem::setDisplayCursorPosition(CHILD::X_POS, CHILD::Y_POS);
    CHILD::clearTitle();
    MenuItem::setDisplayCursorPosition(CHILD::X_POS, CHILD::Y_POS);
    MenuItem::writeStringToDisplay(CHILD::getTitle());
  }

  static constexpr void upAction() {
    CHILD::specificUpAction();
  }

  static constexpr void downAction() {
    CHILD::specificDownAction();
  }

  static constexpr void leftAction() {
    CHILD::specificLeftAction();
  }

  static constexpr void rightAction() {
    CHILD::specificRightAction();
  }

  static constexpr void enterAction() {
    CHILD::specificEnterAction();
  }

private:
  static constexpr void displayMenu() {
    MenuItem::clearDisplay();
    MenuItem::setDisplayCursorPosition(0, 0);
    MenuItem::writeStringToDisplay(CHILD::getMenuText());
  }
};
}  // namespace AdvancedMicrotech
#endif  // ADVANVED_MICROTECH_MENU_HPP
