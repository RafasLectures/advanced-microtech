#ifndef ADVANVED_MICROTECH_MENU_HPP
#define ADVANVED_MICROTECH_MENU_HPP

#include "exercise5/AudioRecorder.hpp"
#include <cstdint>

namespace AdvancedMicrotech {

/**
 * A menu item, selectable by the user.
 * The item could perform a specific action or be another Menu in itself.
 *
 * Menu items must have:
 *  * A getTitle() method
 *  * A clearTitle() method
 *  * A DISPLAY attribute
 */
template <typename T, uint8_t X_POS, uint8_t Y_POS>
class MenuItem {
public:
  static constexpr void initialize(AudioRecorder* const audioRecorderPtr, Joystick* const joystickPtr){
    audioRecorder = audioRecorderPtr;
    joystick = joystickPtr;
    T::specificInitialize(audioRecorderPtr, joystickPtr);
  }

  static constexpr void showTitle() {
    printString(T::getTitle());
  }

  static void printString(const char* stringToPrint) {
    T::clearTitle();
    T::DISPLAY::setCursorPosition(X_POS, Y_POS);
    while (*stringToPrint != 0x00) {
      if(*stringToPrint == *"\n") {
        T::DISPLAY::setCursorPosition(0, Y_POS+1);
      } else {
        T::DISPLAY::writeChar(*stringToPrint);
      }
      stringToPrint++;
    }
  }
protected:
  static AudioRecorder* audioRecorder;
  static Joystick* joystick;
};

template <typename T, uint8_t X_POS, uint8_t Y_POS>
AudioRecorder*  MenuItem<T, X_POS, Y_POS>::audioRecorder = nullptr;

template <typename T, uint8_t X_POS, uint8_t Y_POS>
Joystick*  MenuItem<T, X_POS, Y_POS>::joystick = nullptr;

/**
 * Menus must implement:
 *  * A getTitle() method
 *  * A getMenuText() method
 *  * A clearText() method
 *  * A showItems() method
 *  * A initialize(Joystick*) method
 *  * A DISPLAY attribute
 * @tparam T
 */
template <typename T, uint8_t X_POS, uint8_t Y_POS>
class Menu : public MenuItem<T, X_POS, Y_POS>{
  using Base = MenuItem<T, X_POS, Y_POS>;
public:
  static void select() {
    T::DISPLAY::clearDisplay();
    MenuItem<T, 0, 0>::printString(T::getMenuText());
    T::showItems();
    Base::joystick->registerUpEventCallback(&upAction);
    Base::joystick->registerDownEventCallback(&downAction);
    Base::joystick->registerLeftEventCallback(&leftAction);
    Base::joystick->registerRightEventCallback(&rightAction);
    Base::joystick->registerPressEventCallback(&enterAction);
  }

  static constexpr void upAction() {
    T::specificUpAction();
  }

  static constexpr void downAction() {
    T::specificDownAction();
  }

  static constexpr void leftAction() {
    T::specificLeftAction();
  }

  static constexpr void rightAction() {
    T::specificRightAction();
  }

  static constexpr void enterAction() {
    T::specificEnterAction();
  }

};
} // namespace AdvancedMicrotech
#endif  // ADVANVED_MICROTECH_MENU_HPP
