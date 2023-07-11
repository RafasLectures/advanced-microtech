#ifndef ADVANVED_MICROTECH_AUDIOMENU_HPP
#define ADVANVED_MICROTECH_AUDIOMENU_HPP

#include "Menu.hpp"

#include <cstdint>
namespace  AdvancedMicrotech{
template<typename LCD, uint8_t X_POS, uint8_t Y_POS>
class AudioMenu : public Menu<AudioMenu<LCD, X_POS, Y_POS>, X_POS, Y_POS> {
  using Base = Menu<AudioMenu<LCD, X_POS, Y_POS>, X_POS, Y_POS>;
  enum class CurrentSelection {
    PLAY = 0,
    RECORD,
    BACK
  };
public:
  using DISPLAY = LCD;
  static constexpr void specificInitialize(AudioRecorder* const /*audioRecorderPtr*/, Joystick* const /*joystickPtr*/) {

  }
  static constexpr const char* getTitle() {
    return Base::audioRecorder->getCurrentAudioName();
  }
  static constexpr const char* getMenuText() { return "      \x7F\x7ENav \x02Sel\n\xDB\Play \xDB\Rec \xDB\Back"; }

  static constexpr void clearTitle() {
    DISPLAY::setCursorPosition(X_POS, Y_POS);
    DISPLAY::writeString("          ");
  }

  static constexpr void showItems() {
    DISPLAY::setCursorPosition(0, 0);
    DISPLAY::writeString(Base::audioRecorder->getCurrentAudioName());
    DISPLAY::setCursorPosition(0, 1);
    currentSelection = CurrentSelection::PLAY;
    DISPLAY::blinkCursor(true);
  }
  static constexpr void specificUpAction() {

  }
  static constexpr void specificLeftAction() {
    switch (currentSelection) {
      case CurrentSelection::PLAY:
        return;
      case CurrentSelection::RECORD:
        currentSelection = CurrentSelection::PLAY;
        DISPLAY::setCursorPosition(0, 1);
        break;
      case CurrentSelection::BACK:
        currentSelection = CurrentSelection::RECORD;
        DISPLAY::setCursorPosition(6, 1);
        break;
    };

  }
  static constexpr void specificRightAction() {
    switch (currentSelection) {
      case CurrentSelection::PLAY:
        currentSelection = CurrentSelection::RECORD;
        DISPLAY::setCursorPosition(6, 1);
        break;
      case CurrentSelection::RECORD:
        currentSelection = CurrentSelection::BACK;
        DISPLAY::setCursorPosition(11, 1);
        break;
      case CurrentSelection::BACK:
        return;
    };
  }
  static constexpr void specificDownAction() {

  }
  static constexpr void specificEnterAction() {
    switch (currentSelection) {
      case CurrentSelection::PLAY:

      break;
      case CurrentSelection::RECORD:
      break;
      case CurrentSelection::BACK:

      break;
    };
  }

private:
  static CurrentSelection currentSelection;
};

template<typename LCD, uint8_t X_POS, uint8_t Y_POS>
AudioMenu<LCD, X_POS, Y_POS>::CurrentSelection AudioMenu<LCD, X_POS, Y_POS>::currentSelection = AudioMenu<LCD, X_POS, Y_POS>::CurrentSelection::PLAY;
}
#endif  // ADVANVED_MICROTECH_AUDIOMENU_HPP
