#ifndef ADVANVED_MICROTECH_NEWAUDIOMENU_HPP
#define ADVANVED_MICROTECH_NEWAUDIOMENU_HPP

#include "Menu.hpp"
namespace AdvancedMicrotech {
template <typename LCD, uint8_t X_POS, uint8_t Y_POS>
class NewAudioMenu : public Menu<NewAudioMenu<LCD, X_POS, Y_POS>, X_POS, Y_POS> {
  using Base = Menu<NewAudioMenu<LCD, X_POS, Y_POS>, X_POS, Y_POS>;
public:
  using DISPLAY = LCD;
  static constexpr void specificInitialize(AudioRecorder* const audioRecorderPtr, Joystick* const joystickPtr) {
    newAudioName.clear();
    audioMenu.initialize(audioRecorderPtr, joystickPtr);
  }
  static constexpr const char* getTitle() { return "New Audio"; }
  static constexpr const char* getMenuText() { return "Song Name: \x01Sel\n      \x7F\x7ENav \x02Save"; }

  static constexpr void clearTitle() {
    DISPLAY::setCursorPosition(X_POS, Y_POS);
    DISPLAY::writeString("          ");
  }

  static constexpr void showItems() {
    DISPLAY::setCursorPosition(0, 1);
    newAudioName.clear();
    DISPLAY::blinkCursor(true);
  }

  static constexpr void specificUpAction() {
    DISPLAY::writeChar(newAudioName.setPreviousCharacter());
    DISPLAY::setCursorPosition(newAudioName.getCurrentPosition(), 1);
  }
  static constexpr void specificDownAction() {
    DISPLAY::writeChar(newAudioName.setNextCharacter());
    DISPLAY::setCursorPosition(newAudioName.getCurrentPosition(), 1);
  }
  static constexpr void specificLeftAction() {
    newAudioName.previousPosition();
    DISPLAY::setCursorPosition(newAudioName.getCurrentPosition(), 1);
  }
  static constexpr void specificRightAction() {
    newAudioName.nextPosition();
    DISPLAY::setCursorPosition(newAudioName.getCurrentPosition(), 1);
  }

  static constexpr void specificEnterAction() {
    Base::audioRecorder->createNewAudio(newAudioName.getString());
    audioMenu.select();
  }

private:
  static constexpr AudioMenu<LCD, 1, 1> audioMenu{};
  static StringBuilder<AudioRecorder::MAX_SIZE_AUDIO_NAME> newAudioName;
};
template <typename LCD, uint8_t X_POS, uint8_t Y_POS>
StringBuilder<AudioRecorder::MAX_SIZE_AUDIO_NAME> NewAudioMenu<LCD, X_POS, Y_POS>::newAudioName;
}
#endif  // ADVANVED_MICROTECH_NEWAUDIOMENU_HPP
