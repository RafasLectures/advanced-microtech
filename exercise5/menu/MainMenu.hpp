#ifndef ADVANVED_MICROTECH_MAINMENU_HPP
#define ADVANVED_MICROTECH_MAINMENU_HPP

#include "AudioMenu.hpp"
#include "NewAudioMenu.hpp"
#include "Menu.hpp"
#include "exercise5/AudioRecorder.hpp"

namespace AdvancedMicrotech {
// ========================= Free Time Item =========================
template<typename LCD, uint8_t X_POS, uint8_t Y_POS>
class Time : public MenuItem<Time<LCD, X_POS, Y_POS>, X_POS, Y_POS> {
  using Base = MenuItem<Time<LCD, X_POS, Y_POS>, X_POS, Y_POS>;
public:
  using DISPLAY = LCD;
  static constexpr void specificInitialize(AudioRecorder* const /*audioRecorderPtr*/, Joystick* const /*joystickPtr*/) {
    // Nothing to do here
  }

  static constexpr const char* getTitle() {
    return Base::audioRecorder->getFreeTimeText();
  }
  static constexpr void clearTitle() {
    DISPLAY::setCursorPosition(X_POS, Y_POS);
    DISPLAY::writeString("     ");
  }
};


template<typename LCD, uint8_t X_POS, uint8_t Y_POS>
class EraseAll : public MenuItem<EraseAll<LCD, X_POS, Y_POS>, X_POS, Y_POS> {
  using Base = MenuItem<EraseAll<LCD, X_POS, Y_POS>, X_POS, Y_POS>;
public:
  using DISPLAY = LCD;
  static constexpr void specificInitialize(AudioRecorder* const /*audioRecorderPtr*/, Joystick* const /*joystickPtr*/) {
    // Nothing to do here;
  }

  static constexpr const char*getTitle() {
    return "Erase All";
  }
  static constexpr void clearTitle() {
    DISPLAY::setCursorPosition(X_POS, Y_POS);
    DISPLAY::writeString("          ");
  }

  static constexpr void execute() {
    Base::audioRecorder->eraseAllAudios();
  }

};


// ======================== Main Menu =================
/**
 *
 * @tparam LCD
 */
template<typename LCD>
class MainMenu : public Menu<MainMenu<LCD>, 0, 0> {
  using Base = Menu<MainMenu<LCD>, 0, 0>;
  enum class MenuType {
    NEW_AUDIO = 0,
    ERASE_ALL,
    AUDIO
  };
public:
  using DISPLAY = LCD;
  static constexpr void specificInitialize(AudioRecorder* const audioRecorderPtr, Joystick* const joystickPtr) {
    timeItem.initialize(audioRecorderPtr, joystickPtr);
    newAudioMenu.initialize(audioRecorderPtr, joystickPtr);
    audioName.initialize(audioRecorderPtr, joystickPtr);
    eraseAll.initialize(audioRecorderPtr, joystickPtr);
  }
  static constexpr const char* getTitle() {
    return "Main Menu";
  }
  static constexpr const char* getMenuText() {
    return "Free time:      \n\x01          \x02 Sel";
  }
  static constexpr void clearTitle() {
    DISPLAY::clearDisplay();
  }


  static constexpr void showItems() {
    timeItem.showTitle();
    selectedMenuType = MenuType::NEW_AUDIO;
    newAudioMenu.showTitle();
  }

  static constexpr void specificUpAction() {
    switch (selectedMenuType) {
      case MenuType::NEW_AUDIO:
        return;
      case MenuType::ERASE_ALL:
        selectedMenuType = MenuType::NEW_AUDIO;
        newAudioMenu.showTitle();
        break;
      case MenuType::AUDIO:
        const bool selectionResult = Base::audioRecorder->selectPreviousAudio();
        if(!selectionResult) {
          selectedMenuType = MenuType::ERASE_ALL;
          eraseAll.showTitle();
        } else {
          audioName.showTitle();
        }
        break;
    }
  }
  static constexpr void specificLeftAction() {

  }
  static constexpr void specificRightAction() {

  }
  static constexpr void specificDownAction() {
    switch (selectedMenuType) {
      case MenuType::NEW_AUDIO:
        selectedMenuType = MenuType::ERASE_ALL;
        eraseAll.showTitle();
        break;
      case MenuType::ERASE_ALL:
        if(Base::audioRecorder->getNumberOfStoredAudios() == 0) {
          return;
        }
        audioName.showTitle();
        break;
      case MenuType::AUDIO:
        Base::audioRecorder->selectNextAudio();
        audioName.showTitle();
        break;
    }
  }

  static constexpr void specificEnterAction() {
    switch (selectedMenuType) {
      case MenuType::NEW_AUDIO:
        newAudioMenu.select();
        break;
      case MenuType::ERASE_ALL:
        eraseAll.execute();
        break;
      case MenuType::AUDIO:
        audioName.select();
        break;
    }
  }

private:
  static constexpr Time<LCD, 11, 0> timeItem{};
  static constexpr AudioMenu<LCD, 1, 1> audioName{};
  static constexpr NewAudioMenu<LCD, 1, 1> newAudioMenu{};
  static constexpr EraseAll<LCD, 1, 1> eraseAll{};
  static MenuType selectedMenuType;
};

template<typename LCD>
MainMenu<LCD>::MenuType MainMenu<LCD>::selectedMenuType = MainMenu<LCD>::MenuType::NEW_AUDIO;


}
#endif  // ADVANVED_MICROTECH_MAINMENU_HPP
