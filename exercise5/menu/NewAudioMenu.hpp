#ifndef ADVANVED_MICROTECH_NEWAUDIOMENU_HPP
#define ADVANVED_MICROTECH_NEWAUDIOMENU_HPP

#include "Menu.hpp"
#include "libs/common/StringBuilder.hpp"

namespace AdvancedMicrotech {

class NewAudioMenu : public Menu<NewAudioMenu> {
public:
  static constexpr uint8_t X_POS = 1;
  static constexpr uint8_t Y_POS = 1;

  static constexpr const char* getTitle() {
    return "New Audio";
  }
  static constexpr const char* getMenuText() {
    return "Audio Name: \x01Sel\n      \x7F\x7ENav \x02Save";
  }

  static void clearTitle();

  static void showItems();

  static void specificUpAction();
  static void specificDownAction();
  static void specificLeftAction();
  static void specificRightAction();
  static void specificEnterAction();

private:
  static StringBuilder<Audio::MAX_SIZE_AUDIO_NAME> newAudioName;
};
}  // namespace AdvancedMicrotech
#endif  // ADVANVED_MICROTECH_NEWAUDIOMENU_HPP
