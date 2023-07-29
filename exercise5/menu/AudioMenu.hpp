#ifndef ADVANVED_MICROTECH_AUDIOMENU_HPP
#define ADVANVED_MICROTECH_AUDIOMENU_HPP

#include "Menu.hpp"

namespace AdvancedMicrotech {

class AudioMenu : public Menu<AudioMenu> {
public:
  static constexpr uint8_t X_POS = 1;
  static constexpr uint8_t Y_POS = 1;

  enum class MenuSelection { PLAY = 0, RECORD, BACK };
  static const char* getTitle() {
    return (const char*)getAudioRecorderPtr()->getCurrentAudioName();
  }
  static constexpr const char* getMenuText() {
    return "      \x7F\x7ENav \x02Sel\n\xDBPlay \xDBRec \xDB\Back";
  }

  static void clearTitle();
  static void showItems();
  static void specificUpAction();
  static void specificDownAction();
  static void specificLeftAction();
  static void specificRightAction();
  static void specificEnterAction();

private:
  static constexpr void changeMenuSelection(MenuSelection newMenuSelection);
  static MenuSelection currentMenuSelection;
};
}  // namespace AdvancedMicrotech
#endif  // ADVANVED_MICROTECH_AUDIOMENU_HPP
