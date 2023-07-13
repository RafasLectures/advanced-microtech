#ifndef ADVANVED_MICROTECH_PLAYAUDIOMENU_HPP
#define ADVANVED_MICROTECH_PLAYAUDIOMENU_HPP

#include "Menu.hpp"

namespace AdvancedMicrotech {
class PlayAudioMenu : public Menu<PlayAudioMenu> {
public:
  static constexpr uint8_t X_POS = 1;
  static constexpr uint8_t Y_POS = 1;

  static constexpr const char* getTitle() {
    return "Play";
  }
  static constexpr const char* getMenuText() {
    return "Playing         \n\x02Stop Playing   ";
  }

  static void clearTitle();

  static void showItems();

  static void specificUpAction();
  static void specificDownAction();
  static void specificLeftAction();
  static void specificRightAction();
  static void specificEnterAction();

private:
};
}  // namespace AdvancedMicrotech
#endif  // ADVANVED_MICROTECH_PLAYAUDIOMENU_HPP
