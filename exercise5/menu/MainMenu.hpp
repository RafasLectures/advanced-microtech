#ifndef ADVANVED_MICROTECH_MAINMENU_HPP
#define ADVANVED_MICROTECH_MAINMENU_HPP

#include "Menu.hpp"

namespace AdvancedMicrotech {

class FreeTime : public MenuItem {
  static constexpr uint8_t X_POS = 11;
  static constexpr uint8_t Y_POS = 0;

public:
  static const char* getTitle();
  static void displayTitle();
  static void clearTitle();
};

class EraseAll : public MenuItem {
  static constexpr uint8_t X_POS = 1;
  static constexpr uint8_t Y_POS = 1;

public:
  static constexpr const char* getTitle() {
    return "Erase All";
  }
  static void displayTitle();
  static void clearTitle();
  static void execute();
};

class MainMenu : public Menu<MainMenu> {
public:
  static constexpr uint8_t X_POS = 0;
  static constexpr uint8_t Y_POS = 0;
  enum class MenuSelection { NEW_AUDIO = 0, ERASE_ALL, AUDIO };
  static constexpr const char* getTitle() {
    return "Main Menu";
  }
  static constexpr const char* getMenuText() {
    return "Free time:      \n\x01          \x02 Sel";
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
#endif  // ADVANVED_MICROTECH_MAINMENU_HPP
