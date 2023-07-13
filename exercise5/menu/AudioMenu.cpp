#include "AudioMenu.hpp"
#include "MainMenu.hpp"
#include "PlayAudioMenu.hpp"
#include "RecordAudioMenu.hpp"
namespace AdvancedMicrotech {
AudioMenu::MenuSelection AudioMenu::currentMenuSelection;

void AudioMenu::clearTitle() {
  MenuItem::setDisplayCursorPosition(X_POS, Y_POS);
  MenuItem::writeStringToDisplay("          ");
}

void AudioMenu::showItems() {
  MenuItem::setDisplayCursorPosition(0, 0);
  MenuItem::writeStringToDisplay(getAudioRecorderPtr()->getCurrentAudioName());
  changeMenuSelection(MenuSelection::PLAY);
  MenuItem::blinkDisplayCursor(true);
}
void AudioMenu::specificUpAction() {}
void AudioMenu::specificDownAction() {}
void AudioMenu::specificLeftAction() {
  switch (currentMenuSelection) {
    case MenuSelection::PLAY: return;
    case MenuSelection::RECORD: changeMenuSelection(MenuSelection::PLAY); break;
    case MenuSelection::BACK: changeMenuSelection(MenuSelection::RECORD); break;
  };
}
void AudioMenu::specificRightAction() {
  switch (currentMenuSelection) {
    case MenuSelection::PLAY: changeMenuSelection(MenuSelection::RECORD); break;
    case MenuSelection::RECORD: changeMenuSelection(MenuSelection::BACK); break;
    case MenuSelection::BACK: return;
  };
}

void AudioMenu::specificEnterAction() {
  switch (currentMenuSelection) {
    case MenuSelection::PLAY: PlayAudioMenu::select(); break;
    case MenuSelection::RECORD: RecordAudioMenu::select(); break;
    case MenuSelection::BACK: MainMenu::select(); break;
  };
}

constexpr void AudioMenu::changeMenuSelection(MenuSelection newMenuSelection) {
  currentMenuSelection = newMenuSelection;
  switch (newMenuSelection) {
    case MenuSelection::PLAY: MenuItem::setDisplayCursorPosition(0, 1); break;
    case MenuSelection::RECORD: MenuItem::setDisplayCursorPosition(6, 1); break;
    case MenuSelection::BACK: MenuItem::setDisplayCursorPosition(11, 1); break;
  }
}

}  // namespace AdvancedMicrotech
