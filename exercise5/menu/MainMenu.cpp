#include "MainMenu.hpp"
#include "AudioMenu.hpp"
#include "NewAudioMenu.hpp"

namespace AdvancedMicrotech {

MainMenu::MenuSelection MainMenu::currentMenuSelection = MainMenu::MenuSelection::NEW_AUDIO;

// Free Time field
const char* FreeTime::getTitle() {
  return getAudioRecorderPtr()->getFreeTimeText();
}
void FreeTime::displayTitle() {
  MenuItem::setDisplayCursorPosition(X_POS, Y_POS);
  clearTitle();
  MenuItem::setDisplayCursorPosition(X_POS, Y_POS);
  MenuItem::writeStringToDisplay(getTitle());
}
void FreeTime::clearTitle() {
  MenuItem::setDisplayCursorPosition(X_POS, Y_POS);
  MenuItem::writeStringToDisplay("     ");
}

// Erase All
void EraseAll::displayTitle() {
  MenuItem::setDisplayCursorPosition(X_POS, Y_POS);
  clearTitle();
  MenuItem::setDisplayCursorPosition(X_POS, Y_POS);
  MenuItem::writeStringToDisplay(getTitle());
}
void EraseAll::clearTitle() {
  MenuItem::setDisplayCursorPosition(X_POS, Y_POS);
  MenuItem::writeStringToDisplay("         ");
}

void EraseAll::execute() {
  getAudioRecorderPtr()->eraseAllAudios();
}

// Main menu
void MainMenu::clearTitle() {
  MenuItem::clearDisplay();
}

void MainMenu::showItems() {
  MenuItem::blinkDisplayCursor(false);
  FreeTime::displayTitle();
  changeMenuSelection(MenuSelection::NEW_AUDIO);
}

void MainMenu::specificUpAction() {
  switch (currentMenuSelection) {
    case MenuSelection::NEW_AUDIO: return;
    case MenuSelection::ERASE_ALL: changeMenuSelection(MenuSelection::NEW_AUDIO); break;
    case MenuSelection::AUDIO:
      const bool selectionResult = getAudioRecorderPtr()->selectPreviousAudio();
      if (!selectionResult) {
        changeMenuSelection(MenuSelection::ERASE_ALL);
      } else {
        AudioMenu::displayTitle();
      }
      break;
  }
}
void MainMenu::specificDownAction() {
  switch (currentMenuSelection) {
    case MenuSelection::NEW_AUDIO: changeMenuSelection(MenuSelection::ERASE_ALL); break;
    case MenuSelection::ERASE_ALL:
      if (getAudioRecorderPtr()->getNumberOfStoredAudios() == 0) {
        return;
      }
      changeMenuSelection(MenuSelection::AUDIO);
      break;
    case MenuSelection::AUDIO:
      getAudioRecorderPtr()->selectNextAudio();
      AudioMenu::displayTitle();
      break;
  }
}
void MainMenu::specificLeftAction() {}
void MainMenu::specificRightAction() {}
void MainMenu::specificEnterAction() {
  switch (currentMenuSelection) {
    case MenuSelection::NEW_AUDIO: NewAudioMenu::select(); break;
    case MenuSelection::ERASE_ALL: EraseAll::execute(); break;
    case MenuSelection::AUDIO: AudioMenu::select(); break;
  }
}

constexpr void MainMenu::changeMenuSelection(MenuSelection newMenuSelection) {
  currentMenuSelection = newMenuSelection;
  switch (newMenuSelection) {
    case MenuSelection::NEW_AUDIO: NewAudioMenu::displayTitle(); break;
    case MenuSelection::ERASE_ALL: EraseAll::displayTitle(); break;
    case MenuSelection::AUDIO: AudioMenu::displayTitle(); break;
  }
}

}  // namespace AdvancedMicrotech
