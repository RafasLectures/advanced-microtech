#include "NewAudioMenu.hpp"
#include "AudioMenu.hpp"

namespace AdvancedMicrotech {

StringBuilder<AudioRecorder::MAX_SIZE_AUDIO_NAME> NewAudioMenu::newAudioName;

void NewAudioMenu::clearTitle() {
  MenuItem::setDisplayCursorPosition(X_POS, Y_POS);
  MenuItem::writeStringToDisplay("          ");
}

void NewAudioMenu::showItems() {
  MenuItem::setDisplayCursorPosition(0, 1);
  newAudioName.clear();
  MenuItem::blinkDisplayCursor(true);
}

void NewAudioMenu::specificUpAction() {
  MenuItem::writeCharToDisplay(newAudioName.setPreviousCharacter());
  MenuItem::setDisplayCursorPosition(newAudioName.getCurrentPosition(), 1);
}
void NewAudioMenu::specificDownAction() {
  MenuItem::writeCharToDisplay(newAudioName.setNextCharacter());
  MenuItem::setDisplayCursorPosition(newAudioName.getCurrentPosition(), 1);
}
void NewAudioMenu::specificLeftAction() {
  newAudioName.previousPosition();
  MenuItem::setDisplayCursorPosition(newAudioName.getCurrentPosition(), 1);
}
void NewAudioMenu::specificRightAction() {
  newAudioName.nextPosition();
  MenuItem::setDisplayCursorPosition(newAudioName.getCurrentPosition(), 1);
}

void NewAudioMenu::specificEnterAction() {
  getAudioRecorderPtr()->createNewEmptyAudio(newAudioName.getBuffer());
  AudioMenu::select();
}
}  // namespace AdvancedMicrotech
