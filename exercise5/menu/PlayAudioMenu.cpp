#include "PlayAudioMenu.hpp"
#include "AudioMenu.hpp"

namespace AdvancedMicrotech {

void PlayAudioMenu::clearTitle() {}
void PlayAudioMenu::showItems() {
  MenuItem::blinkDisplayCursor(false);
  getAudioRecorderPtr()->startPlayingCurrentAudio();
}
void PlayAudioMenu::specificUpAction() {}
void PlayAudioMenu::specificDownAction() {}
void PlayAudioMenu::specificLeftAction() {}
void PlayAudioMenu::specificRightAction() {}
void PlayAudioMenu::specificEnterAction() {
  getAudioRecorderPtr()->stopPlayingCurrentAudio();
  AudioMenu::select();
}
}  // namespace AdvancedMicrotech