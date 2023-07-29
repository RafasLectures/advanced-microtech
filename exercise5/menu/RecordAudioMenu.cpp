#include "RecordAudioMenu.hpp"
#include "AudioMenu.hpp"

namespace AdvancedMicrotech {

void RecordAudioMenu::clearTitle() {}
void RecordAudioMenu::showItems() {
  MenuItem::blinkDisplayCursor(false);
  getAudioRecorderPtr()->startRecordingCurrentAudio(&recordingFinishedCallback);
}
void RecordAudioMenu::specificUpAction() {}
void RecordAudioMenu::specificDownAction() {}
void RecordAudioMenu::specificLeftAction() {}
void RecordAudioMenu::specificRightAction() {}
void RecordAudioMenu::specificEnterAction() {
  getAudioRecorderPtr()->stopRecordingCurrentAudio();
  AudioMenu::select();
}

void RecordAudioMenu::recordingFinishedCallback() {
  AudioMenu::select();
}
}  // namespace AdvancedMicrotech
