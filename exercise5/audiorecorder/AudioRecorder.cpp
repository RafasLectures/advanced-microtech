#include "AudioRecorder.hpp"
#include "libs/common/helpers.hpp"

namespace AdvancedMicrotech {

void AudioRecorder::setFreeTime(std::chrono::seconds newFreeTime) {
  const std::chrono::minutes minutes = std::chrono::duration_cast<std::chrono::minutes>(newFreeTime);
  newFreeTime -= minutes;
  convertIntToCString(freeTimeText.data(), minutes.count(), 2, true);
  freeTimeText[2] = *":";
  convertIntToCString(&freeTimeText[3], newFreeTime.count(), 2, true);
}


bool AudioRecorder::createNewEmptyAudio(const char* newName) {
  if(nextFreeIndex >= MAX_NUM_AUDIOS) {
    return false;
  }
  audios.at(nextFreeIndex++) = std::move(Audio(newName));
  return true;
}

void AudioRecorder::startRecordingCurrentAudio() {}
void AudioRecorder::stopRecordingCurrentAudio() {}
void AudioRecorder::startPlayingCurrentAudio() {}
void AudioRecorder::stopPlayingCurrentAudio() {}

void AudioRecorder::eraseAllAudios() {
  for(auto& audio : audios) {
    audio.erase();
  }
  nextFreeIndex = 0;
  selectedAudioIndex = 0;
}

bool AudioRecorder::selectNextAudio() {
  if(selectedAudioIndex >= (MAX_NUM_AUDIOS - 1)) {
    return false;
  }
  if(selectedAudioIndex+1 == nextFreeIndex) {
    return false;
  }
  selectedAudioIndex++;
  return true;
}
bool AudioRecorder::selectPreviousAudio() {
  if (selectedAudioIndex == 0) {
    return false;
  }
  selectedAudioIndex--;
  return true;
}

const char* AudioRecorder::getFreeTimeText() const {
  return freeTimeText.data();
}
const char* AudioRecorder::getCurrentAudioName() const {
  if(nextFreeIndex == 0) {
    return "None";
  }
  return audios.at(selectedAudioIndex).getName();
}

uint8_t AudioRecorder::getNumberOfStoredAudios() const{
  return nextFreeIndex;
}

}
