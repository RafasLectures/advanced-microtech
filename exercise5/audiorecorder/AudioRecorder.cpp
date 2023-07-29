#include "AudioRecorder.hpp"
#include "libs/common/helpers.hpp"

namespace AdvancedMicrotech {

MemoryManager AudioRecorder::memoryManager;

void AudioRecorder::initialize() {
  memoryManager.initialize(this);
}

void AudioRecorder::setFreeTime(std::chrono::seconds newFreeTime) {
  const std::chrono::minutes minutes = std::chrono::duration_cast<std::chrono::minutes>(newFreeTime);
  newFreeTime -= minutes;
  convertIntToCString(freeTimeText.data(), minutes.count(), 2, true);
  freeTimeText[2] = *":";
  convertIntToCString(&freeTimeText[3], newFreeTime.count(), 2, true);
}


bool AudioRecorder::createNewEmptyAudio(const uint8_t* newName) {
  if(nextFreeIndex >= MAX_NUM_AUDIOS) {
    return false;
  }
  const uint32_t newAudioAddress = memoryManager.addSongToNextSlot(newName);
  addAudio(newName, newAudioAddress);
  return true;
}

void AudioRecorder::startRecordingCurrentAudio(void(*finishedCallback)()) {
  audios[selectedAudioIndex].record(finishedCallback);
}
void AudioRecorder::stopRecordingCurrentAudio() {}
void AudioRecorder::startPlayingCurrentAudio(void(*finishedCallback)()) {
  audios[selectedAudioIndex].play(finishedCallback);
}
void AudioRecorder::stopPlayingCurrentAudio() {}

void AudioRecorder::eraseAllAudios() {
  nextFreeIndex = 0;
  selectedAudioIndex = 0;
  memoryManager.eraseAll();
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
const uint8_t* AudioRecorder::getCurrentAudioName() const {
  if(nextFreeIndex == 0) {
    return (uint8_t*)"None";
  }
  return audios.at(selectedAudioIndex).getName();
}

uint8_t AudioRecorder::getNumberOfStoredAudios() const{
  return nextFreeIndex;
}
void AudioRecorder::addAudio(const uint8_t* name, uint32_t address) {
  selectedAudioIndex = nextFreeIndex;
  audios.at(nextFreeIndex++) = Audio(name, address);
}
}
