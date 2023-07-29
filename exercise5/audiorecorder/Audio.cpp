#include "Audio.hpp"

namespace AdvancedMicrotech {

MemoryManager Audio::memoryManager;

Audio::Audio(const uint8_t* newName, uint32_t newAddress) : address(newAddress){
  for(uint8_t i = 0; i < name.size(); i++) {
    if(!newName) {
      break;
    }
    name[i] = *newName++;
  }
}

void Audio::play(void(*finishedCallback)()) noexcept {
  playingFinishedCallback = finishedCallback;
  memoryManager.play(this);
  // start PWM
}
void Audio::record(void(*finishedCallback)()) noexcept {
  recordingFinishedCallback = finishedCallback;
  memoryManager.record(this);
}

void Audio::recordingFinished() noexcept {
  if(recordingFinishedCallback) {
    recordingFinishedCallback();
  }
}

uint32_t Audio::getAddress() const noexcept{
  return address;
}
const char* Audio::getName() const noexcept {
  return name.data();
}

}
