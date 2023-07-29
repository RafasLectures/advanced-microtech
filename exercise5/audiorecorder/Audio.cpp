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
  finishedActionCallback = finishedCallback;
  memoryManager.play(this);
}
void Audio::record(void(*finishedCallback)()) noexcept {
  finishedActionCallback = finishedCallback;
  memoryManager.record(this);
}

void Audio::finishedAction() noexcept {
  if(finishedActionCallback) {
    finishedActionCallback();
  }
}
uint32_t Audio::getAddress() const noexcept{
  return address;
}
const uint8_t* Audio::getName() const noexcept {
  return name.data();
}

}
