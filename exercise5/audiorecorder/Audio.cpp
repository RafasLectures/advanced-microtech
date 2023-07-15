#include "Audio.hpp"

namespace AdvancedMicrotech {

Audio::Audio(const char* newName) {
  for(uint8_t i = 0; i < name.size(); i++) {
    if(!newName) {
      break;
    }
    name[i] = *newName++;
  }
}

void Audio::play() noexcept{

}
void Audio::record() noexcept{

}
void Audio::erase() noexcept{

}
uint32_t Audio::getAddress() const noexcept{
  return address;
}
const char* Audio::getName() const noexcept {
  return name.data();
}

}
