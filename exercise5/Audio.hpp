#ifndef ADVANVED_MICROTECH_AUDIO_HPP
#define ADVANVED_MICROTECH_AUDIO_HPP

#include "libs/common/StringBuilder.hpp"

namespace AdvancedMicrotech {
class Audio {
public:
  Audio(const char* newName) : name(newName) {}
  void play();
  void record();
  void erase();

  uint32_t getAddress() const {
    return address;
  }
  const char* getName() const noexcept {
    return name;
  }

private:
  const char* name;
  uint32_t address;
};
}
#endif  // ADVANVED_MICROTECH_AUDIO_HPP
