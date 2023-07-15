#ifndef ADVANVED_MICROTECH_AUDIO_HPP
#define ADVANVED_MICROTECH_AUDIO_HPP

#include "libs/common/StringBuilder.hpp"

namespace AdvancedMicrotech {
class Audio {
public:
  Audio() = default;
  Audio(const char* newName);
  void play() noexcept;
  void record() noexcept;
  void erase() noexcept;

  uint32_t getAddress() const noexcept;
  const char* getName() const noexcept;

private:
  std::array<char, 5> name {0, 0, 0, 0, 0};
  uint32_t address = 0;
};
}
#endif  // ADVANVED_MICROTECH_AUDIO_HPP
