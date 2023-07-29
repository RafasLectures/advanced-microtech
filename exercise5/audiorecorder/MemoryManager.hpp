#ifndef ADVANVED_MICROTECH_MEMORYMANAGER_HPP
#define ADVANVED_MICROTECH_MEMORYMANAGER_HPP

#include <array>
#include <cstdint>

namespace AdvancedMicrotech {
class AudioRecorder;
class Audio;

class MemoryManager {
public:
  void(*initialize)(AudioRecorder*) = nullptr;
  void(*eraseAll)() = nullptr;
  uint32_t(*addSongToNextSlot)(const uint8_t*) = nullptr;
  void(*record)(Audio* audio) = nullptr;
  void(*play)(Audio* audio) = nullptr;
};

}
#endif  // ADVANVED_MICROTECH_MEMORYMANAGER_HPP
