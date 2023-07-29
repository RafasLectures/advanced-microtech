#ifndef ADVANVED_MICROTECH_AUDIO_HPP
#define ADVANVED_MICROTECH_AUDIO_HPP

#include "MemoryManager.hpp"

#include <array>
#include <cstdint>

namespace AdvancedMicrotech {

class Audio {
public:
  using FinishedCallback = void(*)();
  static constexpr uint8_t MAX_SIZE_AUDIO_NAME = 6;

  Audio() = default;
  Audio(const uint8_t* newName, uint32_t newAddress);

  template <typename MEMORY_MANAGER_IMPL>
  static constexpr void setMemoryManager() {
    memoryManager.initialize = &MEMORY_MANAGER_IMPL::initialize;
    memoryManager.eraseAll = &MEMORY_MANAGER_IMPL::eraseAll;
    memoryManager.addSongToNextSlot = &MEMORY_MANAGER_IMPL::addSongToNextSlot;
    memoryManager.record = &MEMORY_MANAGER_IMPL::record;
    memoryManager.play = &MEMORY_MANAGER_IMPL::play;
  }

  void play(void(*finishedCallback)()) noexcept;
  void record(void(*finishedCallback)()) noexcept;

  void finishedAction() noexcept;
  uint32_t getAddress() const noexcept;
  const uint8_t* getName() const noexcept;

private:
  std::array<uint8_t, MAX_SIZE_AUDIO_NAME> name {0};
  uint32_t address;

  static MemoryManager memoryManager;
  FinishedCallback finishedActionCallback;

};
}
#endif  // ADVANVED_MICROTECH_AUDIO_HPP
