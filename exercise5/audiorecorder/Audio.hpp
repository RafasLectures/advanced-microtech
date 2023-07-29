#ifndef ADVANVED_MICROTECH_AUDIO_HPP
#define ADVANVED_MICROTECH_AUDIO_HPP

#include "MemoryManager.hpp"

#include <array>
#include <cstdint>

namespace AdvancedMicrotech {

class Audio {
public:
  using FinishedCallback = void(*)();
  static constexpr uint32_t LENGTH = 0x07FFFF;
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

//  template <typename ADC>
//  static constexpr void setADCFunctions() {
//    startADC = &ADC::startConversion;
//    stopADC = &ADC::stopConversion;
//  }
//
//  template <typename PWM>
//  static constexpr void setPWMFunctions() {
//    startPWM = &PWM::start;
//    stopPWM = &PWM::stop;
//  }

  void play(void(*finishedCallback)()) noexcept;
  void record(void(*finishedCallback)()) noexcept;

  void recordingFinished() noexcept;
  uint32_t getAddress() const noexcept;
  const char* getName() const noexcept;

private:
  std::array<char, 5> name {0, 0, 0, 0, 0};
  uint32_t address;

  static MemoryManager memoryManager;
  FinishedCallback recordingFinishedCallback;
  FinishedCallback playingFinishedCallback;
//  static ControlCallback startADC;
//  static ControlCallback stopADC;
//  static ControlCallback startPWM;
//  static ControlCallback stopPWM;

};
}
#endif  // ADVANVED_MICROTECH_AUDIO_HPP
