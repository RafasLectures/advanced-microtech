#ifndef ADVANVED_MICROTECH_AUDIORECORDER_HPP
#define ADVANVED_MICROTECH_AUDIORECORDER_HPP

#include "MemoryManager.hpp"
#include "Audio.hpp"
#include <array>
#include <chrono>
#include <memory>
#include <vector>

namespace AdvancedMicrotech {

class AudioRecorder {
public:
  static constexpr uint8_t MAX_NUM_AUDIOS = 4;

  void initialize();
  void setFreeTime(std::chrono::seconds newFreeTime);
  template <typename MEMORY_MANAGER_IMPL>
  static constexpr void setMemoryManager() {
    memoryManager.initialize = &MEMORY_MANAGER_IMPL::initialize;
    memoryManager.eraseAll = &MEMORY_MANAGER_IMPL::eraseAll;
    memoryManager.addSongToNextSlot = &MEMORY_MANAGER_IMPL::addSongToNextSlot;
    memoryManager.record = &MEMORY_MANAGER_IMPL::record;
    memoryManager.play = &MEMORY_MANAGER_IMPL::play;
  }
  bool createNewEmptyAudio(const uint8_t* newName);

  void startRecordingCurrentAudio(void(*finishedCallback)());
  void stopRecordingCurrentAudio();
  void startPlayingCurrentAudio(void(*finishedCallback)());
  void stopPlayingCurrentAudio();

  void eraseAllAudios();

  bool selectNextAudio();
  bool selectPreviousAudio();

  const char* getFreeTimeText() const;
  const uint8_t* getCurrentAudioName() const;

  uint8_t getNumberOfStoredAudios() const;

  void addAudio(const uint8_t* name, uint32_t address);
private:
  std::array<char, 5> freeTimeText{};
  static MemoryManager memoryManager;
  volatile uint8_t selectedAudioIndex = 0;
  volatile uint8_t nextFreeIndex = 0;
  std::array<Audio,MAX_NUM_AUDIOS> audios;
};

}  // namespace AdvancedMicrotech
#endif  // ADVANVED_MICROTECH_AUDIORECORDER_HPP
