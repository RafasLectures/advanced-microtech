#ifndef ADVANVED_MICROTECH_AUDIORECORDER_HPP
#define ADVANVED_MICROTECH_AUDIORECORDER_HPP

#include "Audio.hpp"
#include <array>
#include <chrono>
#include <memory>
#include <vector>

namespace AdvancedMicrotech {
class AudioRecorder {
public:
  static constexpr uint8_t MAX_SIZE_AUDIO_NAME = 6;
  static constexpr uint8_t MAX_NUM_AUDIOS = 3;

  void setFreeTime(std::chrono::seconds newFreeTime);
  bool createNewEmptyAudio(const char* newName);

  void startRecordingCurrentAudio();
  void stopRecordingCurrentAudio();
  void startPlayingCurrentAudio();
  void stopPlayingCurrentAudio();

  void eraseAllAudios();

  bool selectNextAudio();
  bool selectPreviousAudio();

  const char* getFreeTimeText() const;
  const char* getCurrentAudioName() const;

  uint8_t getNumberOfStoredAudios() const;
private:
  std::array<char, 5> freeTimeText;
  volatile uint8_t selectedAudioIndex = 0;
  volatile uint8_t nextFreeIndex = 0;
  std::array<Audio,MAX_NUM_AUDIOS> audios;
};
}  // namespace AdvancedMicrotech
#endif  // ADVANVED_MICROTECH_AUDIORECORDER_HPP
