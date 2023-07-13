#ifndef ADVANVED_MICROTECH_AUDIORECORDER_HPP
#define ADVANVED_MICROTECH_AUDIORECORDER_HPP

#include "Audio.hpp"
#include <libs/common/helpers.hpp>
#include <array>
#include <chrono>
#include <memory>
#include <vector>

namespace AdvancedMicrotech {
class AudioRecorder {
public:
  void setFreeTime(std::chrono::seconds newFreeTime) {
    const std::chrono::minutes minutes = std::chrono::duration_cast<std::chrono::minutes>(newFreeTime);
    newFreeTime -= minutes;
    convertIntToCString(&freeTimeText[0], minutes.count(), 2, true);
    freeTimeText[2] = *":";
    convertIntToCString(&freeTimeText[3], newFreeTime.count(), 2, true);
  }
  void createNewAudio(const char* newName) {
    auto newAudio = std::make_unique<Audio>(newName);
    selectedAudio = audios.insert(audios.begin(), std::move(newAudio));
  }

  void startRecordingCurrentAudio() {}
  void stopRecordingCurrentAudio() {}
  void startPlayingCurrentAudio() {}
  void stopPlayingCurrentAudio() {}

  void eraseAllAudios() {
    for(const auto& audio : audios) {
      audio->erase();
    }
    audios.erase(audios.begin(), audios.end());
    selectedAudio = audios.end();
  }

  bool selectNextAudio() {
    if (std::next(selectedAudio) == audios.end()) {
      return false;
    }
    selectedAudio++;
    return true;
  }
  bool selectPreviousAudio() {
    if (selectedAudio == audios.begin()) {
      return false;
    }
    selectedAudio--;
    return true;
  }

  const char* getFreeTimeText() const {
    return freeTimeText.data();
  }
  const char* getCurrentAudioName() {
    if (selectedAudio == audios.end()) {
      return "";
    }
    return selectedAudio->get()->getName();
  }

  uint8_t getNumberOfStoredAudios() {
    return audios.size();
  }

  static constexpr uint8_t MAX_SIZE_AUDIO_NAME = 6;

private:
  std::array<char, 5> freeTimeText;
  std::vector<std::unique_ptr<Audio>>::iterator selectedAudio;
  std::vector<std::unique_ptr<Audio>> audios;
};
}  // namespace AdvancedMicrotech
#endif  // ADVANVED_MICROTECH_AUDIORECORDER_HPP
