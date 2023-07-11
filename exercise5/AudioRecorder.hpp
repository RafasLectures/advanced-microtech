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
  void createNewAudio(const char* newName){
    auto newAudio = std::make_unique<Audio>(newName);
    selectedAudio = newAudio.get();
    audios.insert(audios.begin(), std::move(newAudio));
  }
  void eraseAllAudios(){

  }
  bool selectNextAudio() {
    return false;
  }
  bool selectPreviousAudio() {
    return false;
  }
  const char* getFreeTimeText() const {
    return freeTimeText.data();
  }
  const char* getCurrentAudioName() {
    if(selectedAudio == nullptr) {
      return "";
    }
    return selectedAudio->getName();
  }

  uint8_t getNumberOfStoredAudios() {
    return audios.size();
  }

  static constexpr uint8_t MAX_SIZE_AUDIO_NAME = 5;
private:

  std::array<char, 5> freeTimeText;
  Audio* selectedAudio = nullptr;
  std::vector<std::unique_ptr<Audio>> audios;
};
}
#endif  // ADVANVED_MICROTECH_AUDIORECORDER_HPP
