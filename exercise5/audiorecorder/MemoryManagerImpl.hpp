#ifndef ADVANVED_MICROTECH_MEMORYMANAGERIMPL_HPP
#define ADVANVED_MICROTECH_MEMORYMANAGERIMPL_HPP

#include <array>
#include <cstdint>
#include "MemoryManager.hpp"
#include "AudioRecorder.hpp"
#include "Audio.hpp"
#include "IQmathLib.h"

namespace AdvancedMicrotech {

template<typename FLASH, typename ADC, typename EXTERN_ADC_DAC, typename PWM>
class MemoryManagerImpl {
public:
  // Header composition:
  // SIGNATURE(1B) + SONG NAME (5B)
  static constexpr uint32_t HEADER_SIZE = 1 + Audio::MAX_SIZE_AUDIO_NAME;
  static constexpr uint8_t HEADER_SIGNATURE = 0xA5;
  static_assert(AudioRecorder::MAX_NUM_AUDIOS <= 4, "Number of audios is more than 4, memory organization must be reviewed");
  static constexpr uint32_t SLOT_ADDRESS_OFFSET = (1L << 19);

  static constexpr uint8_t WRITE_BUFFER_SIZE = 50;
  static constexpr uint16_t HALF_WRITE_BUFFER_SIZE = WRITE_BUFFER_SIZE / 2;


  static constexpr void initialize(AudioRecorder* newAudioRecorder) {
    ADC::setDTCBuffer(&buffer[0], HALF_WRITE_BUFFER_SIZE);
    ADC::setADCIsrFunctionHandler(&handleWriteToFlashIRQ);
    PWM::setIsrCallback(&handlePWMIsr);
    FLASH::init();

    audioRecorder = newAudioRecorder;
    latestAvailableSongSlot = 0;
    selectAudio(nullptr);

    findStoredSongsInFlash();

    EXTERN_ADC_DAC::initialize();
  }

  static constexpr void eraseAll() {
    FLASH::init();
    FLASH::bulkErase();
    EXTERN_ADC_DAC::initialize();
  }

  static constexpr void record(Audio* audio) {
    selectAudio(audio);
    transferingData = true;
    recording = true;

    FLASH::init();
    uint32_t audioAddress = currentAudio->getAddress();
    for(uint8_t i = 0; i < 8; i++) {
      FLASH::sectorErase(audioAddress);
      audioAddress += SLOT_ADDRESS_OFFSET;
    }
    writeSongHeader(audio->getAddress(), audio->getName());
    ADC::startConversion();
  }

  static constexpr void play(Audio* audio) {
    selectAudio(audio);
    transferingData = true;
    recording = false;
    dutyCycleConstant = _IQdiv(_IQ(PWM::getCCR0()), _IQ(1023));

    FLASH::init();
    FLASH::prepareRead(transferAddress);
    // read the first 16bits
    FLASH::keepReading(2, (uint8_t*)&buffer[0]);
    PWM::start();
  }

  // Returns the song data address
  static constexpr uint32_t addSongToNextSlot(const uint8_t* name) {
    if(latestAvailableSongSlot > AudioRecorder::MAX_NUM_AUDIOS) {
      return 0xFFFFFFFF;
    }

    FLASH::init();

    const uint32_t headerAddress = latestAvailableSongSlot * SLOT_ADDRESS_OFFSET;
    writeSongHeader(headerAddress, name);
    latestAvailableSongSlot++;

    EXTERN_ADC_DAC::initialize();
    return headerAddress;
  }

  static constexpr bool executeAction() {
    if(!transferDataReady) {
      return transferingData;
    }
    bool finishedAction = false;
    if(recording) {
      finishedAction = recordAction();
    } else {
      finishedAction = playAction();
    }
    if(finishedAction) {
      EXTERN_ADC_DAC::initialize();
      transferingData = false;
      currentAudio->finishedAction();
    }
    transferDataReady = false;
    return transferingData;
  };

private:
  static constexpr void findStoredSongsInFlash() {
    // Loops through memory and check if the song headers are available
    for(uint32_t songSlot = 0; songSlot < AudioRecorder::MAX_NUM_AUDIOS; songSlot++) {
      std::array<uint8_t, HEADER_SIZE> header{};
      const uint32_t headerAddress = songSlot * SLOT_ADDRESS_OFFSET;
      FLASH::read(headerAddress, HEADER_SIZE, header.data());
      // Check if header signature is correct
      if(header[0] != HEADER_SIGNATURE) {
        // if the first signature is wrong, we consider the memory is corrupted, so clean it up
        if(songSlot == 0) {
          eraseAll();
          return;
        }
      } else {
        // If song signature is correct, add to the audio recorder
        audioRecorder->addAudio(&header[1], headerAddress);
        latestAvailableSongSlot++;
      }
    }
  }

  /**
   * Stores a song header in the flash
   * @param address Address to store
   * @param name Name of the song
   * @return
   */
  static constexpr void writeSongHeader(uint32_t address, const uint8_t* name) {
    std::array<uint8_t,HEADER_SIZE> songHeader{HEADER_SIGNATURE, name[0], name[1],  name[2],  name[3],  name[4]};
    FLASH::write(address, HEADER_SIZE, songHeader.data());
  }
  /**
   * Action to be executed when recording
   * @return if it finished or not
   */
  static constexpr bool recordAction() {
    constexpr uint16_t HALF_BUFFER = sizeof(buffer) / 2;
    // Initializes the length to be written by checking if half of the buffer would fit within the interval.
    const uint16_t length = ((transferAddress + HALF_BUFFER) <= lastTransferAddress) ? HALF_BUFFER : (lastTransferAddress - transferAddress);
    // Sets which half of the buffer to use.
    const uint16_t bufferIndex = ADC::getFinishedBlock() ? 0 : HALF_WRITE_BUFFER_SIZE;

    FLASH::write(transferAddress, length, (uint8_t*)&buffer[bufferIndex]);
    transferAddress += length;

    const bool finishedTransfer = transferAddress >= lastTransferAddress;
    if(finishedTransfer) {
      ADC::stopConversion();
    }
    return finishedTransfer;
  }

  /**
   * Action to be executed when playing
   * @return if it finished or not
   */
  static constexpr bool playAction() {
    constexpr uint16_t LENGTH = 2;
    constexpr uint16_t BUFFER_INDEX = 0;

    const _iq newOutput = _IQ(buffer[BUFFER_INDEX]);
    // Calculates the new duty cycle
    const uint16_t dutyCyclePWM = _IQint(_IQmpy(newOutput, dutyCycleConstant));
    PWM::setNextCCR2Val(dutyCyclePWM);

    FLASH::keepReading(LENGTH, (uint8_t*)&buffer[BUFFER_INDEX]);
    transferAddress += LENGTH;

    const bool finishedTransfer = transferAddress >= lastTransferAddress;
    if(finishedTransfer) {
      PWM::stop();
      FLASH::finishRead();
    }
    transferDataReady = false;
    return finishedTransfer;
  }

  static constexpr bool selectAudio(Audio* audio) {
    currentAudio = audio;
    if(currentAudio == nullptr) {
      transferAddress = 0;
      lastTransferAddress = SLOT_ADDRESS_OFFSET - 1;
    } else {
      transferAddress = currentAudio->getAddress() + HEADER_SIZE;
      lastTransferAddress = currentAudio->getAddress() + SLOT_ADDRESS_OFFSET - 1;
    }
    return true;
  }

  static constexpr void handleWriteToFlashIRQ() {
    transferDataReady = true;
  }
  static constexpr void handlePWMIsr() {
    transferDataReady = true;
  }

  static volatile uint16_t buffer[WRITE_BUFFER_SIZE] ;
  static volatile bool transferDataReady;

  static uint32_t latestAvailableSongSlot;

  static AudioRecorder* audioRecorder;
  static Audio* currentAudio;

  static uint32_t transferAddress;
  static uint32_t lastTransferAddress;
  static bool transferingData;
  static bool recording;
  static _iq dutyCycleConstant;
};
template<typename FLASH, typename ADC, typename EXTERN_ADC_DAC, typename PWM>
volatile uint16_t MemoryManagerImpl<FLASH, ADC, EXTERN_ADC_DAC, PWM>::buffer[MemoryManagerImpl<FLASH, ADC, EXTERN_ADC_DAC, PWM>::WRITE_BUFFER_SIZE] ={0};


template<typename FLASH, typename ADC, typename EXTERN_ADC_DAC, typename PWM>
uint32_t MemoryManagerImpl<FLASH, ADC, EXTERN_ADC_DAC, PWM>::latestAvailableSongSlot = 0;
template<typename FLASH, typename ADC, typename EXTERN_ADC_DAC, typename PWM>
uint32_t MemoryManagerImpl<FLASH, ADC, EXTERN_ADC_DAC, PWM>::transferAddress = 0;
template<typename FLASH, typename ADC, typename EXTERN_ADC_DAC, typename PWM>
uint32_t MemoryManagerImpl<FLASH, ADC, EXTERN_ADC_DAC, PWM>::lastTransferAddress = 0;
template<typename FLASH, typename ADC, typename EXTERN_ADC_DAC, typename PWM>
AudioRecorder* MemoryManagerImpl<FLASH, ADC, EXTERN_ADC_DAC, PWM>::audioRecorder = nullptr;
template<typename FLASH, typename ADC, typename EXTERN_ADC_DAC, typename PWM>
Audio* MemoryManagerImpl<FLASH, ADC, EXTERN_ADC_DAC, PWM>::currentAudio = nullptr;
template<typename FLASH, typename ADC, typename EXTERN_ADC_DAC, typename PWM>
volatile bool MemoryManagerImpl<FLASH, ADC, EXTERN_ADC_DAC, PWM>::transferDataReady = false;
template<typename FLASH, typename ADC, typename EXTERN_ADC_DAC, typename PWM>
bool MemoryManagerImpl<FLASH, ADC, EXTERN_ADC_DAC, PWM>::transferingData = false;
template<typename FLASH, typename ADC, typename EXTERN_ADC_DAC, typename PWM>
bool MemoryManagerImpl<FLASH, ADC, EXTERN_ADC_DAC, PWM>::recording = false;
template<typename FLASH, typename ADC, typename EXTERN_ADC_DAC, typename PWM>
_iq MemoryManagerImpl<FLASH, ADC, EXTERN_ADC_DAC, PWM>::dutyCycleConstant = 0;

}
#endif  // ADVANVED_MICROTECH_MEMORYMANAGERIMPL_HPP
