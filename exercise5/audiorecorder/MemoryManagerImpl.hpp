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

  static constexpr uint32_t HEADER_SIZE = 1 + AudioRecorder::MAX_SIZE_AUDIO_NAME;
  static constexpr uint8_t HEADER_SIGNATURE = 0xA5;
  static constexpr uint8_t NUM_SONG_SLOTS = 4;

  static constexpr uint8_t WRITE_BUFFER_SIZE = 50;
  static constexpr uint16_t HALF_WRITE_BUFFER_SIZE = WRITE_BUFFER_SIZE / 2;
  static constexpr uint32_t SLOT_ADDRESS_OFFSET = (1L << 19);

  static constexpr void initialize(AudioRecorder* newAudioRecorder) {
    ADC::setDTCBuffer(&buffer[0], HALF_WRITE_BUFFER_SIZE);
    ADC::setADCIsrFunctionHandler(&handleWriteToFlashIRQ);
    PWM::setIsrCallback(&handleWriteToFlashIRQ);
    audioRecorder = newAudioRecorder;
    FLASH::init();
    latestAvailableSongSlot = 0;
    selectAudio(0);
    for(uint32_t songSlot = 0; songSlot < NUM_SONG_SLOTS; songSlot++) {
      // because it is latest available song * 8 << 16. since *8 is the same as << 3, 3+16= 19, so << 19.
      std::array<uint8_t, HEADER_SIZE> header{};
      const uint32_t headerAddress = songSlot * SLOT_ADDRESS_OFFSET;
      FLASH::read(headerAddress, HEADER_SIZE, header.data());
      // Check if header signature is correct
      if(header[0] != HEADER_SIGNATURE) {
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
    EXTERN_ADC_DAC::initialize();
  }

  static constexpr void eraseAll() {
//    delay_us(20);
    FLASH::init();
    FLASH::bulkErase();
    EXTERN_ADC_DAC::initialize();
  }

//  static constexpr bool readSongFromFlash(uint8_t* data, uint16_t length) {
//    if(transferAddress <= latestAvailableSongSlot) {
//      return false;
//    }
//    if((transferAddress + length) > latestAvailableSongSlot) {
//      length = latestAvailableSongSlot - transferAddress;
//    }
//    FLASH::read(transferAddress, data, length);
//    transferAddress += length;
//    return !(transferAddress <= latestAvailableSongSlot);
//  }

  static constexpr void record(Audio* audio) {
    currentAudio = audio;
    selectAudio(audio->getAddress());
    transferingData = true;
    writeAction = true;
//    delay_us(500);
    FLASH::init();
    ADC::startConversion();
  }

  static constexpr void play(Audio* audio) {
    currentAudio = audio;
    selectAudio(audio->getAddress());
    transferingData = true;
    writeAction = false;
//    delay_us(500);
    FLASH::init();
    FLASH::prepareRead(transferAddress);
    //PWM::start();
  }
  // Returns the song data address
  static constexpr uint32_t addSongToNextSlot(const uint8_t* name) {
    if(latestAvailableSongSlot > NUM_SONG_SLOTS) {
      return 0xFFFFFFFF;
    }
//    delay_us(20);
    FLASH::init();
    // because it is latest available song * 8 << 16. since *8 is the same as << 3, 3+16= 19, so << 19.
    const uint32_t headerAddress = latestAvailableSongSlot * SLOT_ADDRESS_OFFSET;
    std::array<uint8_t,HEADER_SIZE> songHeader{HEADER_SIGNATURE, name[0], name[1],  name[2],  name[3],  name[4]};
    FLASH::write(headerAddress, HEADER_SIZE, songHeader.data());
    latestAvailableSongSlot++;
//    delay_us(20);
    EXTERN_ADC_DAC::initialize();
    return headerAddress;
  }

  static constexpr bool executeAction() {
    if(!transferDataReady) {
      return transferingData;
    }

    uint16_t length = 0;
    uint16_t bufferIndex = 0;
    //if(writeAction) {
      length = sizeof(buffer) / 2;
      // true block 1, false block 2
      bufferIndex = ADC::getFinishedBlock() ? 0 : HALF_WRITE_BUFFER_SIZE;
//    } else {
//      length = 2;
//      bufferIndex = 0;
//    }

    if((transferAddress + length) > lastTransferAddress) {
      length = lastTransferAddress - transferAddress;
    }

//    if(writeAction) {
      FLASH::write(transferAddress, length, (uint8_t*)&buffer[bufferIndex]);
//    } else {
//      FLASH::keepReading(length, (uint8_t*)&buffer[bufferIndex]);
//      const _iq newOutput = _IQ(buffer[bufferIndex]);
//      // Calculates the new duty cycle
//      const _iq dutyCyclePWM = _IQmpy(_IQdiv(newOutput, _IQ(1023)),PWM::MAX_DUTY_CYCLE);
//      PWM::setNextDutyCycle(dutyCyclePWM);
//    }
    transferAddress += length;

    if(transferAddress >= lastTransferAddress) {
//      if(writeAction) {
        ADC::stopConversion();
//      } else {
//        FLASH::finishRead();
//      }
      EXTERN_ADC_DAC::initialize();
      transferDataReady = false;
      transferingData = false;
      currentAudio->recordingFinished();
    }
    transferDataReady = false;
    return transferingData;
  };
private:

  static constexpr bool selectAudio(uint32_t audioAddress) {
    transferAddress = audioAddress + HEADER_SIZE;
    lastTransferAddress = audioAddress + SLOT_ADDRESS_OFFSET - 1;
    return true;
  }

  static constexpr void handleWriteToFlashIRQ() {
    transferDataReady = true;
  }



  static volatile uint16_t buffer[WRITE_BUFFER_SIZE] ;
  static uint32_t latestAvailableSongSlot;
  static uint32_t transferAddress;
  static uint32_t lastTransferAddress;
  static AudioRecorder* audioRecorder;
  static Audio* currentAudio;
  static volatile bool transferDataReady;
  static bool transferingData;
  static bool writeAction;
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
bool MemoryManagerImpl<FLASH, ADC, EXTERN_ADC_DAC, PWM>::writeAction = false;
}
#endif  // ADVANVED_MICROTECH_MEMORYMANAGERIMPL_HPP
