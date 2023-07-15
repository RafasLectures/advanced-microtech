
#define NO_TEMPLATE_UART

#include "exercise5/audiorecorder/AudioRecorder.hpp"

#include <cstdint>
#include <iostream>
using namespace AdvancedMicrotech;

AudioRecorder audioRecorder;


int main(void) {
  audioRecorder.createNewEmptyAudio("Test1");
  std::cout << "Selected Audio: " << audioRecorder.getCurrentAudioName() << std::endl;
  audioRecorder.createNewEmptyAudio("Test2");
  std::cout << "Selected Audio: " << audioRecorder.getCurrentAudioName() << std::endl;
  audioRecorder.createNewEmptyAudio("Test3");
  std::cout << "Selected Audio: " << audioRecorder.getCurrentAudioName() << std::endl;

  bool selected = audioRecorder.selectPreviousAudio();
  std::cout << "Selected previous Audio with result: " << selected << std::endl;
  std::cout << "Selected Audio: " << audioRecorder.getCurrentAudioName() << std::endl;
  selected = audioRecorder.selectPreviousAudio();
  std::cout << "Selected previous Audio with result: " << selected << std::endl;
  std::cout << "Selected Audio: " << audioRecorder.getCurrentAudioName() << std::endl;
  selected = audioRecorder.selectPreviousAudio();
  std::cout << "Selected previous Audio with result: " << selected << std::endl;
  std::cout << "Selected Audio: " << audioRecorder.getCurrentAudioName() << std::endl;

  selected = audioRecorder.selectNextAudio();
  std::cout << "Selected next Audio with result: " << selected << std::endl;
  std::cout << "Selected Audio: " << audioRecorder.getCurrentAudioName() << std::endl;
  selected = audioRecorder.selectNextAudio();
  std::cout << "Selected next Audio with result: " << selected << std::endl;
  std::cout << "Selected Audio: " << audioRecorder.getCurrentAudioName() << std::endl;
  selected = audioRecorder.selectNextAudio();
  std::cout << "Selected next Audio with result: " << selected << std::endl;
  std::cout << "Selected Audio: " << audioRecorder.getCurrentAudioName() << std::endl;



}
