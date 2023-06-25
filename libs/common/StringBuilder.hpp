/******************************************************************************
* @file                    StringBuilder.hpp
* @author                  Rafael Andrioli Bauer
* @date                    24.05.2023
* @matriculation number    5163344
* @e-mail contact          abauer.rafael@gmail.com
* @brief                   Header file for StringBuilder class
*
******************************************************************************/
#ifndef ADVANVED_MICROTECH_STRINGBUILDER_HPP
#define ADVANVED_MICROTECH_STRINGBUILDER_HPP

#include <array>

namespace AdvancedMicrotech {
template<uint8_t STRING_SIZE>
class StringBuilder {
public:
  StringBuilder() = default;
  StringBuilder(const char* initialString) noexcept {
    setInitialMessage(initialString);
  }

  void setInitialMessage(const char* initialString) noexcept {
    clear();
    bool stringEnded = false;
    for (uint8_t index = 0; index < STRING_SIZE ; index++){
      if(initialString[index] == 0x00 || stringEnded) {
          buffer[index] = ASCII_SPACE;
          stringEnded = true;
      } else {
          buffer[index] = initialString[index];
      }
    }
  }

  void clear() noexcept {
    buffer.fill(0x00);
  }

  const char* getString() noexcept {
    return (const char*)buffer.data();
  }

  uint8_t* getBuffer() noexcept {
    return buffer.data();
  }
  uint8_t getBufferSize() noexcept {
    return buffer.size();
  }

  uint8_t setNextCharacter() noexcept {
    // do upper limit here
    buffer[currentIndex]++;

    if(buffer[currentIndex] > *"z") {   // 0x7A
      buffer[currentIndex] = ASCII_SPACE;
    } else if(buffer[currentIndex] > *"Z" && buffer[currentIndex] < *"a") {  // 0x5A
      buffer[currentIndex] = *"a";
    } else if(buffer[currentIndex] > *"9" && buffer[currentIndex] < *"A") {
      buffer[currentIndex] = *"A";
    } else if(buffer[currentIndex] > *"!" && buffer[currentIndex] < *"0") {
      buffer[currentIndex] = *"0";
    } else if(buffer[currentIndex] < ASCII_SPACE) { //0x32
      buffer[currentIndex] = ASCII_SPACE;
    }
    return buffer[currentIndex];
  }

  uint8_t setPreviousCharacter() noexcept {
    // do upper limit here
    buffer[currentIndex]--;

    if(buffer[currentIndex] > *"z") {   // 0x7A
      buffer[currentIndex] = *"z";
    } else if(buffer[currentIndex] > *"Z" && buffer[currentIndex] < *"a") {  // 0x5A
      buffer[currentIndex] = *"Z";
    } else if(buffer[currentIndex] > *"9" && buffer[currentIndex] < *"A") {
      buffer[currentIndex] = *"9";
    } else if(buffer[currentIndex] > *"!" && buffer[currentIndex] < *"0") {
      buffer[currentIndex] = *"!";
    } else if(buffer[currentIndex] < ASCII_SPACE) { //0x32
      buffer[currentIndex] = *"z";
    }
    return buffer[currentIndex];
    //return --buffer[currentIndex];
  }

  bool nextPosition() noexcept {
    if(currentIndex == STRING_SIZE-1) {
      return false;
    }
    currentIndex++;
    return true;
  }

  bool previousPosition() noexcept {
    if(currentIndex == 0) {
      return false;
    }
    currentIndex--;
    return true;
  }

  uint8_t setCurrentPosition(uint8_t newPosition) noexcept {
    if(newPosition > 16) {
        currentIndex = 16;
    }
    return currentIndex;
  }

  uint8_t getCurrentPosition() noexcept {
    return currentIndex;
  }

private:
  static constexpr uint8_t ASCII_SPACE = 0x20;

  // +1 because we want to include the null terminator
  std::array<uint8_t, STRING_SIZE + 1> buffer;
  uint8_t currentIndex = 0;
};
}

#endif  // ADVANVED_MICROTECH_STRINGBUILDER_HPP
