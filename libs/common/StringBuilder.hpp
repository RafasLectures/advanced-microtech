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

/**
 * Class that provides a simple string builder for a string of a fixed size.
 *
 * A user can edit one character at a time, set a complete string.
 * @tparam STRING_SIZE The size of the string with the null terminator.
 */
template<uint8_t STRING_SIZE, uint8_t NULL_TERMINATED_STRING_SIZE = STRING_SIZE - 1>
class StringBuilder {
public:
  StringBuilder() = default;
  /**
   * Constructor of the class with an initial string.
   * @param initialString Initial string
   */
  explicit StringBuilder(const char* initialString) noexcept {
    setString(initialString);
  }

  /**
   * Method to set the string to be edited. If the string is bigger than the STRING_SIZE, the string is truncated.
   * @param initialString the string to be edited
   */
  void setString(const char* initialString) noexcept {
    clear();
    bool stringEnded = false;
    for (uint8_t index = 0; index < STRING_SIZE; index++) {
      if (initialString[index] == 0x00 || stringEnded) {
        buffer[index] = ASCII_SPACE;
        stringEnded = true;
      } else {
        buffer[index] = initialString[index];
      }
    }
  }

  /**
   * Method to clear the string.
   */
  void clear() noexcept {
    buffer.fill(0x00);
    currentIndex = 0;
  }

  /**
   * Method to get the string.
   * @return The string
   */
  const char* getString() const noexcept {
    return (const char*)buffer.data();
  }

  /**
   * Method to get the string as an uint8 buffer.
   * @return The string length
   */
  uint8_t* getBuffer() noexcept {
    return buffer.data();
  }
  /**
   * Method to get the string length (with null terminator).
   * @return The string length
   */
  constexpr uint8_t getBufferSize() noexcept {
    return STRING_SIZE;
  }

  /**
   * Method to increase the character by one at the current position.
   * @return The character at the current position after being incremented.
   */
  uint8_t setNextCharacter() noexcept {
    buffer[currentIndex]++;

    if (buffer[currentIndex] > *"z") {                                        // 0x7A
      buffer[currentIndex] = ASCII_SPACE;
    } else if (buffer[currentIndex] > *"Z" && buffer[currentIndex] < *"a") {  // 0x5A
      buffer[currentIndex] = *"a";
    } else if (buffer[currentIndex] > *"9" && buffer[currentIndex] < *"A") {
      buffer[currentIndex] = *"A";
    } else if (buffer[currentIndex] > *"!" && buffer[currentIndex] < *"0") {
      buffer[currentIndex] = *"0";
    } else if (buffer[currentIndex] < ASCII_SPACE) {  // 0x32
      buffer[currentIndex] = ASCII_SPACE;
    }
    return buffer[currentIndex];
  }

  /**
   * Method to decrease the character by one at the current position.
   * @return The character at the current position after being decremented.
   */
  uint8_t setPreviousCharacter() noexcept {
    buffer[currentIndex]--;

    if (buffer[currentIndex] > *"z") {                                        // 0x7A
      buffer[currentIndex] = *"z";
    } else if (buffer[currentIndex] > *"Z" && buffer[currentIndex] < *"a") {  // 0x5A
      buffer[currentIndex] = *"Z";
    } else if (buffer[currentIndex] > *"9" && buffer[currentIndex] < *"A") {
      buffer[currentIndex] = *"9";
    } else if (buffer[currentIndex] > *"!" && buffer[currentIndex] < *"0") {
      buffer[currentIndex] = *"!";
    } else if (buffer[currentIndex] < ASCII_SPACE) {  // 0x32
      buffer[currentIndex] = *"z";
    }
    return buffer[currentIndex];
  }

  /**
   * Method to go to the next position of the string. If we are at the end of the string,
   * the position will stay the same
   * @return If the position was incremented or not
   */
  bool nextPosition() noexcept {
    // -1 because we want the string to have a null terminator
    if (currentIndex == NULL_TERMINATED_STRING_SIZE - 1) {
      return false;
    }
    currentIndex++;
    return true;
  }

  /**
   * Method to go to the previous position of the string. If we are at the beginning if the string,
   * the position will stay the same.
   * @return If the position was decremented or not
   */
  bool previousPosition() noexcept {
    if (currentIndex == 0) {
      return false;
    }
    currentIndex--;
    return true;
  }

  /**
   * Method to get the current position of the string editor.
   * @return the current position of the string editor
   */
  uint8_t getCurrentPosition() noexcept {
    return currentIndex;
  }

private:
  static constexpr uint8_t ASCII_SPACE = 0x20;  ///< Space character in ASCII

  std::array<uint8_t, STRING_SIZE> buffer;  ///< The buffer that stores the string
  uint8_t currentIndex = 0;                     ///< The current position of the string editor
};
}  // namespace AdvancedMicrotech

#endif  // ADVANVED_MICROTECH_STRINGBUILDER_HPP
