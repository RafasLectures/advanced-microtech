/*******************************************************************************
 * @file    LCD.h
 * @author  Rafael Andrioli Bauer
 * @date    <date of creation>
 *
 * @brief   Header file with the prototypes of the LCD C functions as well as the
 *          declaration of the LCD class.
 ******************************************************************************/

#ifndef LIBS_LCD_H_
#define LIBS_LCD_H_

/******************************************************************************
 * INCLUDES
 *****************************************************************************/

#include "libs/common/GPIOs.hpp"

#include <msp430g2553.h>

#include <cstdint>

namespace AdvancedMicrotech {
/**
 *  4 bit control interface
 */
class LCD {
public:
  LCD() = delete;
  constexpr LCD(const Microtech::OutputHandle& selectRegisterHandle, const Microtech::OutputHandle& readWriteHandle,
      const Microtech::OutputHandle& enableReadWriteHandle) : selectRegister(selectRegisterHandle), readWrite(readWriteHandle), enableReadWrite(enableReadWriteHandle){}
  ~LCD() = default;

  /**
   * Initialization of the LCD; set all pin directions, basic setup of the LCD, etc.
   */
  void initialize() noexcept;
  /**
   * Enable (1) or disable (0) the display (i.e. hide all text)
   */
  void enable(const bool enable) noexcept;

  /**
   * Set the cursor to a certain x/y-position
   */
  void setCursorPosition(const uint8_t x, const uint8_t y) noexcept;

  /**
   *  Show or hide the cursor
   *  @param show boolean to choose if the cursor should be shown. True to show, false to hide.
   */
  void showCursor(const bool show) noexcept;

  /**
   * Method to blink or not the cursor of the LCD.
   * @param blink boolean to choose if the cursor should blink. True to blink, false not to.
   */
  void blinkCursor(const bool blink) noexcept;

  /**
   * Delete everything on the LCD
   */
  void clearDisplay() noexcept;

  /**
   * Put a single character on the display at the cursor's current position
   * @param character char to be written.
   */
  void writeChar(const char character) noexcept;

  /**
   * Show a given string on the display. If the text is too long to display,
   * there is no line break, the string is being cut.
   * @param text string to be written
   */
  void writeString(const char* text) noexcept;

  /**
   * Show a given number at the cursor's current position.
   * @param number
   */
  void writeNumber(const int16_t number) noexcept;

private:
  enum class RegisterSelection {
      INSTRUCTION = 0,
      DATA
  };
  enum class DataOperation {
      WRITE = 0,
      READ
  };

  constexpr void setOperation(const DataOperation newOperation) noexcept {
      if(newOperation == DataOperation::READ) {
          readWrite.setState(Microtech::IOState::HIGH);
      } else {
          readWrite.setState(Microtech::IOState::LOW);
      }
  }
  constexpr void setRegister(const RegisterSelection newSelection) noexcept {
      if(newSelection == RegisterSelection::DATA) {
          selectRegister.setState(Microtech::IOState::HIGH);
      } else {
          selectRegister.setState(Microtech::IOState::LOW);
      }
  }

  void sendInstruction(const uint8_t instruction) noexcept;

  void writeWordToBus(const uint8_t dataToWrite) noexcept;
  void writeByteToBus(const uint8_t dataToWrite) noexcept;
  bool isBusy() noexcept;
  /**
   * Signal RS. Selects registers.
   *    False = Instruction register (for write)
   *            Busy flag: address counter (for read)
   *    True  = Data register (for write and read)
   */
  const Microtech::OutputHandle selectRegister;
  const Microtech::OutputHandle readWrite;          ///< Signal R/W. Selects read or write. False = write; True = read
  const Microtech::OutputHandle enableReadWrite;     ///< Signal E. Starts data read write

  Microtech::IoBus<Microtech::IOPort::PORT_2, 0x0F> dataBus;
};

}
#ifdef __cplusplus
extern "C" {
#endif

/** Initialization */

// Initialization of the LCD; set all pin directions,
// basic setup of the LCD, etc. (1 pt.)
void lcd_init(void);

/** Control functions */

// Enable (1) or disable (0) the display (i.e. hide all text) (0.5 pts.)
void lcd_enable(unsigned char on);

// Set the cursor to a certain x/y-position (0.5 pts.)
void lcd_cursorSet(unsigned char x, unsigned char y);

// Show (1) or hide (0) the cursor (0.5 pts.)
void lcd_cursorShow(unsigned char on);

// Blink (1) or don't blink (0) the cursor (0.5 pts.)
void lcd_cursorBlink(unsigned char on);

/** Data manipulation */

// Delete everything on the LCD (1 pt.)
void lcd_clear(void);

// Put a single character on the display at the cursor's current position (1 pt.)
void lcd_putChar(char character);

// Show a given string on the display. If the text is too long to display,
// don't show the rest (i.e. don't break into the next line) (1 pt.).
void lcd_putText(char* text);

// Show a given number at the cursor's current position.
// Note that this is a signed variable! (1 pt.)
void lcd_putNumber(int number);
#ifdef __cplusplus
}
#endif
#endif /* LIBS_LCD_H_ */
