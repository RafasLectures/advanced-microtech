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
    constexpr LCD() {};
  //constexpr LCD(const Microtech::OutputHandle& selectRegisterHandle, const Microtech::OutputHandle& readWriteHandle,
  //    const Microtech::OutputHandle& enableReadWriteHandle) : selectRegister(selectRegisterHandle), readWrite(readWriteHandle), enableReadWrite(enableReadWriteHandle){}
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
   * (x,y) -> (0,0) is top left and (15, 1) is bottom right.
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

  constexpr void enableOperation(const bool enableOperation) noexcept {
      enableReadWrite.setState(enableOperation);
  }

  void sendInstruction(const uint8_t instruction) noexcept;

  /**
   * Method to write word (4 bits) to the bus.
   * @param dataToWrite only the 4 LSB bits will be written
   */
  void writeWordToBus(const uint8_t dataToWrite) noexcept;
  /**
   * Method to write a byte (8 bits) to the bus.
   * @param dataToWrite byte to be written
   */
  void writeByteToBus(const uint8_t dataToWrite) noexcept;
  /**
   * Method to wait for the display until it is not busy anymore.
   * It is a blocking call. So once called it will only return when the display
   * is not busy anymore.
   */
  void waitUntilNotBusy() noexcept;

  void writeDigitOfNumber(int16_t number, uint8_t digit);

  /**
   * Signal RS. Selects registers.
   *    False = Instruction register (for write)
   *            Busy flag: address counter (for read)
   *    True  = Data register (for write and read)
   */
  const Microtech::OutputHandle selectRegister{Microtech::GPIOs::getOutputHandle<Microtech::IOPort::PORT_3, static_cast<uint8_t>(0)>()};
  const Microtech::OutputHandle readWrite = Microtech::GPIOs::getOutputHandle<Microtech::IOPort::PORT_3, static_cast<uint8_t>(1)>();          ///< Signal R/W. Selects read or write. False = write; True = read
  const Microtech::OutputHandle enableReadWrite = Microtech::GPIOs::getOutputHandle<Microtech::IOPort::PORT_3, static_cast<uint8_t>(2)>();     ///< Signal E. Enables/disables data read or write


  Microtech::IoBus<Microtech::IOPort::PORT_2, 0x0F> dataBus; ///< Databus that connects the display to the microcontroller

  uint8_t displayControl = 0;           ///< Attribute that holds the current "state" of the display control.
  uint8_t functionSet = 0;              ///< Attribute that holds the current "state" of the function set.
};

}
#if 0
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
#endif
#endif /* LIBS_LCD_H_ */
