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

#include "LcdCommands.hpp"
#include "libs/common/clocks.hpp"
#include <cstdint>

namespace AdvancedMicrotech {

template <typename RS,
          typename RW,
          typename E,
          typename BUS>
class LCD_T {
public:
  /**
   * Initialization of the LCD; set all pin directions, basic setup of the LCD, etc.
   */
  static constexpr void initialize() {
      displayControl = DisplayControl::BASE_VALUE;
      functionSet = FunctionSet::BASE_VALUE;

      // Initialize IOs
      RS::init();
      E::init();
      RW::init();
      BUS::init();

      // Create the function set frame to how we want to work:
      //  - Interface of 4 bits
      //  - 2 Lines
      //  - Character size 5x8 dots
      FunctionSet::INTERFACE_DATA_LENGTH.encode(functionSet, FunctionSet::InterfaceDataLength::BITS_4);
      FunctionSet::NUMBER_OF_LINES.encode(functionSet, FunctionSet::NumberOfLines::LINES_2);
      FunctionSet::CHARACHTER_FONT.encode(functionSet, FunctionSet::CharachterFont::DOTS_5X8);

      // Sets the function set to 4 bits.
      // According to datasheet, we first need to write the MSB of the functionSet frame once
      // and then write the whole frame again, so we set it first
      // to 4-bit operation.
      waitUntilNotBusy();
      writeWordToBus(functionSet >> 4);
      sendInstruction(functionSet);
      clearDisplay();

  }
  /**
   * Enable (1) or disable (0) the display (i.e. hide all text)
   */
  static void enable(const bool enable) noexcept {
      DisplayControl::DISPLAY_ON.encode(displayControl, enable);
      sendInstruction(displayControl);
  }

  /**
   * Set the cursor to a certain x/y-position
   * (x,y) -> (0,0) is top left and (15, 1) is bottom right.
   */
  static void setCursorPosition(const uint8_t x, const uint8_t y) noexcept {
      static constexpr uint8_t SET_ADDRESS_BASE = 0x80;
      uint8_t address = SET_ADDRESS_BASE;
      if(y >= 1) {
          address |= 0x40;
      }
      if(x > 0x0F) {
          address |= 0x0F;
      } else {
          address |= x;
      }

      // waitUntilNotBusy();
      setRegister(RegisterSelection::INSTRUCTION);
      writeByteToBus(address);
  }

  /**
   *  Show or hide the cursor
   *  @param show boolean to choose if the cursor should be shown. True to show, false to hide.
   */
  static void showCursor(const bool show) noexcept {
      DisplayControl::CURSOR_ON.encode(displayControl, show);
      sendInstruction(displayControl);
  }

  /**
   * Method to blink or not the cursor of the LCD.
   * @param blink boolean to choose if the cursor should blink. True to blink, false not to.
   */
  static void blinkCursor(const bool blink) noexcept {
      DisplayControl::BLINK_CURSOR.encode(displayControl, blink);
      sendInstruction(displayControl);
  }

  /**
   * Delete everything on the LCD
   */
  static void clearDisplay() noexcept {
      sendInstruction(CLEAR_DISPLAY);
  }

  /**
   * Put a single character on the display at the cursor's current position
   * @param character char to be written.
   */
  static void writeChar(const char character) noexcept {
      setRegister(RegisterSelection::DATA);
      writeByteToBus(character);
  }

  /**
   * Show a given string on the display. If the text is too long to display,
   * there is no line break, the string is being cut.
   * @param text string to be written
   */
  static void writeString(const char* text) noexcept {
      while(*text != 0x00) {
          writeChar(*text);
          text++;
      }
  }

  /**
   * Show a given number at the cursor's current position.
   * @param number
   */
  static void writeNumber(int16_t number) noexcept {
      static constexpr int16_t SIGN_MASK = 0x8000;
      // Check if number is negative
      if(SIGN_MASK & number) {
          writeChar(0x2D); // Minus sign in ASCII

          // Flip every bit of it so its like a normal number
          // without the sign and add 1 because -1 is 0xFFFF, so after flipping
          // it is 0x0000 + 1 = 0x0001;
          number = ~number;
          number++;
      }
      int8_t i = 4;
      while(i >= 0) {
          writeDigitOfNumber(number, i);
          i--;
      }
  }

private:
  enum class RegisterSelection {
      INSTRUCTION = 0,
      DATA
  };
  enum class DataOperation {
      WRITE = 0,
      READ
  };

  static constexpr void setOperation(const DataOperation newOperation) noexcept {
      if(newOperation == DataOperation::READ) {
          RW::set_high();
      } else {
          RW::set_low();
      }
  }
  static constexpr void setRegister(const RegisterSelection newSelection) noexcept {
      if(newSelection == RegisterSelection::DATA) {
          RS::set_high();
      } else {
          RS::set_low();
      }
  }

  static constexpr void enableOperation(const bool enableOperation) noexcept {
      E::set(enableOperation);
  }

  static void sendInstruction(const uint8_t instruction) noexcept {
      //waitUntilNotBusy();
      setRegister(RegisterSelection::INSTRUCTION);
      writeByteToBus(instruction);
  }

  /**
   * Method to write word (4 bits) to the bus.
   * @param dataToWrite only the 4 LSB bits will be written
   */
  static void writeWordToBus(const uint8_t dataToWrite) noexcept {
     enableOperation(false);  // Disable write
     setOperation(DataOperation::WRITE);                 // Set to write mode
     BUS::write(dataToWrite);                         // Put data to bus
     enableOperation(true); // Enable write
     delay_us(30);
     enableOperation(false);  // Disable write
  }
  /**
   * Method to write a byte (8 bits) to the bus.
   * @param dataToWrite byte to be written
   */
  static void writeByteToBus(const uint8_t dataToWrite) noexcept {
      const uint8_t dataMsb = (dataToWrite) >> 4;
      const uint8_t dataLsb = (0x0F & dataToWrite);
      waitUntilNotBusy();
      writeWordToBus(dataMsb);
      writeWordToBus(dataLsb);
  }
  /**
   * Method to wait for the display until it is not busy anymore.
   * It is a blocking call. So once called it will only return when the display
   * is not busy anymore.
   */
  static void waitUntilNotBusy() noexcept {
      static constexpr uint8_t BUSY_MASK = 0x08;      // The mask to decode the busy bit
      const bool currentSelection = RS::get();
      setRegister(RegisterSelection::INSTRUCTION);
      setOperation(DataOperation::READ);              // Set to read mode
      BUS::read();                                 // Perform a read, but main goal is to set the data bus as input.
      enableOperation(true);                          // Enable read
      delay_us(30);
      uint8_t dataVal = BUS::read();               // Read value from Bus
      while(dataVal & BUSY_MASK) {
          dataVal = BUS::read();                   // Read value from Bus
      }

      uint8_t addrCounter = (0xE & dataVal) << 4;
      enableOperation(false);      // Disable read
      delay_us(30);
      enableOperation(true);     // Enable read
      delay_us(30);
      addrCounter |= BUS::read();                          // Get value from Bus
      enableOperation(false);      // Disable read
      RS::set(currentSelection);
  }

  static void writeDigitOfNumber(int16_t number, uint8_t digit) {
      static constexpr int16_t POW10[] = {1, 10, 100, 1000, 10000};
      static constexpr int16_t BASE_ASCII = 0x30;
      if(digit > 4) {
          // Only support up to 4 digits. Early return
          return;
      }
      uint8_t digitToBeRemoved = 0;
      if(digit < 4) {
          digitToBeRemoved = digit + 1;
      } else {
          digitToBeRemoved = digit;
      }
      const int16_t originalNumber = number;
      const int16_t numberToProcess = number % POW10[digitToBeRemoved];
      if (originalNumber >= POW10[digit]) {
          const int16_t toBeWritten = numberToProcess / POW10[digit];
          writeChar(BASE_ASCII + toBeWritten);
      }
  }

  /**
   * Signal RS. Selects registers.
   *    False = Instruction register (for write)
   *            Busy flag: address counter (for read)
   *    True  = Data register (for write and read)
   */
  //const Microtech::OutputHandle selectRegister{Microtech::GPIOs::getOutputHandle<Microtech::IOPort::PORT_3, static_cast<uint8_t>(0)>()};
  //const Microtech::OutputHandle readWrite = Microtech::GPIOs::getOutputHandle<Microtech::IOPort::PORT_3, static_cast<uint8_t>(1)>();          ///< Signal R/W. Selects read or write. False = write; True = read
  //const Microtech::OutputHandle enableReadWrite = Microtech::GPIOs::getOutputHandle<Microtech::IOPort::PORT_3, static_cast<uint8_t>(2)>();     ///< Signal E. Enables/disables data read or write


 // Microtech::IoBus<Microtech::IOPort::PORT_2, 0x0F> dataBus; ///< Databus that connects the display to the microcontroller

  static uint8_t displayControl;           ///< Attribute that holds the current "state" of the display control.
  static uint8_t functionSet;              ///< Attribute that holds the current "state" of the function set.
};

template <typename RS,typename RW, typename E, typename BUS>
uint8_t LCD_T<RS, RW, E, BUS>::displayControl;

template <typename RS,typename RW, typename E, typename BUS>
uint8_t LCD_T<RS, RW, E, BUS>::functionSet;

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
