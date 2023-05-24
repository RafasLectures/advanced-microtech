/*******************************************************************************
 * @file                    LCD.hpp
 * @author                  Rafael Andrioli Bauer
 * @date                    10.05.2023
 * @matriculation number    5163344
 * @e-mail contact          abauer.rafael@gmail.com
 *
 * @brief   Header file with the prototypes of the LCD C functions as well as the
 *          declaration of the LCD class.
 *          Its implementation must be in the header because it is templated.
 ******************************************************************************/

#ifndef LIBS_LCD_H_
#define LIBS_LCD_H_

#include "LcdCommands.hpp"
#include "libs/common/clocks.hpp"
#include <cstdint>

namespace AdvancedMicrotech {

/**
 * Class that implements the abstraction for a LCD. It provides methods to make it easier to manipulate the LCD.
 *
 * @tparam RS Display RS signal. Signal used to select either the instruction register or the data register of the
 * display. 0 = instruction; 1 = data
 * @tparam RW Display R/W signal. Selects read or write. 0 = write; 1 = read
 * @tparam E Display E signal. Enables/disables data read or write;
 * @tparam BUS Display data bus.
 */
template<typename RS, typename RW, typename E, typename BUS>
class LCD_T {
public:
  /**
   * Initialization of the LCD; set all pin directions, basic setup of the LCD, etc.
   */
  static constexpr void initialize() {
    // Initializes the static attributes that hold the current value of the "DisplayControl" and "FunctionSet" frames
    // defined in the displays' documentation.
    displayControl = DisplayControl::BASE_VALUE;
    functionSet = FunctionSet::BASE_VALUE;

    // Initialize IOs
    RS::init();
    E::init();
    RW::init();
    BUS::init();

    // Makes sure that the LCD is in a known state for its initialization. If the LCD was already ON, and it had been
    // set to 4-bits interface, that is different from when it powers on. By default, when the LCD powers on it is 8-bit
    // interface, therefore, we first set it to an 8-bit interface to make sure we are in a known "state" for the LCD.
    setRegister(RegisterSelection::INSTRUCTION);
    functionSet = FunctionSet::INTERFACE_DATA_LENGTH.encode(functionSet, FunctionSet::InterfaceDataLength::BITS_8);
    sendInstruction(functionSet);

    // Create the function set frame to how we want to work:
    //  - Interface of 4 bits
    //  - 2 Lines
    //  - Character size 5x8 dots
    functionSet = FunctionSet::INTERFACE_DATA_LENGTH.encode(functionSet, FunctionSet::InterfaceDataLength::BITS_4);
    functionSet = FunctionSet::NUMBER_OF_LINES.encode(functionSet, FunctionSet::NumberOfLines::LINES_2);
    functionSet = FunctionSet::CHARACHTER_FONT.encode(functionSet, FunctionSet::CharachterFont::DOTS_5X8);

    // Sets the data interface to 4 bits.
    // Since initially we are as 8-bit interface, according to datasheet,
    // we first need to write the MSB of the functionSet frame once
    // and then write the whole frame again, so we set it first
    // to 4-bit operation.
    setRegister(RegisterSelection::INSTRUCTION);
    writeWordToBus(functionSet >> 4);
    sendInstruction(functionSet);
  }
  /**
   * Enable (1) or disable (0) the display (i.e. hide all text)
   */
  static void enable(const bool enable) noexcept {
    // encodes the "enable" information in the "DisplayControl" frame, defined in the documentation
    displayControl = DisplayControl::DISPLAY_ON.encode(displayControl, enable);
    sendInstruction(displayControl);
  }

  /**
   * Set the cursor to a certain x/y-position
   * (x,y) -> (0,0) is top left and (15, 1) is bottom right.
   */
  static void setCursorPosition(const uint8_t x, const uint8_t y) noexcept {
    static constexpr uint8_t SET_ADDRESS_BASE = 0x80;  // Base value to perform "Set DDRAM address" instruction.
    uint8_t address = SET_ADDRESS_BASE;                // Create variable that will hold the instruction + address;
    if (y >= 1) {  // Make sure that the Y (vertical index) is not bigger than 1, since we have only 2 lines.
      address |= 0x40;
    }
    if (x > 0x0F) {  // Make sure that the X (horizontal index) is not bigger than 15, since we have only 16 columns.
      address |= 0x0F;
    } else {
      address |= x;
    }

    sendInstruction(address);
  }

  /**
   *  Show or hide the cursor
   *  @param show boolean to choose if the cursor should be shown. True to show, false to hide.
   */
  static void showCursor(const uint8_t show) noexcept {
    // encodes the "cursor on" information in the "DisplayControl" frame, defined in the documentation
    displayControl = DisplayControl::CURSOR_ON.encode(displayControl, show);
    sendInstruction(displayControl);
  }

  /**
   * Method to blink or not the cursor of the LCD.
   * @param blink boolean to choose if the cursor should blink. True to blink, false not to.
   */
  static void blinkCursor(const uint8_t blink) noexcept {
    // encodes the "blink cursor" information in the "DisplayControl" frame, defined in the documentation
    displayControl = DisplayControl::BLINK_CURSOR.encode(displayControl, blink);
    sendInstruction(displayControl);
  }

  /**
   * Delete everything on the LCD and puts the cursor in the position (0,0)
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
    // A C string (const char* string) is null terminated (ends with 0x00), so keep writing until we reach that point.
    while (*text != 0x00) {
      writeChar(*text);
      text++;
    }
  }

  /**
   * Show a given number at the cursor's current position.
   * @param number
   */
  static void writeNumber(int16_t number) noexcept {
    if (number == 0) {
      writeChar(0x30);
      return;
    }
    // In a signed number the most significant bit is 1. Since it is an int16, the mask checks only the last bit
    static constexpr int16_t SIGN_MASK = 0x8000;
    // Check if number is negative
    if (SIGN_MASK & number) {
      static constexpr uint8_t MINUS_IN_ASCII = 0x2D;
      writeChar(MINUS_IN_ASCII);

      // Flip every bit of it so it is like a normal number
      // without the sign and add 1 because -1 is 0xFFFF, so after flipping
      // it is 0x0000 + 1 = 0x0001;
      number = ~number;
      number++;
    }
    // Writes each digit in the screen starting from the 4th digit.
    int8_t i = 4;
    while (i >= 0) {
      writeDigitOfNumber(number, i);
      i--;
    }
  }

private:
  /**
   * Type to define the different types of registers are present.
   */
  enum class RegisterSelection { INSTRUCTION = 0, DATA };
  /**
   * Type to define the 2 operations that can be performed
   */
  enum class DataOperation { WRITE = 0, READ };

  /**
   * Method just to make it easier to read the code and be easier to understand
   * when we are setting the operation to read or write.
   * @param newOperation The desired operation to be performed.
   */
  static constexpr void setOperation(const DataOperation newOperation) noexcept {
    if (newOperation == DataOperation::READ) {
      RW::set_high();
    } else {
      RW::set_low();
    }
  }

  /**
   * Method just to make it easier to read the code and be easier to understand
   * when we are selecting which register.
   * @param newSelection The desired register selection
   */
  static constexpr void setRegister(const RegisterSelection newSelection) noexcept {
    if (newSelection == RegisterSelection::DATA) {
      RS::set_high();
    } else {
      RS::set_low();
    }
  }

  /**
   * Method just to make it easier to read the code and be easier to understand
   * when enabling the active operation from the last "setOperation" call.
   * @param enableOperation if it is to enable or not
   */
  static constexpr void enableOperation(const bool enableOperation) noexcept {
    E::set(enableOperation);
  }

  /**
   * Method that sends an instruction to the display. It is responsible of
   * performing all the steps to write the given instruction, such as
   * setting the register as the instruction register.
   * @param instruction the data to be written in the bus
   */
  static void sendInstruction(const uint8_t instruction) noexcept {
    setRegister(RegisterSelection::INSTRUCTION);
    writeByteToBus(instruction);
  }

  /**
   * Method to write word (4 bits) to the bus.
   * @param dataToWrite only the 4 LSB bits will be written
   */
  static void writeWordToBus(const uint8_t dataToWrite) noexcept {
    setOperation(DataOperation::WRITE);  // Set to write mode
    BUS::write(dataToWrite);             // Put data to bus
    enableOperation(true);               // Enable write
    enableOperation(false);              // Disable write
  }
  /**
   * Method to write a byte (8 bits) to the bus.
   * @param dataToWrite byte to be written
   */
  static void writeByteToBus(const uint8_t dataToWrite) noexcept {
    waitUntilNotBusy();  // Make sure the display is not busy, and we can write to it
    const uint8_t dataMsb = (dataToWrite) >> 4;
    const uint8_t dataLsb = (0x0F & dataToWrite);

    writeWordToBus(dataMsb);
    writeWordToBus(dataLsb);
  }
  /**
   * Method to wait for the display until it is not busy anymore.
   * It is a blocking call. So once called it will only return when the display
   * is not busy anymore.
   */
  static void waitUntilNotBusy() noexcept {
    static constexpr uint8_t BUSY_MASK = 0x08;  // The mask to decode the busy bit
    const bool currentSelection = RS::get();    // Stores current register selection, so we can reset it at the end
    setRegister(RegisterSelection::INSTRUCTION);
    setOperation(DataOperation::READ);          // Set to read mode
    BUS::read();                                // Perform a read, but main goal is to set the data bus as input.
    enableOperation(true);                      // Enable read
    delay_us(20);
    uint8_t dataVal = BUS::read();              // Read value from Bus
    while (dataVal & BUSY_MASK) {               // Keep reading it until we are not busy anymore
      dataVal = BUS::read();                    // Read value from Bus
    }

    // According to the datasheet, the current address counter is also reported together with the busy flag.
    // At the moment it is being read, but not used.
    uint8_t addrCounter = (0xE & dataVal) << 4;
    enableOperation(false);      // Disable read
    enableOperation(true);       // Enable read
    addrCounter |= BUS::read();  // Get value from Bus
    enableOperation(false);      // Disable read

    RS::set(currentSelection);   // Sets register selection as before call of this function.
  }

  /**
   * Function to write the nth-digit of a number
   * @param number number to get the digit from
   * @param digit the nth-digit of the number
   */
  static void writeDigitOfNumber(int16_t number, uint8_t digit) {
    // Table with predefined powers of 10 to have fewer computations.
    // There are 5 numbers, because in further steps we need to remove the
    // nth-digit + 1
    static constexpr int32_t POW10[] = {1, 10, 100, 1000, 10000, 100000};
    static constexpr int16_t BASE_NUM_ASCII = 0x30;  // Base hex of a number in ASCII
    if (digit > 4) {
      // Only support up to 4-digit numbers. Early return
      return;
    }

    // We first need to remove the digit higher to the digit we want to print.
    const int16_t originalNumber = number;
    const int16_t numberToProcess = number % POW10[digit + 1];
    if (originalNumber >= POW10[digit]) {
      const int16_t toBeWritten = numberToProcess / POW10[digit];
      writeChar(BASE_NUM_ASCII + toBeWritten);
    }
  }

  static volatile uint8_t displayControl;  ///< Attribute that holds the current "state" of the "DisplayControl" frame.
  static volatile uint8_t functionSet;     ///< Attribute that holds the current "state" of the "FunctionSet" frame.
};

// Needs to declare the static attributes so the linker allocates memory to it.
template<typename RS, typename RW, typename E, typename BUS>
volatile uint8_t LCD_T<RS, RW, E, BUS>::displayControl;

template<typename RS, typename RW, typename E, typename BUS>
volatile uint8_t LCD_T<RS, RW, E, BUS>::functionSet;

}  // namespace AdvancedMicrotech
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
