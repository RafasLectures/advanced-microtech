/***************************************************************************//**
 * @file    LCD.c
 * @author  <your name>
 * @date    <date of creation>
 *
 * @brief   <brief description>
 *
 * Here goes a detailed description if required.
 ******************************************************************************/

#include "libs/LCD.hpp"

#include <type_traits>
#include <limits>
#include <cstdio>
#include <cstdlib>

namespace AdvancedMicrotech {

/**
 * Helper class to define a field within a frame. It provides with functionality to encode and decode the field within
 * a frame.
 *
 * @tparam FIELD_TYPE The type of the field. Can be any integral type. The decode will return this type, and the encode
 * will take it as an input parameter.
 * @tparam FRAME_TYPE The type of the frame. Can be any integral type. If it is a 16bit, 8bit value and so on.
 * @tparam mask The mask of where the bits with the information is within the frame.
 * @tparam position Is the right most position. The number of shifts to the right when decoding, and number of left
 *                  shifts when encoding.
 */
template<typename FIELD_TYPE, typename FRAME_TYPE, FRAME_TYPE mask, FRAME_TYPE position>
class Field {
  static_assert(std::is_integral<FRAME_TYPE>::value, "Frame type is not integral");
  static_assert(std::numeric_limits<FRAME_TYPE>::digits > position, "Position shift bigger than value coming");

public:
  /**
   * Method to decode a field within a frame.
   * @param value The raw value of the frame
   * @return The decoded value.
   */
  static constexpr FIELD_TYPE decode(const FRAME_TYPE value) {
    return static_cast<FIELD_TYPE>((value & mask) >> position);
  }

  /**
   * Method to encode the field into the frame to be send.
   *
   * @param value [output] The frame will be populated with the encoded value.
   * @param toEncode Value to encode from the field type
   * @return void
   */
  static constexpr void encode(FRAME_TYPE& value, FIELD_TYPE toEncode) {
    const FRAME_TYPE shiftedValue = static_cast<FRAME_TYPE>(toEncode) << POSITION;
    const FRAME_TYPE setVal = MASK & shiftedValue;
    const FRAME_TYPE resetVal = MASK & ~shiftedValue;
    value |= setVal;
    value &= ~resetVal;
  }

  static constexpr FRAME_TYPE MASK = mask;
  static constexpr FRAME_TYPE POSITION = position;
};
/******************************************************************************
 * VARIABLES
 *****************************************************************************/

/**
 * Clears entire display and sets DDRAM address 0 in address counter.
 */
static constexpr uint8_t CLEAR_DISPLAY = 0x01;
///**
// * Sets DDRAM address 0 in address counter. Also returns display from being
// * shifted to original position. DDRAM contents remain unchanged.
// */
static constexpr uint8_t RETURN_HOME = 0x02;

/**
 * Class specifying fields of the EntryModeSet instruction.
 */
class EntryModeSet {
public:
    using RawType = uint8_t;

    enum class CursorMoveDirection {
        DECREMENT = 0,
        INCREMENT
    };

    static constexpr RawType BASE_VALUE = 0x04;

    static constexpr Field<bool, RawType, 0x01, 0> ACCOMPANIES_DISPLAY_SHIFT{};
    static constexpr Field<CursorMoveDirection, RawType, 0x02, 1> CURSOR_MOVE_DIRECTION{};

};

class DisplayControl {
public:
    using RawType = uint8_t;

    static constexpr RawType BASE_VALUE = 0x08;

    static constexpr Field<bool, RawType, 0x01, 0> BLINK_CURSOR{};
    static constexpr Field<bool, RawType, 0x02, 1> CURSOR_ON{};
    static constexpr Field<bool, RawType, 0x04, 2> DISPLAY_ON{};
};

class CursorOrDisplayShift {
public:
    using RawType = uint8_t;

    enum class ShiftSelection {
        CURSOR_MOVE = 0,
        DISPLAY_SHIFT,
    };

    enum class ShiftDirection {
        LEFT = 0,
        RIGHT,
    };
    static constexpr RawType BASE_VALUE = 0x10;

    static constexpr Field<ShiftDirection, RawType, 0x04, 2> SHIFT_DIRECTION{};
    static constexpr Field<ShiftSelection, RawType, 0x08, 3> SHIFT_SELECTION{};
};

class FunctionSet {
public:
    using RawType = uint8_t;

    enum class InterfaceDataLength : RawType {
        BITS_4 = 0,
        BITS_8,
    };

    enum class NumberOfLines : RawType{
        LINE_1 = 0,
        LINES_2
    };

    enum class CharachterFont : RawType{
        DOTS_5X8 = 0,
        DOTS_5X10 = 1,
    };

    static constexpr RawType BASE_VALUE = 0x20;

    static constexpr Field<CharachterFont, RawType, 0x04, 2> CHARACHTER_FONT{};
    static constexpr Field<NumberOfLines, RawType, 0x08, 3> NUMBER_OF_LINES{};
    static constexpr Field<InterfaceDataLength, RawType, 0x10, 4> INTERFACE_DATA_LENGTH{};
};

void LCD::initialize() noexcept{
    displayControl = DisplayControl::BASE_VALUE;
    functionSet = FunctionSet::BASE_VALUE;

    // Initialize IOs
    selectRegister.init();
    readWrite.init();
    enableReadWrite.init();
    dataBus.initialize();

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
    writeWordToBus(functionSet >> 4);
    sendInstruction(functionSet);
    clearDisplay();

}
void LCD::enable(const bool enable) noexcept {
    DisplayControl::DISPLAY_ON.encode(displayControl, enable);
    sendInstruction(displayControl);
}
void LCD::setCursorPosition(const uint8_t x, const uint8_t y) noexcept {
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

    waitUntilNotBusy();
    setRegister(RegisterSelection::INSTRUCTION);
    writeByteToBus(address);
}
void LCD::showCursor(const bool show) noexcept{
    DisplayControl::CURSOR_ON.encode(displayControl, show);
    sendInstruction(displayControl);
}
void LCD::blinkCursor(const bool blink) noexcept{
    DisplayControl::BLINK_CURSOR.encode(displayControl, blink);
    sendInstruction(displayControl);
}
void LCD::clearDisplay() noexcept {
    sendInstruction(CLEAR_DISPLAY);
}

void LCD::writeChar(const char character) noexcept {
    setRegister(RegisterSelection::DATA);
    writeByteToBus(character);
}
void LCD::writeString(const char *text) noexcept {
    while(*text != 0x00) {
        writeChar(*text);
        text++;
    }
}
void LCD::writeDigitOfNumber(const int16_t number, uint8_t digit) {
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
    // Remove the nth-digit.
//    return number % POW10[digit];
}
void LCD::writeNumber(int16_t number) noexcept {
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
    /*
    int16_t j = number;
    // If the number is between 10000 and 32767, print the 10000-
    // digit.
    if (j >= 10000) {
        writeChar(0x30 + number / 10000);
    }
    // Remove the 10000-digit.
    number = number % 10000;
    // Print the 1000-digit, if the number bigger then 999.
    if (j >= 1000)
    {
        writeChar(0x30 + number / 1000);
    }
    // Now remove the 1000-digit.
    number = number % 1000;
    // Print the 100-digit if the number is big enough ...
    if (j >= 100)
    {
        writeChar(0x30 + number / 100);
    }
    // ... remove it ...
    number = number % 100;
    // ... same for 10-digit ...
    if (j >= 10)
    {
        writeChar(0x30 + number / 10);
    }
    // ...
    number = number % 10;
    // Print the last digit, no matter how big the number is (so if the
    // number is 0, we'll just print that).
    writeChar(0x30 + number / 1);
    */
}

void LCD::sendInstruction(const uint8_t instruction) noexcept {
    waitUntilNotBusy();
    setRegister(RegisterSelection::INSTRUCTION);
    writeByteToBus(instruction);
}

void LCD::writeWordToBus(const uint8_t dataToWrite) noexcept {
   enableOperation(false);  // Disable write
    setOperation(DataOperation::WRITE);                 // Set to write mode
    dataBus.write(dataToWrite);                         // Put data to bus
    enableOperation(true); // Enable write

    //__delay_cycles(100);
    enableOperation(false);  // Disable write
}

void LCD::writeByteToBus(const uint8_t dataToWrite) noexcept {
    const uint8_t dataMsb = (dataToWrite) >> 4;
    const uint8_t dataLsb = (0x0F & dataToWrite);
    writeWordToBus(dataMsb);
    writeWordToBus(dataLsb);
}


void LCD::waitUntilNotBusy() {
    static constexpr uint8_t BUSY_MASK = 0x08;      // The mask to decode the busy bit
    setRegister(RegisterSelection::INSTRUCTION);
    setOperation(DataOperation::READ);              // Set to read mode
    dataBus.read();                                 // Perform a read, but main goal is to set the data bus as input.
    enableOperation(true);                          // Enable read
    //__delay_cycles(20);
    uint8_t dataVal = dataBus.read();               // Read value from Bus
    while(dataVal & BUSY_MASK) {
        __no_operation();
        dataVal = dataBus.read();                   // Read value from Bus
    }

    uint8_t addrCounter = (0xE & dataVal) << 4;
    enableOperation(false);      // Disable read
    //__delay_cycles(100);
    enableOperation(true);     // Enable read
    //__delay_cycles(100);
    addrCounter |= dataBus.read();                          // Get value from Bus
    enableOperation(false);      // Disable read

}

}
