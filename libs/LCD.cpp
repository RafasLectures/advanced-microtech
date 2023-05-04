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
    const FRAME_TYPE shiftedValue = static_cast<FRAME_TYPE>(toEncode) << position;
    value &= shiftedValue;
    value |= shiftedValue;
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
//static constexpr uint8_t CLEAR_DISPLAY = 0x01;
///**
// * Sets DDRAM address 0 in address counter. Also returns display from being
// * shifted to original position. DDRAM contents remain unchanged.
// */
//static constexpr uint8_t RETURN_HOME = 0x02;

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

    static constexpr Field<bool, RawType, 0x01, 0> BLINK_DISPLAY{};
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

    static constexpr Field<ShiftDirection, RawType, 0x04, 3> SHIFT_DIRECTION{};
    static constexpr Field<ShiftSelection, RawType, 0x08, 4> SHIFT_SELECTION{};
};

class FunctionSet {
public:
    using RawType = uint8_t;

    enum class InterfaceDataLength {
        BITS_4 = 0,
        BITS_8,
    };

    enum class NumberOfLines {
        LINES_2 = 0,
        LINE_1,
    };

    enum class CharachterFont {
        DOTS_5X8 = 0,
        DOTS_5X10,
    };

    static constexpr Field<CharachterFont, RawType, 0x04, 3> CHARACHTER_FONT{};
    static constexpr Field<NumberOfLines, RawType, 0x08, 4> NUMBER_OF_LINES{};
    static constexpr Field<InterfaceDataLength, RawType, 0x10, 5> INTERFACE_DATA_LENGTH{};

    static constexpr RawType encode(const InterfaceDataLength dataLength, const NumberOfLines numLines, const CharachterFont charFont) {
        RawType retVal = BASE_VALUE;
        CHARACHTER_FONT.encode(retVal, charFont);
        NUMBER_OF_LINES.encode(retVal, numLines);
        INTERFACE_DATA_LENGTH.encode(retVal, dataLength);
        return retVal;
    }
private:
    static constexpr RawType BASE_VALUE = 0x10;
};
/******************************************************************************
 * LOCAL FUNCTION PROTOTYPES
 *****************************************************************************/

/******************************************************************************
 * LOCAL FUNCTION IMPLEMENTATION
 *****************************************************************************/

/******************************************************************************
 * FUNCTION IMPLEMENTATION
 *****************************************************************************/

void LCD::initialize() noexcept{
    selectRegister.init();
    readWrite.init();
    enableReadWrite.init();

    const FunctionSet::RawType functionSet = FunctionSet::encode(FunctionSet::InterfaceDataLength::BITS_4,
                                                                 FunctionSet::NumberOfLines::LINES_2,
                                                                 FunctionSet::CharachterFont::DOTS_5X10);
    setRegister(RegisterSelection::INSTRUCTION);
    setOperation(DataOperation::READ);
    // wait until we can write an instruction to the
    while(isBusy()) {
        __no_operation();
    }
    // According to datasheet, we fist need to write the MSB once
    // and then write the whole thing again, so we set it first
    // to 4-bit operation.
    writeWordToBus(functionSet >> 4);
    sendInstruction(functionSet);
}
void LCD::enable(const bool enable) noexcept {

}
void LCD::setCursorPosition(const uint8_t x, const uint8_t y) noexcept {

}
void LCD::showCursor(const bool show) noexcept{

}
void LCD::blinkCursor(const bool blink) noexcept{

}
void LCD::clearDisplay() noexcept {

}
void LCD::writeChar(const char character) noexcept {

}
void LCD::writeString(const char *text) noexcept {

}
void LCD::writeNumber(const int16_t number) noexcept {

}

void LCD::sendInstruction(const uint8_t instruction) noexcept {
    setRegister(RegisterSelection::INSTRUCTION);
    setOperation(DataOperation::READ);
    // wait until we can write an instruction to the
    while(isBusy()) {
        __no_operation();
    }
    writeByteToBus(instruction);
}

void LCD::writeWordToBus(const uint8_t dataToWrite) noexcept {
    enableReadWrite.setState(Microtech::IOState::LOW);  // Disable write
    setOperation(DataOperation::WRITE);                 // Set to write mode
    dataBus.write(dataToWrite);                         // Put data to bus
    enableReadWrite.setState(Microtech::IOState::HIGH); // Enable write

   // probably need to put some delay here.
    enableReadWrite.setState(Microtech::IOState::LOW);  // Disable write
}

void LCD::writeByteToBus(const uint8_t dataToWrite) noexcept {
    const uint8_t dataMsb = (dataToWrite) >> 4;
    const uint8_t dataLsb = (0x0F & dataToWrite);
    writeWordToBus(dataMsb);
    writeWordToBus(dataLsb);
}
bool LCD::isBusy() noexcept {
    static constexpr uint8_t BUSY_MASK = 0x80;
    setOperation(DataOperation::READ);                      // Set to read mode
    enableReadWrite.setState(Microtech::IOState::HIGH);     // Enable read
    // probably need to put a delay here.
    const uint8_t dataVal = dataBus.read();                 // Get value from Bus
    enableReadWrite.setState(Microtech::IOState::LOW);      // Disable read
    return dataVal & BUSY_MASK;
}
}
