/*
 * LcdCommands.hpp
 *
 *  Created on: 10.05.2023
 *      Author: Andrio01
 */

#ifndef LIBS_LCDCOMMANDS_HPP_
#define LIBS_LCDCOMMANDS_HPP_

#include <cstdint>
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
    const FRAME_TYPE shiftedValue = static_cast<FRAME_TYPE>(toEncode) << POSITION;
    const FRAME_TYPE setVal = MASK & shiftedValue;
    const FRAME_TYPE resetVal = MASK & ~shiftedValue;
    value |= setVal;
    value &= ~resetVal;
  }

  static constexpr FRAME_TYPE MASK = mask;
  static constexpr FRAME_TYPE POSITION = position;
};



/**
 * Clears entire display and sets DDRAM address 0 in address counter.
 */
static constexpr uint8_t CLEAR_DISPLAY = 0x01;

/**
 * Sets DDRAM address 0 in address counter. Also returns display from being
 * shifted to original position. DDRAM contents remain unchanged.
 */
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

}

#endif /* LIBS_LCDCOMMANDS_HPP_ */
