/*******************************************************************************
 * @file    LcdCommands.hpp
 * @author  Rafael Andrioli Bauer
 * @date    10.05.2023
 *
 * @brief   Header file with the commands accepted by the Lcd
 ******************************************************************************/

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
   * Method to encode the field into the frame.
   *
   * @param value  The frame will be populated with the encoded value.
   * @param toEncode Value to encode from the field type
   * @return the input value with the encoded information
   */
  static constexpr FRAME_TYPE encode(FRAME_TYPE value, FIELD_TYPE toEncode) {
    const FRAME_TYPE shiftedValue = static_cast<FRAME_TYPE>(toEncode) << position;
    value &= ~mask;
    value |= shiftedValue;
    return value;
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
 * Class specifying fields of the EntryModeSet frame specified in the datasheet.
 */
class EntryModeSet {
public:
    using RawType = uint8_t;

    /**
     * Specification of different cursor move directions
     */
    enum class CursorMoveDirection {
        DECREMENT = 0,
        INCREMENT
    };

    /**
     * Specifies the base value to perform this instruction as defined in the datasheet
     */
    static constexpr RawType BASE_VALUE = 0x04;

    static constexpr Field<bool, RawType, 0x01, 0> DISPLAY_SHIFT{};    ///< To specify display shift.
    /**
     * Specifies the cursor move direction. If it should increment or decrement 1
     */
    static constexpr Field<CursorMoveDirection, RawType, 0x02, 1> CURSOR_MOVE_DIRECTION{};

};

/**
 * Class specifying fields of the DisplayControl frame specified in the datasheet.
 */
class DisplayControl {
public:
    using RawType = uint8_t;

    /**
     * Specifies the base value to perform this instruction as defined in the datasheet
     */
    static constexpr RawType BASE_VALUE = 0x08;

    static constexpr Field<bool, RawType, 0x01, 0> BLINK_CURSOR{}; ///< Field to make cursor blink
    static constexpr Field<bool, RawType, 0x02, 1> CURSOR_ON{}; ///< Field to show cursor
    static constexpr Field<bool, RawType, 0x04, 2> DISPLAY_ON{}; ///< Field to turn on the display
};

class CursorOrDisplayShift {
public:
    using RawType = uint8_t;

    /**
     * Type to define the different shift selection options
     */
    enum class ShiftSelection {
        CURSOR_MOVE = 0,
        DISPLAY_SHIFT,
    };

    /**
     * Type to define the different shift direction options
     */
    enum class ShiftDirection {
        LEFT = 0,
        RIGHT,
    };

    /**
     * Specifies the base value to perform this instruction as defined in the datasheet
     */
    static constexpr RawType BASE_VALUE = 0x10;

    /**
     * Selects the shift to the right or left
     */
    static constexpr Field<ShiftDirection, RawType, 0x04, 2> SHIFT_DIRECTION{};
    /**
     * Selects if cursor or display must shift
     */
    static constexpr Field<ShiftSelection, RawType, 0x08, 3> SHIFT_SELECTION{};
};

/**
 * Class specifying fields of the FunctionSet frame specified in the datasheet.
 */
class FunctionSet {
public:
    using RawType = uint8_t;

    /**
     * Type to define the different interface data length options
     */
    enum class InterfaceDataLength : RawType {
        BITS_4 = 0,
        BITS_8,
    };

    /**
     * Type to define the different number of line options
     */
    enum class NumberOfLines : RawType{
        LINE_1 = 0,
        LINES_2
    };

    /**
     * Type to define the different character font options
     */
    enum class CharachterFont : RawType{
        DOTS_5X8 = 0,
        DOTS_5X10 = 1,
    };

    /**
     * Specifies the base value to perform this instruction as defined in the datasheet
     */
    static constexpr RawType BASE_VALUE = 0x20;

    static constexpr Field<CharachterFont, RawType, 0x04, 2> CHARACHTER_FONT{}; ///< Field to set the character font
    static constexpr Field<NumberOfLines, RawType, 0x08, 3> NUMBER_OF_LINES{}; ///< Field to set number of lines
    static constexpr Field<InterfaceDataLength, RawType, 0x10, 4> INTERFACE_DATA_LENGTH{}; ///< Field to set interface data length
};

}

#endif /* LIBS_LCDCOMMANDS_HPP_ */
