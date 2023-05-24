/*******************************************************************************
 * @file    LcdCommands.hpp
 * @author  Rafael Andrioli Bauer
 * @date    10.05.2023
 *
 * @brief   Header file with the commands accepted by the Lcd
 ******************************************************************************/

#ifndef LIBS_LCDCOMMANDS_HPP_
#define LIBS_LCDCOMMANDS_HPP_

#include "common/fields.hpp"

namespace AdvancedMicrotech {

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
  enum class CursorMoveDirection { DECREMENT = 0, INCREMENT };

  /**
   * Specifies the base value to perform this instruction as defined in the datasheet
   */
  static constexpr RawType BASE_VALUE = 0x04;

  static constexpr Field<bool, RawType, 0x01, 0> DISPLAY_SHIFT{};  ///< To specify display shift.
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

  static constexpr Field<bool, RawType, 0x01, 0> BLINK_CURSOR{};  ///< Field to make cursor blink
  static constexpr Field<bool, RawType, 0x02, 1> CURSOR_ON{};     ///< Field to show cursor
  static constexpr Field<bool, RawType, 0x04, 2> DISPLAY_ON{};    ///< Field to turn on the display
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
  enum class NumberOfLines : RawType { LINE_1 = 0, LINES_2 };

  /**
   * Type to define the different character font options
   */
  enum class CharachterFont : RawType {
    DOTS_5X8 = 0,
    DOTS_5X10 = 1,
  };

  /**
   * Specifies the base value to perform this instruction as defined in the datasheet
   */
  static constexpr RawType BASE_VALUE = 0x20;

  static constexpr Field<CharachterFont, RawType, 0x04, 2> CHARACHTER_FONT{};  ///< Field to set the character font
  static constexpr Field<NumberOfLines, RawType, 0x08, 3> NUMBER_OF_LINES{};   ///< Field to set number of lines
  static constexpr Field<InterfaceDataLength, RawType, 0x10, 4>
    INTERFACE_DATA_LENGTH{};                                                   ///< Field to set interface data length
};

}  // namespace AdvancedMicrotech

#endif /* LIBS_LCDCOMMANDS_HPP_ */
