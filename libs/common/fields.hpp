/******************************************************************************
 * @file                    fields.hpp
 * @author                  Rafael Andrioli Bauer
 * @date                    23.05.2023
 * @matriculation number    5163344
 * @e-mail contact          abauer.rafael@gmail.com
 *
 * @brief   Header contains a helper class to encode and decode fields within a frame
 *
 ******************************************************************************/

#ifndef LIBS_COMMON_FIELDS_HPP_
#define LIBS_COMMON_FIELDS_HPP_

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
}


#endif /* LIBS_COMMON_FIELDS_HPP_ */
