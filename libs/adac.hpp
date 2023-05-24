/******************************************************************************
 * @file                    adac.hpp
 * @author                  Rafael Andrioli Bauer
 * @date                    22.05.2023
 * @matriculation number    5163344
 * @e-mail contact          abauer.rafael@gmail.com
 * @brief                   Header file with the ADC/DAC class
 *
 * Its implementation must be in the header because it is templated.
 ******************************************************************************/

#ifndef EXERCISE_LIBS_ADAC_H_
#define EXERCISE_LIBS_ADAC_H_

#include "common/fields.hpp"

#include <cstdint>

namespace AdvancedMicrotech {

/**
 * Class specifying fields of the ControlByte frame specified in the datasheet of the ADC/DAC PCF8591.
 */
class Pcf8591ControlByte {
public:
  using RawType = uint8_t;  // Raw type of the control byte

  /**
   * Specification of the available channels in the ADC
   */
  enum class AdChannel : RawType {
    CHANNEL_0 = 0,  ///< Channel 0
    CHANNEL_1,      ///< Channel 1
    CHANNEL_2,      ///< Channel 2
    CHANNEL_3,      ///< Channel 3
  };

  /**
   * Different types of inputs
   */
  enum class InputProgramming : RawType {
    ALL_SINGLE_ENDED_INPUTS = 0,           ///< Every channel is single ended.
    ALL_DIFFERENTIAL_INPUTS,               ///< Every channel is a differential input compared to channel 3
    SINGLE_ENDED_AND_DIFFERENTIAL_INPUTS,  ///<  Channels 0 and 1 are single ended; channels 2 and 3 are differential.
    TWO_DIFFERENTIAL_INPUTS,               ///< Channel 0 and 1 are differential; channels 2 and 3 are differential.
  };

  static constexpr Field<AdChannel, RawType, 0x03, 0> AD_CHANNEL{};  ///< To specify which ad channel number to read

  static constexpr Field<bool, RawType, 0x04, 2> AUTO_INCREMENT{};
  static constexpr Field<InputProgramming, RawType, 0x30, 4> ANALOG_INPUT_PROGRAMMING{};
  static constexpr Field<bool, RawType, 0x40, 6> ANALOG_OUTPUT_ENABLED{};
};

/**
 * Class implements an abstraction for the ADC/DAC PCF8591. It provides methods to read the ADC channels
 * and set the DAC value.
 * to make easier to use this peripheral
 * @tparam I2C The I2C interface.
 */
template<typename I2C>
class ADC_DAC_T {
public:
  static constexpr uint8_t I2C_ADDRESS = 0x48;      ///< I2C address of ADC/DAC device
  static constexpr uint8_t NUMBER_AD_CHANNELS = 4;  ///< Number of channels (later could be a template argument)

  /**
   * Initialize the ADC / DAC
   * @return
   */
  static bool initialize() {
    I2C::initialize(I2C_ADDRESS);
    controlByte = Pcf8591ControlByte::AD_CHANNEL.encode(controlByte, Pcf8591ControlByte::AdChannel::CHANNEL_0);
    controlByte = Pcf8591ControlByte::AUTO_INCREMENT.encode(controlByte, true);
    controlByte = Pcf8591ControlByte::ANALOG_INPUT_PROGRAMMING.encode(
      controlByte, Pcf8591ControlByte::InputProgramming::ALL_SINGLE_ENDED_INPUTS);
    controlByte = Pcf8591ControlByte::ANALOG_OUTPUT_ENABLED.encode(controlByte, true);

    return I2C::write(sizeof(controlByte), &controlByte, true);
  }

  /**
   * Read all ADC-values and write it into the passed values-array.
   *
   * @param values Pointer to where the ADC values will be written.
   * @warning always pass an array of size four (at least)
   * @return If the read was successful.
   *
   * (1 pt.)
   */
  static bool read(uint8_t* values) {
    I2C::read(NUMBER_AD_CHANNELS, values);
    return true;
  }

  /**
   * Write a certain value to the DAC.
   * @param value Value to be written in the DAC
   * @return If the write was successful (there was no NACK)
   * (1 pt.)
   */
  static bool write(const uint8_t value) {
    // According to the PCF8591, we should write the control byte, followed by the value we want.
    uint8_t data[] = {controlByte, value};
    return I2C::write(sizeof(data), data, true);
  }

private:
  static Pcf8591ControlByte::RawType controlByte;  ///< Variable to hold the control byte value
};

template<typename I2C>
Pcf8591ControlByte::RawType ADC_DAC_T<I2C>::controlByte;

#if 0
// All functions return 0 if everything went fine
// and anything but 0 if not.

// Initialize the ADC / DAC
unsigned char adac_init(void);

// Read all ADC-values and write it into the passed values-array.
// (Important: always pass an array of size four (at least).) (1 pt.)
unsigned char adac_read(unsigned char * values);

// Write a certain value to the DAC. (1 pt.)
unsigned char adac_write(unsigned char value);
#endif
}  // namespace AdvancedMicrotech
#endif /* EXERCISE_LIBS_ADAC_H_ */
