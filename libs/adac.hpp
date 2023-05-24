/******************************************************************************
 * @file                    adac.hpp
 * @author                  Rafael Andrioli Bauer
 * @date                    14.05.2023
 * @matriculation number    5163344
 * @e-mail contact          abauer.rafael@gmail.com
 * @brief                   <brief description>
 *
 * Here goes a detailed description if required.
 ******************************************************************************/

#ifndef EXERCISE_LIBS_ADAC_H_
#define EXERCISE_LIBS_ADAC_H_


/******************************************************************************
 * INCLUDES
 *****************************************************************************/

#include "i2c.hpp"
#include "common/fields.hpp"

#include <cstdint>
/******************************************************************************
 * CONSTANTS
 *****************************************************************************/



/******************************************************************************
 * VARIABLES
 *****************************************************************************/



/******************************************************************************
 * FUNCTION PROTOTYPES
 *****************************************************************************/
namespace AdvancedMicrotech {

/**
 * Class specifying fields of the ControlByte frame specified in the datasheet of the ADC/DAC PCF8591.
 */
class Pcf8591ControlByte {
public:
    using RawType = uint8_t;

    /**
     * Specification of different cursor move directions
     */
    enum class AdChannel : RawType{
        CHANNEL_0 = 0,
        CHANNEL_1,
        CHANNEL_2,
        CHANNEL_3,
    };

    enum class InputProgramming : RawType{
        ALL_SINGLE_ENDED_INPUTS = 0,
        ALL_DIFFERENTIAL_INPUTS,
        SINGLE_ENDED_AND_DIFFERENTIAL_INPUTS,
        TWO_DIFFERENTIAL_INPUTS,
    };

    static constexpr Field<AdChannel, RawType, 0x03, 0> AD_CHANNEL{};    ///< To specify which ad channel number to read

    static constexpr Field<bool, RawType, 0x04, 2> AUTO_INCREMENT{};
    static constexpr Field<InputProgramming, RawType, 0x30, 4> ANALOG_INPUT_PROGRAMMING{};
    static constexpr Field<bool, RawType, 0x40, 6> ANALOG_OUTPUT_ENABLED{};
};

template<typename I2C>
class ADC_DAC_T {
public:
    static constexpr uint8_t I2C_ADDRESS = 0x48;
    static constexpr uint8_t NUMBER_AD_CHANNELS = 4;

    static bool initialize() {
        I2C::initialize(I2C_ADDRESS);
        controlByte = Pcf8591ControlByte::AD_CHANNEL.encode(controlByte, Pcf8591ControlByte::AdChannel::CHANNEL_0);
        controlByte = Pcf8591ControlByte::AUTO_INCREMENT.encode(controlByte, true);
        controlByte = Pcf8591ControlByte::ANALOG_INPUT_PROGRAMMING.encode(controlByte, Pcf8591ControlByte::InputProgramming::ALL_SINGLE_ENDED_INPUTS);
        controlByte = Pcf8591ControlByte::ANALOG_OUTPUT_ENABLED.encode(controlByte, true);

        return I2C::write(sizeof(controlByte), &controlByte, true);
    }

    static bool read(uint8_t* values) {
        const bool retVal = I2C::write(sizeof(controlByte), &controlByte, true);
        I2C::read(NUMBER_AD_CHANNELS, values);
        return retVal;
    }

    static bool write(const uint8_t value) {
      uint8_t data[] = {controlByte, value};
      return I2C::write(sizeof(data), data, true);
    }

private:
    static Pcf8591ControlByte::RawType controlByte;
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
}
#endif /* EXERCISE_LIBS_ADAC_H_ */
