#ifndef ADVANVED_MICROTECH_MMAREGISTERS_HPP
#define ADVANVED_MICROTECH_MMAREGISTERS_HPP

#include "common/fields.hpp"

namespace AdvancedMicrotech {
namespace MMA {

template<typename I2C, uint8_t NEW_REG_ADDRESS, bool NEW_CAN_WRITE, typename newRawType>
class RegisterBase {
public:
  using RawType = newRawType;  // Raw type of the register
  static constexpr bool CAN_WRITE = NEW_CAN_WRITE;
  static constexpr uint8_t REG_ADDRESS = NEW_REG_ADDRESS;

  static RawType readRegister() {
    uint8_t regAddress = NEW_REG_ADDRESS;
    I2C::write(1, &regAddress, false);
    I2C::read(sizeof(currentVal), &currentVal);
    return currentVal;
  }

  static bool writeCacheToRegister() {
    static constexpr uint8_t WRITE_BUFFER_SIZE = 2;
    uint8_t regAddress = NEW_REG_ADDRESS;
    RawType writeBuffer[WRITE_BUFFER_SIZE] = {regAddress, currentVal};
    return I2C::write(WRITE_BUFFER_SIZE, writeBuffer, true);
  }

  template<typename FIELD>
  static constexpr void encodeField(FIELD /*field*/, typename FIELD::FieldType toEncode) {
    currentVal = FIELD::encode(currentVal, toEncode);
  }

  template<typename FIELD>
  static constexpr typename FIELD::FieldType getFieldValue(FIELD /*field*/) {
    return FIELD::decode(currentVal);
  }
  static RawType getCachedValue() {
    return currentVal;
  }

private:
  static RawType currentVal;
};

/**
 * System Control 1 Register
 * @tparam I2C
 * @tparam RawType
 */
template<typename I2C, typename RawType = uint8_t>
class CTRL_REG1 : public RegisterBase<I2C, 0x2A, true, RawType> {
public:
  enum class Resolution : RawType {
    BITS_14 = 0,
    BITS_8,
  };

  enum class AutoWakeSampleFrequency : RawType {
    FREQ_50HZ,
    FREQ_12_5HZ,
    FREQ_6_25HZ,
    FREQ_1_56HZ,
  };

  enum class DataRateSelection : RawType {
    FREQ_800HZ,
    FREQ_400HZ,
    FREQ_200HZ,
    FREQ_100HZ,
    FREQ_50HZ,
    FREQ_12_5HZ,
    FREQ_6_25HZ,
    FREQ_1_56HZ,
  };

  static constexpr Field<bool, RawType, 0x01, 0> ACTIVE{};

  static constexpr Field<Resolution, RawType, 0x02, 1> RESOLUTION{};

  static constexpr Field<bool, RawType, 0x04, 2> REDUCED_NOISE{};

  /**
   * Selects the Output Data Rate (ODR) for acceleration samples. The default value is 800 Hz.
   */
  static constexpr Field<DataRateSelection, RawType, 0x38, 3> DATA_RATE{};

  /**
   * Configures the Auto-WAKE sample frequency when the device is in SLEEP Mode. Default value: 00.
   */
  static constexpr Field<AutoWakeSampleFrequency, RawType, 0xC0, 6> ALSP_RATE{};
};

template<typename I2C, typename RawType = uint8_t>
class CTRL_REG2 : public RegisterBase<I2C, 0x2B, true, RawType> {
public:
  enum class PowerSelection : RawType {
    NORMAL = 0,
    LOW_NOISE_LOW_POWER,
    HIGH_RESOLUTION,
    LOW_POWER,
  };

  static constexpr Field<PowerSelection, RawType, 0x03, 0> ACTIVE_POWER_SELECTION{};

  static constexpr Field<bool, RawType, 0x04, 2> AUTO_SLEEP{};

  static constexpr Field<PowerSelection, RawType, 0x18, 3> SLEEP_POWER_SELECTION{};

  static constexpr Field<bool, RawType, 0x40, 6> SOFTWARE_RESET{};

  static constexpr Field<bool, RawType, 0x80, 7> SELF_TEST_ON{};
};

/**
 * The XYZ_DATA_CFG register sets the dynamic range and sets the high pass filter for the output data. When the HIGH_PASS_FILTER_OUT bit is set, both the FIFO and DATA registers will contain high pass filtered data
 * @tparam I2C
 * @tparam RawType
 */
template<typename I2C, typename RawType = uint8_t>
class XYZ_DATA_CFG : public RegisterBase<I2C, 0x0E, true, RawType> {
public:
  /**
   *
   */
  enum class Range : RawType { RANGE_2G = 0, RANGE_4G, RANGE_8G };

  static constexpr Field<Range, RawType, 0x03, 0> RANGE{};
  static constexpr Field<bool, RawType, 0x10, 4> HIGH_PASS_FILTER_OUT{};
};

template<typename I2C>
class WHO_AM_I_DEVICE_ID : public RegisterBase<I2C, 0x0D, false, uint8_t> {
public:
};

template<typename I2C, uint8_t NEW_REG_ADDRESS, bool NEW_CAN_WRITE, typename newRawType>
newRawType RegisterBase<I2C, NEW_REG_ADDRESS, NEW_CAN_WRITE, newRawType>::currentVal=0;

}
}

#endif  // ADVANVED_MICROTECH_MMAREGISTERS_HPP
