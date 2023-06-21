/******************************************************************************
 * @file                    mmaRegisters.hpp
 * @author                  Rafael Andrioli Bauer
 * @date                    16.06.2023
 * @matriculation number    5163344
 * @e-mail contact          abauer.rafael@gmail.com
 *
 * @brief   Header file with the declaration of the MMA Registers.
 ******************************************************************************/

#ifndef ADVANVED_MICROTECH_MMAREGISTERS_HPP
#define ADVANVED_MICROTECH_MMAREGISTERS_HPP

#include "common/fields.hpp"

namespace AdvancedMicrotech {
namespace MMA {

/**
 * The XYZ_DATA_CFG register sets the dynamic range and sets the high pass filter for the output data.
 * When the HIGH_PASS_FILTER_OUT bit is set, both the FIFO and DATA registers will contain high pass filtered data
 * @tparam I2C
 * @tparam RawType
 */
class XYZ_DATA_CFG {
public:
  using RawType = uint8_t;  // Raw type of the register
  static constexpr bool CAN_WRITE = true;
  static constexpr uint8_t ADDRESS = 0x0E;

  /**
   *
   */
  enum class Range : RawType { RANGE_2G = 0, RANGE_4G, RANGE_8G };

  static constexpr Field<Range, RawType, 0x03, 0> RANGE{};
  static constexpr Field<bool, RawType, 0x10, 4> HIGH_PASS_FILTER_OUT{};
};

class WHO_AM_I_DEVICE_ID {
public:
  using RawType = uint8_t;  // Raw type of the register
  static constexpr bool CAN_WRITE = false;
  static constexpr uint8_t ADDRESS = 0x0D;
};

/**
 * This register configures the event flag for the tap detection for enabling/disabling the detection of a
 * single and double pulse on each of the axes
 * @tparam I2C
 * @tparam RawType
 */
class PULSE_CFG {
public:
  using RawType = uint8_t;  // Raw type of the register
  static constexpr bool CAN_WRITE = true;
  static constexpr uint8_t ADDRESS = 0x21;

  static constexpr Field<bool, RawType, 0x01, 0> X_SINGLE_PULSE_ENABLED{};
  static constexpr Field<bool, RawType, 0x02, 1> X_DOUBLE_PULSE_ENABLED{};

  static constexpr Field<bool, RawType, 0x04, 2> Y_SINGLE_PULSE_ENABLED{};
  static constexpr Field<bool, RawType, 0x08, 3> Y_DOUBLE_PULSE_ENABLED{};

  static constexpr Field<bool, RawType, 0x10, 4> Z_SINGLE_PULSE_ENABLED{};
  static constexpr Field<bool, RawType, 0x20, 5> Z_DOUBLE_PULSE_ENABLED{};

  static constexpr Field<bool, RawType, 0x40, 6> LATCH_EVENT_TO_PULSE_SRC{};

  /**
   * False: Double Pulse detection is not aborted if the start of a pulse is detected during the time
   *        period specified by the PULSE_LTCY register.
   * True: Setting the DPA bit momentarily suspends the double tap detection if the start of a pulse
   *       is detected during the time period specified by the PULSE_LTCY register and the pulse ends
   *       before the end of the time period specified by the PULSE_LTCY register.
   */
  static constexpr Field<bool, RawType, 0x80, 7> DISABLE_DOUBLE_PULSE{};
};

/**
 * The threshold values range from 1 to 127 with steps of 0.63g/LSB at a fixed 8g acceleration range, thus the minimum
 * resolution is always fixed at 0.063g/LSB. If the Low Noise bit in Register 0x2A is set then the maximum threshold
 * will be 4g. The PULSE_THSX, PULSE_THSY and PULSE_THSZ registers define the threshold which is used by the system
 * to start the pulse detection procedure. The threshold value is expressed over 7-bits as an unsigned number.
 * @tparam I2C
 * @tparam RawType
 */
template<uint8_t REG_ADDRESS>
class PULSE_THS {
public:
  using RawType = uint8_t;  // Raw type of the register
  static constexpr uint8_t ADDRESS = REG_ADDRESS;
  static constexpr bool CAN_WRITE = true;

  static constexpr Field<RawType, RawType, 0x7F, 0> PULSE_THRESHOLD{};
};

class PULSE_THSX : public PULSE_THS<0x23> {
public:
};

class PULSE_THSY : public PULSE_THS<0x24> {
public:
};

class PULSE_THSZ : public PULSE_THS<0x25> {
public:
};

/**
 * Pulse Time Window 1 Register
 *
 * The bits TMLT7 through TMLT0 define the maximum time interval that can elapse between the start of the acceleration
 * on the selected axis exceeding the specified threshold and the end when the acceleration on the selected axis must go
 * below the specified threshold to be considered a valid pulse.
 *
 * The minimum time step for the pulse time limit is defined in Table 49 and Table 50. Maximum time for a given ODR and
 * Oversampling mode is the time step pulse multiplied by 255. The time steps available are dependent on the
 * Oversampling mode and whether the Pulse Low Pass Filter option is enabled or not. The Pulse Low Pass Filter is set in
 * Register 0x0F.
 */
class PULSE_TMLT {
public:
  using RawType = uint8_t;  // Raw type of the register
  static constexpr bool CAN_WRITE = true;
  static constexpr uint8_t ADDRESS = 0x26;

  static constexpr Field<RawType, RawType, 0xFF, 0> FIRST_PULSE_TIME{};
};

/**
 * Pulse Latency Timer Register
 *
 * The bits LTCY7 through LTCY0 define the time interval that starts after the first pulse detection. During this time
 * interval, all pulses are ignored. Note: This timer must be set for single pulse and for double pulse.
 *
 * The minimum time step for the pulse latency is defined in Table 52 and Table 53 from the datasheet (page 38).
 * The maximum time is the time step at the ODR and Oversampling mode multiplied by 255. The timing also changes when
 * the Pulse LPF is enabled or disabled.
 */
class PULSE_LTCY {
public:
  using RawType = uint8_t;  // Raw type of the register
  static constexpr bool CAN_WRITE = true;
  static constexpr uint8_t ADDRESS = 0x27;

  static constexpr Field<RawType, RawType, 0xFF, 0> PULSE_LATENCY{};
};

/**
 * Second Pulse Time Window Register
 *
 * The bits WIND7 through WIND0 define the maximum interval of time that can elapse after the end of the latency
 * interval in which the start of the second pulse event must be detected provided the device has been configured for
 * double pulse detection. The detected second pulse width must be shorter than the time limit constraints specified by
 * the PULSE_TMLT register, but the end of the double pulse need not finish within the time specified by the PULSE_WIND
 * register.
 *
 * The minimum time step for the pulse window is defined in Table 55 and Table 56 from the datasheet (page 39).
 * The maximum time is the time step at the ODR, Oversampling mode and LPF Filter Option multiplied by 255.
 */
class PULSE_WIND {
public:
  using RawType = uint8_t;  // Raw type of the register
  static constexpr bool CAN_WRITE = true;
  static constexpr uint8_t ADDRESS = 0x28;

  static constexpr Field<RawType, RawType, 0xFF, 0> SECOND_PULSE_TIME{};
};

/**
 * System Control 1 Register
 */
class CTRL_REG1 {
public:
  using RawType = uint8_t;  // Raw type of the register
  static constexpr bool CAN_WRITE = true;
  static constexpr uint8_t ADDRESS = 0x2A;

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

class CTRL_REG2 {
public:
  using RawType = uint8_t;  // Raw type of the register
  static constexpr bool CAN_WRITE = true;
  static constexpr uint8_t ADDRESS = 0x2B;

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

class CTRL_REG3 {
public:
  using RawType = uint8_t;  // Raw type of the register
  static constexpr bool CAN_WRITE = true;
  static constexpr uint8_t ADDRESS = 0x2C;

  enum class IntPinMode : RawType { PUSH_PULL = 0, OPEN_DRAIN };

  enum class IntPolarity : RawType { ACTIVE_LOW = 0, ACTIVE_HIGH };

  static constexpr Field<IntPinMode, RawType, 0x01, 0> INT_PIN_MODE{};
  static constexpr Field<IntPolarity, RawType, 0x02, 1> INT_POLARITY{};
  static constexpr Field<bool, RawType, 0x08, 3> FREEFALL_INT_BYPASSED_IN_SLEEP_MODE{};
  static constexpr Field<bool, RawType, 0x10, 4> PULSE_INT_BYPASSED_IN_SLEEP_MODE{};
  static constexpr Field<bool, RawType, 0x20, 5> ORIENTATION_INT_BYPASSED_IN_SLEEP_MODE{};
  static constexpr Field<bool, RawType, 0x40, 6> TRANSIENT_INT_BYPASSED_IN_SLEEP_MODE{};
  static constexpr Field<bool, RawType, 0x80, 7> FIFO_GATE_BYPASSED_IN_SLEEP_MODE{};
};

class CTRL_REG4 {
public:
  using RawType = uint8_t;  // Raw type of the register
  static constexpr bool CAN_WRITE = true;
  static constexpr uint8_t ADDRESS = 0x2D;

  static constexpr Field<bool, RawType, 0x01, 0> DATA_READY_INT_ENABLED{};
  static constexpr Field<bool, RawType, 0x04, 2> FREEFALL_INT_ENABLED{};
  static constexpr Field<bool, RawType, 0x08, 3> PULSE_DETECTION_INT_ENABLED{};
  static constexpr Field<bool, RawType, 0x10, 4> ORIENTATION_INT_ENABLED{};
  static constexpr Field<bool, RawType, 0x20, 5> TRANSIENT_INT_ENABLED{};
  static constexpr Field<bool, RawType, 0x40, 6> FIFO_INT_ENABLED{};
  static constexpr Field<bool, RawType, 0x80, 7> AUTO_SLEEP_WAKE_INT_ENABLED{};
};

class CTRL_REG5 {
public:
  using RawType = uint8_t;  // Raw type of the register
  static constexpr bool CAN_WRITE = true;
  static constexpr uint8_t ADDRESS = 0x2E;

  enum class IntPin : RawType {
    PIN_INT2 = 0,
    PIN_INT1,
  };

  static constexpr Field<IntPin, RawType, 0x01, 0> DATA_READY_INT_PIN{};
  static constexpr Field<IntPin, RawType, 0x04, 2> FREEFALL_INT_PIN{};
  static constexpr Field<IntPin, RawType, 0x08, 3> PULSE_DETECTION_INT_PIN{};
  static constexpr Field<IntPin, RawType, 0x10, 4> ORIENTATION_INT_PIN{};
  static constexpr Field<IntPin, RawType, 0x20, 5> TRANSIENT_INT_PIN{};
  static constexpr Field<IntPin, RawType, 0x40, 6> FIFO_INT_PIN{};
  static constexpr Field<IntPin, RawType, 0x80, 7> AUTO_SLEEP_WAKE_INT_PIN{};
};
}  // namespace MMA
}  // namespace AdvancedMicrotech

#endif  // ADVANVED_MICROTECH_MMAREGISTERS_HPP
