/******************************************************************************
 * @file                    mma.hpp
 * @author                  Rafael Andrioli Bauer
 * @date                    13.06.2023
 * @matriculation number    5163344
 * @e-mail contact          abauer.rafael@gmail.com
 *
 * @brief   Header file with the declaration of the MMA class.
 *          Its implementation must be in the header because it is templated.
 *
 * @ note   The X/Y-direction printed on the board are wrong:
 *          X is the real Y and Y is the real X. Sorry for the confusion. :-(
 ******************************************************************************/

#ifndef EXERCISE_3_LIBS_MMA_H_
#define EXERCISE_3_LIBS_MMA_H_

/******************************************************************************
 * INCLUDES
 *****************************************************************************/

#include "common/fields.hpp"
#include "mmaRegisters.hpp"
#include <cstring>

namespace AdvancedMicrotech {
namespace MMA {

/**
 * Defines the available ranges.
 */
enum class Range {
  RANGE_2G,
  RANGE_4G,
  RANGE_8G,
};
/**
 * Defines the available resolutions
 */
enum class Resolution {
  BITS_8,
  BITS_14,
};
/**
 * Struct to bundle the acceleration datapoints
 */
struct Acceleration {
  int16_t x;
  int16_t y;
  int16_t z;

  constexpr Acceleration operator-(const Acceleration& rhs) const {
    return {x - rhs.x, y - rhs.y, z - rhs.z};
  }
  constexpr Acceleration subtractAbs(const Acceleration& rhs) const {
    return {abs(x - rhs.x), abs(y - rhs.y), abs(z - rhs.z)};
  }
};

constexpr bool operator>=(const Acceleration& lhs, const int16_t& rhs) {
  return lhs.x >= rhs && lhs.y >= rhs && lhs.z >= rhs;
}

constexpr bool operator>=(const Acceleration& lhs, const Acceleration& rhs) {
  return lhs.x >= rhs.x && lhs.y >= rhs.y && lhs.z >= rhs.z;
}
constexpr bool operator<=(const Acceleration& lhs, const Acceleration& rhs) {
  return lhs.x <= rhs.x && lhs.y <= rhs.y && lhs.z <= rhs.z;
}
}  // namespace MMA

/**
 * Class implements the Communication with an MMA8451Q
 * @tparam I2C I2C interface
 */
template<typename I2C>
class MMA_T {
public:
  static constexpr uint8_t I2C_ADDRESS = 0x1D;       ///< I2C address of MMA device
  static constexpr uint8_t FREESCALE_DEVICE = 0x1A;  ///< Device ID of Freescacle MMA8451Q device

  /**
   * Initialize the MMA with 8 bit resolution and 4G measurement range.
   * @return 0 if everything went fine, >0 if something went wrong.
   *
   * (1 pt.)
   */
  static constexpr uint8_t initialize() noexcept {
    I2C::initialize(I2C_ADDRESS);
    // Make sure that the device is the correct one.
    MMA::WHO_AM_I_DEVICE_ID::RawType deviceId = 0;
    readRegister(MMA::WHO_AM_I_DEVICE_ID::ADDRESS, 1, &deviceId);
    if (deviceId != FREESCALE_DEVICE) {
      return 3;  // Wrong device. Nothing should work.
    }

    if (setResolution(MMA::Resolution::BITS_8) != 0) {
      return 1;
    }

    // Make sure the highpass filter is enabled
    xyzConfigReg = MMA::XYZ_DATA_CFG::HIGH_PASS_FILTER_OUT.encode(xyzConfigReg, false);
    if (setRange(MMA::Range::RANGE_4G) != 0) {
      return 2;
    }

    return 0;
  }

  /**
   * Changes the measurement range
   * @param newRange new range to be set
   * @return 0 if everything went fine, 1 if something went wrong.
   *
   * (0.5 pt.)
   */
  static uint8_t setRange(const MMA::Range newRange) {
    range = newRange;
    setActiveMode(false);  // Stops converting
    xyzConfigReg = MMA::XYZ_DATA_CFG::RANGE.encode(xyzConfigReg, translateRange(newRange));
    return writeToRegister(MMA::XYZ_DATA_CFG::ADDRESS, 1, &xyzConfigReg);
  }

  /**
   * Changes the resolution
   * @param newResolution new resolution to be set
   * @return 0 if everything went fine, 1 if something went wrong.
   *
   * (0.5 pt.)
   */
  static uint8_t setResolution(const MMA::Resolution newResolution) {
    resolution = newResolution;
    ctrlReg1 = MMA::CTRL_REG1::RESOLUTION.encode(ctrlReg1, translateResolution(newResolution));
    return writeToRegister(MMA::CTRL_REG1::ADDRESS, 1, &ctrlReg1);
  }

  /**
   * Run a self-test on the MMA, verifying that all three axis and all three
   * measurement ranges are working.
   * @return 0 if everything went fine, 1 if something went wrong.
   *
   * (1 pt.)
   */
  static uint16_t runSelfTest() {
    /* HINT:
     * The idea of the self test is that you measure the current acceleration values,
     * then enable the on-chip self-test and then read the values again.
     * The values without self-test enabled and those with self-test enabled
     * should now feature a predefined difference (see the datasheet).
     */
    static constexpr uint16_t DIFF_X_MIN = 160;
    static constexpr uint16_t DIFF_X_MAX = 200;
    static constexpr uint16_t DIFF_Y_MIN = 230;
    static constexpr uint16_t DIFF_Y_MAX = 280;
    static constexpr uint16_t DIFF_Z_MIN = 1300;
    static constexpr uint16_t DIFF_Z_MAX = 2000;
    // Setting the range to 4G to make sure it is the same as specified in Datasheet
    if (setRange(MMA::Range::RANGE_4G) != 0) {
      return 1;
    }

    if (read() != 0) {
      return 1;
    }
    // Store initial values
    const MMA::Acceleration rawInit{get14X(), get14Y(), get14Z()};

    // Stop conversion
    setActiveMode(false);

    // Start self test and read again
    ctrlReg2 = MMA::CTRL_REG2::SELF_TEST_ON.encode(ctrlReg2, true);
    writeToRegister(MMA::CTRL_REG2::ADDRESS, 1, &ctrlReg2);

    if (read() != 0) {
      // return 1;
      return {};
    }

    // Stop conversion and stop self-test mode
    setActiveMode(false);
    ctrlReg2 = MMA::CTRL_REG2::SELF_TEST_ON.encode(ctrlReg2, false);
    writeToRegister(MMA::CTRL_REG2::ADDRESS, 1, &ctrlReg2);

    // Store reading after
    const MMA::Acceleration rawAfter{get14X(), get14Y(), get14Z()};

    // Absolute difference
    const MMA::Acceleration absDiff = rawInit.subtractAbs(rawAfter);

    if (DIFF_X_MIN > absDiff.x || absDiff.x > DIFF_X_MAX) {
      return 2;
    }
    if (DIFF_Y_MIN > absDiff.y || absDiff.y > DIFF_Y_MAX) {
      return 3;
    }
    if (DIFF_Z_MIN > absDiff.z || absDiff.z > DIFF_Z_MAX) {
      return 4;
    }
    return 0;
  }

  /**
   * Sets up the double tap interrupt on the MMA. The MSP interruption is not set
   * in this function.
   * The MMA should change the INT1-pin whenever a double tap is detected.
   * You may freely choose the axis on which the tap has to be received. (You should put a comment in your code,
   * which axis you chose, though). (1 pt.)
   * @return 0 if everything went fine, 1 if something went wrong.
   *
   * (1 pt.)
   */
  static uint8_t enableTapInterrupt() {
    /* HINT:
     * As the datasheet for the MMA is a bit stingy when it comes to the double
     * tap stuff, so here's (roughly) what you should do:
     *
     *  1) Go to standby (as you can only change the registers when in standby)
     *  2) Write MMA_PULSE_CFG  to enable the z-axis for double tap
     *  3) Write MMA_PULSE_THSZ to set the tap threshold (e.g. to 2g)
     *  4) Write MMA_PULSE_TMLT to set the pulse time limit (e.g. to 100 ms)
     *  5) Write MMA_PULSE_LTCY to set the pulse latency timer (e.g. to 200 ms)
     *  6) Write MMA_PULSE_WIND to set the time window for the second tap
     *  7) Write MMA_CTRL_REG4  to set the pulse interrupt
     *  8) Write MMA_CTRL_REG5  to activate the interrupt on INT1
     *  9) Write MMA_CTRL_REG3  to set the interrupt polarity
     * 10) Return to active mode
     */

    // go to standby
    if (setActiveMode(false) != 0) {
      return 1;
    }

    // Enables double tap in Z axis
    pulseCfgReg = MMA::PULSE_CFG::Z_DOUBLE_PULSE_ENABLED.encode(pulseCfgReg, true);
    writeToRegister(MMA::PULSE_CFG::ADDRESS, 1, &pulseCfgReg);

    // Calculates threshold in compile time.
    // And sets threshold
    static constexpr uint8_t THRESHOLD = 2 / 0.063;  // Calculate threshold of 2G in compile time
    static_assert(THRESHOLD < 0x7F, "Threshold is too high");
    pulseThszReg = MMA::PULSE_THSZ::PULSE_THRESHOLD.encode(pulseThszReg, THRESHOLD);
    writeToRegister(MMA::PULSE_THSZ::ADDRESS, 1, &pulseThszReg);

    static constexpr uint8_t TIME_LIMIT = 150 / 0.625;  // Calculate time limit of 150ms in compile time
    static_assert(TIME_LIMIT < 0xFF, "Time limit is too high");
    pulseTmltReg = MMA::PULSE_TMLT::FIRST_PULSE_TIME.encode(pulseTmltReg, TIME_LIMIT);
    writeToRegister(MMA::PULSE_TMLT::ADDRESS, 1, &pulseTmltReg);

    static constexpr uint8_t LATENCY = 200 / 1.25;  // Calculate latency of 200ms in compile time
    static_assert(LATENCY < 0xFF, "Latency is too high");
    pulseLtcyReg = MMA::PULSE_LTCY::PULSE_LATENCY.encode(pulseLtcyReg, LATENCY);
    writeToRegister(MMA::PULSE_LTCY::ADDRESS, 1, &pulseLtcyReg);

    static constexpr uint8_t SECOND_WINDOW = 250 / 1.25;  // Calculate second pulse within 250ms in compile time
    static_assert(LATENCY < 0xFF, "Second window");
    pulseWindReg = MMA::PULSE_WIND::SECOND_PULSE_TIME.encode(pulseWindReg, SECOND_WINDOW);
    writeToRegister(MMA::PULSE_WIND::ADDRESS, 1, &pulseWindReg);

    ctrlReg4 = MMA::CTRL_REG4::PULSE_DETECTION_INT_ENABLED.encode(ctrlReg4, true);
    writeToRegister(MMA::CTRL_REG4::ADDRESS, 1, &ctrlReg4);

    ctrlReg5 = MMA::CTRL_REG5::PULSE_DETECTION_INT_PIN.encode(ctrlReg5, MMA::CTRL_REG5::IntPin::PIN_INT1);
    writeToRegister(MMA::CTRL_REG5::ADDRESS, 1, &ctrlReg5);

    ctrlReg3 = MMA::CTRL_REG3::INT_POLARITY.encode(ctrlReg3, MMA::CTRL_REG3::IntPolarity::ACTIVE_HIGH);
    writeToRegister(MMA::CTRL_REG3::ADDRESS, 1, &ctrlReg3);

    if (setActiveMode(true) != 0) {
      return 1;
    }
    return 0;
  }

  /**
   * Disable the double-tap-interrupt on the MMA.
   * @return 0 if everything went fine, 1 if something went wrong.
   *
   * (0.5 pt.)
   */
  static uint8_t disableTapInterrupt() {
    if (setActiveMode(false) != 0) {
      return 1;
    }
    // Disables double pulse and interruption
    pulseCfgReg = MMA::PULSE_CFG::Z_DOUBLE_PULSE_ENABLED.encode(pulseCfgReg, false);
    writeToRegister(MMA::PULSE_CFG::ADDRESS, 1, &pulseCfgReg);

    ctrlReg4 = MMA::CTRL_REG4::PULSE_DETECTION_INT_ENABLED.encode(ctrlReg4, false);
    return writeToRegister(MMA::CTRL_REG4::ADDRESS, 1, &ctrlReg4);
  }

  /**
   * Read the values of all three axis from the chip and store the values
   * internally. Take the requested resolution into account.
   * @return 0 if everything went fine, 1 if something went wrong.
   *
   * (1 pt.)
   */
  static uint8_t read() {
    static constexpr uint8_t X_MSB_REG_ADDRESS = 0x01;
    static constexpr uint8_t MAX_BUFFER_SIZE = 6;

    uint8_t bufferSize = MAX_BUFFER_SIZE;
    uint8_t readBuffer[MAX_BUFFER_SIZE] = {0};
    if (resolution == MMA::Resolution::BITS_8) {
      bufferSize = 3;
    }

    setActiveMode(true);

    readRegister(X_MSB_REG_ADDRESS, bufferSize, &readBuffer[0]);
    memset(&xRaw[0], 0, sizeof(xRaw));
    memset(&yRaw[0], 0, sizeof(yRaw));
    memset(&zRaw[0], 0, sizeof(zRaw));
    // Make it the same as if we read a 14 bits message
    if (resolution == MMA::Resolution::BITS_8) {
      xRaw[0] = readBuffer[0];
      yRaw[0] = readBuffer[1];
      zRaw[0] = readBuffer[2];
    } else {
      memcpy(&xRaw[0], &readBuffer[0], sizeof(xRaw));
      memcpy(&yRaw[0], &readBuffer[2], sizeof(yRaw));
      memcpy(&zRaw[0], &readBuffer[4], sizeof(zRaw));
    }

    return 0;
  }

  // ============ Get Functions (1 pt. total) =============
  /**
   * Method to get the 8-bit value in the X axis.
   * If the resolution during read was 14 bit, translate the data to 8 bit
   * @return The 8-bit acceleration in the X axis
   *
   * (~0.11 pt)
   */
  static int8_t get8X() {
    return xRaw[0];
  }

  /**
   * Method to get the 8-bit value in the Y axis.
   * If the resolution during read was 14 bit, translate the data to 8 bit
   * @return The 8-bit acceleration in the Y axis
   *
   * (~0.11 pt)
   */
  static int8_t get8Y() {
    return yRaw[0];
  }

  /**
   * Method to get the 8-bit value in the Z axis.
   * If the resolution during read was 14 bit, translate the data to 8 bit
   * @return The 8-bit acceleration in the Z axis
   *
   * (~0.11 pt)
   */
  static int8_t get8Z() {
    return zRaw[0];
  }

  /**
   * Method to get the 14-bit value in the X axis.
   * If the resolution during read was 8 bit, translate the data to 14 bit
   * @return The 14-bit acceleration in the X axis
   *
   * (~0.11 pt)
   */
  static int16_t get14X() {
    return convert14To16Bit(&xRaw[0]);
  }

  /**
   * Method to get the 14-bit value in the Y axis.
   * If the resolution during read was 8 bit, translate the data to 14 bit
   * @return The 14-bit acceleration in the Y axis
   *
   * (~0.11 pt)
   */
  static int16_t get14Y() {
    return convert14To16Bit(&yRaw[0]);
  }

  /**
   * Method to get the 14-bit value in the Z axis.
   * If the resolution during read was 8 bit, translate the data to 14 bit
   * @return The 14-bit acceleration in the Z axis
   *
   * (~0.11 pt)
   */
  static int16_t get14Z() {
    return convert14To16Bit(&zRaw[0]);
  }

  /**
   * Method to get the acceleration from the in the X axis in m*s^-2.
   * @return The acceleration in the X axis in m*s^-2
   *
   * (~0.11 pt)
   */
  static double getRealX() {
    return convertAcceleration(get14X());
  }

  /**
   * Method to get the acceleration from the in the Y axis in m*s^-2.
   * @return The acceleration in the Y axis in m*s^-2
   *
   * (~0.11 pt)
   */
  static double getRealY() {
    return convertAcceleration(get14Y());
  }

  /**
   * Method to get the acceleration from the in the Z axis in m*s^-2.
   * @return The acceleration in the Z axis in m*s^-2
   *
   * (~0.11 pt)
   */
  static double getRealZ() {
    return convertAcceleration(get14Z());
  }

private:
  /**
   * Method to read from a MMA register
   * @param address Address to be read
   * @param length length of data to be read
   * @param result result buffer.
   * @return if read was successful
   */
  static constexpr uint8_t readRegister(uint8_t address, uint8_t length, uint8_t* result) {
    I2C::write(1, &address, false);
    I2C::read(length, result);
    return 0;
  }

  /**
   * Method to write to a register
   * @param address Address to write in
   * @param value value to be written
   * @return if the write was successful
   */
  static constexpr uint8_t writeToRegister(uint8_t address, uint8_t /*length*/, uint8_t* value) {
    uint8_t infoBuffer[] = {address, *value};
    // I2C::write(1, &address, false);    // I wanted to do like the commented code,
    // I2C::write(length, value, true);   // but I am getting erros. Couldn't solve it
    return I2C::write(2, infoBuffer, true);
  }

  /**
   * Method to translate an MMA::Range to a range value from one of the registers
   * @param range Range to be translated
   * @return The range from the register type
   */
  static constexpr typename MMA::XYZ_DATA_CFG::Range translateRange(const MMA::Range range) noexcept {
    switch (range) {
      case MMA::Range::RANGE_2G: return MMA::XYZ_DATA_CFG::Range::RANGE_2G;
      case MMA::Range::RANGE_4G: return MMA::XYZ_DATA_CFG::Range::RANGE_4G;
      case MMA::Range::RANGE_8G: return MMA::XYZ_DATA_CFG::Range::RANGE_8G;
    }
    return MMA::XYZ_DATA_CFG::Range::RANGE_2G;
  }

  /**
   * Method to translate an MMA::Resolution to the resolution value from one of the registers
   * @param resolution resolution to be translated
   * @return The resolution from the register
   */
  static constexpr typename MMA::CTRL_REG1::Resolution translateResolution(const MMA::Resolution resolution) noexcept {
    switch (resolution) {
      case MMA::Resolution::BITS_8: return MMA::CTRL_REG1::Resolution::BITS_8;
      case MMA::Resolution::BITS_14: return MMA::CTRL_REG1::Resolution::BITS_14;
    }
    return MMA::CTRL_REG1::Resolution::BITS_14;
  }

  /**
   * Helper function to set the accelerator as active or not
   * @param set Whether it should be active or no
   * @return If the operation was successfully completed (0 is success)
   */
  static constexpr uint8_t setActiveMode(const bool set) {
    uint8_t ret = 0;
    if (MMA::CTRL_REG1::ACTIVE.decode(ctrlReg1) ^ set) {
      ctrlReg1 = MMA::CTRL_REG1::ACTIVE.encode(ctrlReg1, set);
      ret = writeToRegister(MMA::CTRL_REG1::ADDRESS, 1, &ctrlReg1);
      delay_ms(5);
    }
    return ret;
  }

  /**
   * Method to convert a 14 2'complement to 16 bits.
   * @param bufferResult
   * @return
   */
  static constexpr int16_t convert14To16Bit(uint8_t* bufferResult) {
    if (*bufferResult > 0x7F) {
      return 0xC000 | (*bufferResult++ << 6 | *bufferResult >> 2);
    }
    return *bufferResult++ << 6 | *bufferResult >> 2;
  }

  /**
   * Converts a raw acceleration into double
   * @param accelRaw Raw value
   * @return Double acceleration value
   */
  static constexpr double convertAcceleration(const int16_t accelRaw) {
    // Multipliers taken from datasheet
    switch (range) {
      case MMA::Range::RANGE_2G: return accelRaw * 0.00025;
      case MMA::Range::RANGE_4G: return accelRaw * 0.00049;
      case MMA::Range::RANGE_8G: return accelRaw * 0.00098;
    }
    return 0.0;
  }

  static MMA::XYZ_DATA_CFG::RawType xyzConfigReg;
  static MMA::CTRL_REG1::RawType ctrlReg1;
  static MMA::CTRL_REG2::RawType ctrlReg2;
  static MMA::CTRL_REG3::RawType ctrlReg3;
  static MMA::CTRL_REG4::RawType ctrlReg4;
  static MMA::CTRL_REG5::RawType ctrlReg5;
  static MMA::PULSE_CFG::RawType pulseCfgReg;
  static MMA::PULSE_THSZ::RawType pulseThszReg;
  static MMA::PULSE_TMLT::RawType pulseTmltReg;
  static MMA::PULSE_LTCY::RawType pulseLtcyReg;
  static MMA::PULSE_WIND::RawType pulseWindReg;

  static MMA::Resolution resolution;
  static MMA::Range range;

  static uint8_t xRaw[2];
  static uint8_t yRaw[2];
  static uint8_t zRaw[2];
};

template<typename I2C>
MMA::XYZ_DATA_CFG::RawType MMA_T<I2C>::xyzConfigReg;

template<typename I2C>
MMA::CTRL_REG1::RawType MMA_T<I2C>::ctrlReg1;
template<typename I2C>
MMA::CTRL_REG2::RawType MMA_T<I2C>::ctrlReg2;
template<typename I2C>
MMA::CTRL_REG3::RawType MMA_T<I2C>::ctrlReg3;
template<typename I2C>
MMA::CTRL_REG4::RawType MMA_T<I2C>::ctrlReg4;
template<typename I2C>
MMA::CTRL_REG5::RawType MMA_T<I2C>::ctrlReg5;
template<typename I2C>
MMA::PULSE_CFG::RawType MMA_T<I2C>::pulseCfgReg;
template<typename I2C>
MMA::PULSE_THSZ::RawType MMA_T<I2C>::pulseThszReg;
template<typename I2C>
MMA::PULSE_TMLT::RawType MMA_T<I2C>::pulseTmltReg;
template<typename I2C>
MMA::PULSE_LTCY::RawType MMA_T<I2C>::pulseLtcyReg;
template<typename I2C>
MMA::PULSE_WIND::RawType MMA_T<I2C>::pulseWindReg;

template<typename I2C>
MMA::Resolution MMA_T<I2C>::resolution;

template<typename I2C>
MMA::Range MMA_T<I2C>::range;

template<typename I2C>
uint8_t MMA_T<I2C>::xRaw[2];
template<typename I2C>
uint8_t MMA_T<I2C>::yRaw[2];
template<typename I2C>
uint8_t MMA_T<I2C>::zRaw[2];

}  // namespace AdvancedMicrotech

#endif /* EXERCISE_3_LIBS_MMA_H_ */
