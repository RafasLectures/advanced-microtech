/******************************************************************************
 * @file    mma.hpp
 * @author                  Rafael Andrioli Bauer
 * @date                    13.06.2023
 * @matriculation number    5163344
 * @e-mail contact          abauer.rafael@gmail.com
 *
 * @brief   <brief description>
 *
 * Here goes a detailed description if required.
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

enum class Range {
  RANGE_2G,
  RANGE_4G,
  RANGE_8G,
};
enum class Resolution {
  BITS_8,
  BITS_14,
};
enum class Axis {
  UNDEFINED = 0,
  AXIS_X,
  AXIS_Y,
  AXIS_Z,
};


struct DiffSelfTest {
    int16_t xDiff;
    int16_t yDiff;
    int16_t zDiff;
};
}

template<typename I2C>
class MMA_T {
public:
  static constexpr uint8_t I2C_ADDRESS = 0x1D;      ///< I2C address of MMA device
  static constexpr uint8_t FREESCALE_DEVICE = 0x1A;      ///< Device ID of Freescacle MMA8451Q device

  /**
   * Initialize the MMA with 8 bit resolution and 4G measurement range.
   * @return 0 if everything went fine, >0 if something went wrong.
   *
   * (1 pt.)
   */
  static uint8_t initialize() {
    I2C::initialize(I2C_ADDRESS);
    // Make sure that the device is the correct one.

    if(MMA::WHO_AM_I_DEVICE_ID<I2C>::readRegister() != FREESCALE_DEVICE) {
      // Wrong device. nothing should work.
     return 3;
    }

    if(setResolution(MMA::Resolution::BITS_8) != 0) {
      return 1;
    }
    
    xyzConfigReg.encodeField(xyzConfigReg.HIGH_PASS_FILTER_OUT, false);

    if(setRange(MMA::Range::RANGE_4G) != 0) {
      return 2;
    }

    ctrlReg2.writeCacheToRegister();
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
    setActiveMode(false);
    xyzConfigReg.encodeField(xyzConfigReg.RANGE, translateRange(newRange));
    return xyzConfigReg.writeCacheToRegister();
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
      ctrlReg1.encodeField(ctrlReg1.RESOLUTION, translateResolution(newResolution));
      return ctrlReg1.writeCacheToRegister();
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
     * The values without self-test enabled and those with selftest enabled
     * should now feature a predefined difference (see the datasheet).
     */
      static constexpr uint16_t DIFF_X_MIN = 160;
      static constexpr uint16_t DIFF_X_MAX = 200;
      static constexpr uint16_t DIFF_Y_MIN = 230;
      static constexpr uint16_t DIFF_Y_MAX = 280;
      static constexpr uint16_t DIFF_Z_MIN = 1300;
      static constexpr uint16_t DIFF_Z_MAX = 2000;
      // Setting the range to 4G to make sure it is the same as specified in Datasheet
      if(setRange(MMA::Range::RANGE_4G) != 0) {
        //return 1;
          return {};
      }

      if(read() != 0) {
        //return 1;
        return {};
      }

      int16_t xRawInit = get14X();
      int16_t yRawInit = get14Y();
      int16_t zRawInit = get14Z();

      // Stop conversion
      setActiveMode(false);

      // Start self test and read again
      ctrlReg2.encodeField(ctrlReg2.SELF_TEST_ON, true);
      //ctrlReg2.encodeField(ctrlReg2.SOFTWARE_RESET, true);
      ctrlReg2.writeCacheToRegister();

      if(read() != 0) {
        //return 1;
        return {};
      }

      // Stop conversion
      setActiveMode(false);
      ctrlReg2.encodeField(ctrlReg2.SELF_TEST_ON, false);
      ctrlReg2.writeCacheToRegister();

      int16_t xRawAfter = get14X();
      int16_t yRawAfter = get14Y();
      int16_t zRawAfter = get14Z();

      uint16_t xDiff = abs(xRawInit - xRawAfter);
      uint16_t yDiff = abs(yRawInit - yRawAfter);
      uint16_t zDiff = abs(zRawInit - zRawAfter);

      if(DIFF_X_MIN > xDiff || xDiff > DIFF_X_MAX){
        return 2;
      }
      if(DIFF_Y_MIN > yDiff || yDiff > DIFF_Y_MAX){
        return 3;
      }
      if(DIFF_Z_MIN > zDiff || zDiff > DIFF_Z_MAX){
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
  static uint8_t enableTapInterrupt(const MMA::Axis tapAxis) {
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
  }

  /**
   * Disable the double-tap-interrupt on the MMA.
   * @return 0 if everything went fine, 1 if something went wrong.
   *
   * (0.5 pt.)
   */
  static uint8_t disableTapInterrupt() {

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
      if(resolution == MMA::Resolution::BITS_8) {
        bufferSize = 3;
      }

      setActiveMode(true);

      uint8_t readBuffer[MAX_BUFFER_SIZE]={0};
      uint8_t regAddress = X_MSB_REG_ADDRESS;
      I2C::write(sizeof(regAddress), &regAddress, false);
      I2C::read(bufferSize, readBuffer);
      memset(xRaw, 0, sizeof(xRaw));
      memset(yRaw, 0, sizeof(yRaw));
      memset(zRaw, 0, sizeof(zRaw));
      // Make it the same as if we read a 14 bits message
      if(resolution == MMA::Resolution::BITS_8) {
        xRaw[0] = readBuffer[0];
        yRaw[0] = readBuffer[1];
        zRaw[0] = readBuffer[2];
      } else {
        memcpy(xRaw, &readBuffer[0], sizeof(xRaw));
        memcpy(yRaw, &readBuffer[2], sizeof(yRaw));
        memcpy(zRaw, &readBuffer[4], sizeof(zRaw));
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
  static constexpr typename MMA::XYZ_DATA_CFG<I2C>::Range translateRange(const MMA::Range range) noexcept {
    switch (range) {
      case MMA::Range::RANGE_2G: return MMA::XYZ_DATA_CFG<I2C>::Range::RANGE_2G;
      case MMA::Range::RANGE_4G: return MMA::XYZ_DATA_CFG<I2C>::Range::RANGE_4G;
      case MMA::Range::RANGE_8G: return MMA::XYZ_DATA_CFG<I2C>::Range::RANGE_8G;
    }
    return MMA::XYZ_DATA_CFG<I2C>::Range::RANGE_2G;
  }

  static constexpr typename MMA::CTRL_REG1<I2C>::Resolution translateResolution(const MMA::Resolution resolution) noexcept {
      switch (resolution) {
      case MMA::Resolution::BITS_8: return MMA::CTRL_REG1<I2C>::Resolution::BITS_8;
      case MMA::Resolution::BITS_14: return MMA::CTRL_REG1<I2C>::Resolution::BITS_14;
      }
      return MMA::CTRL_REG1<I2C>::Resolution::BITS_14;
  }

  static uint8_t setActiveMode(const bool set) {
      uint8_t ret = 0;
      if(ctrlReg1.getFieldValue(ctrlReg1.ACTIVE) ^ set) {
          ctrlReg1.encodeField(ctrlReg1.ACTIVE, set);
          ret = ctrlReg1.writeCacheToRegister();
          delay_ms(5);
      }
      return ret;
  }

  static constexpr int16_t convert14To16Bit(uint8_t* bufferResult) {
      if(*bufferResult > 0x7F) {
          return 0xC000 | (*bufferResult++ << 6 | *bufferResult>>2);
      }
      return *bufferResult++ << 6 | *bufferResult>>2;
  }

  static constexpr double convertAcceleration(const int16_t accelRaw) {
      switch (range) {
        case MMA::Range::RANGE_2G: return accelRaw*0.00025;
        case MMA::Range::RANGE_4G: return accelRaw*0.00049;
        case MMA::Range::RANGE_8G: return accelRaw*0.00098;
      }
      return 0.0;
  }

  static MMA::XYZ_DATA_CFG<I2C> xyzConfigReg;
  static MMA::CTRL_REG1<I2C> ctrlReg1;
  static MMA::CTRL_REG2<I2C> ctrlReg2;

  static MMA::Resolution resolution;
  static MMA::Range range;

  static uint8_t xRaw[2];
  static uint8_t yRaw[2];
  static uint8_t zRaw[2];
};

template<typename I2C>
MMA::XYZ_DATA_CFG<I2C> MMA_T<I2C>::xyzConfigReg;

template<typename I2C>
MMA::CTRL_REG1<I2C> MMA_T<I2C>::ctrlReg1;
template<typename I2C>
MMA::CTRL_REG2<I2C> MMA_T<I2C>::ctrlReg2;

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

}

#if 0
// All configuration functions return 0 if everything went fine
// and anything but 0 if not (they are the ones with a unsigned char return type).

// Initialize the MMA with 8 bit resolution and 4G measurement range (1 pt.)
unsigned char mma_init(void);

// Change the measurement range. (0: 2g, 1: 4g, >1: 8g) (0.5 pt.)
unsigned char mma_setRange(unsigned char range);
// Change the resolution (0: 8 Bit, >= 1: 14 Bit) (0.5 pt.)
unsigned char mma_setResolution(unsigned char resolution);

// Run a self-test on the MMA, verifying that all three axis and all three
// measurement ranges are working. (1 pt.)
/* HINT:
 * The idea of the self test is that you measure the current acceleration values,
 * then enable the on-chip self-test and then read the values again.
 * The values without selftest enabled and those with selftest enabled
 * should now feature a predefined difference (see the datasheet).
 */
unsigned char mma_selftest(void);

// Set up the double tap interrupt on the MMA (do not set up the interrupt on
// the MSP in this function!). This means that the MMA should change the INT1-
// pin whenever a double tap is detected. You may freely choose the axis on
// which the tap has to be received. (You should put a comment in your code,
// which axis you chose, though). (1 pt.)
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
unsigned char mma_enableTapInterrupt(void);
// Disable the double-tap-interrupt on the MMA. (0.5 pt.)
unsigned char mma_disableTapInterrupt(void);

// Read the values of all three axis from the chip and store the values
// internally. Take the requested resolution into account. (1 pt.)
unsigned char mma_read(void);

/* Get Functions (1 pt. total): */

// Return the appropriate 8 bit values
// If the resolution during mma_read was 14 bit, translate the data to 8 bit
signed char mma_get8X(void);
signed char mma_get8Y(void);
signed char mma_get8Z(void);

// Return the appropriate 14 bit values
// If the resolution during mma_read was 8 bit, translate the data to 14 bit
int mma_get14X(void);
int mma_get14Y(void);
int mma_get14Z(void);

// Return the appropriate values in m*s^-2.
double mma_getRealX(void);
double mma_getRealX(void);
double mma_getRealX(void);

#endif
#endif /* EXERCISE_3_LIBS_MMA_H_ */
