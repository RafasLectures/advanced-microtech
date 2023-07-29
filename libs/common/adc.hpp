#ifndef MICROTECH_ADC_HPP
#define MICROTECH_ADC_HPP

#include "movingaverage.hpp"
#include "helpers.hpp"
#include "clocks.hpp"
#include <msp430g2553.h>
#include <cstdint>
#include <array>
#include <chrono>

namespace Microtech {

extern void (*handleADC10IsrFunc)(void);
///**
// * Class to provide access to a ADC port
// */
//template <typename FILTER>
//class ADC_HANDLE {
//public:
//  /**
//   * Protected constructor so only the ADC class can create a handle.
//   * @param adcValueRef the reference to where the raw value will be written
//   */
//  constexpr ADC_HANDLE(uint16_t* adcValuePtr) : rawValue(adcValuePtr) {}
//  /**
//   * Method to get the latest raw ADC read
//   * @return the latest raw value
//   */
//  constexpr uint16_t getRawValue() const noexcept {
//    return *rawValue;
//  }
//  /**
//   * Method to get the latest filtered ADC value
//   * @return the latest filtered value
//   */
//  constexpr uint16_t getFilteredValue() noexcept {
//    return filter.filterNewSample(rawValue);
//  }
//
//private:
//  FILTER filter;             // filter
//  uint16_t* rawValue;                 ///< Reference to the raw value.
//};
//
enum class SampleAndHoldCycles {
  ADC10CLK_4_CYCLES = 4,
  ADC10CLK_8_CYCLES = 8,
  ADC10CLK_16_CYCLES = 16,
  ADC10CLK_64_CYCLES = 64
};

enum class ClockDiv {
  ADC10CLK_DIV_1 = 1,
  ADC10CLK_DIV_2 = 2,
  ADC10CLK_DIV_3 = 3,
  ADC10CLK_DIV_4 = 4,
  ADC10CLK_DIV_5 = 5,
  ADC10CLK_DIV_6 = 6,
  ADC10CLK_DIV_7 = 7,
  ADC10CLK_DIV_8 = 8,

};

template<typename CLOCK, typename MCLK, SampleAndHoldCycles sampleAndHoldCycles, ClockDiv DIV>
class ADC_T {
  static constexpr float SAMPLE_AND_HOLD_TIME = static_cast<float>(sampleAndHoldCycles) * (static_cast<float>(DIV)/CLOCK::frequency);
  // Conversion time is always 13 ADC clock cycles
  static constexpr float CONVERSION_TIME = 13.0 * (static_cast<float>(DIV)/CLOCK::frequency);
public:
  ADC_T() = default;
  ~ADC_T() = default;

  /**
   * Method that initialized the adc
   */
  static constexpr void initialize() noexcept {
    // Make sure DTC is disabled
    ADC10DTC0 = 0;
    ADC10DTC1 = 0;

    // Make sure the ADC is not running.
    ADC10CTL0 &= ~ENC;
    while (ADC10CTL1 & ADC10BUSY){}
    // ADC10 on
    // Multiple sample and conversion on.
    ADC10CTL0 = ADC10ON + getSampleAndHoldRegValue() + MSC;

    // Total conversion of 1 channel = Sample and hold + convert time
    // DTC can take up to 4 MCKL cycles, so the sample and hold time should be
    // at least that time.
    static_assert((SAMPLE_AND_HOLD_TIME + CONVERSION_TIME) > ((1.0/MCLK::frequency) * 4.0),
                  "Cannot perform DTC like this. Total conversion time is too fast");

    // Source of sample and hold from ADC10SC bit
    // Always start sampling from the ADC0, since we are populating the adcValues array with DTC
    ADC10CTL1 = CONSEQ_2 + getClockType() + getDivRegValue() + SHS_0 + INCH_0;
    ADC10AE0 = 0x1; // Only enable channel 0
    // Leave DTC to be set by user
  }

  static constexpr void setADCIsrFunctionHandler(void (*pFunction)()) {
    handleADC10IsrFunc = pFunction;
  }

  static constexpr void setDTCBuffer(volatile uint16_t* buffer, uint8_t length) {
    // Setup Data transfer control 0
    // The basic idea is that everytime the ADC does a conversion, the
    // Data transfer control automatically writes the results (without any need of CPU) back a buffer
    // without the user having to actively fetch any data from the ADC10MEM.
    ADC10DTC0 = ADC10CT + ADC10TB;             // enable continuous transfer and two block transfer mode
    ADC10DTC1 = length;                        // Number of transfers is equal to the size of array.
    ADC10SA = (uint16_t)buffer;                // Starts at address is the first entry of the array;
  }
  static constexpr uint8_t getFinishedBlock()  {
    return (ADC10DTC0 & ADC10B1) >> 1;
  }
  /**
   * Method that starts the ADC conversion
   * Previous to this call, one has to already have requested the ADC handles, as well as
   * initialized the ADC.
   */
  static constexpr void startConversion() {
    ADC10CTL0 |= ADC10IE;
    ADC10CTL0 |= ADC10SC + ENC;
  }

  /**
   * Method that stops the ADC conversion
   * Previous to this call, one has to already have requested the ADC handles, as well as
   * initialized the ADC.
   */
  static constexpr void stopConversion() {
    ADC10CTL0 &= ~(ADC10IE + ADC10SC + ENC);
  }

  static constexpr std::chrono::microseconds getPeriodForNewSample() {
//    return std::chrono::microseconds(((SAMPLE_AND_HOLD_TIME + CONVERSION_TIME) + ((1.0/MCLK::frequency) * 4.0))*1000000);
    std::chrono::duration<float> fSeconds(SAMPLE_AND_HOLD_TIME + CONVERSION_TIME);
    return std::chrono::duration_cast<std::chrono::microseconds>(fSeconds);
  }

//  /**
//   * Method to retrieve an ADC Handle.
//   * @tparam pinNumber specify pin number of ADC to be retrieved
//   * @tparam bitMask Not needed to be filled. There is a default value
//   * @return The handle of that ADC pin
//   */
//  template<uint8_t pinNumber, typename FILTER, uint8_t bitMask = 0x01 << pinNumber>
//  static constexpr ADC_HANDLE<FILTER> getAdcHandle() {
//    constexpr uint8_t MAX_NUM_ADC_CHANNELS = 7;
//    static_assert(pinNumber <= MAX_NUM_ADC_CHANNELS, "Cannot set ADC to pin higher than 7");
//    setRegisterBits(ADC10AE0, bitMask);  // Sets pin as an ADC input
//
//    // Creates the AdcHandle and passes the array entry equivalent to the pin to the handle.
//    ADC_HANDLE<FILTER> retVal(&adcValues[pinNumber]);
//    return std::move(retVal);
//  }

private:

  static constexpr uint16_t getClockType() {
    switch (CLOCK::type) {
      case CLOCK_TYPE_ADCOSC: return ADC10SSEL_0;
      case CLOCK_TYPE_ACLK: return ADC10SSEL_1;
      case CLOCK_TYPE_MCLK: return ADC10SSEL_2;
      case CLOCK_TYPE_SMCLK: return ADC10SSEL_3;
    }
    return ADC10SSEL_0;
  }
  static constexpr uint16_t getSampleAndHoldRegValue() {
    switch (sampleAndHoldCycles) {
      case SampleAndHoldCycles::ADC10CLK_4_CYCLES: return ADC10SHT_0;
      case SampleAndHoldCycles::ADC10CLK_8_CYCLES: return ADC10SHT_1;
      case SampleAndHoldCycles::ADC10CLK_16_CYCLES: return ADC10SHT_2;
      case SampleAndHoldCycles::ADC10CLK_64_CYCLES: return ADC10SHT_3;
    }
    return ADC10SHT_0;
  }
  static constexpr uint16_t getDivRegValue() {
    switch (DIV) {
      case ClockDiv::ADC10CLK_DIV_1: return ADC10DIV_0;
      case ClockDiv::ADC10CLK_DIV_2: return ADC10DIV_1;
      case ClockDiv::ADC10CLK_DIV_3: return ADC10DIV_2;
      case ClockDiv::ADC10CLK_DIV_4: return ADC10DIV_3;
      case ClockDiv::ADC10CLK_DIV_5: return ADC10DIV_4;
      case ClockDiv::ADC10CLK_DIV_6: return ADC10DIV_5;
      case ClockDiv::ADC10CLK_DIV_7: return ADC10DIV_6;
      case ClockDiv::ADC10CLK_DIV_8: return ADC10DIV_7;

    }
  }
//  /**
//   * Array that stores the conversion values from the ADC.
//   * It is automatically populated by the DTC
//   */
//  static std::array<uint16_t, 1> adcValues;
};
//template<typename CLOCK, typename MCKL, SampleAndHoldCycles sampleAndHoldCycles>
//std::array<uint16_t, 1> ADC_T<CLOCK, MCKL, sampleAndHoldCycles>::adcValues{0};
}  // namespace Microtech

#endif  // MICROTECH_ADC_HPP
