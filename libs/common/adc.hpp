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

/**
 * Class to provide access to a ADC port
 */
template <typename FILTER>
class ADC_HANDLE {
public:
  /**
   * Protected constructor so only the ADC class can create a handle.
   * @param adcValueRef the reference to where the raw value will be written
   */
  constexpr ADC_HANDLE(uint16_t* adcValuePtr) : rawValue(adcValuePtr) {}
  /**
   * Method to get the latest raw ADC read
   * @return the latest raw value
   */
  constexpr uint16_t getRawValue() const noexcept {
    return *rawValue;
  }
  /**
   * Method to get the latest filtered ADC value
   * @return the latest filtered value
   */
  constexpr uint16_t getFilteredValue() noexcept {
    return filter.filterNewSample(rawValue);
  }

private:
  FILTER filter;             // filter
  uint16_t* rawValue;                 ///< Reference to the raw value.
};

enum class SampleAndHoldCycles {
  ADC10CLK_4_CYCLES = 4,
  ADC10CLK_8_CYCLES = 8,
  ADC10CLK_16_CYCLES = 16,
  ADC10CLK_64_CYCLES = 64
};

template<typename CLOCK, typename MCLK, SampleAndHoldCycles sampleAndHoldCycles>
class ADC_T {
  static constexpr float SAMPLE_AND_HOLD_TIME = static_cast<float>(sampleAndHoldCycles) * (1.0/CLOCK::frequency);
  // Conversion time is always 13 ADC clock cycles
  static constexpr float CONVERSION_TIME = 13.0 * (1.0/CLOCK::frequency);
public:
  ADC_T() = default;
  // Deleted copy and move constructors
//  ADC_T(ADC_T&) = delete;
//  ADC_T(ADC_T&&) = delete;
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
    ADC10CTL1 = CONSEQ_3 + getClockType() + ADC10DIV_0 + SHS_0 + INCH_0;

    // Setup Data transfer control 0
    // The basic idea is that everytime the ADC does a conversion, the
    // Data transfer control automatically writes the results (without any need of CPU) back to the
    // adcValues array.
    // References of the adcValues array are passed to the AdcHandles
    // so when the user gets the AdcHandles, the latest raw value will always be available
    // without the user having to actively fetch any data from the ADC10MEM.
    constexpr uint8_t NUMBER_OF_CHANNELS = sizeof(adcValues) / sizeof(adcValues[0]);
    ADC10DTC0 = ADC10CT;                                   // enable continuous transfer
    ADC10DTC1 = NUMBER_OF_CHANNELS;                        // Number of transfers is equal to the size of array.
    ADC10SA = (size_t)(&adcValues[0]);                     // Starts at address is the first entry of the array;
  }

  /**
   * Method that starts the ADC conversion
   * Previous to this call, one has to already have requested the ADC handles, as well as
   * initialized the ADC.
   */
  static constexpr void startConversion() {
    setRegisterBits(ADC10CTL0, static_cast<uint16_t>(ADC10SC + ENC));
  }

  static constexpr std::chrono::nanoseconds getPeriodForNewSample() {
    return std::chrono::nanoseconds(((SAMPLE_AND_HOLD_TIME + CONVERSION_TIME) + ((1.0/MCLK::frequency) * 4.0))*1000000000);
  }
  /**
   * Method to retrieve an ADC Handle.
   * @tparam pinNumber specify pin number of ADC to be retrieved
   * @tparam bitMask Not needed to be filled. There is a default value
   * @return The handle of that ADC pin
   */
  template<uint8_t pinNumber, typename FILTER, uint8_t bitMask = 0x01 << pinNumber>
  static constexpr ADC_HANDLE<FILTER> getAdcHandle() {
    constexpr uint8_t MAX_NUM_ADC_CHANNELS = 7;
    static_assert(pinNumber <= MAX_NUM_ADC_CHANNELS, "Cannot set ADC to pin higher than 7");
    setRegisterBits(ADC10AE0, bitMask);  // Sets pin as an ADC input

    // Creates the AdcHandle and passes the array entry equivalent to the pin to the handle.
    ADC_HANDLE<FILTER> retVal(&adcValues[pinNumber]);
    return std::move(retVal);
  }

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
  /**
   * Array that stores the conversion values from the ADC.
   * It is automatically populated by the DTC
   */
  static std::array<uint16_t, 1> adcValues;
};
template<typename CLOCK, typename MCKL, SampleAndHoldCycles sampleAndHoldCycles>
std::array<uint16_t, 1> ADC_T<CLOCK, MCKL, sampleAndHoldCycles>::adcValues{0};
}  // namespace Microtech

// ADC10 Interruption
// #pragma vector = ADC10_VECTOR
//__interrupt void ADC10_ISR(void) {
//
//}

#endif  // MICROTECH_ADC_HPP
