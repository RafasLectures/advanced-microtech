#ifndef MICROTECH_ADC_HPP
#define MICROTECH_ADC_HPP

#include "MovingAverage.hpp"
#include "helpers.hpp"
#include <msp430g2553.h>
#include <cstdint>
#include <array>

namespace Microtech {

/**
 * Class to provide access to a ADC port
 */
class AdcHandle {
  friend class Adc;

public:
  AdcHandle() = delete;
  /**
   * Method to get the latest raw ADC read
   * @return the latest raw value
   */
  uint16_t getRawValue() const noexcept {
    return rawValue;
  }
  /**
   * Method to get the latest filtered ADC value
   * @return the latest filtered value
   */
  //uint16_t getFilteredValue() noexcept {
  //  return smaFilter.filterNewSample(rawValue);
  //}

protected:
  /**
   * Protected constructor so only the ADC class can create a handle.
   * @param adcValueRef the reference to where the raw value will be written
   */
  constexpr AdcHandle(uint16_t& adcValueRef) : rawValue(adcValueRef) {}

private:
  //SimpleMovingAverage<30> smaFilter;  ///< Moving Averaget filter of 30 samples. This can be templated in the future.
  uint16_t& rawValue;                 ///< Reference to the raw value.
};

class Adc {
  Adc() = default;

public:
  // Deleted copy and move constructors
  Adc(Adc&) = delete;
  Adc(Adc&&) = delete;
  ~Adc() = default;

  /**
   * Method that guarantees that there is only one instance of the ADC class in the software
   * @return A reference to the instance
   */
  static Adc& getInstance() {
    static Adc instance;
    return instance;
  }

  /**
   * Method that initialized the adc
   */
  void init() noexcept {
    // Make sure DTC is disabled
    ADC10DTC0 = 0;
    ADC10DTC1 = 0;

    // Make sure the ADC is not running.
    ADC10CTL0 &= ~ENC;
    while (ADC10CTL1 & ADC10BUSY){}
    // ADC10 on
    // sample and hold time = 16 ADC Clock cycles = 8*0.2us = 1.6 us
    // Multiple sample and conversion on.
    ADC10CTL0 = ADC10ON + ADC10SHT_1 + MSC;

    // Repeat-sequence-of-channels mode
    // CLk source = ADC10OSC => around 5 MHz
    // DTC can take up to 4 MCKL cycles (4 us), so the sample and hold time should be
    // at least 4us.
    // sample and hold time = 16 ADC Clock cycles = 8*0.2us = 1.6 us
    // Convert time = 13 ADC Clock cycles = 13*0.2us = 2.6us
    // Total conversion of 1 channel = Sample and hold + convert time = 1.6us + 2.6us = 4.2us
    // Source of sample and hold from ADC10SC bit
    // Always start sampling from the ADC7, since we are populating the adcValues array with DTC
    ADC10CTL1 = CONSEQ_3 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_0;

    // Setup Data transfer control 0
    // The basic idea is that everytime the ADC does a conversion, the
    // Data transfer control automatically writes the results (without any need of CPU) back to the
    // adcValues array.
    // References of the adcValues array are passed to the AdcHandles
    // so when the user gets the AdcHandles, the latest raw value will always be available
    // without the user having to actively fetch any data from the ADC10MEM.
    ADC10DTC0 = ADC10CT;                                   // enable continuous transfer
    ADC10DTC1 = 1; //sizeof(adcValues) / sizeof(adcValues[0]);  // Number of transfers is equal to the size of array.
    ADC10SA = (size_t)(&adcValues[0]);                     // Starts at address is the first entry of the array;
  }

  /**
   * Method that starts the ADC conversion
   * Previous to this call, one has to already have requested the ADC handles, as well as
   * initialized the ADC.
   */
  void startConversion() {
    setRegisterBits(ADC10CTL0, static_cast<uint16_t>(ADC10SC + ENC));
  }

  /**
   * Method to retrieve an ADC Handle.
   * @tparam pinNumber specify pin number of ADC to be retrieved
   * @tparam bitMask Not needed to be filled. There is a default value
   * @return The handle of that ADC pin
   */
  template<uint8_t pinNumber, uint8_t bitMask = 0x01 << pinNumber>
  AdcHandle getAdcHandle() {
    constexpr uint8_t MAX_NUM_ADC_CHANNELS = 7;
    static_assert(pinNumber <= MAX_NUM_ADC_CHANNELS, "Cannot set ADC to pin higher than 7");
    setRegisterBits(ADC10AE0, bitMask);  // Sets pin as an ADC input

    // Creates the AdcHandle and passes the array entry equivalent to the pin to the handle.
    AdcHandle retVal(adcValues[pinNumber]);

    return retVal;
  }

private:
  /**
   * Array that stores the conversion values from the ADC.
   * It is automatically populated by the DTC
   */
  std::array<uint16_t, 8> adcValues{0, 0, 0, 0, 0, 0, 0, 0};
};
}  // namespace Microtech

// ADC10 Interruption
// #pragma vector = ADC10_VECTOR
//__interrupt void ADC10_ISR(void) {
//  Microtech::Adc::getInstance().interruptionHappened();
//}

#endif  // MICROTECH_ADC_HPP
