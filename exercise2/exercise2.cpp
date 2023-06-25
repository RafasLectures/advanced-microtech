/******************************************************************************
 * @file    main.cpp
 * @author  Rafael Andrioli Bauer
 * @date    22.05.2023
 *
 * @brief   Exercise 2 - I2C
 *
 * Description and pin connections go here.
 *
 * Pin connections:  CON5:D4-CON5:D7 <-> CON3:P2.0-CON3:P2.3
 *                   CON5:RS <-> CON4:P3.0
 *                   CON5:RW <-> CON4:P3.1
 *                   CON5:E <-> CON4:P3.2
 *                   CON6:I2C_SPI <-> CON2:P1.3
 *                   CON6:XSCL <-> CON2:P1.6
 *                   CON6:XSDA <-> CON2:P1.7
 *                   CON6:UDAC <-> CON5:BCKL
 *                   JP2:BKL_ON
 *                   CON3:P3:6 <-> X1:Buzzer
 *
 * @note    The project was exported using CCS 12.3.0.
 *          UART is disabled within templateEMP.h in order to avoid
 *          interference with your I2C routine!
 *
 ******************************************************************************/

#define NO_TEMPLATE_UART

#include "libs/LCD.hpp"
#include "libs/adac.hpp"
#include "libs/common/gpio.hpp"
#include "libs/common/parallelbus.hpp"
#include "libs/i2c.hpp"
#include "libs/templateEMP.h"  // UART disabled, see @note!

using namespace AdvancedMicrotech;

// =========== Clocks ============
typedef DCOCLK_T<16000000> DCO;  // Set DCO clock as 16MHz (eventhough is already done in the initMSP)
// Setting DCO as SMCLK clock source. (necessary for later when using the I2C,
// so we can set the BR0 and BR1 on compile time.
typedef SMCLK_T<DCO> SMCLK;
// ============ LCD ==============
// Define pins
typedef GPIO_OUTPUT_T<3, 0, LOW> RS;  // Pin P3.0 as output and initial value is 0
typedef GPIO_OUTPUT_T<3, 1, LOW> RW;  // Pin P3.1 as output and initial value is 0
typedef GPIO_OUTPUT_T<3, 2, LOW> E;   // Pin P3.2 as output and initial value is 0

// Defines bus at port 2 with mask 0x0F
typedef PARALLEL_BUS_T<2, 0x0F> LCD_BUS;

// Defines the LCD with the pins and bus from above
typedef LCD_T<RS, RW, E, LCD_BUS> LCD;

// ============ I2C ==============
typedef GPIO_OUTPUT_T<1, 3, LOW> I2C_SPI;  // Pin P1.3 as output and initial value is 1
typedef GPIO_MODULE_T<1, 6, 3> SCL;         // Setting P1.6 as its function 3 (SCL)
typedef GPIO_MODULE_T<1, 7, 3> SDA;         // Setting P1.7 as its function 3 (SDA)
typedef I2C_T<SDA, SCL, SMCLK, I2C_SPI> I2C;         // Create I2C and set the SDA and SCL pins. SMCLK is set as clock source.

// ============ ADC ===========
typedef ADC_DAC_T<I2C> ADC_DAC;  // Create the ADC and pass the I2C as communication means.

int main(void) {
  initMSP();

  // Initialize LCD
  LCD::initialize();
  LCD::enable(true);
  LCD::clearDisplay();
  LCD::writeString("AD0:    AD1:   ");
  LCD::setCursorPosition(0, 1);
  LCD::writeString("AD2:    AD3:   ");

  // Initialize I2C and ADC/DAC
  I2C_SPI::init();
  ADC_DAC::initialize();

  static constexpr uint8_t X_AD_CHANNEL = 1;         // Variable just to make easier to extract the ADC value
  uint8_t adcValues[ADC_DAC::NUMBER_AD_CHANNELS]{};  // Buffer used to retrieve the ADC values

  while (1) {
    ADC_DAC::read(adcValues);
    ADC_DAC::write(adcValues[X_AD_CHANNEL]);

    // Write ADC results in the LCD, mostly for debugging.
    uint8_t adcIndex = 0;
    uint8_t line = 0;
    while (adcIndex < ADC_DAC::NUMBER_AD_CHANNELS) {
      LCD::setCursorPosition(4, line);
      LCD::writeString("    ");
      LCD::setCursorPosition(4, line);
      LCD::writeNumber(adcValues[adcIndex]);
      adcIndex++;

      LCD::setCursorPosition(12, line);
      LCD::writeString("    ");
      LCD::setCursorPosition(12, line);
      LCD::writeNumber(adcValues[adcIndex]);
      adcIndex++;
      line++;
    }
  }
}
