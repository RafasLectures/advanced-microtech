/***************************************************************************//**
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
 * @note    The project was exported using CCS 8.0.0.
 *          UART is disabled within templateEMP.h in order to avoid
 *          interference with your I2C routine!
 *
 ******************************************************************************/

#define NO_TEMPLATE_UART

#include "libs/templateEMP.h"   // UART disabled, see @note!
#include "libs/LCD.hpp"
#include "libs/common/parallelbus.hpp"
#include "libs/common/gpio.hpp"
#include "libs/i2c.hpp"
#include "libs/adac.hpp"

//#include "libs/adac.hpp"

using namespace AdvancedMicrotech;

// =========== Clocks ============
typedef DCOCLK_T<> DCO;
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
typedef GPIO_OUTPUT_T<1, 3, HIGH> I2C_SPI;   // Pin P1.3 as output and initial value is 1
typedef GPIO_MODULE_T<1, 6, 3> SCL;
typedef GPIO_MODULE_T<1, 7, 3> SDA;
typedef I2C_T<SDA, SCL, SMCLK> I2C;

// ============ ADC ===========
typedef ADC_DAC_T<I2C> ADC_DAC;

int main(void) {

    initMSP();

    LCD::initialize();
    LCD::enable(true);
    LCD::clearDisplay();
    LCD::writeString("AD0:    AD1:   ");
    LCD::setCursorPosition(0,1);
    LCD::writeString("AD2:    AD3:   ");
    I2C_SPI::init();
    ADC_DAC::initialize();

    //static constexpr uint16_t DELAY_TIME = 1000;
    uint8_t adcValues[ADC_DAC::NUMBER_AD_CHANNELS]{};
    uint8_t adcIndex = 0;

    ADC_DAC::read(adcValues);
    while (1) {

        ADC_DAC::read(adcValues);

        adcIndex = 0;
        LCD::setCursorPosition(2,0);
        LCD::writeNumber(adcIndex);
        LCD::setCursorPosition(4,0);
        LCD::writeString("    ");
        LCD::setCursorPosition(4,0);
        LCD::writeNumber(adcValues[adcIndex]);

        adcIndex++;
        LCD::setCursorPosition(10,0);
        LCD::writeNumber(adcIndex);
        LCD::setCursorPosition(12,0);
        LCD::writeString("    ");
        LCD::setCursorPosition(12,0);
        LCD::writeNumber(adcValues[adcIndex]);


        adcIndex++;
        LCD::setCursorPosition(2,1);
        LCD::writeNumber(adcIndex);
        LCD::setCursorPosition(4,1);
        LCD::writeString("    ");
        LCD::setCursorPosition(4,1);
        LCD::writeNumber(adcValues[adcIndex]);


        adcIndex++;
        LCD::setCursorPosition(10,1);
        LCD::writeNumber(adcIndex);
        LCD::setCursorPosition(12,1);
        LCD::writeString("    ");
        LCD::setCursorPosition(12,1);
        LCD::writeNumber(adcValues[adcIndex]);

        //delay_ms(DELAY_TIME);

    }
}
