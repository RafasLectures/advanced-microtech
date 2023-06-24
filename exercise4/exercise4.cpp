/***************************************************************************//**
 * @file    main.c
 * @author  <your name>
 * @date    <date of creation>
 *
 * @brief   Exercise 4 - SPI
 *
 * Description and pin connections go here.
 *
 * @note    The project was exported using CCS 12.3.0
 *
 ******************************************************************************/
#define NO_TEMPLATE_UART

#include "libs/flash.hpp"
#include "libs/LCD.hpp"
#include "libs/common/gpio.hpp"
#include "libs/common/parallelbus.hpp"
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
typedef GPIO_OUTPUT_T<1, 3, LOW> I2C_SPI;  // Pin P1.3 as output and initial value is 0
typedef GPIO_MODULE_T<1, 5, 3, OUTPUT> SCLK;       // Setting P1.5 as its function 3 (SCLK)
typedef GPIO_MODULE_T<1, 6, 3, INPUT> SCL_MISO;   // Setting P1.6 as its function 3 (SCL/MISO)
typedef GPIO_MODULE_T<1, 7, 3, OUTPUT> SDA_MOSI;   // Setting P1.7 as its function 3 (SDA/MOSI)
typedef SPI_T<SDA_MOSI,SCL_MISO, SCLK, SMCLK, I2C_SPI> SPI;         // Create SPI and set the CS, MOSI, MISO and SCLK pins. SMCLK is set as clock source.

// ============ Flash ==============
typedef GPIO_OUTPUT_T<3, 3, HIGH> HOLD;      // Setting P3.4 as output and initial value is 1
typedef GPIO_OUTPUT_T<3, 4, HIGH> CS;      // Setting P3.4 as output and initial value is 1
typedef GPIO_OUTPUT_T<3, 5, HIGH> WP;      // Setting P3.4 as output and initial value is 1
typedef FLASH_T<CS, HOLD, WP, SPI> FLASH;


int main(void) {
    initMSP();
    // Initialize LCD
    LCD::initialize();
    LCD::enable(true);
    LCD::clearDisplay();

    // Initialize I2C and MMA
    I2C_SPI::init();
    FLASH::init();

    //uint8_t someData[]{*"S", *"o", *"m", *"e"};
    //uint8_t* someData = (uint8_t*)"Some";
    uint8_t someData[5]{0};
    FLASH::read(0x1F0000, 5, someData);
    LCD::writeString((const char*)someData);
    while (1) {
        // TODO: Add your main program here.
    }
}
