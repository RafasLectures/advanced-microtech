/***************************************************************************//**
 * @file    main.c
 * @author  <your name>
 * @date    <date of creation>
 *
 * @brief   Exercise 2 - I2C
 *
 * Description and pin connections go here.
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

//#include "libs/adac.hpp"

using namespace AdvancedMicrotech;

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

int main(void) {
    initMSP();
    LCD::initialize();
    I2C_SPI::init();

    // TODO: Add your initialization here.

    while (1) {
    }
}
