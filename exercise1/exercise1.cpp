/***************************************************************************//**
 * @file    main.cpp
 * @author  Rafael Andrioli Bauer
 * @date    01.05.2023
 *
 * @brief   Exercise 1 - Display Interface
 *
 * Implemented a class to abstract LCD operations
 *
 * Pin connections:  CON5:D4-CON5:D7 <-> CON3:P2.0-CON3:P2.3
 *                   CON5:RS <-> CON4:P3.0
 *                   CON5:RW <-> CON4:P3.1
 *                   CON5:E <-> CON4:P3.2
 *
 *
 * @note    The project was exported using CCS 12.1.0
 ******************************************************************************/

#define NO_TEMPLATE_UART

#include "libs/templateEMP.h"
#include "libs/LCD.hpp"
#include "libs/common/parallelbus.hpp"
#include "libs/common/gpio.hpp"

using namespace AdvancedMicrotech;

// Define pins
typedef GPIO_OUTPUT_T<3, 0, LOW> RS;  // Pin P3.0 as output and initial value is 0
typedef GPIO_OUTPUT_T<3, 1, LOW> RW;  // Pin P3.1 as output and initial value is 0
typedef GPIO_OUTPUT_T<3, 2, LOW> E;   // Pin P3.2 as output and initial value is 0

// Defines bus at port 2 with mask 0x0F
typedef PARALLEL_BUS_T<2, 0x0F> LCD_BUS;

// Defines the LCD with the pins and bus from above
typedef LCD_T<RS, RW, E, LCD_BUS> LCD;

int main(void) {
    initMSP();

    LCD::initialize();
    LCD::enable(true);

    static constexpr uint16_t DELAY_TIME = 1000;
    while (1) {
        LCD::clearDisplay();
        LCD::writeString("Halli hallo!");
        LCD::setCursorPosition(0,1);
        LCD::writeString("Check features");
        delay_ms(DELAY_TIME);

        LCD::clearDisplay();
        LCD::writeString("Clear Display...");
        delay_ms(DELAY_TIME);

        LCD::clearDisplay();
        LCD::writeString("Enable/Disable..");
        delay_ms(DELAY_TIME);
        LCD::enable(false);
        delay_ms(DELAY_TIME);
        LCD::enable(true);

        LCD::clearDisplay();
        LCD::writeString("Show/hide");
        LCD::setCursorPosition(0,1);
        LCD::writeString("cursor");
        LCD::showCursor(1);
        delay_ms(DELAY_TIME);
        LCD::showCursor(0);

        LCD::clearDisplay();
        LCD::writeString("Blink cursor");
        LCD::blinkCursor(1);
        delay_ms(3000);
        LCD::blinkCursor(0);

        LCD::clearDisplay();
        LCD::writeString("Write numbers");
        LCD::setCursorPosition(0,1);
        LCD::writeNumber(32767);
        LCD::writeString("  ");
        LCD::writeNumber(-32767);
        delay_ms(DELAY_TIME);

        LCD::clearDisplay();
        LCD::writeChar(*"J");
        LCD::setCursorPosition(1,1);
        LCD::writeChar(*"U");
        LCD::setCursorPosition(2,0);
        LCD::writeChar(*"M");
        LCD::setCursorPosition(3,1);
        LCD::writeChar(*"P");
        LCD::setCursorPosition(7,1);
        LCD::writeChar(*"C");
        LCD::setCursorPosition(8,0);
        LCD::writeChar(*"U");
        LCD::setCursorPosition(9,1);
        LCD::writeChar(*"R");
        LCD::setCursorPosition(10,0);
        LCD::writeChar(*"S");
        LCD::setCursorPosition(11,1);
        LCD::writeChar(*"O");
        LCD::setCursorPosition(12,0);
        LCD::writeChar(*"R");
        delay_ms(DELAY_TIME);
    }
}
