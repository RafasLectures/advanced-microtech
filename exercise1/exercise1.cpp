/***************************************************************************//**
 * @file    main.cpp
 * @author  Rafael Andrioli Bauer
 * @date    01.05.2023
 *
 * @brief   Exercise 1 - Display Interface
 *
 * Description and pin connections go here.
 *
 * @note    The project was exported using CCS 12.1.0
 ******************************************************************************/

#define NO_TEMPLATE_UART

#include "libs/templateEMP.h"
#include "libs/LCD.hpp"
#include "libs/common/parallelbus.hpp"
#include "libs/common/gpio.hpp"

using namespace AdvancedMicrotech;

/**
 * Microtech::GPIOs::getOutputHandle<Microtech::IOPort::PORT_3, static_cast<uint8_t>(0)>(),
        Microtech::GPIOs::getOutputHandle<Microtech::IOPort::PORT_3, static_cast<uint8_t>(1)>(),
        Microtech::GPIOs::getOutputHandle<Microtech::IOPort::PORT_3, static_cast<uint8_t>(2)>()
 */
typedef GPIO_OUTPUT_T<3, 0, LOW> RS;
typedef GPIO_OUTPUT_T<3, 1, LOW> RW;
typedef GPIO_OUTPUT_T<3, 2, LOW> E;
typedef PARALLEL_BUS_T<2, 0x0F> LCD_BUS;
typedef LCD_T<RS, RW, E, LCD_BUS> LCD;

int main(void) {
    initMSP();

    LCD::initialize();
    LCD::enable(true);
    LCD::showCursor(true);
    //LCD::blinkCursor(true);
    //LCD::writeString("Hello world");
    //LCD::setCursorPosition(1,10);
    //LCD::writeNumber(-1000);
    //LCD::clearDisplay();
    //LCD::writeNumber(-1000);

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
        LCD::enable(false);
        delay_ms(DELAY_TIME);
        LCD::enable(true);

        LCD::clearDisplay();
        LCD::writeString("Show/hide");
        LCD::setCursorPosition(0,1);
        LCD::writeString("cursor");
        LCD::showCursor(false);
        delay_ms(DELAY_TIME);
        LCD::showCursor(true);

        LCD::clearDisplay();
        LCD::writeString("Blink cursor");
        LCD::blinkCursor(true);
        delay_ms(3000);
        LCD::blinkCursor(false);

        LCD::clearDisplay();
        LCD::writeString("Write numbers");
        LCD::setCursorPosition(0,1);
        LCD::writeNumber(24);
        LCD::writeString("  ");
        LCD::writeNumber(-12);
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
