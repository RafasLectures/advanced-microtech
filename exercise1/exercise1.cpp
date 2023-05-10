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

#include "libs/templateEMP.h"
#include "libs/LCD.hpp"
using namespace AdvancedMicrotech;

/**
 * Microtech::GPIOs::getOutputHandle<Microtech::IOPort::PORT_3, static_cast<uint8_t>(0)>(),
        Microtech::GPIOs::getOutputHandle<Microtech::IOPort::PORT_3, static_cast<uint8_t>(1)>(),
        Microtech::GPIOs::getOutputHandle<Microtech::IOPort::PORT_3, static_cast<uint8_t>(2)>()
 */
LCD lcd;

int main(void) {
    initMSP();
    lcd.initialize();
    lcd.enable(true);
    lcd.showCursor(false);
    lcd.blinkCursor(false);
    lcd.writeString("Hello world");
    lcd.setCursorPosition(1,10);
    lcd.writeNumber(-1000);
    lcd.clearDisplay();
    lcd.writeNumber(-1000);
    while (1) {
        lcd.writeString("Halli hallo!");
        lcd.setCursorPosition(1,0);
        lcd.writeString("Check features");
        __delay_cycles(100000);
        lcd.clearDisplay();
        lcd.setCursorPosition(0,0);
        lcd.writeString("Clear Display...");
        lcd.clearDisplay();
        lcd.setCursorPosition(0,0);
        __delay_cycles(100000);
    }
}
