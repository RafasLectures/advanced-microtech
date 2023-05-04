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

LCD lcd(Microtech::GPIOs::getOutputHandle<Microtech::IOPort::PORT_3, static_cast<uint8_t>(0)>(),
        Microtech::GPIOs::getOutputHandle<Microtech::IOPort::PORT_3, static_cast<uint8_t>(1)>(),
        Microtech::GPIOs::getOutputHandle<Microtech::IOPort::PORT_3, static_cast<uint8_t>(2)>());

int main(void) {
    initMSP();
    lcd.initialize();
    while (1) {
        // TODO: Show a funky demo of what you did.
    }
}
