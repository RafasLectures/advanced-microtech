/***************************************************************************//**
 * @file    LCD.c
 * @author  <your name>
 * @date    <date of creation>
 *
 * @brief   <brief description>
 *
 * Here goes a detailed description if required.
 ******************************************************************************/

#include "libs/LCD.hpp"

#include <type_traits>
#include <limits>
#include <cstdio>
#include <cstdlib>

namespace AdvancedMicrotech {

/******************************************************************************
 * VARIABLES
 *****************************************************************************/
#if 0
void LCD::initialize() noexcept{
    displayControl = DisplayControl::BASE_VALUE;
    functionSet = FunctionSet::BASE_VALUE;

    // Initialize IOs
    selectRegister.init();
    readWrite.init();
    enableReadWrite.init();
    dataBus.initialize();

    // Create the function set frame to how we want to work:
    //  - Interface of 4 bits
    //  - 2 Lines
    //  - Character size 5x8 dots
    FunctionSet::INTERFACE_DATA_LENGTH.encode(functionSet, FunctionSet::InterfaceDataLength::BITS_4);
    FunctionSet::NUMBER_OF_LINES.encode(functionSet, FunctionSet::NumberOfLines::LINES_2);
    FunctionSet::CHARACHTER_FONT.encode(functionSet, FunctionSet::CharachterFont::DOTS_5X8);

    // Sets the function set to 4 bits.
    // According to datasheet, we first need to write the MSB of the functionSet frame once
    // and then write the whole frame again, so we set it first
    // to 4-bit operation.
    writeWordToBus(functionSet >> 4);
    sendInstruction(functionSet);
    clearDisplay();

}
void LCD::enable(const bool enable) noexcept {
    DisplayControl::DISPLAY_ON.encode(displayControl, enable);
    sendInstruction(displayControl);
}
void LCD::setCursorPosition(const uint8_t x, const uint8_t y) noexcept {
    static constexpr uint8_t SET_ADDRESS_BASE = 0x80;
    uint8_t address = SET_ADDRESS_BASE;
    if(y >= 1) {
        address |= 0x40;
    }
    if(x > 0x0F) {
        address |= 0x0F;
    } else {
        address |= x;
    }

    waitUntilNotBusy();
    setRegister(RegisterSelection::INSTRUCTION);
    writeByteToBus(address);
}
void LCD::showCursor(const bool show) noexcept{
    DisplayControl::CURSOR_ON.encode(displayControl, show);
    sendInstruction(displayControl);
}
void LCD::blinkCursor(const bool blink) noexcept{
    DisplayControl::BLINK_CURSOR.encode(displayControl, blink);
    sendInstruction(displayControl);
}
void LCD::clearDisplay() noexcept {
    sendInstruction(CLEAR_DISPLAY);
}

void LCD::writeChar(const char character) noexcept {
    setRegister(RegisterSelection::DATA);
    writeByteToBus(character);
}
void LCD::writeString(const char *text) noexcept {
    while(*text != 0x00) {
        writeChar(*text);
        text++;
    }
}
void LCD::writeDigitOfNumber(const int16_t number, uint8_t digit) {
    static constexpr int16_t POW10[] = {1, 10, 100, 1000, 10000};
    static constexpr int16_t BASE_ASCII = 0x30;
    if(digit > 4) {
        // Only support up to 4 digits. Early return
        return;
    }
    uint8_t digitToBeRemoved = 0;
    if(digit < 4) {
        digitToBeRemoved = digit + 1;
    } else {
        digitToBeRemoved = digit;
    }
    const int16_t originalNumber = number;
    const int16_t numberToProcess = number % POW10[digitToBeRemoved];
    if (originalNumber >= POW10[digit]) {
        const int16_t toBeWritten = numberToProcess / POW10[digit];
        writeChar(BASE_ASCII + toBeWritten);
    }
    // Remove the nth-digit.
//    return number % POW10[digit];
}
void LCD::writeNumber(int16_t number) noexcept {
    static constexpr int16_t SIGN_MASK = 0x8000;
    // Check if number is negative
    if(SIGN_MASK & number) {
        writeChar(0x2D); // Minus sign in ASCII

        // Flip every bit of it so its like a normal number
        // without the sign and add 1 because -1 is 0xFFFF, so after flipping
        // it is 0x0000 + 1 = 0x0001;
        number = ~number;
        number++;
    }
    int8_t i = 4;
    while(i >= 0) {
        writeDigitOfNumber(number, i);
        i--;
    }
    /*
    int16_t j = number;
    // If the number is between 10000 and 32767, print the 10000-
    // digit.
    if (j >= 10000) {
        writeChar(0x30 + number / 10000);
    }
    // Remove the 10000-digit.
    number = number % 10000;
    // Print the 1000-digit, if the number bigger then 999.
    if (j >= 1000)
    {
        writeChar(0x30 + number / 1000);
    }
    // Now remove the 1000-digit.
    number = number % 1000;
    // Print the 100-digit if the number is big enough ...
    if (j >= 100)
    {
        writeChar(0x30 + number / 100);
    }
    // ... remove it ...
    number = number % 100;
    // ... same for 10-digit ...
    if (j >= 10)
    {
        writeChar(0x30 + number / 10);
    }
    // ...
    number = number % 10;
    // Print the last digit, no matter how big the number is (so if the
    // number is 0, we'll just print that).
    writeChar(0x30 + number / 1);
    */
}

void LCD::sendInstruction(const uint8_t instruction) noexcept {
    waitUntilNotBusy();
    setRegister(RegisterSelection::INSTRUCTION);
    writeByteToBus(instruction);
}

void LCD::writeWordToBus(const uint8_t dataToWrite) noexcept {
   enableOperation(false);  // Disable write
    setOperation(DataOperation::WRITE);                 // Set to write mode
    dataBus.write(dataToWrite);                         // Put data to bus
    enableOperation(true); // Enable write

    //__delay_cycles(100);
    enableOperation(false);  // Disable write
}

void LCD::writeByteToBus(const uint8_t dataToWrite) noexcept {
    const uint8_t dataMsb = (dataToWrite) >> 4;
    const uint8_t dataLsb = (0x0F & dataToWrite);
    writeWordToBus(dataMsb);
    writeWordToBus(dataLsb);
}


void LCD::waitUntilNotBusy() {

}
#endif
}
