/******************************************************************************
 * @file    main.cpp
 * @author  Rafael Andrioli Bauer
 * @date    23.06.2023
 *
 * @brief   Exercise 4 - SPI
 *
 * Pin connections: CON5:D4-CON5:D7 <-> CON3:P2.0-CON3:P2.3
 *                  CON5:RS <-> CON4:P3.0
 *                  CON5:RW <-> CON4:P3.1
 *                  CON5:E <-> CON4:P3.2
 *                  CON6:I2C_SPI <-> CON2:P1.3
 *                  CON6:XSCL <-> CON2:P1.6
 *                  CON6:XSDA <-> CON2:P1.7
 *                  CON6:UDAC <-> CON5:BCKL
 *                  JP2:BKL_ON
 *                  CON3:P3:6 <-> X1:Buzzer
 *                  CON6:MMA_INT1 <-> CON2:P1.4
 *                  CON2:P1.5 <-> CON6:CC_CLK
 *                  CON6:CC_SO <-> CON9:F_SO
 *                  CON6:CC_SI <-> CON9:F_SI
 *                  CON6:CC_CLK <-> CON9:F_CLK
 *                  CON4:P3.4 <-> CON9:F_/CS
 *                  CON4:P3.3 <-> CON9:F_/HOLD
 *                  CON4:P3.5 <-> CON9:F_/WP
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
#include "libs/flash.hpp"
#include "libs/i2c.hpp"
#include "libs/templateEMP.h"  // UART disabled, see @note!

#include "libs/common/StringBuilder.hpp"
#include "libs/common/Joystick.hpp"

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

// ============ I2C and SPI ==============
typedef GPIO_OUTPUT_T<1, 3, LOW> I2C_SPI;  // Pin P1.3 as output and initial value is 0
typedef GPIO_MODULE_T<1, 5, 3> SCLK;       // Setting P1.5 as its function 3 (SCLK)
typedef GPIO_MODULE_T<1, 6, 3> SCL_MISO;   // Setting P1.6 as its function 3 (SCL/MISO)
typedef GPIO_MODULE_T<1, 7, 3> SDA_MOSI;   // Setting P1.7 as its function 3 (SDA/MOSI)
typedef SPI_T<SDA_MOSI,SCL_MISO, SCLK, SMCLK, I2C_SPI> SPI;         // Create SPI and set the CS, MOSI, MISO and SCLK pins. SMCLK is set as clock source.
typedef I2C_T<SDA_MOSI, SCL_MISO, SMCLK, I2C_SPI> I2C;         // Create I2C and set the SDA and SCL pins. SMCLK is set as clock source.

// ============ ADC ===========
typedef ADC_DAC_T<I2C> ADC_DAC;  // Create the ADC and pass the I2C as communication means.

// ============ Flash ==============
typedef GPIO_OUTPUT_T<3, 3, HIGH> HOLD;      // Setting P3.4 as output and initial value is 1
typedef GPIO_OUTPUT_T<3, 4, HIGH> CS;      // Setting P3.4 as output and initial value is 1
typedef GPIO_OUTPUT_T<3, 5, HIGH> WP;      // Setting P3.4 as output and initial value is 1
typedef FLASH_T<CS, HOLD, WP, SPI> FLASH;

static constexpr uint8_t WELCOME_MESSAGE_SIZE = 17;   // Size of the welcome message + null terminator
static constexpr uint32_t FLASH_ADDRESS = 0x00FFFD;       // Address to read/store the welcome message

// Custom LCD characters
constexpr LcdCustomCharacter ARROW_UP_DOWN{{0x04, 0x0E, 0x15, 0x04, 0x04, 0x15, 0x0E, 0x04}, 0x00};
constexpr LcdCustomCharacter ENTER{{0x01, 0x01, 0x01, 0x05, 0x09, 0x1F, 0x08, 0x04}, 0x01};

// Instantiation of welcome message and joystick classes
Joystick joystick;
StringBuilder<WELCOME_MESSAGE_SIZE> welcomeMessage;

/**
 * Method that prints the instructions on how to edit the welcome message.
 */
void printInstructionsLine() {
  static constexpr uint8_t ARROW_LEFT = 0x7F;
  static constexpr uint8_t ARROW_RIGHT = 0x7E;
  LCD::writeChar(ARROW_UP_DOWN.address);
  LCD::writeString("Sel ");
  LCD::writeChar(ARROW_LEFT);
  LCD::writeChar(ARROW_RIGHT);
  LCD::writeString("Nav ");
  LCD::writeChar(ENTER.address);
  LCD::writeString("Save");
}

/**
 * Method that sets up the screen to edit the welcome message
 */
void prepareStringEdit() {
    LCD::clearDisplay();
    printInstructionsLine();
    LCD::setCursorPosition(0, 1);
    LCD::writeString(welcomeMessage.getString());
    LCD::setCursorPosition(welcomeMessage.getCurrentPosition(), 1);
    LCD::blinkCursor(true);
}

/**
 * Function that handles the joystick up event.
 */
void upEventCallback() {
  LCD::writeChar(welcomeMessage.setPreviousCharacter());
  LCD::setCursorPosition(welcomeMessage.getCurrentPosition(), 1);
}

/**
 * Function that handles the joystick down event.
 */
void downEventCallback() {
  LCD::writeChar(welcomeMessage.setNextCharacter());
  LCD::setCursorPosition(welcomeMessage.getCurrentPosition(), 1);
}

/**
 * Function that handles the joystick left event.
 */
void leftEventCallback() {
  welcomeMessage.previousPosition();
  LCD::setCursorPosition(welcomeMessage.getCurrentPosition(), 1);
}

/**
 * Function that handles the joystick right event.
 */
void rightEventCallback() {
  welcomeMessage.nextPosition();
  LCD::setCursorPosition(welcomeMessage.getCurrentPosition(), 1);
}

/**
 * Function that handles the event when the joystick gets pressed.
 */
void pressEventCallback() {
  FLASH::init();  // Call init to change from I2C to SPI
  FLASH::write(FLASH_ADDRESS, welcomeMessage.getBufferSize(), welcomeMessage.getBuffer());

  // Verify if write was successful by readying the flash and comparing the data
  uint8_t checkBuffer[WELCOME_MESSAGE_SIZE];
  FLASH::read(FLASH_ADDRESS, welcomeMessage.getBufferSize(), checkBuffer);

  bool savedSuccessfully = true;
  uint8_t bufferSize = welcomeMessage.getBufferSize();
  for(uint8_t i = 0; i < bufferSize; i++) {
    if(checkBuffer[i] != welcomeMessage.getBuffer()[i]) {
      savedSuccessfully = false;
      break;
    }
  }
	// If the write to flash was successful, show message
  if(savedSuccessfully) {
    LCD::blinkCursor(false);
    LCD::setCursorPosition(0, 1);
    LCD::writeString("Saved!          ");
    delay_ms(500);
    prepareStringEdit();
  }
  ADC_DAC::initialize();  // Call init to change from SPI to I2C
}


int main(void) {
    initMSP();
    // Initialize LCD
    LCD::initialize();
    LCD::enable(true);
    LCD::clearDisplay();
    LCD::createCustomChar(&ARROW_UP_DOWN);
    LCD::createCustomChar(&ENTER);

    // Initialize I2C and ADC
    I2C_SPI::init();
    ADC_DAC::initialize();
    ADC_DAC::write(0xFF);   // Backlight ON

    // Initialize FLASH
    FLASH::init();

    // Read welcome message from flash and write to LCD
    uint8_t storedWelcomeMessage[WELCOME_MESSAGE_SIZE]{0};
    FLASH::read(FLASH_ADDRESS, welcomeMessage.getBufferSize(), &storedWelcomeMessage[0]);
    LCD::writeString((const char*)storedWelcomeMessage);
    welcomeMessage.setString((const char*)storedWelcomeMessage);

    // Show hoe to edit the welcome message
    delay_ms(1000);
    LCD::clearDisplay();
    LCD::writeString("Edit welcome msg");
    LCD::setCursorPosition(0, 1);
    printInstructionsLine();
    delay_ms(1000);

    // Register events from Joystick
    joystick.registerUpEventCallback(&upEventCallback);
    joystick.registerDownEventCallback(&downEventCallback);
    joystick.registerRightEventCallback(&rightEventCallback);
    joystick.registerLeftEventCallback(&leftEventCallback);
    joystick.registerPressEventCallback(&pressEventCallback);

    prepareStringEdit();

    ADC_DAC::initialize();
    delay_ms(20);
    std::array<uint8_t,ADC_DAC::NUMBER_AD_CHANNELS> adcValues{0, 0, 0, 0};  // Buffer used to retrieve the ADC values
    while (1) {
        ADC_DAC::read(&adcValues[0]);
        joystick.evaluateJoystick(&adcValues);
        delay_ms(2);
    }
}
