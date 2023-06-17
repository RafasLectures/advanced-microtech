/***************************************************************************//**
 * @file    main.cpp
 * @author  <your name>
 * @date    <date of creation>
 *
 * @brief   Exercise 3 - Advanced Sensors
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
 *                   CON6:MMA_INT1 <-> CON2:P1.3
 *
 * @note    The project was exported using CCS 8.0.0.
 *
 ******************************************************************************/

#define NO_TEMPLATE_UART

#include "libs/LCD.hpp"
#include "libs/mma.hpp"
#include "libs/common/gpio.hpp"
#include "libs/common/parallelbus.hpp"
#include "libs/i2c.hpp"
#include "libs/templateEMP.h"  // UART disabled, see @note!

using namespace AdvancedMicrotech;

// =========== Clocks ============
typedef DCOCLK_T<1000000> DCO;  // Set DCO clock as 1MHz (eventhough is already done in the initMSP)
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
typedef GPIO_OUTPUT_T<1, 3, HIGH> I2C_SPI;  // Pin P1.3 as output and initial value is 1
typedef GPIO_MODULE_T<1, 6, 3> SCL;         // Setting P1.6 as its function 3 (SCL)
typedef GPIO_MODULE_T<1, 7, 3> SDA;         // Setting P1.7 as its function 3 (SDA)
typedef I2C_T<SDA, SCL, SMCLK> I2C;         // Create I2C and set the SDA and SCL pins. SMCLK is set as clock source.

// ============ MMA ===========
typedef MMA_T<I2C> Mma;


void runSelfTest() {
    LCD::writeString("Running");
    LCD::setCursorPosition(0, 1);
    LCD::writeString("Self-test...");
    delay_ms(2000);

    uint16_t selfTestResult = Mma::runSelfTest();
    while(selfTestResult != 0) {
      LCD::clearDisplay();
      LCD::writeString("Self-test failed");
      LCD::setCursorPosition(0, 1);
      switch (selfTestResult) {
        case 1:
          LCD::writeString("Control problem");
          break;
        case 2:
          LCD::writeString("X mismatch");
          break;
        case 3:
          LCD::writeString("Y mismatch");
          break;
        case 4:
          LCD::writeString("Z mismatch");
          break;
        default:
          break;
      }
      delay_ms(1000);

      selfTestResult = Mma::runSelfTest();

    }
    LCD::clearDisplay();
    LCD::writeString("Self-test");
    LCD::setCursorPosition(0, 1);
    LCD::writeString("successful");
    delay_ms(2000);
    return;
}

int main(void) {
    initMSP();

    // Initialize LCD
    LCD::initialize();
    LCD::enable(true);
    LCD::clearDisplay();

    // Initialize I2C and MMA
    I2C_SPI::init();
    Mma::initialize();
    runSelfTest();
    Mma::setRange(MMA::Range::RANGE_2G);
    Mma::setResolution(MMA::Resolution::BITS_14);

    LCD::clearDisplay();
    LCD::writeString("X:      Y:     ");
    LCD::setCursorPosition(0, 1);
    LCD::writeString("Z:");

    while (1) {
      Mma::read();
      LCD::setCursorPosition(2, 0);
      LCD::writeString("      ");
      LCD::setCursorPosition(2, 0);
      LCD::writeNumber(Mma::get14X());

      LCD::setCursorPosition(10, 0);
      LCD::writeString("      ");
      LCD::setCursorPosition(10, 0);
      LCD::writeNumber(Mma::get14Y());

      LCD::setCursorPosition(2, 1);
      LCD::writeString("      ");
      LCD::setCursorPosition(2, 1);
      LCD::writeNumber(Mma::get14Z());
    }
}
