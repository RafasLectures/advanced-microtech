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
#include "libs/common/adc.hpp"
#include "libs/common/Timer.hpp"
#include "libs/common/ringbuffer.hpp"
#include "libs/templateEMP.h"  // UART disabled, see @note!

#include "audiorecorder/AudioRecorder.hpp"
#include "libs/common/Joystick.hpp"

#include "audiorecorder/MemoryManagerImpl.hpp"
#include "libs/common/pwm.hpp"
#include "libs/digipoti.hpp"
#include "menu/MainMenu.hpp"

using namespace AdvancedMicrotech;

// =========== Clocks ============
typedef DCOCLK_T<16000000> DCO;  // Set DCO clock as 16MHz (eventhough is already done in the initMSP)
// Setting DCO as SMCLK clock source. (necessary for later when using the I2C,
// so we can set the BR0 and BR1 on compile time.
typedef SMCLK_T<DCO> SMCLK;
typedef MCLK_T<DCO> MCLK;
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
typedef SPI_T<SDA_MOSI, SCL_MISO, SCLK, SMCLK, I2C_SPI>
  SPI;  // Create SPI and set the CS, MOSI, MISO and SCLK pins. SMCLK is set as clock source.
typedef I2C_T<SDA_MOSI, SCL_MISO, SMCLK, I2C_SPI>
  I2C;  // Create I2C and set the SDA and SCL pins. SMCLK is set as clock source.

// ============ External ADC ===========
typedef ADC_DAC_T<I2C> ADC_DAC;  // Create the external ADC and pass the I2C as communication means.

// ============ Flash ==============
typedef GPIO_OUTPUT_T<3, 3, HIGH> HOLD;  // Setting P3.4 as output and initial value is 1
typedef GPIO_OUTPUT_T<3, 4, HIGH> CS;    // Setting P3.4 as output and initial value is 1
typedef GPIO_OUTPUT_T<3, 5, HIGH> WP;    // Setting P3.4 as output and initial value is 1
typedef FLASH_T<CS, HOLD, WP, SPI> FLASH;

// ========= Internal ADC =============
typedef Microtech::ADC_T<ADCOSC_T, MCLK, Microtech::SampleAndHoldCycles::ADC10CLK_8_CYCLES, Microtech::ClockDiv::ADC10CLK_DIV_5> ADC;
typedef AdvancedMicrotech::DigiPoti_T<I2C> DIGI_POTI;

// ========= Timers =============
//typedef Microtech::TIMER_T<Microtech::TIMER_A, 1, 1, SMCLK> TimerA1;

// ============== PWM =============
typedef Microtech::TIMER_T<Microtech::TIMER_A, 0, 1, SMCLK, true> PwmTimerA0;
typedef GPIO_MODULE_T<3, 6, 1> PWM_OUTPUT;       // Setting P3.6 as its function 1 (Timer 0_A3.TA2)
typedef Microtech::PWM_T<PwmTimerA0, PWM_OUTPUT> PWM;

// Custom LCD characters
constexpr LcdCustomCharacter ARROW_UP_DOWN{{0x04, 0x0E, 0x15, 0x04, 0x04, 0x15, 0x0E, 0x04}, 0x01};
constexpr LcdCustomCharacter ENTER{{0x01, 0x01, 0x01, 0x05, 0x09, 0x1F, 0x08, 0x04}, 0x02};

typedef AdvancedMicrotech::MemoryManagerImpl<FLASH, ADC, ADC_DAC, PWM> memoryManager;

//void getDataFromAdc();

AudioRecorder audioRecorder;

// Instantiation of welcome message and joystick classes
Joystick joystick;

//Microtech::ADC_HANDLE<Microtech::SimpleMovingAverage<1>> micInput = ADC::getAdcHandle<0, Microtech::SimpleMovingAverage<1>>();

//Microtech::TaskHandler countingTask(&getDataFromAdc, true);

//void getDataFromAdc() {
//  uint8_t const micValue8Bits = (micInput.getRawValue() >> 2);
//  //  ringBuffer.put(micValue8Bits);
//  //  ringBuffer.get();
//  const _iq micValue = _IQ(micValue8Bits);
//  // Calculates the new duty cycle
//  const _iq dutyCyclePWM = _IQmpy(_IQdiv(micValue, _IQ(255)),PWM::MAX_DUTY_CYCLE);
//  PWM::setDutyCycle(dutyCyclePWM);
//}

//void recordEvent(const bool startRecording) {
//  if(startRecording) {
//    recording = true;
//    TimerA1::start();
//    PWM::start();
//  } else {
//    recording = false;
//    TimerA1::stop();
//    PWM::stop();
//  }
//}
//void playEvent(const bool startPlaying) {
//  if(startPlaying) {
//    playing = true;
//    TimerA1::start();
//    PWM::start();
//  } else {
//    playing = false;
//    TimerA1::stop();
//    PWM::stop();
//  }
//}

int main(void) {
  initMSP();
  SMCLK::init();
  MCLK::init();

  I2C_SPI::init();

  // Initialize LCD
  LCD::initialize();
  LCD::enable(true);
  LCD::clearDisplay();
  LCD::createCustomChar(&ARROW_UP_DOWN);
  LCD::createCustomChar(&ENTER);
  LCD::blinkCursor(false);

  // Initialize internal ADC
  ADC::initialize();
  //ADC::startConversion();

  DIGI_POTI::initialize();
  DIGI_POTI::setResistance(240);

  // Initialize I2C and ADC
  ADC_DAC::initialize();
  ADC_DAC::write(0xFF);  // Backlight ON

  MenuItem::setDependencies(&audioRecorder, &joystick);
  MenuItem::setDisplay<LCD>();

  audioRecorder.setFreeTime(std::chrono::seconds(130));

  // Initialize FLASH
  FLASH::init();

  PWM::initialize();
  PWM::setPwmPeriod<ADC::getPeriodForNewSample().count()>();
  PWM::stop();

//  TimerA1::initialize();
//  TimerA1::registerTask<22>(&countingTask);
//  TimerA1::stop();

  AudioRecorder::setMemoryManager<memoryManager>();
  Audio::setMemoryManager<memoryManager>();
  audioRecorder.initialize();
//  audioRecorder.setRecordEventCallback(&recordEvent);
//  audioRecorder.setPlayEventCallback(&playEvent);


  ADC_DAC::initialize();
  MainMenu::select();

  delay_ms(20);
  std::array<uint8_t,ADC_DAC::NUMBER_AD_CHANNELS> adcValues{0, 0, 0, 0};  // Buffer used to retrieve the ADC values
  while (1) {
    if(!memoryManager::executeAction()) {
      ADC_DAC::read(adcValues.data());
      joystick.evaluateJoystick(&adcValues);
    }


    // write ADC values from buffer to flash memory
    // or
    // read ADC values from flash into buffer and write values from buffer to PWM

  }
}
