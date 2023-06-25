
#define NO_TEMPLATE_UART

#include "libs/common/StringBuilder.hpp"
#include "libs/common/Joystick.hpp"

#include <cstdint>
#include <iostream>
using namespace AdvancedMicrotech;

StringBuilder<16> welcomeMessage;
Joystick joystick;
void upEventCallback() {
  std::cout << "Up event! Previous Character Result: " << welcomeMessage.setPreviousCharacter() << std::endl;
  std::cout << "New String: " << welcomeMessage.getString() << std::endl;
}

void downEventCallback() {
  std::cout << "Down event! Next Character Result: " << welcomeMessage.setNextCharacter() << std::endl;
  std::cout << "New String: " << welcomeMessage.getString() << std::endl;
}

void leftEventCallback() {
  std::cout << "Left event! Previous position result: " << welcomeMessage.previousPosition() << std::endl;
  std::cout << "New position: " << static_cast<int16_t>(welcomeMessage.getCurrentPosition()) << std::endl;
}

void rightEventCallback() {
  std::cout << "Right event! Next position result: " << welcomeMessage.nextPosition() << std::endl;
  std::cout << "New position: " << static_cast<int16_t>(welcomeMessage.getCurrentPosition()) << std::endl;
}

int main(void) {
  welcomeMessage.setInitialMessage("Hello World!");

  joystick.registerUpEventCallback(&upEventCallback);
  joystick.registerDownEventCallback(&downEventCallback);
  joystick.registerRightEventCallback(&rightEventCallback);
  joystick.registerLeftEventCallback(&leftEventCallback);

  // Simulate ADC values
  uint8_t adcValues[4]{127,127,127,127};  // Buffer used to retrieve the ADC values

  std::cout << " ============= AD1 = 0 ==============" << std::endl;
  adcValues[1] = 0;
  joystick.evaluateJoystick(&adcValues);
  adcValues[1] = 127;
  joystick.evaluateJoystick(&adcValues);

  std::cout << " ============= AD1 = 255 ==============" << std::endl;
  adcValues[1] = 255;
  joystick.evaluateJoystick(&adcValues);
  adcValues[1] = 127;
  joystick.evaluateJoystick(&adcValues);

  std::cout << " ============= AD2 = 0 ==============" << std::endl;
  adcValues[2] = 0;
  joystick.evaluateJoystick(&adcValues);
  adcValues[2] = 127;
  joystick.evaluateJoystick(&adcValues);

  std::cout << " ============= AD2 = 255 ==============" << std::endl;
  adcValues[2] = 255;
  joystick.evaluateJoystick(&adcValues);


}
