#ifndef ADVANVED_MICROTECH_JOYSTICK_HPP
#define ADVANVED_MICROTECH_JOYSTICK_HPP

#include <cstdint>

namespace AdvancedMicrotech {

class Joystick {
public:
  using Callback = void (*)();

  Joystick() = default;
  ~Joystick() = default;

  void registerUpEventCallback(Callback callback) noexcept;
  void registerDownEventCallback(Callback callback) noexcept;
  void registerLeftEventCallback(Callback callback) noexcept;
  void registerRightEventCallback(Callback callback) noexcept;
  void registerPressEventCallback(Callback callback) noexcept;

  void evaluateJoystick(const uint8_t (*adcValues)[4]) noexcept;

private:
  //  void evaluateBiggerThanAndCallCallback(uint8_t lhs, uint8_t rhs, bool stillActive, Callback* callback);
  Callback upEventCallback = nullptr;
  Callback downEventCallback = nullptr;
  Callback leftEventCallback = nullptr;
  Callback rightEventCallback = nullptr;
  Callback pressEventCallback = nullptr;
};

}  // namespace AdvancedMicrotech

#endif  // ADVANVED_MICROTECH_JOYSTICK_HPP
