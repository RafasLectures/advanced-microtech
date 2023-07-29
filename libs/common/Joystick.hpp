/******************************************************************************
 * @file                    Joystick.hpp
 * @author                  Rafael Andrioli Bauer
 * @date                    25.05.2023
 * @matriculation number    5163344
 * @e-mail contact          abauer.rafael@gmail.com
 * @brief                   Header file for Joystick class
 *
 ******************************************************************************/
#ifndef ADVANVED_MICROTECH_JOYSTICK_HPP
#define ADVANVED_MICROTECH_JOYSTICK_HPP

#include <array>
#include <cstdint>

namespace AdvancedMicrotech {

/**
 *  Class provides an abstraction of a joystick. Allowing the user to subscribe to events.
 */
class Joystick {
public:
  using Callback = void (*)();  ///< Callback function type

  Joystick() = default;
  ~Joystick() = default;

  /**
   * Method to subscribe to joystick UP event. One can unsubscribe from this event, by setting the callback to nullptr.
   * @param callback Callback to be called when joystick UP event occurs.
   */
  void registerUpEventCallback(Callback callback) noexcept;
  /**
   * Method to subscribe to joystick DOWN event. One can unsubscribe from this event, by setting the callback to
   * nullptr.
   * @param callback Callback to be called when joystick DOWN event occurs.
   */
  void registerDownEventCallback(Callback callback) noexcept;
  /**
   * Method to subscribe to joystick LEFT event. One can unsubscribe from this event, by setting the callback to
   * nullptr.
   * @param callback Callback to be called when joystick LEFT event occurs.
   */
  void registerLeftEventCallback(Callback callback) noexcept;
  /**
   * Method to subscribe to joystick RIGHT event. One can unsubscribe from this event, by setting the callback to
   * nullptr.
   * @param callback Callback to be called when joystick RIGHT event occurs.
   */
  void registerRightEventCallback(Callback callback) noexcept;
  /**
   * Method to subscribe to joystick PRESSED event. One can unsubscribe from this event, by setting the callback to
   * nullptr.
   * @param callback Callback to be called when joystick PRESSED event occurs.
   */
  void registerPressEventCallback(Callback callback) noexcept;

  /**
   * Method that triggers an evaluation of the joystick. This method is a blocking call, and, in case of an event,
   * the callback will be called within the same context/thread.
   * @param adcValues The values to be evaluated and converted into joystick events.
   */
  void evaluateJoystick(const std::array<uint8_t,4>* adcValues) noexcept;

private:
  Callback upEventCallback = nullptr;     ///< Callback to be called when joystick UP event occurs.
  Callback downEventCallback = nullptr;   ///< Callback to be called when joystick DOWN event occurs.
  Callback leftEventCallback = nullptr;   ///< Callback to be called when joystick LEFT event occurs.
  Callback rightEventCallback = nullptr;  ///< Callback to be called when joystick RIGHT event occurs.
  Callback pressEventCallback = nullptr;  ///< Callback to be called when joystick PRESSED event occurs.
};

}  // namespace AdvancedMicrotech

#endif  // ADVANVED_MICROTECH_JOYSTICK_HPP
