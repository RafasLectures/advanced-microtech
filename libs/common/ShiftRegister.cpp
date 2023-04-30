#include "common/ShiftRegister.hpp"

namespace Microtech {

void ShiftRegisterBase::setMode(const Mode mode) const noexcept {
  switch (mode) {
    case ShiftRegisterBase::Mode::PAUSE:
      s0.setState(IOState::LOW);
      s1.setState(IOState::LOW);
      break;
    case ShiftRegisterBase::Mode::SHIFT_RIGHT:
      s0.setState(IOState::HIGH);
      s1.setState(IOState::LOW);
      break;
    case ShiftRegisterBase::Mode::SHIFT_LEFT:
      s0.setState(IOState::LOW);
      s1.setState(IOState::HIGH);
      break;
    case ShiftRegisterBase::Mode::MIRROR_PARALLEL:
      s0.setState(IOState::HIGH);
      s1.setState(IOState::HIGH);
      break;
  };
}

void ShiftRegisterBase::init() const noexcept {
  clock.init();
  clear.init();
  clock.setState(IOState::LOW);
  stopAndReset();

  s0.init();
  s1.init();
  setMode(Mode::PAUSE);
}

void ShiftRegisterBase::start() const noexcept {
  clear.setState(IOState::HIGH);
}

void ShiftRegisterBase::stopAndReset() const noexcept {
  clear.setState(IOState::LOW);
}

void ShiftRegisterBase::reset() const noexcept {
  clear.setState(IOState::LOW);
  clear.setState(IOState::HIGH);
}

void ShiftRegisterBase::clockOneCycle() const noexcept {
  clock.setState(IOState::HIGH);
  // According to datasheet it takes approximately 6ns for the clock to stabilize.
  // In theory there is no need for a delay, since the microcontroller clock is 1us
  //__delay_cycles(10);
  clock.setState(IOState::LOW);
}

}  // namespace Microtech
