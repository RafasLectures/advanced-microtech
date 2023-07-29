#ifndef ADVANVED_MICROTECH_DIGIPOTI_HPP
#define ADVANVED_MICROTECH_DIGIPOTI_HPP

#include <cstdint>
#include <array>

namespace AdvancedMicrotech {

template<typename I2C>
class DigiPoti_T {
public:
  static constexpr uint8_t ADDRESS = 0x28;
  static constexpr void initialize() {
    I2C::initialize(ADDRESS);
  }

  static constexpr void setResistance(uint8_t resistanceScale) {
    std::array<uint8_t, 2> data{0, resistanceScale};
    I2C::write(data.size(), data.data(), true);
  }
};
}
#endif  // ADVANVED_MICROTECH_DIGIPOTI_HPP
