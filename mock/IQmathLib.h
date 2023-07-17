#ifndef MICROTECH_IQMATHLIB_H
#define MICROTECH_IQMATHLIB_H

#include <cstdint>
#include <cmath>

typedef double _iq15;
typedef double _iq;
#define _IQ15(X) _iq15(X)
#define _IQ(X) _iq(X)

constexpr _iq15 _IQ15mpy(const _iq15 val1, const _iq15 val2) {
  return val1 * val2;
}
constexpr _iq _IQmpy(const _iq val1, const _iq val2) {
  return val1 * val2;
}

constexpr _iq15 _IQ15div(const _iq15 val1, const _iq15 val2) {
  return val1/val2;
}
constexpr _iq _IQdiv(const _iq val1, const _iq val2) {
  return val1/val2;
}

constexpr _iq15 _IQ15sin(const _iq15 phase) {
  return std::sin(phase);
}

constexpr _iq _IQsin(const _iq phase) {
  return std::sin(phase);
}

constexpr _iq15 _IQ15int(const _iq15 val) {
  return (int16_t)(val);
}

constexpr _iq _IQint(const _iq val) {
  return (int16_t)(val);
}



#endif  // MICROTECH_IQMATHLIB_H
