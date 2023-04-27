#ifndef MICROTECH_IQMATHLIB_H
#define MICROTECH_IQMATHLIB_H

#include <cmath>

typedef double _iq15;
#define _IQ15(X) _iq15(X)

constexpr _iq15 _IQ15mpy(const _iq15 val1, const _iq15 val2) {
  return val1 * val2;
}

constexpr _iq15 _IQ15div(const _iq15 val1, const _iq15 val2) {
  return val1/val2;
}

constexpr _iq15 _IQ15sin(const _iq15 phase) {
  return std::sin(phase);
}



#endif  // MICROTECH_IQMATHLIB_H
