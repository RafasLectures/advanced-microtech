/*******************************************************************************
 * @file    clocks.cpp
 * @author  ekoeppen
 * @date    25.12.2014
 *
 * @brief   Implementation file that implements delays. Implementation taken from:
 *          https://github.com/ekoeppen/msp430-template-library/blob/master/clocks.h
 ******************************************************************************/

#include "clocks.hpp"

void delay_ms(uint16_t milliseconds) {
  uint8_t m;
  switch (BCSCTL1 & 0b00001111) {
    case 6: m = 1; break;
    case 13: m = 8; break;
    case 14: m = 12; break;
    default: m = 16; break;
  }
  while (milliseconds--) {
    for (uint16_t i = 0; i < m; i++) __delay_cycles(1000);
  }
}

void delay_us(int16_t microseconds) {
  switch (BCSCTL1 & 0b00001111) {
    case 6:
      while (microseconds--) __delay_cycles(1);
      break;
    case 13:
      while (microseconds--) __delay_cycles(8);
      break;
    case 14:
      while (microseconds--) __delay_cycles(12);
      break;
    default:
      while (microseconds--) __delay_cycles(16);
      break;
  }
}
