/*******************************************************************************
 * @file    usci.hpp
 * @author  ekoeppen
 * @date    30.10.2015
 *
 * @brief   Header file that abstracts USCI registers of MSP430.
 *          It was taken from the URL below, but it had to be adapted.
 *          https://github.com/ekoeppen/msp430-template-library/blob/master/usci.h
 ******************************************************************************/
#ifndef LIBS_COMMON_USCI_HPP_
#define LIBS_COMMON_USCI_HPP_

#include <msp430g2553.h>
#include <cstdint>
#include <type_traits>

// Function pointers that are called by the interruptions.
// They are set in the initialize methods of the I2C.
extern void (*handleUSCIB0TxIsrFunc)(void);
extern void (*handleUSCIB0RxIsrFunc)(void);


enum USCI_MODULE { USCI_A, USCI_B };

static constexpr volatile uint8_t *usci_a_registers[][9] = {
  {&UCA0CTL0, &UCA0CTL1, &UCA0BR0, &UCA0BR1, &UCA0MCTL, &UCA0STAT, const_cast<uint8_t *>(&UCA0RXBUF), &UCA0TXBUF, 0},
#ifdef UCA1CTL0_
  {&UCA1CTL0, &UCA1CTL1, &UCA1BR0, &UCA1BR1, &UCA1MCTL, &UCA1STAT, const_cast<uint8_t *>(&UCA1RXBUF), &UCA1TXBUF, 0}
#endif
};

static constexpr volatile uint16_t *usci_a_registers_addr[][2] = {{0, 0},
#ifdef UCA1CTL0_
                                                                  {0, 0}
#endif
};

static constexpr volatile uint8_t *usci_b_registers[][9] = {
  {&UCB0CTL0, &UCB0CTL1, &UCB0BR0, &UCB0BR1, 0, &UCB0STAT, const_cast<uint8_t *>(&UCB0RXBUF), &UCB0TXBUF, &UCB0I2CIE},
#ifdef UCB1CTL0_
  {&UCB1CTL0, &UCB1CTL1, &UCB1BR0, &UCB1BR1, 0, &UCB1STAT, const_cast<uint8_t *>(&UCB1RXBUF), &UCB1TXBUF, &UCB1I2CIE}
#endif
};

static constexpr volatile uint16_t *usci_b_registers_addr[][2] = {{&UCB0I2COA, &UCB0I2CSA},
#ifdef UCB1CTL0_
                                                                  {&UCB1I2COA, &UCB1I2CSA}
#endif
};

template<typename TYPE, const USCI_MODULE MODULE, const int INSTANCE, int REG,
         typename std::enable_if<std::is_same<TYPE, uint8_t>::value, TYPE>::type = 0>
constexpr volatile TYPE *USCI_REGISTER() {
  return (MODULE == USCI_A ? usci_a_registers : usci_b_registers)[INSTANCE][REG];
}

template<typename TYPE, const USCI_MODULE MODULE, const int INSTANCE, int REG,
         typename std::enable_if<std::is_same<TYPE, uint16_t>::value, TYPE>::type = 0>
constexpr volatile TYPE *USCI_REGISTER() {
  return (MODULE == USCI_A ? usci_a_registers_addr : usci_b_registers_addr)[INSTANCE][REG];
}

template<const USCI_MODULE MODULE, const int INSTANCE>
struct USCI_T {
  static constexpr volatile uint8_t *CTL0 = USCI_REGISTER<uint8_t, MODULE, INSTANCE, 0>();
  static constexpr volatile uint8_t *CTL1 = USCI_REGISTER<uint8_t, MODULE, INSTANCE, 1>();
  static constexpr volatile uint8_t *BR0 = USCI_REGISTER<uint8_t, MODULE, INSTANCE, 2>();
  static constexpr volatile uint8_t *BR1 = USCI_REGISTER<uint8_t, MODULE, INSTANCE, 3>();
  static constexpr volatile uint8_t *MCTL = USCI_REGISTER<uint8_t, MODULE, INSTANCE, 4>();
  static constexpr volatile uint8_t *STAT = USCI_REGISTER<uint8_t, MODULE, INSTANCE, 5>();
  static constexpr volatile uint8_t *RXBUF = USCI_REGISTER<uint8_t, MODULE, INSTANCE, 6>();
  static constexpr volatile uint8_t *TXBUF = USCI_REGISTER<uint8_t, MODULE, INSTANCE, 7>();
  static constexpr volatile uint8_t *I2CIE = USCI_REGISTER<uint8_t, MODULE, INSTANCE, 8>();
  static constexpr volatile uint16_t *I2COA = USCI_REGISTER<uint16_t, MODULE, INSTANCE, 0>();
  static constexpr volatile uint16_t *I2CSA = USCI_REGISTER<uint16_t, MODULE, INSTANCE, 1>();

  static void (*spiTxISRFunction)(void);
  static void (*spiRxISRFunction)(void);
  static void (*i2cTxISRFunction)(void);
  static void (*i2cRxISRFunction)(void);

  static void set_spi_active(bool spiActive) {
    if(spiActive) {
      handleUSCIB0TxIsrFunc = spiTxISRFunction;
      handleUSCIB0RxIsrFunc = spiRxISRFunction;
    } else {
      handleUSCIB0TxIsrFunc = i2cTxISRFunction;
      handleUSCIB0RxIsrFunc = i2cRxISRFunction;
    }
  }

  static void set_i2c_active(bool i2cActive) {
    if(!i2cActive) {
      handleUSCIB0TxIsrFunc = spiTxISRFunction;
      handleUSCIB0RxIsrFunc = spiRxISRFunction;
    } else {
      handleUSCIB0TxIsrFunc = i2cTxISRFunction;
      handleUSCIB0RxIsrFunc = i2cRxISRFunction;
    }
  }

  static void enable_rx_irq(void) {
    IE2 |= (MODULE == USCI_A ? UCA0RXIE : UCB0RXIE);
  }

  static void enable_tx_irq(void) {
    IE2 |= (MODULE == USCI_A ? UCA0TXIE : UCB0TXIE);
  }

  static void enable_rx_tx_irq(void) {
    IE2 |= (MODULE == USCI_A ? UCA0TXIE | UCA0RXIE : UCB0TXIE | UCB0RXIE);
  }

  static void disable_tx_irq(void) {
    IE2 &= ~(MODULE == USCI_A ? UCA0TXIE : UCB0TXIE);
  }

  static void disable_rx_irq(void) {
    IE2 &= ~(MODULE == USCI_A ? UCA0RXIE : UCB0RXIE);
  }

  static void disable_rx_tx_irq(void) {
    IE2 &= ~(MODULE == USCI_A ? UCA0TXIE | UCA0RXIE : UCB0TXIE | UCB0RXIE);
  }

  static void clear_rx_irq(void) {
    IFG2 &= ~(MODULE == USCI_A ? UCA0RXIFG : UCB0RXIFG);
  }

  static void clear_tx_irq(void) {
    IFG2 &= ~(MODULE == USCI_A ? UCA0TXIFG : UCB0TXIFG);
  }

  static bool tx_irq_pending(void) {
    return (IFG2 & (MODULE == USCI_A ? UCA0TXIFG : UCB0TXIFG));
  }

  static bool rx_irq_pending(void) {
    return (IFG2 & (MODULE == USCI_A ? UCA0RXIFG : UCB0RXIFG));
  }
};

template<const USCI_MODULE MODULE, const int INSTANCE>
void (*USCI_T<MODULE, INSTANCE>::spiTxISRFunction)(void) = nullptr;
template<const USCI_MODULE MODULE, const int INSTANCE>
void (*USCI_T<MODULE, INSTANCE>::spiRxISRFunction)(void) = nullptr;
template<const USCI_MODULE MODULE, const int INSTANCE>
void (*USCI_T<MODULE, INSTANCE>::i2cTxISRFunction)(void) = nullptr;
template<const USCI_MODULE MODULE, const int INSTANCE>
void (*USCI_T<MODULE, INSTANCE>::i2cRxISRFunction)(void) = nullptr;

#endif /* LIBS_COMMON_USCI_HPP_ */
