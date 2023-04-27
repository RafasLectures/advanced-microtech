/***************************************************************************//**
 * @file    spi.h
 * @author  <your name>
 * @date    <date of creation>
 *
 * @brief   <brief description>
 *
 * Here goes a detailed description if required.
 ******************************************************************************/

#ifndef LIBS_SPI_H_
#define LIBS_SPI_H_

/******************************************************************************
 * INCLUDES
 *****************************************************************************/

#include <msp430g2553.h>

/******************************************************************************
 * CONSTANTS
 *****************************************************************************/



/******************************************************************************
 * VARIABLES
 *****************************************************************************/



/******************************************************************************
 * FUNCTION PROTOTYPES
 *****************************************************************************/

// Set the USCI-machine to SPI and switch the 74HCT4066 (1 pt.)
void spi_init(void);

// Read <length> bytes into <rxData> (1 pt.)
void spi_read(unsigned char length, unsigned char * rxData);

// Write <length> bytes from <txData> (1 pt.)
void spi_write(unsigned char length, unsigned char * txData);

// Interrupt service routines in your spi.c (1 pt.)

// Returns 1 if the SPI is still busy or 0 if not.
// Note: this is optional. You will most likely need this, but you don't have
// to implement or use this.
unsigned char spi_busy(void);

#endif /* LIBS_SPI_H_ */
