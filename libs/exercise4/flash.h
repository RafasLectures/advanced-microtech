/***************************************************************************//**
 * @file    flash.h
 * @author  <your name>
 * @date    <date of creation>
 *
 * @brief   <brief description>
 *
 * Here goes a detailed description if required.
 ******************************************************************************/

#ifndef LIBS_FLASH_H_
#define LIBS_FLASH_H_

/******************************************************************************
 * INCLUDES
 *****************************************************************************/

#include "./spi.h"

/******************************************************************************
 * CONSTANTS
 *****************************************************************************/



/******************************************************************************
 * VARIABLES
 *****************************************************************************/



/******************************************************************************
 * FUNCTION PROTOTYPES
 *****************************************************************************/

// Initialise the flash chip (in case you need it, else leave this function
// empty).
void flash_init(void);

// Read <length> bytes into <rxData> starting from address <address> (1 pt.)
void flash_read(long int address, unsigned char length, unsigned char * rxData);

// Write <length> bytes from <txData>, starting at address <address> (1 pt.)
void flash_write(long int address, unsigned char length, unsigned char * txData);

// Returns 1 if the FLASH is busy or 0 if not.
// Note: this is optional. You will probably need this, but you don't have to
// implement this if you solve it differently.
unsigned char flash_busy(void);

#endif /* LIBS_FLASH_H_ */
