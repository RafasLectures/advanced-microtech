/***************************************************************************//**
 * @file    i2c.cpp
 * @author  <your name>
 * @date    <date of creation>
 *
 * @brief   <brief description>
 *
 * Here goes a detailed description if required.
 ******************************************************************************/

#include "./i2c.hpp"

namespace AdvancedMicrotech {
/******************************************************************************
 * VARIABLES
 *****************************************************************************/


/******************************************************************************
 * LOCAL FUNCTION PROTOTYPES
 *****************************************************************************/



/******************************************************************************
 * LOCAL FUNCTION IMPLEMENTATION
 *****************************************************************************/
void (*handleTxIsrFunc)(void);
void (*handleRxIsrFunc)(void);


/******************************************************************************
 * FUNCTION IMPLEMENTATION
 *****************************************************************************/

#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void) {
    handleTxIsrFunc();
}

#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void) {
    handleRxIsrFunc();
}
}
#if 0
// TODO: Implement these functions:

void i2c_init (unsigned char addr) {

}

unsigned char i2c_write(unsigned char length, unsigned char * txData, unsigned char stop) {
	// Before writing, you should always check if the last STOP-condition has already been sent.
	while (UCB0CTL1 & UCTXSTP);


	// Wait for transfer to be finished.
	// Info: In TI's sample code, low-power mode statements are inserted,
	// also waiting for the transfer to be finished.
	while(!transferFinished) {}
}

void i2c_read(unsigned char length, unsigned char * rxData) {
	// Before writing, you should always check if the last STOP-condition has already been sent.
	while (UCB0CTL1 & UCTXSTP);


	if (length == 1) {
		// Todo: If you only want to receive one byte, you instantly have to write a STOP-condition
		// after the START-condition got sent.
	}

	// Wait for transfer to be finished.
	// Info: In TI's sample code, low-power mode statements are inserted,
	// also waiting for the transfer to be finished.
	while(!transferFinished);
}


#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void) {
	// TODO: Read RX-Buffer or write TX-Buffer, depending on what you'd like to do.

    // Exit waiting mode after this interrupt, i.e. set the transferFinished variable.
	// TODO: Call this only when necessary
	transferFinished = 1;
}

#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void) {
    // If there is a NACK, try to transmit again.
    if (*USCI::STAT & UCNACKIFG) {
        *USCI::CTL1 |= UCTXSTT;
        *USCI::STAT &= ~UCNACKIFG;
    }
}
#endif
