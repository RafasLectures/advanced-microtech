/***************************************************************************//**
 * @file    templateEMP.c
 * @author  Sebastian Sester, Marc Schink & Sebastian Stoecklin
 * @date    April 19, 2018
 *
 * @brief   A basic library for MSP430 init and serial interface.
 *
 * Include in your main.c and call the initMSP() function to start
 * the controller with a 1 MHz clock and to initialize an interrupt
 * triggered serial communication.
 * If you want to implement UART on your own, please write
 * #define NO_TEMPLATE_UART 1
 * right before you include this file.
 * If you want to implement the ISR on your own, but still want to keep the
 * other functions, please write
 * #define NO_TEMPLATE_ISR 1
 * right before you include this file.
 ******************************************************************************/

/***************************************************************************//**
 * @file    templateEMP.c
 * @author  Sebastian Sester, Marc Schink & Sebastian Stoecklin
 * @date    April 19, 2018
 *
 * @brief   A basic library for MSP430 init and serial interface.
 *
 * Include in your main.c and call the initMSP() function to start
 * the controller with a 1 MHz clock and to initialize an interrupt
 * triggered serial communication.
 * If you want to implement UART on your own, please write
 * #define NO_TEMPLATE_UART 1
 * right before you include this file.
 * If you want to implement the ISR on your own, but still want to keep the
 * other functions, please write
 * #define NO_TEMPLATE_ISR 1
 * right before you include this file.
 ******************************************************************************/


/******************************************************************************
 * INCLUDES
 *****************************************************************************/

/* ============================================================================ */
/* Copyright (c) 2020, Texas Instruments Incorporated                           */
/*  All rights reserved.                                                        */
/*                                                                              */
/*  Redistribution and use in source and binary forms, with or without          */
/*  modification, are permitted provided that the following conditions          */
/*  are met:                                                                    */
/*                                                                              */
/*  *  Redistributions of source code must retain the above copyright           */
/*     notice, this list of conditions and the following disclaimer.            */
/*                                                                              */
/*  *  Redistributions in binary form must reproduce the above copyright        */
/*     notice, this list of conditions and the following disclaimer in the      */
/*     documentation and/or other materials provided with the distribution.     */
/*                                                                              */
/*  *  Neither the name of Texas Instruments Incorporated nor the names of      */
/*     its contributors may be used to endorse or promote products derived      */
/*     from this software without specific prior written permission.            */
/*                                                                              */
/*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" */
/*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,       */
/*  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR      */
/*  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR            */
/*  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,       */
/*  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,         */
/*  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; */
/*  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,    */
/*  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR     */
/*  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,              */
/*  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                          */
/* ============================================================================ */

/********************************************************************
*
* Standard register and bit definitions for the Texas Instruments
* MSP430 microcontroller.
*
* This file supports assembler and C development for
* MSP430G2553 devices.
*
* Texas Instruments, Version 1.2
*
* Rev. 1.0, Setup
* Rev. 1.1, added additional Cal Data Labels
* Rev. 1.2, added dummy TRAPINT_VECTOR interrupt vector as bugfix for USCI29
*
********************************************************************/



extern "C" {


/*----------------------------------------------------------------------------*/
/* PERIPHERAL FILE MAP                                                        */
/*----------------------------------------------------------------------------*/

/* External references resolved by a device-specific linker command file */


/************************************************************
* STANDARD BITS
************************************************************/


/************************************************************
* STATUS REGISTER BITS
************************************************************/


/* Low Power Modes coded with Bits 4-7 in SR */


/* ============================================================================ */
/* Copyright (c) 2013, Texas Instruments Incorporated                           */
/*  All rights reserved.                                                        */
/*                                                                              */
/*  Redistribution and use in source and binary forms, with or without          */
/*  modification, are permitted provided that the following conditions          */
/*  are met:                                                                    */
/*                                                                              */
/*  *  Redistributions of source code must retain the above copyright           */
/*     notice, this list of conditions and the following disclaimer.            */
/*                                                                              */
/*  *  Redistributions in binary form must reproduce the above copyright        */
/*     notice, this list of conditions and the following disclaimer in the      */
/*     documentation and/or other materials provided with the distribution.     */
/*                                                                              */
/*  *  Neither the name of Texas Instruments Incorporated nor the names of      */
/*     its contributors may be used to endorse or promote products derived      */
/*     from this software without specific prior written permission.            */
/*                                                                              */
/*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" */
/*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,       */
/*  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR      */
/*  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR            */
/*  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,       */
/*  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,         */
/*  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; */
/*  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,    */
/*  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR     */
/*  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,              */
/*  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                          */
/* ============================================================================ */

/*----------------------------------------------------------------------------*/
/* INTRINSIC MAPPING FOR IAR V1.XX                                            */
/*----------------------------------------------------------------------------*/


/*****************************************************************************/
/*  INTRINSICS.H                                                             */
/*                                                                           */
/* Copyright (c) 2005 Texas Instruments Incorporated                         */
/* http://www.ti.com/                                                        */
/*                                                                           */
/*  Redistribution and  use in source  and binary forms, with  or without    */
/*  modification,  are permitted provided  that the  following conditions    */
/*  are met:                                                                 */
/*                                                                           */
/*     Redistributions  of source  code must  retain the  above copyright    */
/*     notice, this list of conditions and the following disclaimer.         */
/*                                                                           */
/*     Redistributions in binary form  must reproduce the above copyright    */
/*     notice, this  list of conditions  and the following  disclaimer in    */
/*     the  documentation  and/or   other  materials  provided  with  the    */
/*     distribution.                                                         */
/*                                                                           */
/*     Neither the  name of Texas Instruments Incorporated  nor the names    */
/*     of its  contributors may  be used to  endorse or  promote products    */
/*     derived  from   this  software  without   specific  prior  written    */
/*     permission.                                                           */
/*                                                                           */
/*  THIS SOFTWARE  IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS    */
/*  "AS IS"  AND ANY  EXPRESS OR IMPLIED  WARRANTIES, INCLUDING,  BUT NOT    */
/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR    */
/*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT    */
/*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,    */
/*  SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL  DAMAGES  (INCLUDING, BUT  NOT    */
/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,    */
/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    */
/*  THEORY OF  LIABILITY, WHETHER IN CONTRACT, STRICT  LIABILITY, OR TORT    */
/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE    */
/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.     */
/*                                                                           */
/*****************************************************************************/

extern "C"
{

/*---------------------------------------------------------------------------*/
/* Handle legacy conflicts                                                   */
/*---------------------------------------------------------------------------*/
/*****************************************************************************/
/*  INTRINSICS_LEGACY_UNDEFS.H                                               */
/*                                                                           */
/* Copyright (c) 2005 Texas Instruments Incorporated                         */
/* http://www.ti.com/                                                        */
/*                                                                           */
/*  Redistribution and  use in source  and binary forms, with  or without    */
/*  modification,  are permitted provided  that the  following conditions    */
/*  are met:                                                                 */
/*                                                                           */
/*     Redistributions  of source  code must  retain the  above copyright    */
/*     notice, this list of conditions and the following disclaimer.         */
/*                                                                           */
/*     Redistributions in binary form  must reproduce the above copyright    */
/*     notice, this  list of conditions  and the following  disclaimer in    */
/*     the  documentation  and/or   other  materials  provided  with  the    */
/*     distribution.                                                         */
/*                                                                           */
/*     Neither the  name of Texas Instruments Incorporated  nor the names    */
/*     of its  contributors may  be used to  endorse or  promote products    */
/*     derived  from   this  software  without   specific  prior  written    */
/*     permission.                                                           */
/*                                                                           */
/*  THIS SOFTWARE  IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS    */
/*  "AS IS"  AND ANY  EXPRESS OR IMPLIED  WARRANTIES, INCLUDING,  BUT NOT    */
/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR    */
/*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT    */
/*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,    */
/*  SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL  DAMAGES  (INCLUDING, BUT  NOT    */
/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,    */
/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    */
/*  THEORY OF  LIABILITY, WHETHER IN CONTRACT, STRICT  LIABILITY, OR TORT    */
/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE    */
/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.     */
/*                                                                           */
/*****************************************************************************/

extern "C"
{

/*---------------------------------------------------------------------------*/
/* Handle in430.h conflicts with legacy intrinsic names                      */
/*---------------------------------------------------------------------------*/

} /* extern "C" */


/*---------------------------------------------------------------------------*/
/* General MSP Intrinsics                                                    */
/*---------------------------------------------------------------------------*/
void           __no_operation(void);

unsigned short __bic_SR_register         (unsigned short mask);
unsigned short __bic_SR_register_on_exit (unsigned short mask);
unsigned short __bis_SR_register         (unsigned short mask);
unsigned short __bis_SR_register_on_exit (unsigned short mask);
unsigned short __get_SR_register         (void);
unsigned short __get_SR_register_on_exit (void);

unsigned short __get_SP_register(void);
void           __set_SP_register(unsigned short value);

void           __delay_cycles(unsigned long cycles);

unsigned int   __even_in_range(unsigned int val, unsigned int range);

void           __op_code(unsigned short op);

/*---------------------------------------------------------------------------*/
/* General MSP Macros                                                        */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* MSP430/430X Intrinsics                                                    */
/*---------------------------------------------------------------------------*/
void             __disable_interrupt(void);
void             __enable_interrupt(void);
void             __set_interrupt_state(unsigned short state);

unsigned short   __get_R4_register(void);
void             __set_R4_register(unsigned short value);
unsigned short   __get_R5_register(void);
void             __set_R5_register(unsigned short value);

unsigned short   __bcd_add_short(unsigned short, unsigned short);
unsigned long    __bcd_add_long(unsigned long, unsigned long);

unsigned short   __swap_bytes(unsigned short a);

/*---------------------------------------------------------------------------*/
/* MSP430/430X Macros                                                        */
/*---------------------------------------------------------------------------*/




/*---------------------------------------------------------------------------*/
/* Legacy Macros                                                             */
/*---------------------------------------------------------------------------*/

} /* extern "C" */



/************************************************************
* PERIPHERAL FILE MAP
************************************************************/

/************************************************************
* SPECIAL FUNCTION REGISTER ADDRESSES + CONTROL BITS
************************************************************/

extern volatile unsigned char IE1;                                /* Interrupt Enable 1 */

extern volatile unsigned char IFG1;                               /* Interrupt Flag 1 */

extern volatile unsigned char IE2;                                /* Interrupt Enable 2 */

extern volatile unsigned char IFG2;                               /* Interrupt Flag 2 */

/************************************************************
* ADC10
************************************************************/

extern volatile unsigned char ADC10DTC0;                          /* ADC10 Data Transfer Control 0 */
extern volatile unsigned char ADC10DTC1;                          /* ADC10 Data Transfer Control 1 */
extern volatile unsigned char ADC10AE0;                           /* ADC10 Analog Enable 0 */

extern volatile unsigned int ADC10CTL0;                         /* ADC10 Control 0 */
extern volatile unsigned int ADC10CTL1;                         /* ADC10 Control 1 */
extern volatile unsigned int ADC10MEM;                          /* ADC10 Memory */
extern volatile unsigned int ADC10SA;                           /* ADC10 Data Transfer Start Address */

/* ADC10CTL0 */


/* ADC10CTL1 */






/* ADC10DTC0 */

/************************************************************
* Basic Clock Module
************************************************************/

extern volatile unsigned char DCOCTL;                             /* DCO Clock Frequency Control */
extern volatile unsigned char BCSCTL1;                            /* Basic Clock System Control 1 */
extern volatile unsigned char BCSCTL2;                            /* Basic Clock System Control 2 */
extern volatile unsigned char BCSCTL3;                            /* Basic Clock System Control 3 */












/************************************************************
* Comparator A
************************************************************/

extern volatile unsigned char CACTL1;                             /* Comparator A Control 1 */
extern volatile unsigned char CACTL2;                             /* Comparator A Control 2 */
extern volatile unsigned char CAPD;                               /* Comparator A Port Disable */





/*************************************************************
* Flash Memory
*************************************************************/

extern volatile unsigned int FCTL1;                             /* FLASH Control 1 */
extern volatile unsigned int FCTL2;                             /* FLASH Control 2 */
extern volatile unsigned int FCTL3;                             /* FLASH Control 3 */






/************************************************************
* DIGITAL I/O Port1/2 Pull up / Pull down Resistors
************************************************************/


extern volatile unsigned char P1IN;                               /* Port 1 Input */
extern volatile unsigned char P1OUT;                              /* Port 1 Output */
extern volatile unsigned char P1DIR;                              /* Port 1 Direction */
extern volatile unsigned char P1IFG;                              /* Port 1 Interrupt Flag */
extern volatile unsigned char P1IES;                              /* Port 1 Interrupt Edge Select */
extern volatile unsigned char P1IE;                               /* Port 1 Interrupt Enable */
extern volatile unsigned char P1SEL;                              /* Port 1 Selection */
extern volatile unsigned char P1SEL2;                             /* Port 1 Selection 2 */
extern volatile unsigned char P1REN;                              /* Port 1 Resistor Enable */

extern volatile unsigned char P2IN;                               /* Port 2 Input */
extern volatile unsigned char P2OUT;                              /* Port 2 Output */
extern volatile unsigned char P2DIR;                              /* Port 2 Direction */
extern volatile unsigned char P2IFG;                              /* Port 2 Interrupt Flag */
extern volatile unsigned char P2IES;                              /* Port 2 Interrupt Edge Select */
extern volatile unsigned char P2IE;                               /* Port 2 Interrupt Enable */
extern volatile unsigned char P2SEL;                              /* Port 2 Selection */
extern volatile unsigned char P2SEL2;                             /* Port 2 Selection 2 */
extern volatile unsigned char P2REN;                              /* Port 2 Resistor Enable */

/************************************************************
* DIGITAL I/O Port3 Pull up / Pull down Resistors
************************************************************/


extern volatile unsigned char P3IN;                               /* Port 3 Input */
extern volatile unsigned char P3OUT;                              /* Port 3 Output */
extern volatile unsigned char P3DIR;                              /* Port 3 Direction */
extern volatile unsigned char P3SEL;                              /* Port 3 Selection */
extern volatile unsigned char P3SEL2;                             /* Port 3 Selection 2 */
extern volatile unsigned char P3REN;                              /* Port 3 Resistor Enable */

/************************************************************
* Timer0_A3
************************************************************/

extern volatile unsigned int TA0IV;                             /* Timer0_A3 Interrupt Vector Word */
extern volatile unsigned int TA0CTL;                            /* Timer0_A3 Control */
extern volatile unsigned int TA0CCTL0;                          /* Timer0_A3 Capture/Compare Control 0 */
extern volatile unsigned int TA0CCTL1;                          /* Timer0_A3 Capture/Compare Control 1 */
extern volatile unsigned int TA0CCTL2;                          /* Timer0_A3 Capture/Compare Control 2 */
extern volatile unsigned int TA0R;                              /* Timer0_A3 Counter Register */
extern volatile unsigned int TA0CCR0;                           /* Timer0_A3 Capture/Compare 0 */
extern volatile unsigned int TA0CCR1;                           /* Timer0_A3 Capture/Compare 1 */
extern volatile unsigned int TA0CCR2;                           /* Timer0_A3 Capture/Compare 2 */

/* Alternate register names */

/* Alternate register names 2 */





/* T0_A3IV Definitions */

/************************************************************
* Timer1_A3
************************************************************/

extern volatile unsigned int TA1IV;                             /* Timer1_A3 Interrupt Vector Word */
extern volatile unsigned int TA1CTL;                            /* Timer1_A3 Control */
extern volatile unsigned int TA1CCTL0;                          /* Timer1_A3 Capture/Compare Control 0 */
extern volatile unsigned int TA1CCTL1;                          /* Timer1_A3 Capture/Compare Control 1 */
extern volatile unsigned int TA1CCTL2;                          /* Timer1_A3 Capture/Compare Control 2 */
extern volatile unsigned int TA1R;                              /* Timer1_A3 Counter Register */
extern volatile unsigned int TA1CCR0;                           /* Timer1_A3 Capture/Compare 0 */
extern volatile unsigned int TA1CCR1;                           /* Timer1_A3 Capture/Compare 1 */
extern volatile unsigned int TA1CCR2;                           /* Timer1_A3 Capture/Compare 2 */

/* Bits are already defined within the Timer0_Ax */

/* T1_A3IV Definitions */

/************************************************************
* USCI
************************************************************/

extern volatile unsigned char UCA0CTL0;                           /* USCI A0 Control Register 0 */
extern volatile unsigned char UCA0CTL1;                           /* USCI A0 Control Register 1 */
extern volatile unsigned char UCA0BR0;                            /* USCI A0 Baud Rate 0 */
extern volatile unsigned char UCA0BR1;                            /* USCI A0 Baud Rate 1 */
extern volatile unsigned char UCA0MCTL;                           /* USCI A0 Modulation Control */
extern volatile unsigned char UCA0STAT;                           /* USCI A0 Status Register */
extern volatile unsigned char UCA0RXBUF;                          /* USCI A0 Receive Buffer */
extern volatile unsigned char UCA0TXBUF;                          /* USCI A0 Transmit Buffer */
extern volatile unsigned char UCA0ABCTL;                          /* USCI A0 LIN Control */
extern volatile unsigned char UCA0IRTCTL;                         /* USCI A0 IrDA Transmit Control */
extern volatile unsigned char UCA0IRRCTL;                         /* USCI A0 IrDA Receive Control */



extern volatile unsigned char UCB0CTL0;                           /* USCI B0 Control Register 0 */
extern volatile unsigned char UCB0CTL1;                           /* USCI B0 Control Register 1 */
extern volatile unsigned char UCB0BR0;                            /* USCI B0 Baud Rate 0 */
extern volatile unsigned char UCB0BR1;                            /* USCI B0 Baud Rate 1 */
extern volatile unsigned char UCB0I2CIE;                          /* USCI B0 I2C Interrupt Enable Register */
extern volatile unsigned char UCB0STAT;                           /* USCI B0 Status Register */
extern volatile unsigned char UCB0RXBUF;                          /* USCI B0 Receive Buffer */
extern volatile unsigned char UCB0TXBUF;                          /* USCI B0 Transmit Buffer */
extern volatile unsigned int UCB0I2COA;                         /* USCI B0 I2C Own Address */
extern volatile unsigned int UCB0I2CSA;                         /* USCI B0 I2C Slave Address */

// UART-Mode Bits

// SPI-Mode Bits

// I2C-Mode Bits
//#define res               (0x10)    /* reserved */

// UART-Mode Bits

// SPI-Mode Bits
//#define res               (0x20)    /* reserved */
//#define res               (0x10)    /* reserved */
//#define res               (0x08)    /* reserved */
//#define res               (0x04)    /* reserved */
//#define res               (0x02)    /* reserved */

// I2C-Mode Bits
//#define res               (0x20)    /* reserved */





//#define res               (0x80)    /* reserved */
//#define res               (0x40)    /* reserved */
//#define res               (0x20)    /* reserved */
//#define res               (0x10)    /* reserved */




//#define res               (0x80)    /* reserved */
//#define res               (0x40)    /* reserved */
//#define res               (0x02)    /* reserved */



/************************************************************
* WATCHDOG TIMER
************************************************************/

extern volatile unsigned int WDTCTL;                            /* Watchdog Timer Control */
/* The bit names have been prefixed with "WDT" */


/* WDT-interval times [1ms] coded with Bits 0-2 */
/* WDT is clocked by fSMCLK (assumed 1MHz) */
/* WDT is clocked by fACLK (assumed 32KHz) */
/* Watchdog mode -> reset after expired time */
/* WDT is clocked by fSMCLK (assumed 1MHz) */
/* WDT is clocked by fACLK (assumed 32KHz) */

/* INTERRUPT CONTROL */
/* These two bits are defined in the Special Function Registers */
/* #define WDTIE               0x01 */
/* #define WDTIFG              0x01 */

/************************************************************
* Calibration Data in Info Mem
************************************************************/


extern volatile unsigned char CALDCO_16MHZ;                       /* DCOCTL  Calibration Data for 16MHz */
extern volatile unsigned char CALBC1_16MHZ;                       /* BCSCTL1 Calibration Data for 16MHz */
extern volatile unsigned char CALDCO_12MHZ;                       /* DCOCTL  Calibration Data for 12MHz */
extern volatile unsigned char CALBC1_12MHZ;                       /* BCSCTL1 Calibration Data for 12MHz */
extern volatile unsigned char CALDCO_8MHZ;                        /* DCOCTL  Calibration Data for 8MHz */
extern volatile unsigned char CALBC1_8MHZ;                        /* BCSCTL1 Calibration Data for 8MHz */
extern volatile unsigned char CALDCO_1MHZ;                        /* DCOCTL  Calibration Data for 1MHz */
extern volatile unsigned char CALBC1_1MHZ;                        /* BCSCTL1 Calibration Data for 1MHz */


/************************************************************
* Calibration Data in Info Mem
************************************************************/

/* TLV Calibration Data Structure */

extern volatile unsigned int TLV_CHECKSUM;                      /* TLV CHECK SUM */
extern volatile unsigned char TLV_DCO_30_TAG;                     /* TLV TAG_DCO30 TAG */
extern volatile unsigned char TLV_DCO_30_LEN;                     /* TLV TAG_DCO30 LEN */
extern volatile unsigned char TLV_ADC10_1_TAG;                    /* TLV ADC10_1 TAG */
extern volatile unsigned char TLV_ADC10_1_LEN;                    /* TLV ADC10_1 LEN */




/************************************************************
* Interrupt Vectors (offset from 0xFFE0)
************************************************************/



/************************************************************
* End of Modules
************************************************************/

}



extern "C" {
/******************************************************************************
 * CONSTANTS
 *****************************************************************************/


/******************************************************************************
 * VARIABLES
 *****************************************************************************/


/******************************************************************************
 * FUNCTION PROTOTYPES
 *****************************************************************************/

/**
 * serialEchoBack
 * This determines if the user's input should be echoed back or not.
 *
 * @param e   0 if no echo is required, anything else if it is.
 */
void serialEchoBack(char e);

/**
 * This function can be used to check for an buffer-error such as a buffer
 * overflow. Calling this function will also reset the error-variable.
 *
 * @return 0 if there is no error, anything elese if there is one.
 */
char serialError(void);

/**
 * Echo one character to the serial connection. Please note that this
 * function will not work with UTF-8-characters so you should stick
 * to ANSI or ASCII.
 *
 * @param char The character to be displayed.
 */
void serialWrite(char tx);

/**
 * Print a given integer as a readable number to serial connection (using
 * the ASCII charmap).
 *
 * @param i   The number to be displayed; 16 bit max.
 */
void serialPrintInt(int i);

/**
 * Print a sequence of characters to the serial connection.
 *
 * @example     serialPrint("output");
 * @param tx    A pointer to the text that shall be printed. Has to be
 *              terminated by \0
 */
void serialPrint(char* tx);

/**
 * Print a sequence of characters to the serial connection and terminate
 * the string with a linebreak. (Note that you'll have to enable "Newline
 * at LF+CR" within HTerm - if you use HTerm.)
 *
 * @example     serialPrint("output");
 * @param tx    A pointer to the text that shall be printed. Has to be
 *              terminated by \0
 */
void serialPrintln(char* tx);

/**
 * Returns 1 if the serial buffer is not empty i.e. some data has been
 * received on the serial connection (e.g. by sending something with HTerm)
 *
 * @return 1 if there is data, 0 if not.
 */
char serialAvailable(void);

/**
 * Clear the serial buffer; all content will be lost.
 */
void serialFlush(void);

/**
 * Returns the first byte from the serial buffer without modifying the
 * same. Returns -1 if the buffer is empty.
 *
 * @return The first byte within the buffer or -1 if the buffer is empty.
 */
int serialPeek(void);

/**
 * Returns the first byte from the serial buffer and removes it from the
 * same. Returns -1 if the buffer is empty.
 *
 * @return The first byte within the buffer or -1 if the buffer is empty.
 */
int serialRead(void);

/**
 * Reads in a number from the serial interface, terminated by any
 * non-numeric character.
 *
 * WARNING: This is a *very basic* implementation and you might want to
 * write your own depending on your scenario and your needs.
 *
 * @return The read-in-number.
 */
int serialReadInt(void);


/**
 * Initialization of the controller
 * This function sets the clock, stops the watchdog and - if allowed -
 * initialize the USART-machine.
 */
void initMSP(void);

}

/******************************************************************************
 * VARIABLES
 *****************************************************************************/


// Buffer type definition:
typedef struct {
    char data[32];
    char start;
    char end;
    char error;
} Buffer_t;

// Ring buffer definition:
Buffer_t ringBuffer = { .start = 0, .end = 0, .error = 0, };

// Echo flag definition:
char echoBack = 0;


/******************************************************************************
 * LOCAL FUNCTION PROTOTYPES
 *****************************************************************************/

/******************************************************************************
 * LOCAL FUNCTION IMPLEMENTATION
 *****************************************************************************/

/******************************************************************************
 * FUNCTION IMPLEMENTATION
 *****************************************************************************/


void serialEchoBack(char e)
{
    /*
     * This is the ternary operator. If e is 0, echoBack will be set 0,
     * else it will be set to 1 (we only need this because you can pass
     * other values but 0 and 1 to the function).
     */

    echoBack = e ? 1 : 0;
}

char serialError()
{
    char r = ringBuffer.error;
    ringBuffer.error = 0;
    return r;
}

void serialWrite(char tx)
{
    /* Loop until the TX buffer is ready.*/
    while (!(IFG2 & (0x02)))
        ;
    /* Write the character into the TX-register */
    UCA0TXBUF = tx;
    /* And wait until it has been transmitted. */
    while (!(IFG2 & (0x02)))
        ;
}

void serialPrintInt(int i)
{
    int j = i;
    // If the number is between 10000 and 65535, print the 10000-
    // digit.
    if (j >= 10000)
    {
        serialWrite(0x30 + i / 10000);
    }
    // Remove the 10000-digit.
    i = i % 10000;
    // Print the 1000-digit, if the number bigger then 999.
    if (j >= 1000)
    {
        serialWrite(0x30 + i / 1000);
    }
    // Now remove the 1000-digit.
    i = i % 1000;
    // Print the 100-digit if the number is big enough ...
    if (j >= 100)
    {
        serialWrite(0x30 + i / 100);
    }
    // ... remove it ...
    i = i % 100;
    // ... same for 10-digit ...
    if (j >= 10)
    {
        serialWrite(0x30 + i / 10);
    }
    // ...
    i = i % 10;
    // Print the last digit, no matter how big the number is (so if the
    // number is 0, we'll just print that).
    serialWrite(0x30 + i / 1);
}

void serialPrint(char* tx)
{
    int b, i = 0;
    // Count the number of bytes we shall display.
    while (tx[i] != 0x00)
    {
        i++;
    }
    // Write each of the bytes we counted.
    for (b = 0; b < i; b++)
    {
        // We already implemented the "write-a-single-character"-function,
        // so we're going to use that function instead of implementing the
        // same stuff here again.
        serialWrite(tx[b]);
    }
}

void serialPrintln(char* tx)
{
    // We don't have to implement this again, just pass tx to the apropriate
    // function.
    serialPrint(tx);
    // Print \n
    serialWrite(0x0D);
    // Print \r
    serialWrite(0x0A);
}

char serialAvailable(void)
{
    // If the buffer's start is not the buffer's end, there's data (return 1)
    if (ringBuffer.start != ringBuffer.end)
    {
        return 1;
    }
    // Else there is none (return 0)
    return 0;
}

void serialFlush(void)
{
    // Set the buffer's start to the buffer's end.
    ringBuffer.start = ringBuffer.end;
}

int serialPeek(void)
{
    // If the buffer's start is the buffer's end, there's no data (return -1)
    if (ringBuffer.start == ringBuffer.end)
    {
        return -1;
    }
    // Return the first byte
    return ringBuffer.data[ringBuffer.start];
}

int serialRead(void)
{
    // If the buffer's start is the buffer's end, there's no data (return -1)
    if (ringBuffer.start == ringBuffer.end)
    {
        return -1;
    }
    // Save the first byte to a temporary variable, move the start-pointer
    char r = ringBuffer.data[ringBuffer.start++];
    ringBuffer.start %= 32;
    // and return the stored byte.
    return r;
}

int serialReadInt(void)
{
    int number = 0;
    char stop = 0;
    char negative = 0;
    // While we didn't meet any non-numeric character
    while (!stop)
    {
        // Wait for data
        while (!serialAvailable())
            ;
        // Read the character
        char letter = serialRead();
        // If it's a minus and this is the first figure, it's a negative number
        if (letter == '-' && number == 0)
        {
            negative = 1;
        }
        // If it's a number, add the it to the resulting number
        else if (letter >= '0' && letter <= '9')
        {
            number = number * 10 + (letter - '0');
        }
        // Stop the interpretation elsewise.
        else
        {
            stop = 1;
        }
    }
    if (negative)
    {
        return number * -1;
    }
    return number;
}

/**
 * The UART interrupt (aka. "Hey, we received something!")
 * You must not call this function directly, it's invoked by the controller
 * whenever some data is received on the serial connection.
 */
#pragma vector=(7 * 1u)
__interrupt void USCI0RX_ISR(void)
{
    // Store the received byte in the serial buffer. Since we're using a
    // ringbuffer, we have to make sure that we only use RXBUFFERSIZE bytes.
    ringBuffer.data[ringBuffer.end++] = UCA0RXBUF;
    ringBuffer.end %= 32;
    // If enabled, print the received data back to user.
    if (echoBack)
    {
        while (!(IFG2 & (0x02)))
            ;
        UCA0TXBUF = UCA0RXBUF;
    }
    // Check for an overflow and set the corresponding variable.
    if (ringBuffer.start == ringBuffer.end)
    {
        ringBuffer.error = 1;
    }
}

void initMSP(void)
{
    // Stop Watchdog Timer
    WDTCTL = (0x5A00) + (0x0080);
    // If the calibration constants were erased, stop here.
    if (CALBC1_1MHZ == 0xFF || CALDCO_1MHZ == 0xFF)
    {
        while (1)
            ;
    }

    // Set clock to 1 MHz. Please don't change this if you have to upload/share
    // your code during the lab.
    // Possible options: _1 _8 _12 _16. Don't forget to adapt UART if you
    // change this!
    BCSCTL1 = CALBC1_1MHZ;
    // Set DCO step + modulation
    DCOCTL = CALDCO_1MHZ;

    // Activate UART on 1.1 / 1.2
    // (fixed connection to PC, shows up there as COMx)
    P1SEL = (0x0002) + (0x0004);           // P1.1 = RXD, P1.2=TXD, set everything
    P1SEL2 = (0x0002) + (0x0004);           // else as a normal GPIO.
    UCA0CTL1 |= (0x80);           // Use the SMCLK
    UCA0BR0 = 104;                  // 9600 Baud at 1 MHz
    UCA0BR1 = 0;                    // 9600 Baud at 1 MHz
    UCA0MCTL = (0x02);              // Modulation UCBRSx = 1
    UCA0CTL1 &= ~(0x01);           // Initialize USCI state machine
    IE2 |= (0x01);                // Enable USCI_A0 RX interrupt

    // Now enable the global interrupts
    __enable_interrupt();

    // Boot-up message
    serialWrite(0x0C);
    serialPrintln("Launchpad booted.");
}
