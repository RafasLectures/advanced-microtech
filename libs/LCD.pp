/***************************************************************************//**
 * @file    LCD.c
 * @author  <your name>
 * @date    <date of creation>
 *
 * @brief   <brief description>
 *
 * Here goes a detailed description if required.
 ******************************************************************************/

/*******************************************************************************
 * @file    LCD.h
 * @author  Rafael Andrioli Bauer
 * @date    <date of creation>
 *
 * @brief   Header file with the prototypes of the LCD C functions as well as the
 *          declaration of the LCD class.
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



// -*- C++ -*-
//===--------------------------- cstdint ----------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is dual licensed under the MIT and the University of Illinois Open
// Source Licenses. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//


/*
    cstdint synopsis

Macros:

    INT8_MIN
    INT16_MIN
    INT32_MIN
    INT64_MIN

    INT8_MAX
    INT16_MAX
    INT32_MAX
    INT64_MAX

    UINT8_MAX
    UINT16_MAX
    UINT32_MAX
    UINT64_MAX

    INT_LEAST8_MIN
    INT_LEAST16_MIN
    INT_LEAST32_MIN
    INT_LEAST64_MIN

    INT_LEAST8_MAX
    INT_LEAST16_MAX
    INT_LEAST32_MAX
    INT_LEAST64_MAX

    UINT_LEAST8_MAX
    UINT_LEAST16_MAX
    UINT_LEAST32_MAX
    UINT_LEAST64_MAX

    INT_FAST8_MIN
    INT_FAST16_MIN
    INT_FAST32_MIN
    INT_FAST64_MIN

    INT_FAST8_MAX
    INT_FAST16_MAX
    INT_FAST32_MAX
    INT_FAST64_MAX

    UINT_FAST8_MAX
    UINT_FAST16_MAX
    UINT_FAST32_MAX
    UINT_FAST64_MAX

    INTPTR_MIN
    INTPTR_MAX
    UINTPTR_MAX

    INTMAX_MIN
    INTMAX_MAX

    UINTMAX_MAX

    PTRDIFF_MIN
    PTRDIFF_MAX

    SIG_ATOMIC_MIN
    SIG_ATOMIC_MAX

    SIZE_MAX

    WCHAR_MIN
    WCHAR_MAX

    WINT_MIN
    WINT_MAX

    INT8_C(value)
    INT16_C(value)
    INT32_C(value)
    INT64_C(value)

    UINT8_C(value)
    UINT16_C(value)
    UINT32_C(value)
    UINT64_C(value)

    INTMAX_C(value)
    UINTMAX_C(value)

namespace std
{

Types:

    int8_t
    int16_t
    int32_t
    int64_t

    uint8_t
    uint16_t
    uint32_t
    uint64_t

    int_least8_t
    int_least16_t
    int_least32_t
    int_least64_t

    uint_least8_t
    uint_least16_t
    uint_least32_t
    uint_least64_t

    int_fast8_t
    int_fast16_t
    int_fast32_t
    int_fast64_t

    uint_fast8_t
    uint_fast16_t
    uint_fast32_t
    uint_fast64_t

    intptr_t
    uintptr_t

    intmax_t
    uintmax_t

}  // std
*/

/* -*- C++ -*- */
/*===--------------------------- complex.h --------------------------------===*/
/*                                                                            */
/*                     The LLVM Compiler Infrastructure                       */
/*                                                                            */
/* This file is dual licensed under the MIT and the University of Illinois Open
** Source Licenses. See LICENSE.TXT for details.
*/
/*===----------------------------------------------------------------------===*/


#pragma diag_push
/* Avoid warning on C++ comments in this file */
#pragma diag_suppress 2581
#pragma CHECK_MISRA("-2.2")
#pragma CHECK_MISRA("-19.4")
#pragma CHECK_MISRA("-19.10")

// The libc++ cmake build system expects to preinclude __config site during
// library builds (_LIBCPP_BUILDING_LIBRARY defined). Then, as part of
// installation, will prepend the contents of __config_site to __config and
// install the result as __config. __config_site does not exist as part of the
// cmake installation.
//
// The TI mkrts system follows the same behavior while bulding the library.
// However, it does not support prepending as part of installation, and so must
// have __config_site exist separately as a pre-generated file.
//
// To ensure that the cmake system still works, we only include __config_site
// when it exists as part of an installation. That is: If a TI compiler is
// being used, the library has been built/installed, and __config_site exists.
//===----------------------------------------------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is dual licensed under the MIT and the University of Illinois Open
// Source Licenses. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//


/* #undef _LIBCPP_ABI_UNSTABLE */
/* #undef _LIBCPP_HAS_NO_GLOBAL_FILESYSTEM_NAMESPACE */
/* #undef _LIBCPP_HAS_NO_STDIN */
/* #undef _LIBCPP_HAS_NO_STDOUT */
/* #undef _LIBCPP_HAS_NO_MONOTONIC_CLOCK */
/* #undef _LIBCPP_HAS_NO_THREAD_UNSAFE_C_FUNCTIONS */
/* #undef _LIBCPP_HAS_MUSL_LIBC */
/* #undef _LIBCPP_HAS_THREAD_API_PTHREAD */
/* #undef _LIBCPP_HAS_THREAD_API_EXTERNAL */
/* #undef _LIBCPP_HAS_THREAD_LIBRARY_EXTERNAL */

/*****************************************************************************/
/* LIBCXX_EXTRA.H                                                            */
/*                                                                           */
/* Copyright (c) 2017 Texas Instruments Incorporated                         */
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

/*
Changes made to this file affect how TI libc++ is BOTH built and used in
production environments.
*/

/*
The TI RTS has all source and header files flattened into a single directory.
*/

/* #pragma diag_suppress 1585,2866 */










// Change short string representation so that string data starts at offset 0,
// improving its alignment in some cases.
// Fix deque iterator type in order to support incomplete types.
// Fix undefined behavior in how std::list stores its linked nodes.
// Fix undefined behavior in  how __tree stores its end and parent nodes.
// Fix undefined behavior in how __hash_table stores its pointer types.
// Don't use a nullptr_t simulation type in C++03 instead using C++11 nullptr
// provided under the alternate keyword __nullptr, which changes the mangling
// of nullptr_t. This option is ABI incompatible with GCC in C++03 mode.
// Define the `pointer_safety` enum as a C++11 strongly typed enumeration
// instead of as a class simulating an enum. If this option is enabled
// `pointer_safety` and `get_pointer_safety()` will no longer be available
// in C++03.
// Define a key function for `bad_function_call` in the library, to centralize
// its vtable and typeinfo to libc++ rather than having all other libraries
// using that class define their own copies.

// Enable optimized version of __do_get_(un)signed which avoids redundant copies.





// '__is_identifier' returns '0' if '__x' is a reserved identifier provided by
// the compiler and '1' otherwise.





// FIXME: ABI detection should be done via compiler builtin macros. This
// is just a placeholder until Clang implements such macros. For now assume
// that Windows compilers pretending to be MSVC++ target the Microsoft ABI,
// and allow the user to explicitly specify the ABI to handle cases where this
// heuristic falls short.

// Need to detect which libc we're using if we're on Linux.












// Add a defined(__TARGET__) in the following check for each TARGET
// that supports LIBCPP atomics

// __builtin_strlen can be trivially replaced, but with a hefty runtime cost

// TI targets do not support aligned operator new()

// Currently a dummy value. std::strerror will return "Unknown" for errors that
// are out of the range of those we can print.




















// EDG supports __is_literal_type, which is analagous to __is_literal

// TI compilers using libc++ always accept inline namespaces

namespace std {
  inline namespace __2 {
  }
}


// Allow for build-time disabling of unsigned integer sanitization

// The TI compiler is strict about the difference between extern "C" and
// extern "C++" functions. One cannot be conflated with the other, even if
// the types are otherwise the same.















































// FIXME: Remove all usages of this macro once compilers catch up.



// Try to find out if RTTI is disabled.
// g++ and cl.exe have RTTI on by default and define a macro when it is.
// g++ only defines the macro in 4.3.2 and onwards.


// Thread API




// Systems that use capability-based security (FreeBSD with Capsicum,
// Nuxi CloudABI) may only provide local filesystem access (using *at()).
// Functions like open(), rename(), unlink() and stat() should not be
// used, as they attempt to access the global filesystem namespace.

// CloudABI is intended for running networked services. Processes do not
// have standard input and output channels.


// Thread-unsafe functions such as strtok() and localtime()
// are not available.


// TODO: Remove "&& !defined(__TI_COMPILER_VERSION__) when we enable threads.













// Decide whether to use availability macros.

// Define availability macros.

// Define availability that depends on _LIBCPP_NO_EXCEPTIONS.

// Availability of stream API in the dylib got dropped and re-added.  The
// extern template should effectively be available at:
//    availability(macosx,introduced=10.9)
//    availability(ios,introduced=7.0)


  // Don't warn about macro conflicts when we can restore them at the
  // end of the header.



#pragma diag_pop

/* -*- C++ -*- */
/*===--------------------------- complex.h --------------------------------===*/
/*                                                                            */
/*                     The LLVM Compiler Infrastructure                       */
/*                                                                            */
/* This file is dual licensed under the MIT and the University of Illinois Open
** Source Licenses. See LICENSE.TXT for details.
*/
/*===----------------------------------------------------------------------===*/


/*
    stdint.h synopsis

Macros:

    INT8_MIN
    INT16_MIN
    INT32_MIN
    INT64_MIN

    INT8_MAX
    INT16_MAX
    INT32_MAX
    INT64_MAX

    UINT8_MAX
    UINT16_MAX
    UINT32_MAX
    UINT64_MAX

    INT_LEAST8_MIN
    INT_LEAST16_MIN
    INT_LEAST32_MIN
    INT_LEAST64_MIN

    INT_LEAST8_MAX
    INT_LEAST16_MAX
    INT_LEAST32_MAX
    INT_LEAST64_MAX

    UINT_LEAST8_MAX
    UINT_LEAST16_MAX
    UINT_LEAST32_MAX
    UINT_LEAST64_MAX

    INT_FAST8_MIN
    INT_FAST16_MIN
    INT_FAST32_MIN
    INT_FAST64_MIN

    INT_FAST8_MAX
    INT_FAST16_MAX
    INT_FAST32_MAX
    INT_FAST64_MAX

    UINT_FAST8_MAX
    UINT_FAST16_MAX
    UINT_FAST32_MAX
    UINT_FAST64_MAX

    INTPTR_MIN
    INTPTR_MAX
    UINTPTR_MAX

    INTMAX_MIN
    INTMAX_MAX

    UINTMAX_MAX

    PTRDIFF_MIN
    PTRDIFF_MAX

    SIG_ATOMIC_MIN
    SIG_ATOMIC_MAX

    SIZE_MAX

    WCHAR_MIN
    WCHAR_MAX

    WINT_MIN
    WINT_MAX

    INT8_C(value)
    INT16_C(value)
    INT32_C(value)
    INT64_C(value)

    UINT8_C(value)
    UINT16_C(value)
    UINT32_C(value)
    UINT64_C(value)

    INTMAX_C(value)
    UINTMAX_C(value)

*/



/* C99 stdlib (e.g. glibc < 2.18) does not provide macros needed
   for C++11 unless __STDC_LIMIT_MACROS and __STDC_CONSTANT_MACROS
   are defined
*/

/*****************************************************************************/
/* STDINT.H                                                                  */
/*                                                                           */
/* Copyright (c) 2002 Texas Instruments Incorporated                         */
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

/*****************************************************************************/
/* _ti_config.h                                                              */
/*                                                                           */
/* Copyright (c) 2017 Texas Instruments Incorporated                         */
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


/*Unsupported pragmas are omitted */
# pragma diag_push
# pragma CHECK_MISRA("-19.7")
# pragma CHECK_MISRA("-19.4")
# pragma CHECK_MISRA("-19.1")
# pragma CHECK_MISRA("-19.15")
# pragma diag_pop

_Pragma("diag_push")
_Pragma("CHECK_MISRA(\"-19.4\")")
_Pragma("CHECK_MISRA(\"-19.1\")")
_Pragma("CHECK_MISRA(\"-19.6\")")

/* Hide uses of the TI proprietary macros behind other macros.
    Implementations that don't implement these features should leave
    these macros undefined. */



/* Common definitions */

/* C++ */
 /* C++11 */





/* _TI_NOEXCEPT_CPP14 is defined to noexcept only when compiling for C++14. It
   is intended to be used for functions like abort and atexit that are supposed
   to be declared noexcept only in C++14 mode. */



/* Target-specific definitions */
/*****************************************************************************/
/* linkage.h                                                                 */
/*                                                                           */
/* Copyright (c) 1998 Texas Instruments Incorporated                         */
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


#pragma diag_push
#pragma CHECK_MISRA("-19.4") /* macros required for implementation */

/* No modifiers are needed to access code or data */


/*--------------------------------------------------------------------------*/
/* Define _IDECL ==> how inline functions are declared                      */
/*--------------------------------------------------------------------------*/

#pragma diag_pop


_Pragma("diag_pop")


_Pragma("diag_push")
_Pragma("CHECK_MISRA(\"-19.1\")") /* no code before #include */
_Pragma("CHECK_MISRA(\"-19.7\")") /* prefer functions to macros */

/*****************************************************************************/
/* _STDINT40.H                                                               */
/*                                                                           */
/* Copyright (c) 2018 Texas Instruments Incorporated                         */
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

_Pragma("diag_push")
_Pragma("CHECK_MISRA(\"-19.7\")") /* prefer functions to macros */


/*
   According to footnotes in the 1999 C standard, "C++ implementations
   should define these macros only when __STDC_LIMIT_MACROS is defined
   before <stdint.h> is included."
*/



_Pragma("diag_pop")

/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2001 Mike Barcroft <mike@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */



/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 1991, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * Berkeley Software Design, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	@(#)cdefs.h	8.8 (Berkeley) 1/9/95
 * $FreeBSD$
 */



_Pragma("diag_push")
_Pragma("CHECK_MISRA(\"none\")")

/*
 * Testing against Clang-specific extensions.
 */


/*
 * This code has been put in place to help reduce the addition of
 * compiler specific defines in FreeBSD code.  It helps to aid in
 * having a compiler-agnostic source tree.
 */







/*
 * Macro to test if we're using a specific version of gcc or later.
 */

/*
 * The __CONCAT macro is used to concatenate parts of symbol names, e.g.
 * with "#define OLD(foo) __CONCAT(old,foo)", OLD(foo) produces oldfoo.
 * The __CONCAT macro is a bit tricky to use if it must work in non-ANSI
 * mode -- there must be no spaces between its arguments, and for nested
 * __CONCAT's, all the __CONCAT's must be at the left.  __CONCAT can also
 * concatenate double-quoted strings produced by the __STRING macro, but
 * this only works with ANSI C.
 *
 * __XSTRING is like __STRING, but it expands any macros in its argument
 * first.  It is only available with ANSI C.
 */



/*
 * Compiler-dependent macros to help declare dead (non-returning) and
 * pure (no side effects) functions, and unused variables.  They are
 * null except for versions of gcc that are known to support the features
 * properly (old versions of gcc-2 supported the dead and pure features
 * in a different (wrong) way).  If we do not provide an implementation
 * for a given compiler, let the compile fail if it is told to use
 * a feature that we cannot live without.
 */

/*
 * TI ADD - check that __GNUC__ is defined before referencing it to avoid
 *          generating an error when __GNUC__ treated as zero warning is
 *          promoted to an error via -pdse195 option.
 */



/*
 * Keywords added in C11.
 */







/*
 * XXX: Some compilers (Clang 3.3, GCC 4.7) falsely announce C++11 mode
 * without actually supporting the thread_local keyword. Don't check for
 * the presence of C++11 when defining _Thread_local.
 */


/*
 * Emulation of C11 _Generic().  Unlike the previously defined C11
 * keywords, it is not possible to implement this using exactly the same
 * syntax.  Therefore implement something similar under the name
 * __generic().  Unlike _Generic(), this macro can only distinguish
 * between a single type, so it requires nested invocations to
 * distinguish multiple cases.
 */


/*
 * C99 Static array indices in function parameter declarations.  Syntax such as:
 * void bar(int myArray[static 10]);
 * is allowed in C99 but not in C++.  Define __min_size appropriately so
 * headers using it can be compiled in either language.  Use like this:
 * void bar(int myArray[__min_size(10)]);
 */







/* XXX: should use `#if __STDC_VERSION__ < 199901'. */


/* C++11 exposes a load of C99 stuff */

/*
 * GCC 2.95 provides `__restrict' as an extension to C90 to support the
 * C99-specific `restrict' type qualifier.  We happen to use `__restrict' as
 * a way to define the `restrict' type qualifier without disturbing older
 * software that is unaware of C99 keywords.
 * The TI compiler supports __restrict in all compilation modes.
 */

/*
 * GNU C version 2.96 adds explicit branch prediction so that
 * the CPU back-end can hint the processor and also so that
 * code blocks can be reordered such that the predicted path
 * sees a more linear flow, thus improving cache behavior, etc.
 *
 * The following two macros provide us with a way to utilize this
 * compiler feature.  Use __predict_true() if you expect the expression
 * to evaluate to true, and __predict_false() if you expect the
 * expression to evaluate to false.
 *
 * A few notes about usage:
 *
 *	* Generally, __predict_false() error condition checks (unless
 *	  you have some _strong_ reason to do otherwise, in which case
 *	  document it), and/or __predict_true() `no-error' condition
 *	  checks, assuming you want to optimize for the no-error case.
 *
 *	* Other than that, if you don't know the likelihood of a test
 *	  succeeding from empirical or other `hard' evidence, don't
 *	  make predictions.
 *
 *	* These are meant to be used in places that are run `a lot'.
 *	  It is wasteful to make predictions in code that is run
 *	  seldomly (e.g. at subsystem initialization time) as the
 *	  basic block reordering that this affects can often generate
 *	  larger code.
 */


/*
 * We define this here since <stddef.h>, <sys/queue.h>, and <sys/types.h>
 * require it.
 */

/*
 * Given the pointer x to the member m of the struct s, return
 * a pointer to the containing structure.  When using GCC, we first
 * assign pointer x to a local variable, to check that its type is
 * compatible with member m.
 */

/*
 * Compiler-dependent macros to declare that functions take printf-like
 * or scanf-like arguments.  They are null except for versions of gcc
 * that are known to support the features properly (old versions of gcc-2
 * didn't permit keeping the keywords out of the application namespace).
 */

/* Compiler-dependent macros that rely on FreeBSD-specific extensions. */



/*
 * The following definition might not work well if used in header files,
 * but it should be better than nothing.  If you want a "do nothing"
 * version, then it should generate some harmless declaration, such as:
 *    #define	__IDSTRING(name,string)	struct __hack
 */


/*
 * Embed the rcs id of a source file in the resulting library.  Note that in
 * more recent ELF binutils, we use .ident allowing the ID to be stripped.
 * Usage:
 *	__FBSDID("$FreeBSD$");
 */








/*-
 * The following definitions are an extension of the behavior originally
 * implemented in <sys/_posix.h>, but with a different level of granularity.
 * POSIX.1 requires that the macros we test be defined before any standard
 * header file is included.
 *
 * Here's a quick run-down of the versions:
 *  defined(_POSIX_SOURCE)		1003.1-1988
 *  _POSIX_C_SOURCE == 1		1003.1-1990
 *  _POSIX_C_SOURCE == 2		1003.2-1992 C Language Binding Option
 *  _POSIX_C_SOURCE == 199309		1003.1b-1993
 *  _POSIX_C_SOURCE == 199506		1003.1c-1995, 1003.1i-1995,
 *					and the omnibus ISO/IEC 9945-1: 1996
 *  _POSIX_C_SOURCE == 200112		1003.1-2001
 *  _POSIX_C_SOURCE == 200809		1003.1-2008
 *
 * In addition, the X/Open Portability Guide, which is now the Single UNIX
 * Specification, defines a feature-test macro which indicates the version of
 * that specification, and which subsumes _POSIX_C_SOURCE.
 *
 * Our macros begin with two underscores to avoid namespace screwage.
 */

/* Deal with IEEE Std. 1003.1-1990, in which _POSIX_C_SOURCE == 1. */

/* Deal with IEEE Std. 1003.2-1992, in which _POSIX_C_SOURCE == 2. */

/* Deal with various X/Open Portability Guides and Single UNIX Spec. */

/*
 * Deal with all versions of POSIX.  The ordering relative to the tests above is
 * important.
 */
/*-
 * Deal with _ANSI_SOURCE:
 * If it is defined, and no other compilation environment is explicitly
 * requested, then define our internal feature-test macros to zero.  This
 * makes no difference to the preprocessor (undefined symbols in preprocessing
 * expressions are defined to have value zero), but makes it more convenient for
 * a test program to print out the values.
 *
 * If a program mistakenly defines _ANSI_SOURCE and some other macro such as
 * _POSIX_C_SOURCE, we will assume that it wants the broader compilation
 * environment (and in fact we will never get here).
 */

/* User override __EXT1_VISIBLE */


/*
 * Old versions of GCC use non-standard ARM arch symbols; acle-compat.h
 * translates them to __ARM_ARCH and the modern feature symbols defined by ARM.
 */

/*
 * Nullability qualifiers: currently only supported by Clang.
 */

/*
 * Type Safety Checking
 *
 * Clang provides additional attributes to enable checking type safety
 * properties that cannot be enforced by the C type system. 
 */


/*
 * Lock annotations.
 *
 * Clang provides support for doing basic thread-safety tests at
 * compile-time, by marking which locks will/should be held when
 * entering/leaving a functions.
 *
 * Furthermore, it is also possible to annotate variables and structure
 * members to enforce that they are only accessed when certain locks are
 * held.
 */


/* Structure implements a lock. */

/* Function acquires an exclusive or shared lock. */

/* Function attempts to acquire an exclusive or shared lock. */

/* Function releases a lock. */

/* Function asserts that an exclusive or shared lock is held. */

/* Function requires that an exclusive or shared lock is or is not held. */

/* Function should not be analyzed. */

/* Guard variables and structure members by lock. */

_Pragma("diag_pop")

/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2002 Mike Barcroft <mike@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */


/*****************************************************************************/
/*  _TYPES.H                                                                 */
/*                                                                           */
/* Copyright (c) 2017 Texas Instruments Incorporated                         */
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



#pragma diag_push
/* This file is required to use base types */
#pragma CHECK_MISRA("-6.3")

/*
 * Basic types upon which most other types are built.
 */
typedef	signed char		__int8_t;
typedef	unsigned char		__uint8_t;
typedef	int			__int16_t;
typedef	unsigned int		__uint16_t;
typedef	long			__int32_t;
typedef	unsigned long		__uint32_t;

/* LONGLONG */
typedef	long long		__int64_t;

/* LONGLONG */
typedef	unsigned long long	__uint64_t;

/*
 * Standard type definitions.
 */
typedef	__uint32_t	__clock_t;		/* clock()... */
typedef	__int32_t	__critical_t;
typedef	double		__double_t;
typedef	float		__float_t;
typedef	__int32_t	__intfptr_t;
typedef	__int64_t	__intmax_t;
typedef __int16_t       __intptr_t;

typedef	__int16_t	__int_fast8_t;
typedef	__int16_t	__int_fast16_t;
typedef	__int32_t	__int_fast32_t;
typedef	__int64_t	__int_fast64_t;
typedef	__int8_t	__int_least8_t;
typedef	__int16_t	__int_least16_t;
typedef	__int32_t	__int_least32_t;
typedef	__int64_t	__int_least64_t;
typedef	int __ptrdiff_t;		/* ptr1 - ptr2 */
typedef	__int16_t	__register_t;
typedef	__int32_t	__segsz_t;		/* segment size (in pages) */
typedef	unsigned	__size_t;		/* sizeof() */
typedef	__int32_t	__ssize_t;		/* byte count or error */
typedef __uint32_t      __time_t;
typedef	__uint32_t	__uintfptr_t;
typedef	__uint64_t	__uintmax_t;
typedef	__uint16_t	__uintptr_t;

typedef	__uint16_t	__uint_fast8_t;
typedef	__uint16_t	__uint_fast16_t;
typedef	__uint32_t	__uint_fast32_t;
typedef	__uint64_t	__uint_fast64_t;
typedef	__uint8_t	__uint_least8_t;
typedef	__uint16_t	__uint_least16_t;
typedef	__uint32_t	__uint_least32_t;
typedef	__uint64_t	__uint_least64_t;
typedef	__uint16_t	__u_register_t;
typedef	__uint32_t	__vm_offset_t;
typedef	__uint32_t	__vm_paddr_t;
typedef	__uint32_t	__vm_size_t;

typedef	unsigned int ___wchar_t;


/*
 * POSIX target specific _off_t type definition
 */
typedef long int _off_t;

/*
 * Unusual type definitions.
 */
typedef char* __va_list;

#pragma diag_pop


_Pragma("diag_push")
/* This file is required to use types without size and signedness */
_Pragma("CHECK_MISRA(\"-6.3\")")

/*
 * Standard type definitions.
 */
typedef	__int32_t	__blksize_t;	/* file block size */
typedef	__int64_t	__blkcnt_t;	/* file block count */
typedef	__int32_t	__clockid_t;	/* clock_gettime()... */
typedef	__uint32_t	__fflags_t;	/* file flags */
typedef	__uint64_t	__fsblkcnt_t;
typedef	__uint64_t	__fsfilcnt_t;
typedef	__uint32_t	__gid_t;
typedef	__int64_t	__id_t;		/* can hold a gid_t, pid_t, or uid_t */
typedef	__uint64_t	__ino_t;	/* inode number */
typedef	long		__key_t;	/* IPC key (for Sys V IPC) */
typedef	__int32_t	__lwpid_t;	/* Thread ID (a.k.a. LWP) */
typedef	__uint16_t	__mode_t;	/* permissions */
typedef	int		__accmode_t;	/* access permissions */
typedef	int		__nl_item;
typedef	__uint64_t	__nlink_t;	/* link count */
typedef	_off_t	        __off_t;	/* file offset (target-specific)  */
typedef	__int64_t	__off64_t;	/* file offset (always 64-bit)    */
typedef	__int32_t	__pid_t;	/* process [group] */
typedef	__int64_t	__rlim_t;	/* resource limit - intentionally */
					/* signed, because of legacy code */
					/* that uses -1 for RLIM_INFINITY */
typedef	__uint8_t	__sa_family_t;
typedef	__uint32_t	__socklen_t;
typedef	long		__suseconds_t;	/* microseconds (signed) */
typedef	struct __timer	*__timer_t;	/* timer_gettime()... */
typedef	struct __mq	*__mqd_t;	/* mq_open()... */
typedef	__uint32_t	__uid_t;
typedef	unsigned int	__useconds_t;	/* microseconds (unsigned) */
typedef	int		__cpuwhich_t;	/* which parameter for cpuset. */
typedef	int		__cpulevel_t;	/* level parameter for cpuset. */
typedef int		__cpusetid_t;	/* cpuset identifier. */

/*
 * Unusual type definitions.
 */
/*
 * rune_t is declared to be an ``int'' instead of the more natural
 * ``unsigned long'' or ``long''.  Two things are happening here.  It is not
 * unsigned so that EOF (-1) can be naturally assigned to it and used.  Also,
 * it looks like 10646 will be a 31 bit standard.  This means that if your
 * ints cannot hold 32 bits, you will be in trouble.  The reason an int was
 * chosen over a long is that the is*() and to*() routines take ints (says
 * ANSI C), but they use __ct_rune_t instead of int.
 *
 * NOTE: rune_t is not covered by ANSI nor other standards, and should not
 * be instantiated outside of lib/libc/locale.  Use wchar_t.  wint_t and
 * rune_t must be the same type.  Also, wint_t should be able to hold all
 * members of the largest character set plus one extra value (WEOF), and
 * must be at least 16 bits.
 */
typedef	int		__ct_rune_t;	/* arg type for ctype funcs */

typedef	__ct_rune_t	__rune_t;	/* rune_t (see above) */
typedef	__ct_rune_t	__wint_t;	/* wint_t (see above) */

/* Clang already provides these types as built-ins, but only in C++ mode. */
typedef	__uint_least16_t __char16_t;
typedef	__uint_least32_t __char32_t;
/* In C++11, char16_t and char32_t are built-in types. */

typedef struct {
	long long __max_align1 __attribute__((aligned(alignof(long long))));
	long double __max_align2 __attribute__((aligned(alignof(long double))));
} __max_align_t;

typedef	__uint64_t	__dev_t;	/* device number */

typedef	__uint32_t	__fixpt_t;	/* fixed point number */

/*
 * mbstate_t is an opaque object to keep conversion state during multibyte
 * stream conversions.
 */

typedef int _Mbstatet;

typedef _Mbstatet __mbstate_t;

typedef __uintmax_t     __rman_res_t;

/*
 * When the following macro is defined, the system uses 64-bit inode numbers.
 * Programs can use this to avoid including <sys/param.h>, with its associated
 * namespace pollution.
 */

_Pragma("diag_pop")


/*****************************************************************************/
/*  _STDINT.H                                                                */
/*                                                                           */
/* Copyright (c) 2017 Texas Instruments Incorporated                         */
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

/*-
 * SPDX-License-Identifier: BSD-2-Clause-NetBSD
 *
 * Copyright (c) 2001, 2002 Mike Barcroft <mike@FreeBSD.org>
 * Copyright (c) 2001 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Klaus Klein.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $FreeBSD$
 */


#pragma diag_push
/* 19.4 is issued for macros that are defined in terms of other macros. */
#pragma CHECK_MISRA("-19.4")
#pragma CHECK_MISRA("-19.7")
#pragma CHECK_MISRA("-19.13")







/*
 * ISO/IEC 9899:1999
 * 7.18.2.1 Limits of exact-width integer types
 */
/* Minimum values of exact-width signed integer types. */

/* Maximum values of exact-width signed integer types. */

/* Maximum values of exact-width unsigned integer types. */

/*
 * ISO/IEC 9899:1999
 * 7.18.2.2  Limits of minimum-width integer types
 */
/* Minimum values of minimum-width signed integer types. */

/* Maximum values of minimum-width signed integer types. */

/* Maximum values of minimum-width unsigned integer types. */

/*
 * ISO/IEC 9899:1999
 * 7.18.2.3  Limits of fastest minimum-width integer types
 */
/* Minimum values of fastest minimum-width signed integer types. */

/* Maximum values of fastest minimum-width signed integer types. */

/* Maximum values of fastest minimum-width unsigned integer types. */

/*
 * ISO/IEC 9899:1999
 * 7.18.2.4  Limits of integer types capable of holding object pointers
 */

/*
 * ISO/IEC 9899:1999
 * 7.18.2.5  Limits of greatest-width integer types
 */

/*
 * ISO/IEC 9899:1999
 * 7.18.3  Limits of other integer types
 */
/* Limits of ptrdiff_t. */

/* Limits of sig_atomic_t. */

/* Limit of size_t. */

/* Limits of wint_t. */


#pragma diag_pop

/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2011 David E. O'Brien <obrien@FreeBSD.org>
 * Copyright (c) 2001 Mike Barcroft <mike@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */


typedef	__int8_t		int8_t;

typedef	__int16_t		int16_t;

typedef	__int32_t		int32_t;

typedef	__int64_t		int64_t;

typedef	__uint8_t		uint8_t;

typedef	__uint16_t		uint16_t;

typedef	__uint32_t		uint32_t;

typedef	__uint64_t		uint64_t;

typedef	__intptr_t		intptr_t;
typedef	__uintptr_t		uintptr_t;
typedef	__intmax_t		intmax_t;
typedef	__uintmax_t		uintmax_t;


typedef	__int_least8_t		int_least8_t;
typedef	__int_least16_t		int_least16_t;
typedef	__int_least32_t		int_least32_t;
typedef	__int_least64_t		int_least64_t;

typedef	__uint_least8_t		uint_least8_t;
typedef	__uint_least16_t	uint_least16_t;
typedef	__uint_least32_t	uint_least32_t;
typedef	__uint_least64_t	uint_least64_t;

typedef	__int_fast8_t		int_fast8_t;
typedef	__int_fast16_t		int_fast16_t;
typedef	__int_fast32_t		int_fast32_t;
typedef	__int_fast64_t		int_fast64_t;

typedef	__uint_fast8_t		uint_fast8_t;
typedef	__uint_fast16_t		uint_fast16_t;
typedef	__uint_fast32_t		uint_fast32_t;
typedef	__uint_fast64_t		uint_fast64_t;

_Pragma("diag_push")
_Pragma("CHECK_MISRA(\"-10.1\")")
/* GNU and Darwin define this and people seem to think it's portable */
_Pragma("diag_pop")

_Pragma("diag_push")
_Pragma("CHECK_MISRA(\"-19.4\")")
/* Limits of wchar_t. */
_Pragma("diag_pop")

/* ISO/IEC 9899:2011 K.3.4.4 */


_Pragma("diag_pop")




namespace std { inline namespace __2 {

using::int8_t;
using::int16_t;
using::int32_t;
using::int64_t;

using::uint8_t;
using::uint16_t;
using::uint32_t;
using::uint64_t;

using::int_least8_t;
using::int_least16_t;
using::int_least32_t;
using::int_least64_t;

using::uint_least8_t;
using::uint_least16_t;
using::uint_least32_t;
using::uint_least64_t;

using::int_fast8_t;
using::int_fast16_t;
using::int_fast32_t;
using::int_fast64_t;

using::uint_fast8_t;
using::uint_fast16_t;
using::uint_fast32_t;
using::uint_fast64_t;

using::intptr_t;
using::uintptr_t;

using::intmax_t;
using::uintmax_t;

} }


enum CLOCK_TYPE {
    CLOCK_TYPE_ACLK,
    CLOCK_TYPE_MCLK,
    CLOCK_TYPE_SMCLK,
    CLOCK_TYPE_ADCOSC
};

enum CLOCK_SOURCE_TYPE {
    CLOCK_SOURCE_TYPE_VLO,
    CLOCK_SOURCE_TYPE_LFXT1,
    CLOCK_SOURCE_TYPE_DCO,
};

uint8_t VLOCLK_usage_count;
uint8_t LFXT1CLK_usage_count;
uint8_t DCOCLK_usage_count;

inline void enter_idle(void) {
    __bis_SR_register((0x0008) + (0x0010) +
        (VLOCLK_usage_count == 0 && LFXT1CLK_usage_count == 0 && DCOCLK_usage_count == 0 ? (0x0020) : 0) +
        (DCOCLK_usage_count == 0 ? (0x0040) + (0x0080) : 0));
}

inline void exit_idle(void) {
    __bic_SR_register_on_exit(((0x0080)+(0x0040)+(0x0020)+(0x0010)));
}

void delay_ms(uint8_t milliseconds) {
    uint8_t m;
    switch (BCSCTL1 & 0b00001111) {
        case 6: m = 1; break;
        case 13: m = 8; break;
        case 14: m = 12; break;
        default: m = 16; break;
    }
    while (milliseconds--) {
        for (int i = 0; i < m; i++) __delay_cycles(1000);
    }
}

void delay_us(int16_t microseconds) {
    switch (BCSCTL1 & 0b00001111) {
        case 6: while (microseconds--) __delay_cycles(1); break;
        case 13: while (microseconds--) __delay_cycles(8); break;
        case 14: while (microseconds--) __delay_cycles(12); break;
        default: while (microseconds--) __delay_cycles(16); break;
    }
}

template<const uint16_t FREQUENCY = 12000, const bool DEBUG_RELEASE = false>
struct VLOCLK_T {
    static constexpr CLOCK_SOURCE_TYPE type = CLOCK_SOURCE_TYPE_VLO;
    static constexpr uint16_t frequency = FREQUENCY;
    static void claim(void) { VLOCLK_usage_count++; }
    static void release(void) {
        if (!DEBUG_RELEASE || VLOCLK_usage_count > 0) {
            VLOCLK_usage_count--;
        } else {
            while (1);
        }
    }
};


template<const uint8_t CAPS, const uint16_t FREQUENCY = 32768, const bool DEBUG_RELEASE = false>
struct LFXT1CLK_T {
    static constexpr CLOCK_SOURCE_TYPE type = CLOCK_SOURCE_TYPE_LFXT1;
    static constexpr uint16_t frequency = FREQUENCY;

    static void init(void) {
        __bis_SR_register((0x0020));
        BCSCTL3 |= CAPS;
        __bic_SR_register((0x0020));
        while (IFG1 & (0x02)) IFG1 &= ~(0x02);
    }

    static void claim(void) { LFXT1CLK_usage_count++; }
    static void release(void) {
        if (!DEBUG_RELEASE || LFXT1CLK_usage_count > 0) {
            LFXT1CLK_usage_count--;
        } else {
            while (1);
        }
    }
};


template<const uint32_t FREQUENCY = 1000000, const bool DEBUG_RELEASE = false>
struct DCOCLK_T {
    static constexpr CLOCK_SOURCE_TYPE type = CLOCK_SOURCE_TYPE_DCO;
    static constexpr uint32_t frequency = FREQUENCY;
    static void init(void) {
        switch (FREQUENCY) {
            case 8000000:
                BCSCTL1 = CALBC1_8MHZ;
                DCOCTL = CALDCO_8MHZ;
                break;
            case 12000000:
                BCSCTL1 = CALBC1_12MHZ;
                DCOCTL = CALDCO_12MHZ;
                break;
            case 16000000:
                BCSCTL1 = CALBC1_16MHZ;
                DCOCTL = CALDCO_16MHZ;
            case 1000000:
                BCSCTL1 = CALBC1_1MHZ;
                DCOCTL = CALDCO_1MHZ;
                break;
            default:
                BCSCTL1 = CALBC1_1MHZ;
                DCOCTL = CALDCO_1MHZ;
                break;
        }
    }
    static void claim(void) { DCOCLK_usage_count++; }
    static void release(void) {
        if (!DEBUG_RELEASE || DCOCLK_usage_count > 0) {
            DCOCLK_usage_count--;
        } else {
            while (1);
        }
    }
};

template<typename SOURCE,
    const int DIVIDER = 1>
struct ACLK_T {
    static constexpr CLOCK_TYPE type = CLOCK_TYPE_ACLK;
    static constexpr long frequency = SOURCE::frequency / DIVIDER;

    static void init(void) {
        if (SOURCE::type == CLOCK_SOURCE_TYPE_VLO) BCSCTL3 |= (0x20);
    };

    static void claim(void) { SOURCE::claim(); }
    static void release(void) { SOURCE::release(); }
};

template<typename SOURCE,
    const int DIVIDER = 1>
struct SMCLK_T {
    static constexpr CLOCK_TYPE type = CLOCK_TYPE_SMCLK;
    static constexpr long frequency = SOURCE::frequency / DIVIDER;

    static void init(void) {
        if (DIVIDER > 1) {
            BCSCTL2 &= (0x06);
            BCSCTL2 |= (DIVIDER == 2 ? (0x02) : (DIVIDER == 4 ? (0x04) : (0x06)));
        }
        if (SOURCE::type != CLOCK_SOURCE_TYPE_DCO) {
            BCSCTL2 |= (0x08);
        }
    };

    static void claim(void) { SOURCE::claim(); }
    static void release(void) { SOURCE::release(); }

    static void set_and_wait_us(uint8_t microseconds) {
        while (microseconds--) {
            __delay_cycles(frequency / 1000000);
        }
    }

    static void set_and_wait(uint8_t milliseconds) {
        while (milliseconds--) {
            __delay_cycles(frequency / 1000);
        }
    }

};

template<typename SOURCE,
    const int DIVIDER = 1>
struct MCLK_T {
    static constexpr CLOCK_TYPE type = CLOCK_TYPE_MCLK;
    static constexpr long frequency = SOURCE::frequency / DIVIDER;

    static void init(void) {
        switch (SOURCE::type) {
            case CLOCK_SOURCE_TYPE_DCO:
                break;
            case CLOCK_SOURCE_TYPE_VLO:
            case CLOCK_SOURCE_TYPE_LFXT1:
                BCSCTL2 |= (0xC0);
                break;
        }
    };

    static void set_and_wait_us(uint8_t microseconds) {
        while (microseconds--) {
            __delay_cycles(frequency / 1000000);
        }
    }

    static void set_and_wait(uint8_t milliseconds) {
        while (milliseconds--) {
            __delay_cycles(frequency / 1000);
        }
    }

    static void claim(void) { SOURCE::claim(); }
    static void release(void) { SOURCE::release(); }
};



template<typename CLOCK>
struct TIMEOUT_T {
    static volatile uint32_t timeout;

    static void set(const uint32_t milliseconds) {
        __bic_SR_register((0x0008));
        timeout = ((uint32_t) milliseconds * (uint32_t) CLOCK::frequency / (uint32_t) 1000);
        __bis_SR_register((0x0008));
        CLOCK::claim();
    };

    static inline bool count_down(void) {
        return !timeout || (--timeout == 0);
    };

    static inline bool triggered(void) {
        return timeout == 0;
    };

    static inline void disable(void) {
        CLOCK::release();
    }

    static inline uint32_t get() {
        return timeout;
    };

    static void set_and_wait(const uint32_t milliseconds) {
        set(milliseconds);
        while (!triggered()) {
            enter_idle();
        }
        disable();
    }
};

struct TIMEOUT_NEVER {
    static void set(const uint32_t milliseconds) { }
    static inline bool count_down(void) { return false; }
    static inline bool triggered(void) { return false; }
    static inline uint32_t get() { return 1; }
    static void disable(void) { }
};

struct TIMEOUT_IMMEDIATELY {
    static void set(const uint32_t milliseconds) { }
    static inline bool count_down(void) { return true; }
    static inline bool triggered(void) { return true; }
    static inline uint32_t get() { return 0; }
    static void disable(void) { }
};

template<typename CLOCK>
volatile uint32_t TIMEOUT_T<CLOCK>::timeout;





enum DIRECTION {
    INPUT = 0,
    OUTPUT = 1
};

enum TRIGGER_EDGE {
    TRIGGER_RISING = 0,
    TRIGGER_FALLING = 1
};

enum LEVEL {
    LOW = false,
    PULL_DOWN = false,
    HIGH = true,
    PULL_UP = true
};

enum INTERRUPT_ENABLE {
    INTERRUPT_DISABLED = false,
    INTERRUPT_ENABLED = true
};

enum RESISTOR_ENABLE {
    RESISTOR_DISABLED = false,
    RESISTOR_ENABLED = true
};

enum PIN_FUNCTION {
    TIMER_OUTPUT,
    TIMER_COMPARE,
    ACLK_OUTPUT,
    CAPACITIVE_SENSE,
    ADCCLK_OUTPUT,
    COMPARATOR_OUTPUT,
    SMCLK_OUTUT,
    XIN,
    XOUT,
    USCI
};

uint8_t P1IFGS;
uint8_t P2IFGS;

static constexpr volatile uint8_t *ports[][10] = {
    {const_cast<uint8_t *>(&P1IN), &P1OUT, &P1DIR, &P1IFG, &P1IES, &P1IE, &P1SEL, &P1SEL2, &P1REN, &P1IFGS},
};

template<const char PORT, const char PIN,
    const DIRECTION PIN_DIRECTION = OUTPUT,
    const LEVEL INITIAL_LEVEL = LOW,
    const INTERRUPT_ENABLE INTERRUPT = INTERRUPT_DISABLED,
    const TRIGGER_EDGE EDGE = TRIGGER_RISING,
    const char FUNCTION_SELECT = 0,
    const RESISTOR_ENABLE RESISTOR = RESISTOR_DISABLED>
struct GPIO_PIN_T {
    static_assert(PORT > 0 && PORT < 4, "Port range must be between 1 and 3");
    static constexpr volatile uint8_t *PxIN = ports[PORT - 1][0];
    static constexpr volatile uint8_t *PxOUT = ports[PORT - 1][1];
    static constexpr volatile uint8_t *PxDIR = ports[PORT - 1][2];
    static constexpr volatile uint8_t *PxIFG = ports[PORT - 1][3];
    static constexpr volatile uint8_t *PxIES = ports[PORT - 1][4];
    static constexpr volatile uint8_t *PxIE = ports[PORT - 1][5];
    static constexpr volatile uint8_t *PxSEL = ports[PORT - 1][6];
    static constexpr volatile uint8_t *PxSEL2 = ports[PORT - 1][7];
    static constexpr volatile uint8_t *PxREN = ports[PORT - 1][8];
    static constexpr volatile uint8_t *PxIFGS = ports[PORT - 1][9];

    static constexpr uint8_t pin = PIN;
    static constexpr uint8_t bit_value = 1 << PIN;

    static constexpr uint8_t direction = (PIN_DIRECTION == OUTPUT ? bit_value : 0);
    static constexpr uint8_t interrupt_enable = (INTERRUPT ? bit_value : 0);
    static constexpr uint8_t interrupt_edge = (EDGE == TRIGGER_FALLING ? bit_value : 0);
    static constexpr uint8_t function_select = (FUNCTION_SELECT & 0b01 ? bit_value : 0);
    static constexpr uint8_t function_select2 = (FUNCTION_SELECT & 0b10 ? bit_value : 0);
    static constexpr uint8_t resistor_enable = (RESISTOR ? bit_value : 0);
    static constexpr uint8_t initial_level = (INITIAL_LEVEL ? bit_value : 0);
    static constexpr uint16_t adc_input = (PORT == 1 ? PIN << 12 : 0);
    static constexpr bool is_unused(void) { return false; }

    static void init(void) {
        if (INITIAL_LEVEL) *PxOUT |= bit_value;
        if (PIN_DIRECTION == OUTPUT) *PxDIR |= bit_value;
        if (INTERRUPT) {
            static_assert(PxIE != 0, "Port not interrupt capable");
            *PxIE |= bit_value;
        }
        if (EDGE == TRIGGER_FALLING) {
            static_assert(PxIES != 0, "Port not interrupt capable");
            *PxIES |= bit_value;
        }
        if (FUNCTION_SELECT & 0b01) {
            static_assert(PxSEL != 0, "Port does not have alternate functions");
            *PxSEL |= bit_value;
        }
        if (FUNCTION_SELECT & 0b10) {
            static_assert(PxSEL2 != 0, "Port does not have alternate functions");
            *PxSEL2 |= bit_value;
        }
        if (RESISTOR) *PxREN |= bit_value;
    };

    static void set_high(void) {
        *PxOUT |= bit_value;
    };

    static void set_low(void) {
        *PxOUT &= ~bit_value;
    }

    static void set(bool value) {
        if (value) set_high();
        else set_low();
    }

    static bool get(void) {
        return *PxIN & bit_value;
    }

    static bool is_high(void) {
        return *PxIN & bit_value;
    }

    static bool is_low(void) {
        return !is_high();
    }

    static void toggle(void) {
        *PxOUT ^= bit_value;
    }

    static void clear_irq(void) {
        *PxIFG &= ~bit_value;
        *PxIFGS &= ~bit_value;
    }

    static bool irq_raised(void) {
        return *PxIFGS & bit_value;
    }

    static void disable_irq(void) {
        *PxIE &= ~bit_value;
    }

    static void enable_irq(void) {
        *PxIE |= bit_value;
    }

    template<typename TIMEOUT = TIMEOUT_NEVER>
    static bool wait_for_irq(void) {
        while (!TIMEOUT::triggered() && !(*PxIFGS & bit_value)) {
            enter_idle();
        }
        return irq_raised();
    }

    static void set_output(void) {
        *PxDIR |= bit_value;
    }

    static void set_input(void) {
        *PxDIR &= ~bit_value;
    }

    static void enable_resistor(void) {
        *PxREN |= bit_value;
    }

    static void disable_resistor(void) {
        *PxREN &= ~bit_value;
    }

    static void pull_up(void) {
        set_high();
    }

    static void pull_down(void) {
        set_low();
    }
};

template<const char PORT, const char PIN,
    const RESISTOR_ENABLE RESISTOR = RESISTOR_DISABLED,
    const LEVEL PULL = PULL_DOWN,
    const INTERRUPT_ENABLE INTERRUPT = INTERRUPT_DISABLED,
    const TRIGGER_EDGE EDGE = TRIGGER_RISING>
struct GPIO_INPUT_T: public GPIO_PIN_T<PORT, PIN, INPUT, PULL, INTERRUPT, EDGE, 0, RESISTOR> {
};

template<const char PORT, const char PIN,
    const LEVEL INITIAL_LEVEL = LOW>
struct GPIO_OUTPUT_T: public GPIO_PIN_T<PORT, PIN, OUTPUT, INITIAL_LEVEL, INTERRUPT_DISABLED, TRIGGER_RISING, 0, RESISTOR_DISABLED> {
};

template<const char PORT, const char PIN,
    const char FUNCTION_SELECT = 0,
    const DIRECTION PIN_DIRECTION = OUTPUT>
struct GPIO_MODULE_T: public GPIO_PIN_T<PORT, PIN, PIN_DIRECTION, LOW, INTERRUPT_DISABLED, TRIGGER_RISING, FUNCTION_SELECT, RESISTOR_DISABLED> {
};

template<const char PORT, const char PIN>
struct GPIO_ANALOG_T {
    static constexpr uint8_t direction = 0;
    static constexpr uint8_t interrupt_enable = 0;
    static constexpr uint8_t interrupt_edge = 0;
    static constexpr uint8_t function_select = 0;
    static constexpr uint8_t function_select2 = 0;
    static constexpr uint8_t resistor_enable = 0;
    static constexpr uint8_t initial_level = 0;
    static constexpr uint8_t pin = PIN;
    static constexpr uint8_t bit_value = 1 << PIN;
    static constexpr uint16_t adc_input = (PORT == 1 ? PIN << 12 : 0);
};

struct PIN_UNUSED {
    static constexpr uint8_t direction = 0;
    static constexpr uint8_t interrupt_enable = 0;
    static constexpr uint8_t interrupt_edge = 0;
    static constexpr uint8_t function_select = 0;
    static constexpr uint8_t function_select2 = 0;
    static constexpr uint8_t resistor_enable = 0;
    static constexpr uint8_t initial_level = 0;
    static constexpr uint16_t adc_input = 0;
    static constexpr uint8_t pin = 0;
    static constexpr uint8_t bit_value = 0;
    static constexpr bool is_unused(void) { return true; }
    static constexpr bool is_high(void) { return false; }
    static constexpr bool is_low(void) { return true; }
    static void toggle(void) { }
    static void set_high(void) { }
    static void set_low(void) { }
    static void clear_irq(void) { }
    static bool wait_for_irq(void) { return true; }
    static constexpr bool irq_raised(void) { return false; }
};

template<const int PORT,
    typename PIN0 = PIN_UNUSED,
    typename PIN1 = PIN_UNUSED,
    typename PIN2 = PIN_UNUSED,
    typename PIN3 = PIN_UNUSED,
    typename PIN4 = PIN_UNUSED,
    typename PIN5 = PIN_UNUSED,
    typename PIN6 = PIN_UNUSED,
    typename PIN7 = PIN_UNUSED>
struct GPIO_PORT_T {
    static constexpr volatile uint8_t *PxIN = ports[PORT - 1][0];
    static constexpr volatile uint8_t *PxOUT = ports[PORT - 1][1];
    static constexpr volatile uint8_t *PxDIR = ports[PORT - 1][2];
    static constexpr volatile uint8_t *PxIFG = ports[PORT - 1][3];
    static constexpr volatile uint8_t *PxIES = ports[PORT - 1][4];
    static constexpr volatile uint8_t *PxIE = ports[PORT - 1][5];
    static constexpr volatile uint8_t *PxSEL = ports[PORT - 1][6];
    static constexpr volatile uint8_t *PxSEL2 = ports[PORT - 1][7];
    static constexpr volatile uint8_t *PxREN = ports[PORT - 1][8];
    static constexpr volatile uint8_t *PxIFGS = ports[PORT - 1][9];

    static void init(void) {
        uint8_t reg;

        reg =
            PIN0::direction | PIN1::direction | PIN2::direction | PIN3::direction |
            PIN4::direction | PIN5::direction | PIN6::direction | PIN7::direction;
        if (reg) *PxDIR = reg;

        reg =
            PIN0::interrupt_enable | PIN1::interrupt_enable | PIN2::interrupt_enable | PIN3::interrupt_enable |
            PIN4::interrupt_enable | PIN5::interrupt_enable | PIN6::interrupt_enable | PIN7::interrupt_enable;
        if (reg) *PxIE = reg;

        reg =
            PIN0::interrupt_edge | PIN1::interrupt_edge | PIN2::interrupt_edge | PIN3::interrupt_edge |
            PIN4::interrupt_edge | PIN5::interrupt_edge | PIN6::interrupt_edge | PIN7::interrupt_edge;
        if (reg) *PxIES = reg;

        reg =
            PIN0::function_select | PIN1::function_select | PIN2::function_select | PIN3::function_select |
            PIN4::function_select | PIN5::function_select | PIN6::function_select | PIN7::function_select;
        if (reg) *PxSEL = reg;

        reg =
            PIN0::function_select2 | PIN1::function_select2 | PIN2::function_select2 | PIN3::function_select2 |
            PIN4::function_select2 | PIN5::function_select2 | PIN6::function_select2 | PIN7::function_select2;
        if (reg) *PxSEL2 = reg;

        reg =
            PIN0::resistor_enable | PIN1::resistor_enable | PIN2::resistor_enable | PIN3::resistor_enable |
            PIN4::resistor_enable | PIN5::resistor_enable | PIN6::resistor_enable | PIN7::resistor_enable;
        if (reg) *PxREN = reg;

        *PxOUT =
            PIN0::initial_level | PIN1::initial_level | PIN2::initial_level | PIN3::initial_level |
            PIN4::initial_level | PIN5::initial_level | PIN6::initial_level | PIN7::initial_level;
    }

    static void clear_irq(void) {
        *PxIFG = 0;
    };

    static bool handle_irq(void) {
        bool resume = false;

        if (*PxIFG) {
            *PxIFGS = *PxIFG;
            clear_irq();
            resume = true;
        }
        return resume;
    }
};



/*
 * LcdCommands.hpp
 *
 *  Created on: 10.05.2023
 *      Author: Andrio01
 */


// -*- C++ -*-
//===------------------------ type_traits ---------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is dual licensed under the MIT and the University of Illinois Open
// Source Licenses. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//


/*
    type_traits synopsis

namespace std
{

    // helper class:
    template <class T, T v> struct integral_constant;
    typedef integral_constant<bool, true>  true_type;   // C++11
    typedef integral_constant<bool, false> false_type;  // C++11
    
    template <bool B>                                   // C++14
    using bool_constant = integral_constant<bool, B>;   // C++14
    typedef bool_constant<true> true_type;              // C++14
    typedef bool_constant<false> false_type;            // C++14

    // helper traits
    template <bool, class T = void> struct enable_if;
    template <bool, class T, class F> struct conditional;

    // Primary classification traits:
    template <class T> struct is_void;
    template <class T> struct is_null_pointer;  // C++14
    template <class T> struct is_integral;
    template <class T> struct is_floating_point;
    template <class T> struct is_array;
    template <class T> struct is_pointer;
    template <class T> struct is_lvalue_reference;
    template <class T> struct is_rvalue_reference;
    template <class T> struct is_member_object_pointer;
    template <class T> struct is_member_function_pointer;
    template <class T> struct is_enum;
    template <class T> struct is_union;
    template <class T> struct is_class;
    template <class T> struct is_function;

    // Secondary classification traits:
    template <class T> struct is_reference;
    template <class T> struct is_arithmetic;
    template <class T> struct is_fundamental;
    template <class T> struct is_member_pointer;
    template <class T> struct is_scalar;
    template <class T> struct is_object;
    template <class T> struct is_compound;

    // Const-volatile properties and transformations:
    template <class T> struct is_const;
    template <class T> struct is_volatile;
    template <class T> struct remove_const;
    template <class T> struct remove_volatile;
    template <class T> struct remove_cv;
    template <class T> struct add_const;
    template <class T> struct add_volatile;
    template <class T> struct add_cv;

    // Reference transformations:
    template <class T> struct remove_reference;
    template <class T> struct add_lvalue_reference;
    template <class T> struct add_rvalue_reference;

    // Pointer transformations:
    template <class T> struct remove_pointer;
    template <class T> struct add_pointer;

    // Integral properties:
    template <class T> struct is_signed;
    template <class T> struct is_unsigned;
    template <class T> struct make_signed;
    template <class T> struct make_unsigned;

    // Array properties and transformations:
    template <class T> struct rank;
    template <class T, unsigned I = 0> struct extent;
    template <class T> struct remove_extent;
    template <class T> struct remove_all_extents;

    // Member introspection:
    template <class T> struct is_pod;
    template <class T> struct is_trivial;
    template <class T> struct is_trivially_copyable;
    template <class T> struct is_standard_layout;
    template <class T> struct is_literal_type;
    template <class T> struct is_empty;
    template <class T> struct is_polymorphic;
    template <class T> struct is_abstract;
    template <class T> struct is_final; // C++14
    template <class T> struct is_aggregate; // C++17

    template <class T, class... Args> struct is_constructible;
    template <class T>                struct is_default_constructible;
    template <class T>                struct is_copy_constructible;
    template <class T>                struct is_move_constructible;
    template <class T, class U>       struct is_assignable;
    template <class T>                struct is_copy_assignable;
    template <class T>                struct is_move_assignable;
    template <class T, class U>       struct is_swappable_with;       // C++17
    template <class T>                struct is_swappable;            // C++17
    template <class T>                struct is_destructible;

    template <class T, class... Args> struct is_trivially_constructible;
    template <class T>                struct is_trivially_default_constructible;
    template <class T>                struct is_trivially_copy_constructible;
    template <class T>                struct is_trivially_move_constructible;
    template <class T, class U>       struct is_trivially_assignable;
    template <class T>                struct is_trivially_copy_assignable;
    template <class T>                struct is_trivially_move_assignable;
    template <class T>                struct is_trivially_destructible;

    template <class T, class... Args> struct is_nothrow_constructible;
    template <class T>                struct is_nothrow_default_constructible;
    template <class T>                struct is_nothrow_copy_constructible;
    template <class T>                struct is_nothrow_move_constructible;
    template <class T, class U>       struct is_nothrow_assignable;
    template <class T>                struct is_nothrow_copy_assignable;
    template <class T>                struct is_nothrow_move_assignable;
    template <class T, class U>       struct is_nothrow_swappable_with; // C++17
    template <class T>                struct is_nothrow_swappable;      // C++17
    template <class T>                struct is_nothrow_destructible;

    template <class T> struct has_virtual_destructor;

    // Relationships between types:
    template <class T, class U> struct is_same;
    template <class Base, class Derived> struct is_base_of;
    template <class From, class To> struct is_convertible;

    template <class, class R = void> struct is_callable; // not defined
    template <class Fn, class... ArgTypes, class R>
      struct is_callable<Fn(ArgTypes...), R>;

    template <class, class R = void> struct is_nothrow_callable; // not defined
    template <class Fn, class... ArgTypes, class R>
      struct is_nothrow_callable<Fn(ArgTypes...), R>;

    // Alignment properties and transformations:
    template <class T> struct alignment_of;
    template <size_t Len, size_t Align = most_stringent_alignment_requirement>
        struct aligned_storage;
    template <size_t Len, class... Types> struct aligned_union;

    template <class T> struct decay;
    template <class... T> struct common_type;
    template <class T> struct underlying_type;
    template <class> class result_of; // undefined
    template <class Fn, class... ArgTypes> class result_of<Fn(ArgTypes...)>;

    // const-volatile modifications:
    template <class T>
      using remove_const_t    = typename remove_const<T>::type;  // C++14
    template <class T>
      using remove_volatile_t = typename remove_volatile<T>::type;  // C++14
    template <class T>
      using remove_cv_t       = typename remove_cv<T>::type;  // C++14
    template <class T>
      using add_const_t       = typename add_const<T>::type;  // C++14
    template <class T>
      using add_volatile_t    = typename add_volatile<T>::type;  // C++14
    template <class T>
      using add_cv_t          = typename add_cv<T>::type;  // C++14
  
    // reference modifications:
    template <class T>
      using remove_reference_t     = typename remove_reference<T>::type;  // C++14
    template <class T>
      using add_lvalue_reference_t = typename add_lvalue_reference<T>::type;  // C++14
    template <class T>
      using add_rvalue_reference_t = typename add_rvalue_reference<T>::type;  // C++14
  
    // sign modifications:
    template <class T>
      using make_signed_t   = typename make_signed<T>::type;  // C++14
    template <class T>
      using make_unsigned_t = typename make_unsigned<T>::type;  // C++14
  
    // array modifications:
    template <class T>
      using remove_extent_t      = typename remove_extent<T>::type;  // C++14
    template <class T>
      using remove_all_extents_t = typename remove_all_extents<T>::type;  // C++14

    // pointer modifications:
    template <class T>
      using remove_pointer_t = typename remove_pointer<T>::type;  // C++14
    template <class T>
      using add_pointer_t    = typename add_pointer<T>::type;  // C++14

    // other transformations:
    template <size_t Len, std::size_t Align=default-alignment>
      using aligned_storage_t = typename aligned_storage<Len,Align>::type;  // C++14
    template <std::size_t Len, class... Types>
      using aligned_union_t   = typename aligned_union<Len,Types...>::type;  // C++14
    template <class T>
      using decay_t           = typename decay<T>::type;  // C++14
    template <bool b, class T=void>
      using enable_if_t       = typename enable_if<b,T>::type;  // C++14
    template <bool b, class T, class F>
      using conditional_t     = typename conditional<b,T,F>::type;  // C++14
    template <class... T>
      using common_type_t     = typename common_type<T...>::type;  // C++14
    template <class T>
      using underlying_type_t = typename underlying_type<T>::type;  // C++14
    template <class F, class... ArgTypes>
      using result_of_t       = typename result_of<F(ArgTypes...)>::type;  // C++14

    template <class...>
      using void_t = void;   // C++17
      
      // See C++14 20.10.4.1, primary type categories
      template <class T> constexpr bool is_void_v
        = is_void<T>::value;                                             // C++17
      template <class T> constexpr bool is_null_pointer_v
        = is_null_pointer<T>::value;                                     // C++17
      template <class T> constexpr bool is_integral_v
        = is_integral<T>::value;                                         // C++17
      template <class T> constexpr bool is_floating_point_v
        = is_floating_point<T>::value;                                   // C++17
      template <class T> constexpr bool is_array_v
        = is_array<T>::value;                                            // C++17
      template <class T> constexpr bool is_pointer_v
        = is_pointer<T>::value;                                          // C++17
      template <class T> constexpr bool is_lvalue_reference_v
        = is_lvalue_reference<T>::value;                                 // C++17
      template <class T> constexpr bool is_rvalue_reference_v
        = is_rvalue_reference<T>::value;                                 // C++17
      template <class T> constexpr bool is_member_object_pointer_v
        = is_member_object_pointer<T>::value;                            // C++17
      template <class T> constexpr bool is_member_function_pointer_v
        = is_member_function_pointer<T>::value;                          // C++17
      template <class T> constexpr bool is_enum_v
        = is_enum<T>::value;                                             // C++17
      template <class T> constexpr bool is_union_v
        = is_union<T>::value;                                            // C++17
      template <class T> constexpr bool is_class_v
        = is_class<T>::value;                                            // C++17
      template <class T> constexpr bool is_function_v
        = is_function<T>::value;                                         // C++17

      // See C++14 20.10.4.2, composite type categories
      template <class T> constexpr bool is_reference_v
        = is_reference<T>::value;                                        // C++17
      template <class T> constexpr bool is_arithmetic_v
        = is_arithmetic<T>::value;                                       // C++17
      template <class T> constexpr bool is_fundamental_v
        = is_fundamental<T>::value;                                      // C++17
      template <class T> constexpr bool is_object_v
        = is_object<T>::value;                                           // C++17
      template <class T> constexpr bool is_scalar_v
        = is_scalar<T>::value;                                           // C++17
      template <class T> constexpr bool is_compound_v
        = is_compound<T>::value;                                         // C++17
      template <class T> constexpr bool is_member_pointer_v
        = is_member_pointer<T>::value;                                   // C++17

      // See C++14 20.10.4.3, type properties
      template <class T> constexpr bool is_const_v
        = is_const<T>::value;                                            // C++17
      template <class T> constexpr bool is_volatile_v
        = is_volatile<T>::value;                                         // C++17
      template <class T> constexpr bool is_trivial_v
        = is_trivial<T>::value;                                          // C++17
      template <class T> constexpr bool is_trivially_copyable_v
        = is_trivially_copyable<T>::value;                               // C++17
      template <class T> constexpr bool is_standard_layout_v
        = is_standard_layout<T>::value;                                  // C++17
      template <class T> constexpr bool is_pod_v
        = is_pod<T>::value;                                              // C++17
      template <class T> constexpr bool is_literal_type_v
        = is_literal_type<T>::value;                                     // C++17
      template <class T> constexpr bool is_empty_v
        = is_empty<T>::value;                                            // C++17
      template <class T> constexpr bool is_polymorphic_v
        = is_polymorphic<T>::value;                                      // C++17
      template <class T> constexpr bool is_abstract_v
        = is_abstract<T>::value;                                         // C++17
      template <class T> constexpr bool is_final_v
        = is_final<T>::value;                                            // C++17
      template <class T> constexpr bool is_aggregate_v
        = is_aggregate<T>::value;                                        // C++17
      template <class T> constexpr bool is_signed_v
        = is_signed<T>::value;                                           // C++17
      template <class T> constexpr bool is_unsigned_v
        = is_unsigned<T>::value;                                         // C++17
      template <class T, class... Args> constexpr bool is_constructible_v
        = is_constructible<T, Args...>::value;                           // C++17
      template <class T> constexpr bool is_default_constructible_v
        = is_default_constructible<T>::value;                            // C++17
      template <class T> constexpr bool is_copy_constructible_v
        = is_copy_constructible<T>::value;                               // C++17
      template <class T> constexpr bool is_move_constructible_v
        = is_move_constructible<T>::value;                               // C++17
      template <class T, class U> constexpr bool is_assignable_v
        = is_assignable<T, U>::value;                                    // C++17
      template <class T> constexpr bool is_copy_assignable_v
        = is_copy_assignable<T>::value;                                  // C++17
      template <class T> constexpr bool is_move_assignable_v
        = is_move_assignable<T>::value;                                  // C++17
      template <class T, class U> constexpr bool is_swappable_with_v
        = is_swappable_with<T, U>::value;                                // C++17
      template <class T> constexpr bool is_swappable_v
        = is_swappable<T>::value;                                        // C++17
      template <class T> constexpr bool is_destructible_v
        = is_destructible<T>::value;                                     // C++17
      template <class T, class... Args> constexpr bool is_trivially_constructible_v
        = is_trivially_constructible<T, Args...>::value;                 // C++17
      template <class T> constexpr bool is_trivially_default_constructible_v
        = is_trivially_default_constructible<T>::value;                  // C++17
      template <class T> constexpr bool is_trivially_copy_constructible_v
        = is_trivially_copy_constructible<T>::value;                     // C++17
      template <class T> constexpr bool is_trivially_move_constructible_v
        = is_trivially_move_constructible<T>::value;                     // C++17
      template <class T, class U> constexpr bool is_trivially_assignable_v
        = is_trivially_assignable<T, U>::value;                          // C++17
      template <class T> constexpr bool is_trivially_copy_assignable_v
        = is_trivially_copy_assignable<T>::value;                        // C++17
      template <class T> constexpr bool is_trivially_move_assignable_v
        = is_trivially_move_assignable<T>::value;                        // C++17
      template <class T> constexpr bool is_trivially_destructible_v
        = is_trivially_destructible<T>::value;                           // C++17
      template <class T, class... Args> constexpr bool is_nothrow_constructible_v
        = is_nothrow_constructible<T, Args...>::value;                   // C++17
      template <class T> constexpr bool is_nothrow_default_constructible_v
        = is_nothrow_default_constructible<T>::value;                    // C++17
      template <class T> constexpr bool is_nothrow_copy_constructible_v
        = is_nothrow_copy_constructible<T>::value;                       // C++17
      template <class T> constexpr bool is_nothrow_move_constructible_v
        = is_nothrow_move_constructible<T>::value;                       // C++17
      template <class T, class U> constexpr bool is_nothrow_assignable_v
        = is_nothrow_assignable<T, U>::value;                            // C++17
      template <class T> constexpr bool is_nothrow_copy_assignable_v
        = is_nothrow_copy_assignable<T>::value;                          // C++17
      template <class T> constexpr bool is_nothrow_move_assignable_v
        = is_nothrow_move_assignable<T>::value;                          // C++17
      template <class T, class U> constexpr bool is_nothrow_swappable_with_v
        = is_nothrow_swappable_with<T, U>::value;                       // C++17
      template <class T> constexpr bool is_nothrow_swappable_v
        = is_nothrow_swappable<T>::value;                               // C++17
      template <class T> constexpr bool is_nothrow_destructible_v
        = is_nothrow_destructible<T>::value;                             // C++17
      template <class T> constexpr bool has_virtual_destructor_v
        = has_virtual_destructor<T>::value;                              // C++17

      // See C++14 20.10.5, type property queries
      template <class T> constexpr size_t alignment_of_v
        = alignment_of<T>::value;                                        // C++17
      template <class T> constexpr size_t rank_v
        = rank<T>::value;                                                // C++17
      template <class T, unsigned I = 0> constexpr size_t extent_v
        = extent<T, I>::value;                                           // C++17

      // See C++14 20.10.6, type relations
      template <class T, class U> constexpr bool is_same_v
        = is_same<T, U>::value;                                          // C++17
      template <class Base, class Derived> constexpr bool is_base_of_v
        = is_base_of<Base, Derived>::value;                              // C++17
      template <class From, class To> constexpr bool is_convertible_v
        = is_convertible<From, To>::value;                               // C++17
      template <class T, class R = void> constexpr bool is_callable_v
        = is_callable<T, R>::value;                                      // C++17
      template <class T, class R = void> constexpr bool is_nothrow_callable_v
        = is_nothrow_callable<T, R>::value;                              // C++17

      // [meta.logical], logical operator traits:
      template<class... B> struct conjunction;                           // C++17
      template<class... B> 
        constexpr bool conjunction_v = conjunction<B...>::value;         // C++17
      template<class... B> struct disjunction;                           // C++17
      template<class... B>
        constexpr bool disjunction_v = disjunction<B...>::value;         // C++17
      template<class B> struct negation;                                 // C++17
      template<class B> 
        constexpr bool negation_v = negation<B>::value;                  // C++17

}

*/
// -*- C++ -*-
//===--------------------------- cstddef ----------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is dual licensed under the MIT and the University of Illinois Open
// Source Licenses. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//


/*
    cstddef synopsis

Macros:

    offsetof(type,member-designator)
    NULL

namespace std
{

Types:

    ptrdiff_t
    size_t
    max_align_t
    nullptr_t
    byte // C++17

}  // std

*/



// Don't include our own <stddef.h>; we don't want to declare ::nullptr_t.
/*****************************************************************************/
/* stddef.h                                                                  */
/*                                                                           */
/* Copyright (c) 1993 Texas Instruments Incorporated                         */
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



_Pragma("diag_push")
_Pragma("CHECK_MISRA(\"-19.7\")") /* macros required for implementation */
_Pragma("CHECK_MISRA(\"-20.1\")") /* standard headers must define standard names */
_Pragma("CHECK_MISRA(\"-20.2\")") /* standard headers must define standard names */

extern "C" {


typedef int ptrdiff_t;


typedef unsigned size_t;


/*----------------------------------------------------------------------------*/
/* C++11 and C11 required max_align_t to be defined. The libc++ cstddef       */
/* header expects the macro __DEFINED_max_align_t to be defined if it is to   */
/* use the definintion of max_align_t from stddef.h. Only define it if        */
/* compiling for C11 or we're in non strict ansi mode.                        */
/*----------------------------------------------------------------------------*/
typedef long double max_align_t;

_Pragma("diag_push")
_Pragma("CHECK_MISRA(\"-19.10\")") /* need types as macro arguments */


_Pragma("diag_pop")

} /* extern "C" */

_Pragma("diag_pop")

// -*- C++ -*-
//===--------------------------- __nullptr --------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is dual licensed under the MIT and the University of Illinois Open
// Source Licenses. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//





namespace std
{
    typedef decltype(nullptr) nullptr_t;
}



namespace std { inline namespace __2 {

using ::ptrdiff_t;
using ::size_t;

// Re-use the compiler's <stddef.h> max_align_t where possible.
using ::max_align_t;

} }




namespace std { inline namespace __2 {

template <class _T1, class _T2> struct  pair;
template <class _Tp> class  reference_wrapper;
template <class _Tp> struct  hash;

template <class>
struct __void_t { typedef void type; };

template <class _Tp>
struct __identity { typedef _Tp type; };

template <class _Tp, bool>
struct  __dependent_type : public _Tp {};

template <bool _Bp, class _If, class _Then>
    struct  conditional {typedef _If type;};
template <class _If, class _Then>
    struct  conditional<false, _If, _Then> {typedef _Then type;};

template <bool _Bp, class _If, class _Then> using conditional_t = typename conditional<_Bp, _If, _Then>::type;

template <bool, class _Tp> struct  __lazy_enable_if {};
template <class _Tp> struct  __lazy_enable_if<true, _Tp> {typedef typename _Tp::type type;};

template <bool, class _Tp = void> struct  enable_if {};
template <class _Tp> struct  enable_if<true, _Tp> {typedef _Tp type;};

template <bool _Bp, class _Tp = void> using enable_if_t = typename enable_if<_Bp, _Tp>::type;

// addressof

template <class _Tp>
inline 
 __attribute__ ((__always_inline__))
_Tp*
addressof(_Tp& __x) noexcept
{
    return __builtin_addressof(__x);
}



template <class _Tp> _Tp* addressof(const _Tp&&) noexcept = delete;

struct __two {char __lx[2];};

// helper class:

template <class _Tp, _Tp __v>
struct  integral_constant
{
    static constexpr const _Tp      value = __v;
    typedef _Tp               value_type;
    typedef integral_constant type;
    __attribute__ ((__always_inline__))
        constexpr operator value_type() const noexcept {return value;}
    __attribute__ ((__always_inline__))
         constexpr value_type operator ()() const noexcept {return value;}
};

template <class _Tp, _Tp __v>
constexpr const _Tp integral_constant<_Tp, __v>::value;


typedef integral_constant<bool,(true)>  true_type;
typedef integral_constant<bool,(false)> false_type;


// __lazy_and

template <bool _Last, class ..._Preds>
struct __lazy_and_impl;

template <class ..._Preds>
struct __lazy_and_impl<false, _Preds...> : false_type {};

template <>
struct __lazy_and_impl<true> : true_type {};

template <class _Pred>
struct __lazy_and_impl<true, _Pred> : integral_constant<bool, _Pred::type::value> {};

template <class _Hp, class ..._Tp>
struct __lazy_and_impl<true, _Hp, _Tp...> : __lazy_and_impl<_Hp::type::value, _Tp...> {};

template <class _P1, class ..._Pr>
struct __lazy_and : __lazy_and_impl<_P1::type::value, _Pr...> {};

// __lazy_or

template <bool _List, class ..._Preds>
struct __lazy_or_impl;

template <class ..._Preds>
struct __lazy_or_impl<true, _Preds...> : true_type {};

template <>
struct __lazy_or_impl<false> : false_type {};

template <class _Hp, class ..._Tp>
struct __lazy_or_impl<false, _Hp, _Tp...>
        : __lazy_or_impl<_Hp::type::value, _Tp...> {};

template <class _P1, class ..._Pr>
struct __lazy_or : __lazy_or_impl<_P1::type::value, _Pr...> {};

// __lazy_not

template <class _Pred>
struct __lazy_not : integral_constant<bool, !_Pred::type::value> {};

// __and_
template<class...> struct __and_;
template<> struct __and_<> : true_type {};

template<class _B0> struct __and_<_B0> : _B0 {};

template<class _B0, class _B1>
struct __and_<_B0, _B1> : conditional<_B0::value, _B1, _B0>::type {};

template<class _B0, class _B1, class _B2, class... _Bn>
struct __and_<_B0, _B1, _B2, _Bn...> 
        : conditional<_B0::value, __and_<_B1, _B2, _Bn...>, _B0>::type {};

// __or_
template<class...> struct __or_;
template<> struct __or_<> : false_type {};

template<class _B0> struct __or_<_B0> : _B0 {};

template<class _B0, class _B1>
struct __or_<_B0, _B1> : conditional<_B0::value, _B0, _B1>::type {};

template<class _B0, class _B1, class _B2, class... _Bn>
struct __or_<_B0, _B1, _B2, _Bn...> 
        : conditional<_B0::value, _B0, __or_<_B1, _B2, _Bn...> >::type {};

// __not_
template<class _Tp> 
struct __not_ : conditional<_Tp::value, false_type, true_type>::type {};


// is_const

template <class _Tp> struct  is_const            : public false_type {};
template <class _Tp> struct  is_const<_Tp const> : public true_type {};


// is_volatile

template <class _Tp> struct  is_volatile               : public false_type {};
template <class _Tp> struct  is_volatile<_Tp volatile> : public true_type {};


// remove_const

template <class _Tp> struct  remove_const            {typedef _Tp type;};
template <class _Tp> struct  remove_const<const _Tp> {typedef _Tp type;};
template <class _Tp> using remove_const_t = typename remove_const<_Tp>::type;

// remove_volatile

template <class _Tp> struct  remove_volatile               {typedef _Tp type;};
template <class _Tp> struct  remove_volatile<volatile _Tp> {typedef _Tp type;};
template <class _Tp> using remove_volatile_t = typename remove_volatile<_Tp>::type;

// remove_cv

template <class _Tp> struct  remove_cv
{typedef typename remove_volatile<typename remove_const<_Tp>::type>::type type;};
template <class _Tp> using remove_cv_t = typename remove_cv<_Tp>::type;

// is_void

template <class _Tp> struct __libcpp_is_void       : public false_type {};
template <>          struct __libcpp_is_void<void> : public true_type {};

template <class _Tp> struct  is_void
    : public __libcpp_is_void<typename remove_cv<_Tp>::type> {};


// __is_nullptr_t

template <class _Tp> struct __is_nullptr_t_impl       : public false_type {};
template <>          struct __is_nullptr_t_impl<nullptr_t> : public true_type {};

template <class _Tp> struct  __is_nullptr_t
    : public __is_nullptr_t_impl<typename remove_cv<_Tp>::type> {};

template <class _Tp> struct  is_null_pointer
    : public __is_nullptr_t_impl<typename remove_cv<_Tp>::type> {};


// is_integral

template <class _Tp> struct __libcpp_is_integral                     : public false_type {};
template <>          struct __libcpp_is_integral<bool>               : public true_type {};
template <>          struct __libcpp_is_integral<char>               : public true_type {};
template <>          struct __libcpp_is_integral<signed char>        : public true_type {};
template <>          struct __libcpp_is_integral<unsigned char>      : public true_type {};
template <>          struct __libcpp_is_integral<wchar_t>            : public true_type {};
template <>          struct __libcpp_is_integral<char16_t>           : public true_type {};
template <>          struct __libcpp_is_integral<char32_t>           : public true_type {};
template <>          struct __libcpp_is_integral<short>              : public true_type {};
template <>          struct __libcpp_is_integral<unsigned short>     : public true_type {};
template <>          struct __libcpp_is_integral<int>                : public true_type {};
template <>          struct __libcpp_is_integral<unsigned int>       : public true_type {};
template <>          struct __libcpp_is_integral<long>               : public true_type {};
template <>          struct __libcpp_is_integral<unsigned long>      : public true_type {};
template <>          struct __libcpp_is_integral<long long>          : public true_type {};
template <>          struct __libcpp_is_integral<unsigned long long> : public true_type {};

template <class _Tp> struct  is_integral
    : public __libcpp_is_integral<typename remove_cv<_Tp>::type> {};


// is_floating_point

template <class _Tp> struct __libcpp_is_floating_point              : public false_type {};
template <>          struct __libcpp_is_floating_point<float>       : public true_type {};
template <>          struct __libcpp_is_floating_point<double>      : public true_type {};
template <>          struct __libcpp_is_floating_point<long double> : public true_type {};

template <class _Tp> struct  is_floating_point
    : public __libcpp_is_floating_point<typename remove_cv<_Tp>::type> {};


// is_array

template <class _Tp> struct  is_array
    : public false_type {};
template <class _Tp> struct  is_array<_Tp[]>
    : public true_type {};
template <class _Tp, size_t _Np> struct  is_array<_Tp[_Np]>
    : public true_type {};


// is_pointer

template <class _Tp> struct __libcpp_is_pointer       : public false_type {};
template <class _Tp> struct __libcpp_is_pointer<_Tp*> : public true_type {};

template <class _Tp> struct  is_pointer
    : public __libcpp_is_pointer<typename remove_cv<_Tp>::type> {};


// is_reference

template <class _Tp> struct  is_lvalue_reference       : public false_type {};
template <class _Tp> struct  is_lvalue_reference<_Tp&> : public true_type {};

template <class _Tp> struct  is_rvalue_reference        : public false_type {};
template <class _Tp> struct  is_rvalue_reference<_Tp&&> : public true_type {};

template <class _Tp> struct  is_reference        : public false_type {};
template <class _Tp> struct  is_reference<_Tp&>  : public true_type {};
template <class _Tp> struct  is_reference<_Tp&&> : public true_type {};

// is_union


template <class _Tp> struct  is_union
    : public integral_constant<bool, __is_union(_Tp)> {};



// is_class


template <class _Tp> struct  is_class
    : public integral_constant<bool, __is_class(_Tp)> {};



// is_same

template <class _Tp, class _Up> struct  is_same           : public false_type {};
template <class _Tp>            struct  is_same<_Tp, _Tp> : public true_type {};


// is_function

namespace __libcpp_is_function_imp
{
struct __dummy_type {};
template <class _Tp> char  __test(_Tp*);
template <class _Tp> char __test(__dummy_type);
template <class _Tp> __two __test(...);
template <class _Tp> _Tp&  __source(int);
template <class _Tp> __dummy_type __source(...);
}

template <class _Tp, bool = is_class<_Tp>::value ||
                            is_union<_Tp>::value ||
                            is_void<_Tp>::value  ||
                            is_reference<_Tp>::value ||
                            __is_nullptr_t<_Tp>::value >
struct __libcpp_is_function
    : public integral_constant<bool, sizeof(__libcpp_is_function_imp::__test<_Tp>(__libcpp_is_function_imp::__source<_Tp>(0))) == 1>
    {};
template <class _Tp> struct __libcpp_is_function<_Tp, true> : public false_type {};

template <class _Tp> struct  is_function
    : public __libcpp_is_function<_Tp> {};


// is_member_function_pointer

// template <class _Tp> struct            __libcpp_is_member_function_pointer             : public false_type {};
// template <class _Tp, class _Up> struct __libcpp_is_member_function_pointer<_Tp _Up::*> : public is_function<_Tp> {};
// 

template <class _MP, bool _IsMemberFunctionPtr, bool _IsMemberObjectPtr>
struct __member_pointer_traits_imp
{  // forward declaration; specializations later
};


template <class _Tp> struct __libcpp_is_member_function_pointer
    : public false_type {};

template <class _Ret, class _Class>
struct __libcpp_is_member_function_pointer<_Ret _Class::*>
    : public is_function<_Ret> {};

template <class _Tp> struct  is_member_function_pointer
    : public __libcpp_is_member_function_pointer<typename remove_cv<_Tp>::type>::type {};


// is_member_pointer

template <class _Tp>            struct __libcpp_is_member_pointer             : public false_type {};
template <class _Tp, class _Up> struct __libcpp_is_member_pointer<_Tp _Up::*> : public true_type {};

template <class _Tp> struct  is_member_pointer
    : public __libcpp_is_member_pointer<typename remove_cv<_Tp>::type> {};


// is_member_object_pointer

template <class _Tp> struct  is_member_object_pointer
    : public integral_constant<bool, is_member_pointer<_Tp>::value &&
                                    !is_member_function_pointer<_Tp>::value> {};


// is_enum


template <class _Tp> struct  is_enum
    : public integral_constant<bool, __is_enum(_Tp)> {};



// is_arithmetic

template <class _Tp> struct  is_arithmetic
    : public integral_constant<bool, is_integral<_Tp>::value      ||
                                     is_floating_point<_Tp>::value> {};


// is_fundamental

template <class _Tp> struct  is_fundamental
    : public integral_constant<bool, is_void<_Tp>::value        ||
                                     __is_nullptr_t<_Tp>::value ||
                                     is_arithmetic<_Tp>::value> {};


// is_scalar

template <class _Tp> struct  is_scalar
    : public integral_constant<bool, is_arithmetic<_Tp>::value     ||
                                     is_member_pointer<_Tp>::value ||
                                     is_pointer<_Tp>::value        ||
                                     __is_nullptr_t<_Tp>::value    ||
                                     is_enum<_Tp>::value           > {};

template <> struct  is_scalar<nullptr_t> : public true_type {};


// is_object

template <class _Tp> struct  is_object
    : public integral_constant<bool, is_scalar<_Tp>::value ||
                                     is_array<_Tp>::value  ||
                                     is_union<_Tp>::value  ||
                                     is_class<_Tp>::value  > {};


// is_compound

template <class _Tp> struct  is_compound
    : public integral_constant<bool, !is_fundamental<_Tp>::value> {};



// __is_referenceable  [defns.referenceable]

struct __is_referenceable_impl {
    template <class _Tp> static _Tp& __test(int);
    template <class _Tp> static __two __test(...);
};

template <class _Tp>
struct __is_referenceable : integral_constant<bool,
    !is_same<decltype(__is_referenceable_impl::__test<_Tp>(0)), __two>::value> {};


// add_const

template <class _Tp, bool = is_reference<_Tp>::value ||
                            is_function<_Tp>::value  ||
                            is_const<_Tp>::value     >
struct __add_const             {typedef _Tp type;};

template <class _Tp>
struct __add_const<_Tp, false> {typedef const _Tp type;};

template <class _Tp> struct  add_const
    {typedef typename __add_const<_Tp>::type type;};

template <class _Tp> using add_const_t = typename add_const<_Tp>::type;

// add_volatile

template <class _Tp, bool = is_reference<_Tp>::value ||
                            is_function<_Tp>::value  ||
                            is_volatile<_Tp>::value  >
struct __add_volatile             {typedef _Tp type;};

template <class _Tp>
struct __add_volatile<_Tp, false> {typedef volatile _Tp type;};

template <class _Tp> struct  add_volatile
    {typedef typename __add_volatile<_Tp>::type type;};

template <class _Tp> using add_volatile_t = typename add_volatile<_Tp>::type;

// add_cv

template <class _Tp> struct  add_cv
    {typedef typename add_const<typename add_volatile<_Tp>::type>::type type;};

template <class _Tp> using add_cv_t = typename add_cv<_Tp>::type;

// remove_reference

template <class _Tp> struct  remove_reference        {typedef _Tp type;};
template <class _Tp> struct  remove_reference<_Tp&>  {typedef _Tp type;};
template <class _Tp> struct  remove_reference<_Tp&&> {typedef _Tp type;};

template <class _Tp> using remove_reference_t = typename remove_reference<_Tp>::type;

// add_lvalue_reference

template <class _Tp, bool = __is_referenceable<_Tp>::value> struct __add_lvalue_reference_impl            { typedef _Tp  type; };
template <class _Tp                                       > struct __add_lvalue_reference_impl<_Tp, true> { typedef _Tp& type; };

template <class _Tp> struct  add_lvalue_reference
{typedef typename __add_lvalue_reference_impl<_Tp>::type type;};

template <class _Tp> using add_lvalue_reference_t = typename add_lvalue_reference<_Tp>::type;


template <class _Tp, bool = __is_referenceable<_Tp>::value> struct __add_rvalue_reference_impl            { typedef _Tp   type; };
template <class _Tp                                       > struct __add_rvalue_reference_impl<_Tp, true> { typedef _Tp&& type; };

template <class _Tp> struct  add_rvalue_reference
{typedef typename __add_rvalue_reference_impl<_Tp>::type type;};

template <class _Tp> using add_rvalue_reference_t = typename add_rvalue_reference<_Tp>::type;



template <class _Tp> _Tp&& __declval(int);
template <class _Tp> _Tp   __declval(long);

template <class _Tp>
decltype(std::__2::__declval<_Tp>(0))
declval() noexcept;


// __uncvref

template <class _Tp>
struct __uncvref  {
    typedef typename remove_cv<typename remove_reference<_Tp>::type>::type type;
};

template <class _Tp>
struct __unconstref {
    typedef typename remove_const<typename remove_reference<_Tp>::type>::type type;
};

template <class _Tp>
using __uncvref_t = typename __uncvref<_Tp>::type;

// __is_same_uncvref

template <class _Tp, class _Up>
struct __is_same_uncvref : is_same<typename __uncvref<_Tp>::type,
                                   typename __uncvref<_Up>::type> {};

struct __any
{
    __any(...);
};

// remove_pointer

template <class _Tp> struct  remove_pointer                      {typedef _Tp type;};
template <class _Tp> struct  remove_pointer<_Tp*>                {typedef _Tp type;};
template <class _Tp> struct  remove_pointer<_Tp* const>          {typedef _Tp type;};
template <class _Tp> struct  remove_pointer<_Tp* volatile>       {typedef _Tp type;};
template <class _Tp> struct  remove_pointer<_Tp* const volatile> {typedef _Tp type;};

template <class _Tp> using remove_pointer_t = typename remove_pointer<_Tp>::type;

// add_pointer

template <class _Tp, 
        bool = __is_referenceable<_Tp>::value || 
                is_same<typename remove_cv<_Tp>::type, void>::value>
struct __add_pointer_impl
    {typedef typename remove_reference<_Tp>::type* type;};
template <class _Tp> struct __add_pointer_impl<_Tp, false> 
    {typedef _Tp type;};

template <class _Tp> struct  add_pointer
    {typedef typename __add_pointer_impl<_Tp>::type type;};

template <class _Tp> using add_pointer_t = typename add_pointer<_Tp>::type;

// is_signed

template <class _Tp, bool = is_integral<_Tp>::value>
struct __libcpp_is_signed_impl : public integral_constant<bool,(_Tp(-1) < _Tp(0))> {};

template <class _Tp>
struct __libcpp_is_signed_impl<_Tp, false> : public true_type {};  // floating point

template <class _Tp, bool = is_arithmetic<_Tp>::value>
struct __libcpp_is_signed : public __libcpp_is_signed_impl<_Tp> {};

template <class _Tp> struct __libcpp_is_signed<_Tp, false> : public false_type {};

template <class _Tp> struct  is_signed : public __libcpp_is_signed<_Tp> {};


// is_unsigned

template <class _Tp, bool = is_integral<_Tp>::value>
struct __libcpp_is_unsigned_impl : public integral_constant<bool,(_Tp(0) < _Tp(-1))> {};

template <class _Tp>
struct __libcpp_is_unsigned_impl<_Tp, false> : public false_type {};  // floating point

template <class _Tp, bool = is_arithmetic<_Tp>::value>
struct __libcpp_is_unsigned : public __libcpp_is_unsigned_impl<_Tp> {};

template <class _Tp> struct __libcpp_is_unsigned<_Tp, false> : public false_type {};

template <class _Tp> struct  is_unsigned : public __libcpp_is_unsigned<_Tp> {};


// rank

template <class _Tp> struct  rank
    : public integral_constant<size_t, 0> {};
template <class _Tp> struct  rank<_Tp[]>
    : public integral_constant<size_t, rank<_Tp>::value + 1> {};
template <class _Tp, size_t _Np> struct  rank<_Tp[_Np]>
    : public integral_constant<size_t, rank<_Tp>::value + 1> {};


// extent

template <class _Tp, unsigned _Ip = 0> struct  extent
    : public integral_constant<size_t, 0> {};
template <class _Tp> struct  extent<_Tp[], 0>
    : public integral_constant<size_t, 0> {};
template <class _Tp, unsigned _Ip> struct  extent<_Tp[], _Ip>
    : public integral_constant<size_t, extent<_Tp, _Ip-1>::value> {};
template <class _Tp, size_t _Np> struct  extent<_Tp[_Np], 0>
    : public integral_constant<size_t, _Np> {};
template <class _Tp, size_t _Np, unsigned _Ip> struct  extent<_Tp[_Np], _Ip>
    : public integral_constant<size_t, extent<_Tp, _Ip-1>::value> {};


// remove_extent

template <class _Tp> struct  remove_extent
    {typedef _Tp type;};
template <class _Tp> struct  remove_extent<_Tp[]>
    {typedef _Tp type;};
template <class _Tp, size_t _Np> struct  remove_extent<_Tp[_Np]>
    {typedef _Tp type;};

template <class _Tp> using remove_extent_t = typename remove_extent<_Tp>::type;

// remove_all_extents

template <class _Tp> struct  remove_all_extents
    {typedef _Tp type;};
template <class _Tp> struct  remove_all_extents<_Tp[]>
    {typedef typename remove_all_extents<_Tp>::type type;};
template <class _Tp, size_t _Np> struct  remove_all_extents<_Tp[_Np]>
    {typedef typename remove_all_extents<_Tp>::type type;};

template <class _Tp> using remove_all_extents_t = typename remove_all_extents<_Tp>::type;

// decay

template <class _Up, bool>
struct __decay {
    typedef typename remove_cv<_Up>::type type;
};

template <class _Up>
struct __decay<_Up, true> {
public:
    typedef typename conditional
                     <
                         is_array<_Up>::value,
                         typename remove_extent<_Up>::type*,
                         typename conditional
                         <
                              is_function<_Up>::value,
                              typename add_pointer<_Up>::type,
                              typename remove_cv<_Up>::type
                         >::type
                     >::type type;
};

template <class _Tp>
struct  decay
{
private:
    typedef typename remove_reference<_Tp>::type _Up;
public:
    typedef typename __decay<_Up, __is_referenceable<_Up>::value>::type type;
};

template <class _Tp> using decay_t = typename decay<_Tp>::type;

// is_abstract

template <class _Tp> struct  is_abstract
    : public integral_constant<bool, __is_abstract(_Tp)> {};


// is_final

template <class _Tp> struct 
__libcpp_is_final : public integral_constant<bool, __is_final(_Tp)> {};

template <class _Tp> struct 
is_final : public integral_constant<bool, __is_final(_Tp)> {};


// is_aggregate

// is_base_of


template <class _Bp, class _Dp>
struct  is_base_of
    : public integral_constant<bool, __is_base_of(_Bp, _Dp)> {};



// is_convertible


template <class _T1, class _T2> struct  is_convertible
    : public integral_constant<bool, __is_convertible_to(_T1, _T2) &&
                                     !is_abstract<_T2>::value> {};



// is_empty


template <class _Tp>
struct  is_empty
    : public integral_constant<bool, __is_empty(_Tp)> {};



// is_polymorphic


template <class _Tp>
struct  is_polymorphic
    : public integral_constant<bool, __is_polymorphic(_Tp)> {};



// has_virtual_destructor


template <class _Tp> struct  has_virtual_destructor
    : public integral_constant<bool, __has_virtual_destructor(_Tp)> {};



// alignment_of

template <class _Tp> struct  alignment_of
    : public integral_constant<size_t, __alignof__(_Tp)> {};


// aligned_storage

template <class _Hp, class _Tp>
struct __type_list
{
    typedef _Hp _Head;
    typedef _Tp _Tail;
};

struct __nat
{
    __nat() = delete;
    __nat(const __nat&) = delete;
    __nat& operator=(const __nat&) = delete;
    ~__nat() = delete;
};

template <class _Tp>
struct __align_type
{
    static const size_t value = alignment_of<_Tp>::value;
    typedef _Tp type;
};

struct __struct_double {long double __lx;};
struct __struct_double4 {double __lx[4];};

typedef
    __type_list<__align_type<unsigned char>,
    __type_list<__align_type<unsigned short>,
    __type_list<__align_type<unsigned int>,
    __type_list<__align_type<unsigned long>,
    __type_list<__align_type<unsigned long long>,
    __type_list<__align_type<double>,
    __type_list<__align_type<long double>,
    __type_list<__align_type<__struct_double>,
    __type_list<__align_type<__struct_double4>,
    __type_list<__align_type<int*>,
    __nat
    > > > > > > > > > > __all_types;

template <class _TL, size_t _Align> struct __find_pod;

template <class _Hp, size_t _Align>
struct __find_pod<__type_list<_Hp, __nat>, _Align>
{
    typedef typename conditional<
                             _Align == _Hp::value,
                             typename _Hp::type,
                             void
                         >::type type;
};

template <class _Hp, class _Tp, size_t _Align>
struct __find_pod<__type_list<_Hp, _Tp>, _Align>
{
    typedef typename conditional<
                             _Align == _Hp::value,
                             typename _Hp::type,
                             typename __find_pod<_Tp, _Align>::type
                         >::type type;
};

template <class _TL, size_t _Len> struct __find_max_align;

template <class _Hp, size_t _Len>
struct __find_max_align<__type_list<_Hp, __nat>, _Len> : public integral_constant<size_t, _Hp::value> {};

template <size_t _Len, size_t _A1, size_t _A2>
struct __select_align
{
private:
    static const size_t __min = _A2 < _A1 ? _A2 : _A1;
    static const size_t __max = _A1 < _A2 ? _A2 : _A1;
public:
    static const size_t value = _Len < __max ? __min : __max;
};

template <class _Hp, class _Tp, size_t _Len>
struct __find_max_align<__type_list<_Hp, _Tp>, _Len>
    : public integral_constant<size_t, __select_align<_Len, _Hp::value, __find_max_align<_Tp, _Len>::value>::value> {};

template <size_t _Len, size_t _Align = __find_max_align<__all_types, _Len>::value>
struct  aligned_storage
{
    typedef typename __find_pod<__all_types, _Align>::type _Aligner;
    static_assert(!is_void<_Aligner>::value, "");
    union type
    {
        _Aligner __align;
        unsigned char __data[(_Len + _Align - 1)/_Align * _Align];
    };
};

template <size_t _Len, size_t _Align = __find_max_align<__all_types, _Len>::value>
    using aligned_storage_t = typename aligned_storage<_Len, _Align>::type;


template <size_t _Len>struct  aligned_storage<_Len, 0x1>{ struct alignas(0x1) type { unsigned char __lx[(_Len + 0x1 - 1)/0x1 * 0x1]; };};
template <size_t _Len>struct  aligned_storage<_Len, 0x2>{ struct alignas(0x2) type { unsigned char __lx[(_Len + 0x2 - 1)/0x2 * 0x2]; };};
template <size_t _Len>struct  aligned_storage<_Len, 0x4>{ struct alignas(0x4) type { unsigned char __lx[(_Len + 0x4 - 1)/0x4 * 0x4]; };};
template <size_t _Len>struct  aligned_storage<_Len, 0x8>{ struct alignas(0x8) type { unsigned char __lx[(_Len + 0x8 - 1)/0x8 * 0x8]; };};
template <size_t _Len>struct  aligned_storage<_Len, 0x10>{ struct alignas(0x10) type { unsigned char __lx[(_Len + 0x10 - 1)/0x10 * 0x10]; };};
template <size_t _Len>struct  aligned_storage<_Len, 0x20>{ struct alignas(0x20) type { unsigned char __lx[(_Len + 0x20 - 1)/0x20 * 0x20]; };};
template <size_t _Len>struct  aligned_storage<_Len, 0x40>{ struct alignas(0x40) type { unsigned char __lx[(_Len + 0x40 - 1)/0x40 * 0x40]; };};
template <size_t _Len>struct  aligned_storage<_Len, 0x80>{ struct alignas(0x80) type { unsigned char __lx[(_Len + 0x80 - 1)/0x80 * 0x80]; };};
template <size_t _Len>struct  aligned_storage<_Len, 0x100>{ struct alignas(0x100) type { unsigned char __lx[(_Len + 0x100 - 1)/0x100 * 0x100]; };};
template <size_t _Len>struct  aligned_storage<_Len, 0x200>{ struct alignas(0x200) type { unsigned char __lx[(_Len + 0x200 - 1)/0x200 * 0x200]; };};
template <size_t _Len>struct  aligned_storage<_Len, 0x400>{ struct alignas(0x400) type { unsigned char __lx[(_Len + 0x400 - 1)/0x400 * 0x400]; };};
template <size_t _Len>struct  aligned_storage<_Len, 0x800>{ struct alignas(0x800) type { unsigned char __lx[(_Len + 0x800 - 1)/0x800 * 0x800]; };};
template <size_t _Len>struct  aligned_storage<_Len, 0x1000>{ struct alignas(0x1000) type { unsigned char __lx[(_Len + 0x1000 - 1)/0x1000 * 0x1000]; };};
template <size_t _Len>struct  aligned_storage<_Len, 0x2000>{ struct alignas(0x2000) type { unsigned char __lx[(_Len + 0x2000 - 1)/0x2000 * 0x2000]; };};
// PE/COFF does not support alignment beyond 8192 (=0x2000)
template <size_t _Len>struct  aligned_storage<_Len, 0x4000>{ struct alignas(0x4000) type { unsigned char __lx[(_Len + 0x4000 - 1)/0x4000 * 0x4000]; };};



// aligned_union

template <size_t _I0, size_t ..._In>
struct __static_max;

template <size_t _I0>
struct __static_max<_I0>
{
    static const size_t value = _I0;
};

template <size_t _I0, size_t _I1, size_t ..._In>
struct __static_max<_I0, _I1, _In...>
{
    static const size_t value = _I0 >= _I1 ? __static_max<_I0, _In...>::value :
                                             __static_max<_I1, _In...>::value;
};

template <size_t _Len, class _Type0, class ..._Types>
struct aligned_union
{
    static const size_t alignment_value = __static_max<__alignof__(_Type0),
                                                       __alignof__(_Types)...>::value;
    static const size_t __len = __static_max<_Len, sizeof(_Type0),
                                             sizeof(_Types)...>::value;
    typedef typename aligned_storage<__len, alignment_value>::type type;
};

template <size_t _Len, class ..._Types> using aligned_union_t = typename aligned_union<_Len, _Types...>::type;


template <class _Tp>
struct __numeric_type
{
   static void __test(...);
   static float __test(float);
   static double __test(char);
   static double __test(int);
   static double __test(unsigned);
   static double __test(long);
   static double __test(unsigned long);
   static double __test(long long);
   static double __test(unsigned long long);
   static double __test(double);
   static long double __test(long double);

   typedef decltype(__test(declval<_Tp>())) type;
   static const bool value = !is_same<type, void>::value;
};

template <>
struct __numeric_type<void>
{
   static const bool value = true;
};

// __promote

template <class _A1, class _A2 = void, class _A3 = void,
          bool = __numeric_type<_A1>::value &&
                 __numeric_type<_A2>::value &&
                 __numeric_type<_A3>::value>
class __promote_imp
{
public:
    static const bool value = false;
};

template <class _A1, class _A2, class _A3>
class __promote_imp<_A1, _A2, _A3, true>
{
private:
    typedef typename __promote_imp<_A1>::type __type1;
    typedef typename __promote_imp<_A2>::type __type2;
    typedef typename __promote_imp<_A3>::type __type3;
public:
    typedef decltype(__type1() + __type2() + __type3()) type;
    static const bool value = true;
};

template <class _A1, class _A2>
class __promote_imp<_A1, _A2, void, true>
{
private:
    typedef typename __promote_imp<_A1>::type __type1;
    typedef typename __promote_imp<_A2>::type __type2;
public:
    typedef decltype(__type1() + __type2()) type;
    static const bool value = true;
};

template <class _A1>
class __promote_imp<_A1, void, void, true>
{
public:
    typedef typename __numeric_type<_A1>::type type;
    static const bool value = true;
};

template <class _A1, class _A2 = void, class _A3 = void>
class __promote : public __promote_imp<_A1, _A2, _A3> {};

// make_signed / make_unsigned

typedef
    __type_list<signed char,
    __type_list<signed short,
    __type_list<signed int,
    __type_list<signed long,
    __type_list<signed long long,
    __nat
    > > > > > __signed_types;

typedef
    __type_list<unsigned char,
    __type_list<unsigned short,
    __type_list<unsigned int,
    __type_list<unsigned long,
    __type_list<unsigned long long,
    __nat
    > > > > > __unsigned_types;

template <class _TypeList, size_t _Size, bool = _Size <= sizeof(typename _TypeList::_Head)> struct __find_first;

template <class _Hp, class _Tp, size_t _Size>
struct __find_first<__type_list<_Hp, _Tp>, _Size, true>
{
    typedef _Hp type;
};

template <class _Hp, class _Tp, size_t _Size>
struct __find_first<__type_list<_Hp, _Tp>, _Size, false>
{
    typedef typename __find_first<_Tp, _Size>::type type;
};

template <class _Tp, class _Up, bool = is_const<typename remove_reference<_Tp>::type>::value,
                             bool = is_volatile<typename remove_reference<_Tp>::type>::value>
struct __apply_cv
{
    typedef _Up type;
};

template <class _Tp, class _Up>
struct __apply_cv<_Tp, _Up, true, false>
{
    typedef const _Up type;
};

template <class _Tp, class _Up>
struct __apply_cv<_Tp, _Up, false, true>
{
    typedef volatile _Up type;
};

template <class _Tp, class _Up>
struct __apply_cv<_Tp, _Up, true, true>
{
    typedef const volatile _Up type;
};

template <class _Tp, class _Up>
struct __apply_cv<_Tp&, _Up, false, false>
{
    typedef _Up& type;
};

template <class _Tp, class _Up>
struct __apply_cv<_Tp&, _Up, true, false>
{
    typedef const _Up& type;
};

template <class _Tp, class _Up>
struct __apply_cv<_Tp&, _Up, false, true>
{
    typedef volatile _Up& type;
};

template <class _Tp, class _Up>
struct __apply_cv<_Tp&, _Up, true, true>
{
    typedef const volatile _Up& type;
};

template <class _Tp, bool = is_integral<_Tp>::value || is_enum<_Tp>::value>
struct __make_signed {};

template <class _Tp>
struct __make_signed<_Tp, true>
{
    typedef typename __find_first<__signed_types, sizeof(_Tp)>::type type;
};

template <> struct __make_signed<bool,               true> {};
template <> struct __make_signed<  signed short,     true> {typedef short     type;};
template <> struct __make_signed<unsigned short,     true> {typedef short     type;};
template <> struct __make_signed<  signed int,       true> {typedef int       type;};
template <> struct __make_signed<unsigned int,       true> {typedef int       type;};
template <> struct __make_signed<  signed long,      true> {typedef long      type;};
template <> struct __make_signed<unsigned long,      true> {typedef long      type;};
template <> struct __make_signed<  signed long long, true> {typedef long long type;};
template <> struct __make_signed<unsigned long long, true> {typedef long long type;};

template <class _Tp>
struct  make_signed
{
    typedef typename __apply_cv<_Tp, typename __make_signed<typename remove_cv<_Tp>::type>::type>::type type;
};

template <class _Tp> using make_signed_t = typename make_signed<_Tp>::type;

template <class _Tp, bool = is_integral<_Tp>::value || is_enum<_Tp>::value>
struct __make_unsigned {};

template <class _Tp>
struct __make_unsigned<_Tp, true>
{
    typedef typename __find_first<__unsigned_types, sizeof(_Tp)>::type type;
};

template <> struct __make_unsigned<bool,               true> {};
template <> struct __make_unsigned<  signed short,     true> {typedef unsigned short     type;};
template <> struct __make_unsigned<unsigned short,     true> {typedef unsigned short     type;};
template <> struct __make_unsigned<  signed int,       true> {typedef unsigned int       type;};
template <> struct __make_unsigned<unsigned int,       true> {typedef unsigned int       type;};
template <> struct __make_unsigned<  signed long,      true> {typedef unsigned long      type;};
template <> struct __make_unsigned<unsigned long,      true> {typedef unsigned long      type;};
template <> struct __make_unsigned<  signed long long, true> {typedef unsigned long long type;};
template <> struct __make_unsigned<unsigned long long, true> {typedef unsigned long long type;};

template <class _Tp>
struct  make_unsigned
{
    typedef typename __apply_cv<_Tp, typename __make_unsigned<typename remove_cv<_Tp>::type>::type>::type type;
};

template <class _Tp> using make_unsigned_t = typename make_unsigned<_Tp>::type;


// bullet 1 - sizeof...(Tp) == 0

template <class ..._Tp>
struct  common_type {};

// bullet 2 - sizeof...(Tp) == 1

template <class _Tp>
struct  common_type<_Tp>
    : public common_type<_Tp, _Tp> {};

// bullet 3 - sizeof...(Tp) == 2

template <class _Tp, class _Up, class = void>
struct __common_type2_imp {};

template <class _Tp, class _Up>
struct __common_type2_imp<_Tp, _Up,
    typename __void_t<decltype(
        true ? std::__2::declval<_Tp>() : std::__2::declval<_Up>()
    )>::type>
{
    typedef typename decay<decltype(
        true ? std::__2::declval<_Tp>() : std::__2::declval<_Up>()
    )>::type type;
};

template <class _Tp, class _Up,
          class _DTp = typename decay<_Tp>::type,
          class _DUp = typename decay<_Up>::type>
using __common_type2 =
  typename conditional<
    is_same<_Tp, _DTp>::value && is_same<_Up, _DUp>::value,
    __common_type2_imp<_Tp, _Up>,
    common_type<_DTp, _DUp>
  >::type;

template <class _Tp, class _Up>
struct  common_type<_Tp, _Up>
    : __common_type2<_Tp, _Up> {};

// bullet 4 - sizeof...(Tp) > 2

template <class ...Tp> struct __common_types;

template <class, class = void>
struct __common_type_impl {};

template <class _Tp, class _Up>
struct __common_type_impl<
    __common_types<_Tp, _Up>,
    typename __void_t<typename common_type<_Tp, _Up>::type>::type>
{
  typedef typename common_type<_Tp, _Up>::type type;
};

template <class _Tp, class _Up, class ..._Vp>
struct __common_type_impl<__common_types<_Tp, _Up, _Vp...>,
    typename __void_t<typename common_type<_Tp, _Up>::type>::type>
  : __common_type_impl<
      __common_types<typename common_type<_Tp, _Up>::type, _Vp...> >
{

};

template <class _Tp, class _Up, class ..._Vp>
struct  common_type<_Tp, _Up, _Vp...>
    : __common_type_impl<__common_types<_Tp, _Up, _Vp...> > {};

template <class ..._Tp> using common_type_t = typename common_type<_Tp...>::type;


// is_assignable

template<typename, typename _Tp> struct __select_2nd { typedef _Tp type; };

template <class _Tp, class _Arg>
typename __select_2nd<decltype((std::__2::declval<_Tp>() = std::__2::declval<_Arg>())), true_type>::type
__is_assignable_test(int);

template <class, class>
false_type __is_assignable_test(...);


template <class _Tp, class _Arg, bool = is_void<_Tp>::value || is_void<_Arg>::value>
struct __is_assignable_imp
    : public decltype((std::__2::__is_assignable_test<_Tp, _Arg>(0))) {};

template <class _Tp, class _Arg>
struct __is_assignable_imp<_Tp, _Arg, true>
    : public false_type
{
};

template <class _Tp, class _Arg>
struct is_assignable
    : public __is_assignable_imp<_Tp, _Arg> {};


// is_copy_assignable

template <class _Tp> struct  is_copy_assignable
    : public is_assignable<typename add_lvalue_reference<_Tp>::type,
                  typename add_lvalue_reference<typename add_const<_Tp>::type>::type> {};


// is_move_assignable

template <class _Tp> struct  is_move_assignable
    : public is_assignable<typename add_lvalue_reference<_Tp>::type,
                           typename add_rvalue_reference<_Tp>::type> {};


// is_destructible

//  if it's a reference, return true
//  if it's a function, return false
//  if it's   void,     return false
//  if it's an array of unknown bound, return false
//  Otherwise, return "std::declval<_Up&>().~_Up()" is well-formed
//    where _Up is remove_all_extents<_Tp>::type

template <class>
struct __is_destructible_apply { typedef int type; };

template <typename _Tp>
struct __is_destructor_wellformed {
    template <typename _Tp1>
    static char  __test (
        typename __is_destructible_apply<decltype(std::__2::declval<_Tp1&>().~_Tp1())>::type
    );

    template <typename _Tp1>
    static __two __test (...);
    
    static const bool value = sizeof(__test<_Tp>(12)) == sizeof(char);
};

template <class _Tp, bool>
struct __destructible_imp;

template <class _Tp>
struct __destructible_imp<_Tp, false> 
   : public std::__2::integral_constant<bool, 
        __is_destructor_wellformed<typename std::__2::remove_all_extents<_Tp>::type>::value> {};

template <class _Tp>
struct __destructible_imp<_Tp, true>
    : public std::__2::true_type {};

template <class _Tp, bool>
struct __destructible_false;

template <class _Tp>
struct __destructible_false<_Tp, false> : public __destructible_imp<_Tp, std::__2::is_reference<_Tp>::value> {};

template <class _Tp>
struct __destructible_false<_Tp, true> : public std::__2::false_type {};

template <class _Tp>
struct is_destructible
    : public __destructible_false<_Tp, std::__2::is_function<_Tp>::value> {};

template <class _Tp>
struct is_destructible<_Tp[]>
    : public std::__2::false_type {};

template <>
struct is_destructible<void>
    : public std::__2::false_type {};


// move


template <class _Tp>
inline __attribute__ ((__always_inline__)) constexpr
typename remove_reference<_Tp>::type&&
move(_Tp&& __t) noexcept
{
    typedef typename remove_reference<_Tp>::type _Up;
    return static_cast<_Up&&>(__t);
}

template <class _Tp>
inline __attribute__ ((__always_inline__)) constexpr
_Tp&&
forward(typename remove_reference<_Tp>::type& __t) noexcept
{
    return static_cast<_Tp&&>(__t);
}

template <class _Tp>
inline __attribute__ ((__always_inline__)) constexpr
_Tp&&
forward(typename remove_reference<_Tp>::type&& __t) noexcept
{
    static_assert(!is_lvalue_reference<_Tp>::value,
                  "can not forward an rvalue as an lvalue");
    return static_cast<_Tp&&>(__t);
}



template <class _Tp>
inline __attribute__ ((__always_inline__))
typename decay<_Tp>::type
__decay_copy(_Tp&& __t)
{
    return std::__2::forward<_Tp>(__t);
}



template <class _Rp, class _Class, class ..._Param>
struct __member_pointer_traits_imp<_Rp (_Class::*)(_Param...), true, false>
{
    typedef _Class _ClassType;
    typedef _Rp _ReturnType;
    typedef _Rp (_FnType) (_Param...);
};

template <class _Rp, class _Class, class ..._Param>
struct __member_pointer_traits_imp<_Rp (_Class::*)(_Param..., ...), true, false>
{
    typedef _Class _ClassType;
    typedef _Rp _ReturnType;
    typedef _Rp (_FnType) (_Param..., ...);
};

template <class _Rp, class _Class, class ..._Param>
struct __member_pointer_traits_imp<_Rp (_Class::*)(_Param...) const, true, false>
{
    typedef _Class const _ClassType;
    typedef _Rp _ReturnType;
    typedef _Rp (_FnType) (_Param...);
};

template <class _Rp, class _Class, class ..._Param>
struct __member_pointer_traits_imp<_Rp (_Class::*)(_Param..., ...) const, true, false>
{
    typedef _Class const _ClassType;
    typedef _Rp _ReturnType;
    typedef _Rp (_FnType) (_Param..., ...);
};

template <class _Rp, class _Class, class ..._Param>
struct __member_pointer_traits_imp<_Rp (_Class::*)(_Param...) volatile, true, false>
{
    typedef _Class volatile _ClassType;
    typedef _Rp _ReturnType;
    typedef _Rp (_FnType) (_Param...);
};

template <class _Rp, class _Class, class ..._Param>
struct __member_pointer_traits_imp<_Rp (_Class::*)(_Param..., ...) volatile, true, false>
{
    typedef _Class volatile _ClassType;
    typedef _Rp _ReturnType;
    typedef _Rp (_FnType) (_Param..., ...);
};

template <class _Rp, class _Class, class ..._Param>
struct __member_pointer_traits_imp<_Rp (_Class::*)(_Param...) const volatile, true, false>
{
    typedef _Class const volatile _ClassType;
    typedef _Rp _ReturnType;
    typedef _Rp (_FnType) (_Param...);
};

template <class _Rp, class _Class, class ..._Param>
struct __member_pointer_traits_imp<_Rp (_Class::*)(_Param..., ...) const volatile, true, false>
{
    typedef _Class const volatile _ClassType;
    typedef _Rp _ReturnType;
    typedef _Rp (_FnType) (_Param..., ...);
};


template <class _Rp, class _Class, class ..._Param>
struct __member_pointer_traits_imp<_Rp (_Class::*)(_Param...) &, true, false>
{
    typedef _Class& _ClassType;
    typedef _Rp _ReturnType;
    typedef _Rp (_FnType) (_Param...);
};

template <class _Rp, class _Class, class ..._Param>
struct __member_pointer_traits_imp<_Rp (_Class::*)(_Param..., ...) &, true, false>
{
    typedef _Class& _ClassType;
    typedef _Rp _ReturnType;
    typedef _Rp (_FnType) (_Param..., ...);
};

template <class _Rp, class _Class, class ..._Param>
struct __member_pointer_traits_imp<_Rp (_Class::*)(_Param...) const&, true, false>
{
    typedef _Class const& _ClassType;
    typedef _Rp _ReturnType;
    typedef _Rp (_FnType) (_Param...);
};

template <class _Rp, class _Class, class ..._Param>
struct __member_pointer_traits_imp<_Rp (_Class::*)(_Param..., ...) const&, true, false>
{
    typedef _Class const& _ClassType;
    typedef _Rp _ReturnType;
    typedef _Rp (_FnType) (_Param..., ...);
};

template <class _Rp, class _Class, class ..._Param>
struct __member_pointer_traits_imp<_Rp (_Class::*)(_Param...) volatile&, true, false>
{
    typedef _Class volatile& _ClassType;
    typedef _Rp _ReturnType;
    typedef _Rp (_FnType) (_Param...);
};

template <class _Rp, class _Class, class ..._Param>
struct __member_pointer_traits_imp<_Rp (_Class::*)(_Param..., ...) volatile&, true, false>
{
    typedef _Class volatile& _ClassType;
    typedef _Rp _ReturnType;
    typedef _Rp (_FnType) (_Param..., ...);
};

template <class _Rp, class _Class, class ..._Param>
struct __member_pointer_traits_imp<_Rp (_Class::*)(_Param...) const volatile&, true, false>
{
    typedef _Class const volatile& _ClassType;
    typedef _Rp _ReturnType;
    typedef _Rp (_FnType) (_Param...);
};

template <class _Rp, class _Class, class ..._Param>
struct __member_pointer_traits_imp<_Rp (_Class::*)(_Param..., ...) const volatile&, true, false>
{
    typedef _Class const volatile& _ClassType;
    typedef _Rp _ReturnType;
    typedef _Rp (_FnType) (_Param..., ...);
};

template <class _Rp, class _Class, class ..._Param>
struct __member_pointer_traits_imp<_Rp (_Class::*)(_Param...) &&, true, false>
{
    typedef _Class&& _ClassType;
    typedef _Rp _ReturnType;
    typedef _Rp (_FnType) (_Param...);
};

template <class _Rp, class _Class, class ..._Param>
struct __member_pointer_traits_imp<_Rp (_Class::*)(_Param..., ...) &&, true, false>
{
    typedef _Class&& _ClassType;
    typedef _Rp _ReturnType;
    typedef _Rp (_FnType) (_Param..., ...);
};

template <class _Rp, class _Class, class ..._Param>
struct __member_pointer_traits_imp<_Rp (_Class::*)(_Param...) const&&, true, false>
{
    typedef _Class const&& _ClassType;
    typedef _Rp _ReturnType;
    typedef _Rp (_FnType) (_Param...);
};

template <class _Rp, class _Class, class ..._Param>
struct __member_pointer_traits_imp<_Rp (_Class::*)(_Param..., ...) const&&, true, false>
{
    typedef _Class const&& _ClassType;
    typedef _Rp _ReturnType;
    typedef _Rp (_FnType) (_Param..., ...);
};

template <class _Rp, class _Class, class ..._Param>
struct __member_pointer_traits_imp<_Rp (_Class::*)(_Param...) volatile&&, true, false>
{
    typedef _Class volatile&& _ClassType;
    typedef _Rp _ReturnType;
    typedef _Rp (_FnType) (_Param...);
};

template <class _Rp, class _Class, class ..._Param>
struct __member_pointer_traits_imp<_Rp (_Class::*)(_Param..., ...) volatile&&, true, false>
{
    typedef _Class volatile&& _ClassType;
    typedef _Rp _ReturnType;
    typedef _Rp (_FnType) (_Param..., ...);
};

template <class _Rp, class _Class, class ..._Param>
struct __member_pointer_traits_imp<_Rp (_Class::*)(_Param...) const volatile&&, true, false>
{
    typedef _Class const volatile&& _ClassType;
    typedef _Rp _ReturnType;
    typedef _Rp (_FnType) (_Param...);
};

template <class _Rp, class _Class, class ..._Param>
struct __member_pointer_traits_imp<_Rp (_Class::*)(_Param..., ...) const volatile&&, true, false>
{
    typedef _Class const volatile&& _ClassType;
    typedef _Rp _ReturnType;
    typedef _Rp (_FnType) (_Param..., ...);
};



template <class _Rp, class _Class>
struct __member_pointer_traits_imp<_Rp _Class::*, false, true>
{
    typedef _Class _ClassType;
    typedef _Rp _ReturnType;
};

template <class _MP>
struct __member_pointer_traits
    : public __member_pointer_traits_imp<typename remove_cv<_MP>::type,
                    is_member_function_pointer<_MP>::value,
                    is_member_object_pointer<_MP>::value>
{
//     typedef ... _ClassType;
//     typedef ... _ReturnType;
//     typedef ... _FnType;
};


template <class _DecayedFp>
struct __member_pointer_class_type {};

template <class _Ret, class _ClassType>
struct __member_pointer_class_type<_Ret _ClassType::*> {
  typedef _ClassType type;
};

// result_of

template <class _Callable> class result_of;


// template <class T, class... Args> struct is_constructible;

namespace __is_construct
{
struct __nat {};
}


template <class _Tp, class... _Args>
struct __libcpp_is_constructible;

template <class _To, class _From>
struct __is_invalid_base_to_derived_cast {
  static_assert(is_reference<_To>::value, "Wrong specialization");
  using _RawFrom = __uncvref_t<_From>;
  using _RawTo = __uncvref_t<_To>;
  static const bool value = __lazy_and<
        __lazy_not<is_same<_RawFrom, _RawTo>>,
        is_base_of<_RawFrom, _RawTo>,
        __lazy_not<__libcpp_is_constructible<_RawTo, _From>>
  >::value;
};

template <class _To, class _From>
struct __is_invalid_lvalue_to_rvalue_cast : false_type {
  static_assert(is_reference<_To>::value, "Wrong specialization");
};

template <class _ToRef, class _FromRef>
struct __is_invalid_lvalue_to_rvalue_cast<_ToRef&&, _FromRef&> {
  using _RawFrom = __uncvref_t<_FromRef>;
  using _RawTo = __uncvref_t<_ToRef>;
  static const bool value = __lazy_and<
      __lazy_not<is_function<_RawTo>>,
      __lazy_or<
        is_same<_RawFrom, _RawTo>,
        is_base_of<_RawTo, _RawFrom>>
    >::value;
};

struct __is_constructible_helper
{
    template <class _To>
    static void __eat(_To);

    // This overload is needed to work around a Clang bug that disallows
    // static_cast<T&&>(e) for non-reference-compatible types.
    // Example: static_cast<int&&>(declval<double>());
    // NOTE: The static_cast implementation below is required to support
    //  classes with explicit conversion operators.
    template <class _To, class _From,
              class = decltype(__eat<_To>(std::__2::declval<_From>()))>
    static true_type __test_cast(int);

    template <class _To, class _From,
              class = decltype(static_cast<_To>(std::__2::declval<_From>()))>
    static integral_constant<bool,
        !__is_invalid_base_to_derived_cast<_To, _From>::value &&
        !__is_invalid_lvalue_to_rvalue_cast<_To, _From>::value
    > __test_cast(long);

    template <class, class>
    static false_type __test_cast(...);

    template <class _Tp, class ..._Args,
        class = decltype(_Tp(std::__2::declval<_Args>()...))>
    static true_type __test_nary(int);
    template <class _Tp, class...>
    static false_type __test_nary(...);

    template <class _Tp, class _A0, class = decltype(::new _Tp(std::__2::declval<_A0>()))>
    static is_destructible<_Tp> __test_unary(int);
    template <class, class>
    static false_type __test_unary(...);
};

template <class _Tp, bool = is_void<_Tp>::value>
struct __is_default_constructible
    : decltype(__is_constructible_helper::__test_nary<_Tp>(0))
{};

template <class _Tp>
struct __is_default_constructible<_Tp, true> : false_type {};

template <class _Tp>
struct __is_default_constructible<_Tp[], false> : false_type {};

template <class _Tp, size_t _Nx>
struct __is_default_constructible<_Tp[_Nx], false>
    : __is_default_constructible<typename remove_all_extents<_Tp>::type>  {};

template <class _Tp, class... _Args>
struct __libcpp_is_constructible
{
  static_assert(sizeof...(_Args) > 1, "Wrong specialization");
  typedef decltype(__is_constructible_helper::__test_nary<_Tp, _Args...>(0))
      type;
};

template <class _Tp>
struct __libcpp_is_constructible<_Tp> : __is_default_constructible<_Tp> {};

template <class _Tp, class _A0>
struct __libcpp_is_constructible<_Tp, _A0>
    : public decltype(__is_constructible_helper::__test_unary<_Tp, _A0>(0))
{};

template <class _Tp, class _A0>
struct __libcpp_is_constructible<_Tp&, _A0>
    : public decltype(__is_constructible_helper::
    __test_cast<_Tp&, _A0>(0))
{};

template <class _Tp, class _A0>
struct __libcpp_is_constructible<_Tp&&, _A0>
    : public decltype(__is_constructible_helper::
    __test_cast<_Tp&&, _A0>(0))
{};


template <class _Tp, class... _Args>
struct  is_constructible
    : public __libcpp_is_constructible<_Tp, _Args...>::type {};



// is_default_constructible

template <class _Tp>
struct  is_default_constructible
    : public is_constructible<_Tp>
    {};


// is_copy_constructible

template <class _Tp>
struct  is_copy_constructible
    : public is_constructible<_Tp, 
                  typename add_lvalue_reference<typename add_const<_Tp>::type>::type> {};


// is_move_constructible

template <class _Tp>
struct  is_move_constructible
    : public is_constructible<_Tp, typename add_rvalue_reference<_Tp>::type>
    {};


// is_trivially_constructible



template <class _Tp, class... _Args>
struct  is_trivially_constructible
    : integral_constant<bool, __is_trivially_constructible(_Tp, _Args...)>
{
};




// is_trivially_default_constructible

template <class _Tp> struct  is_trivially_default_constructible
    : public is_trivially_constructible<_Tp>
    {};


// is_trivially_copy_constructible

template <class _Tp> struct  is_trivially_copy_constructible
    : public is_trivially_constructible<_Tp, typename add_lvalue_reference<const _Tp>::type>
    {};


// is_trivially_move_constructible

template <class _Tp> struct  is_trivially_move_constructible
    : public is_trivially_constructible<_Tp, typename add_rvalue_reference<_Tp>::type>
    {};


// is_trivially_assignable


template <class _Tp, class _Arg>
struct is_trivially_assignable
    : integral_constant<bool, __is_trivially_assignable(_Tp, _Arg)>
{
};



// is_trivially_copy_assignable

template <class _Tp> struct  is_trivially_copy_assignable
    : public is_trivially_assignable<typename add_lvalue_reference<_Tp>::type,
                  typename add_lvalue_reference<typename add_const<_Tp>::type>::type> {};


// is_trivially_move_assignable

template <class _Tp> struct  is_trivially_move_assignable
    : public is_trivially_assignable<typename add_lvalue_reference<_Tp>::type,
                                     typename add_rvalue_reference<_Tp>::type>
    {};


// is_trivially_destructible


template <class _Tp> struct  is_trivially_destructible
    : public integral_constant<bool, is_destructible<_Tp>::value && __has_trivial_destructor(_Tp)> {};



// is_nothrow_constructible




template <bool, bool, class _Tp, class... _Args> struct __libcpp_is_nothrow_constructible;

template <class _Tp, class... _Args>
struct __libcpp_is_nothrow_constructible</*is constructible*/true, /*is reference*/false, _Tp, _Args...>
    : public integral_constant<bool, noexcept(_Tp(declval<_Args>()...))>
{
};

template <class _Tp>
void __implicit_conversion_to(_Tp) noexcept { }

template <class _Tp, class _Arg>
struct __libcpp_is_nothrow_constructible</*is constructible*/true, /*is reference*/true, _Tp, _Arg>
    : public integral_constant<bool, noexcept(__implicit_conversion_to<_Tp>(declval<_Arg>()))>
{
};

template <class _Tp, bool _IsReference, class... _Args>
struct __libcpp_is_nothrow_constructible</*is constructible*/false, _IsReference, _Tp, _Args...>
    : public false_type
{
};

template <class _Tp, class... _Args>
struct  is_nothrow_constructible
    : __libcpp_is_nothrow_constructible<is_constructible<_Tp, _Args...>::value, is_reference<_Tp>::value, _Tp, _Args...>
{
};

template <class _Tp, size_t _Ns>
struct  is_nothrow_constructible<_Tp[_Ns]>
    : __libcpp_is_nothrow_constructible<is_constructible<_Tp>::value, is_reference<_Tp>::value, _Tp>
{
};




// is_nothrow_default_constructible

template <class _Tp> struct  is_nothrow_default_constructible
    : public is_nothrow_constructible<_Tp>
    {};


// is_nothrow_copy_constructible

template <class _Tp> struct  is_nothrow_copy_constructible
    : public is_nothrow_constructible<_Tp,
                  typename add_lvalue_reference<typename add_const<_Tp>::type>::type> {};


// is_nothrow_move_constructible

template <class _Tp> struct  is_nothrow_move_constructible
    : public is_nothrow_constructible<_Tp, typename add_rvalue_reference<_Tp>::type>
    {};


// is_nothrow_assignable


template <bool, class _Tp, class _Arg> struct __libcpp_is_nothrow_assignable;

template <class _Tp, class _Arg>
struct __libcpp_is_nothrow_assignable<false, _Tp, _Arg>
    : public false_type
{
};

template <class _Tp, class _Arg>
struct __libcpp_is_nothrow_assignable<true, _Tp, _Arg>
    : public integral_constant<bool, noexcept(std::__2::declval<_Tp>() = std::__2::declval<_Arg>()) >
{
};

template <class _Tp, class _Arg>
struct  is_nothrow_assignable
    : public __libcpp_is_nothrow_assignable<is_assignable<_Tp, _Arg>::value, _Tp, _Arg>
{
};



// is_nothrow_copy_assignable

template <class _Tp> struct  is_nothrow_copy_assignable
    : public is_nothrow_assignable<typename add_lvalue_reference<_Tp>::type,
                  typename add_lvalue_reference<typename add_const<_Tp>::type>::type> {};


// is_nothrow_move_assignable

template <class _Tp> struct  is_nothrow_move_assignable
    : public is_nothrow_assignable<typename add_lvalue_reference<_Tp>::type,
                                     typename add_rvalue_reference<_Tp>::type>
    {};


// is_nothrow_destructible


template <bool, class _Tp> struct __libcpp_is_nothrow_destructible;

template <class _Tp>
struct __libcpp_is_nothrow_destructible<false, _Tp>
    : public false_type
{
};

template <class _Tp>
struct __libcpp_is_nothrow_destructible<true, _Tp>
    : public integral_constant<bool, noexcept(std::__2::declval<_Tp>().~_Tp()) >
{
};

template <class _Tp>
struct  is_nothrow_destructible
    : public __libcpp_is_nothrow_destructible<is_destructible<_Tp>::value, _Tp>
{
};

template <class _Tp, size_t _Ns>
struct  is_nothrow_destructible<_Tp[_Ns]>
    : public is_nothrow_destructible<_Tp>
{
};

template <class _Tp>
struct  is_nothrow_destructible<_Tp&>
    : public true_type
{
};


template <class _Tp>
struct  is_nothrow_destructible<_Tp&&>
    : public true_type
{
};




// is_pod


template <class _Tp> struct  is_pod
    : public integral_constant<bool, __is_pod(_Tp)> {};



// is_literal_type;

template <class _Tp> struct  is_literal_type
    : public integral_constant<bool, __is_literal_type(_Tp)>
    {};
    

// is_standard_layout;

template <class _Tp> struct  is_standard_layout
    : public integral_constant<bool, __is_standard_layout(_Tp)>
    {};
    

// is_trivially_copyable;

template <class _Tp> struct  is_trivially_copyable
    : public integral_constant<bool, __is_trivially_copyable(_Tp)>
    {};
    

// is_trivial;

template <class _Tp> struct  is_trivial
    : public integral_constant<bool, __is_trivial(_Tp)>
    {};


template <class _Tp> struct __is_reference_wrapper_impl : public false_type {};
template <class _Tp> struct __is_reference_wrapper_impl<reference_wrapper<_Tp> > : public true_type {};
template <class _Tp> struct __is_reference_wrapper
    : public __is_reference_wrapper_impl<typename remove_cv<_Tp>::type> {};


// Check for complete types

template <class ..._Tp> struct __check_complete;

template <>
struct __check_complete<>
{
};

template <class _Hp, class _T0, class ..._Tp>
struct __check_complete<_Hp, _T0, _Tp...>
    : private __check_complete<_Hp>,
      private __check_complete<_T0, _Tp...>
{
};

template <class _Hp>
struct __check_complete<_Hp, _Hp>
    : private __check_complete<_Hp>
{
};

template <class _Tp>
struct __check_complete<_Tp>
{
    static_assert(sizeof(_Tp) > 0, "Type must be complete.");
};

template <class _Tp>
struct __check_complete<_Tp&>
    : private __check_complete<_Tp>
{
};

template <class _Tp>
struct __check_complete<_Tp&&>
    : private __check_complete<_Tp>
{
};

template <class _Rp, class ..._Param>
struct __check_complete<_Rp (*)(_Param...)>
    : private __check_complete<_Rp>
{
};

template <class ..._Param>
struct __check_complete<void (*)(_Param...)>
{
};

template <class _Rp, class ..._Param>
struct __check_complete<_Rp (_Param...)>
    : private __check_complete<_Rp>
{
};

template <class ..._Param>
struct __check_complete<void (_Param...)>
{
};

template <class _Rp, class _Class, class ..._Param>
struct __check_complete<_Rp (_Class::*)(_Param...)>
    : private __check_complete<_Class>
{
};

template <class _Rp, class _Class, class ..._Param>
struct __check_complete<_Rp (_Class::*)(_Param...) const>
    : private __check_complete<_Class>
{
};

template <class _Rp, class _Class, class ..._Param>
struct __check_complete<_Rp (_Class::*)(_Param...) volatile>
    : private __check_complete<_Class>
{
};

template <class _Rp, class _Class, class ..._Param>
struct __check_complete<_Rp (_Class::*)(_Param...) const volatile>
    : private __check_complete<_Class>
{
};

template <class _Rp, class _Class, class ..._Param>
struct __check_complete<_Rp (_Class::*)(_Param...) &>
    : private __check_complete<_Class>
{
};

template <class _Rp, class _Class, class ..._Param>
struct __check_complete<_Rp (_Class::*)(_Param...) const&>
    : private __check_complete<_Class>
{
};

template <class _Rp, class _Class, class ..._Param>
struct __check_complete<_Rp (_Class::*)(_Param...) volatile&>
    : private __check_complete<_Class>
{
};

template <class _Rp, class _Class, class ..._Param>
struct __check_complete<_Rp (_Class::*)(_Param...) const volatile&>
    : private __check_complete<_Class>
{
};

template <class _Rp, class _Class, class ..._Param>
struct __check_complete<_Rp (_Class::*)(_Param...) &&>
    : private __check_complete<_Class>
{
};

template <class _Rp, class _Class, class ..._Param>
struct __check_complete<_Rp (_Class::*)(_Param...) const&&>
    : private __check_complete<_Class>
{
};

template <class _Rp, class _Class, class ..._Param>
struct __check_complete<_Rp (_Class::*)(_Param...) volatile&&>
    : private __check_complete<_Class>
{
};

template <class _Rp, class _Class, class ..._Param>
struct __check_complete<_Rp (_Class::*)(_Param...) const volatile&&>
    : private __check_complete<_Class>
{
};

template <class _Rp, class _Class>
struct __check_complete<_Rp _Class::*>
    : private __check_complete<_Class>
{
};


template <class _Fp, class _A0,
         class _DecayFp = typename decay<_Fp>::type,
         class _DecayA0 = typename decay<_A0>::type,
         class _ClassT = typename __member_pointer_class_type<_DecayFp>::type>
using __enable_if_bullet1 = typename enable_if
    <
        is_member_function_pointer<_DecayFp>::value
        && is_base_of<_ClassT, _DecayA0>::value
    >::type;

template <class _Fp, class _A0,
         class _DecayFp = typename decay<_Fp>::type,
         class _DecayA0 = typename decay<_A0>::type>
using __enable_if_bullet2 = typename enable_if
    <
        is_member_function_pointer<_DecayFp>::value
        && __is_reference_wrapper<_DecayA0>::value
    >::type;

template <class _Fp, class _A0,
         class _DecayFp = typename decay<_Fp>::type,
         class _DecayA0 = typename decay<_A0>::type,
         class _ClassT = typename __member_pointer_class_type<_DecayFp>::type>
using __enable_if_bullet3 = typename enable_if
    <
        is_member_function_pointer<_DecayFp>::value
        && !is_base_of<_ClassT, _DecayA0>::value
        && !__is_reference_wrapper<_DecayA0>::value
    >::type;

template <class _Fp, class _A0,
         class _DecayFp = typename decay<_Fp>::type,
         class _DecayA0 = typename decay<_A0>::type,
         class _ClassT = typename __member_pointer_class_type<_DecayFp>::type>
using __enable_if_bullet4 = typename enable_if
    <
        is_member_object_pointer<_DecayFp>::value
        && is_base_of<_ClassT, _DecayA0>::value
    >::type;

template <class _Fp, class _A0,
         class _DecayFp = typename decay<_Fp>::type,
         class _DecayA0 = typename decay<_A0>::type>
using __enable_if_bullet5 = typename enable_if
    <
        is_member_object_pointer<_DecayFp>::value
        && __is_reference_wrapper<_DecayA0>::value
    >::type;

template <class _Fp, class _A0,
         class _DecayFp = typename decay<_Fp>::type,
         class _DecayA0 = typename decay<_A0>::type,
         class _ClassT = typename __member_pointer_class_type<_DecayFp>::type>
using __enable_if_bullet6 = typename enable_if
    <
        is_member_object_pointer<_DecayFp>::value
        && !is_base_of<_ClassT, _DecayA0>::value
        && !__is_reference_wrapper<_DecayA0>::value
    >::type;

// __invoke forward declarations

// fall back - none of the bullets


template <class ..._Args>
auto __invoke(__any, _Args&& ...__args) -> __nat;

template <class ..._Args>
auto __invoke_constexpr(__any, _Args&& ...__args) -> __nat;

// bullets 1, 2 and 3

template <class _Fp, class _A0, class ..._Args,
          class = __enable_if_bullet1<_Fp, _A0>>
inline __attribute__ ((__always_inline__))
auto
__invoke(_Fp&& __f, _A0&& __a0, _Args&& ...__args)
noexcept(noexcept((std::__2::forward<_A0>(__a0).*__f)(std::__2::forward<_Args>(__args)...))) -> decltype((std::__2::forward<_A0>(__a0).*__f)(std::__2::forward<_Args>(__args)...)) { return (std::__2::forward<_A0>(__a0).*__f)(std::__2::forward<_Args>(__args)...); }

template <class _Fp, class _A0, class ..._Args,
          class = __enable_if_bullet1<_Fp, _A0>>
inline __attribute__ ((__always_inline__))
constexpr auto
__invoke_constexpr(_Fp&& __f, _A0&& __a0, _Args&& ...__args)
noexcept(noexcept((std::__2::forward<_A0>(__a0).*__f)(std::__2::forward<_Args>(__args)...))) -> decltype((std::__2::forward<_A0>(__a0).*__f)(std::__2::forward<_Args>(__args)...)) { return (std::__2::forward<_A0>(__a0).*__f)(std::__2::forward<_Args>(__args)...); }

template <class _Fp, class _A0, class ..._Args,
          class = __enable_if_bullet2<_Fp, _A0>>
inline __attribute__ ((__always_inline__))
auto
__invoke(_Fp&& __f, _A0&& __a0, _Args&& ...__args)
noexcept(noexcept((__a0 . get().*__f)(std::__2::forward<_Args>(__args)...))) -> decltype((__a0 . get().*__f)(std::__2::forward<_Args>(__args)...)) { return (__a0 . get().*__f)(std::__2::forward<_Args>(__args)...); }

template <class _Fp, class _A0, class ..._Args,
          class = __enable_if_bullet2<_Fp, _A0>>
inline __attribute__ ((__always_inline__))
constexpr auto
__invoke_constexpr(_Fp&& __f, _A0&& __a0, _Args&& ...__args)
noexcept(noexcept((__a0 . get().*__f)(std::__2::forward<_Args>(__args)...))) -> decltype((__a0 . get().*__f)(std::__2::forward<_Args>(__args)...)) { return (__a0 . get().*__f)(std::__2::forward<_Args>(__args)...); }

template <class _Fp, class _A0, class ..._Args,
          class = __enable_if_bullet3<_Fp, _A0>>
inline __attribute__ ((__always_inline__))
auto
__invoke(_Fp&& __f, _A0&& __a0, _Args&& ...__args)
noexcept(noexcept(((*std::__2::forward<_A0>(__a0)).*__f)(std::__2::forward<_Args>(__args)...))) -> decltype(((*std::__2::forward<_A0>(__a0)).*__f)(std::__2::forward<_Args>(__args)...)) { return ((*std::__2::forward<_A0>(__a0)).*__f)(std::__2::forward<_Args>(__args)...); }

template <class _Fp, class _A0, class ..._Args,
          class = __enable_if_bullet3<_Fp, _A0>>
inline __attribute__ ((__always_inline__))
constexpr auto
__invoke_constexpr(_Fp&& __f, _A0&& __a0, _Args&& ...__args)
noexcept(noexcept(((*std::__2::forward<_A0>(__a0)).*__f)(std::__2::forward<_Args>(__args)...))) -> decltype(((*std::__2::forward<_A0>(__a0)).*__f)(std::__2::forward<_Args>(__args)...)) { return ((*std::__2::forward<_A0>(__a0)).*__f)(std::__2::forward<_Args>(__args)...); }

// bullets 4, 5 and 6

template <class _Fp, class _A0,
          class = __enable_if_bullet4<_Fp, _A0>>
inline __attribute__ ((__always_inline__))
auto
__invoke(_Fp&& __f, _A0&& __a0)
noexcept(noexcept(std::__2::forward<_A0>(__a0).*__f)) -> decltype(std::__2::forward<_A0>(__a0).*__f) { return std::__2::forward<_A0>(__a0).*__f; }

template <class _Fp, class _A0,
          class = __enable_if_bullet4<_Fp, _A0>>
inline __attribute__ ((__always_inline__))
constexpr auto
__invoke_constexpr(_Fp&& __f, _A0&& __a0)
noexcept(noexcept(std::__2::forward<_A0>(__a0).*__f)) -> decltype(std::__2::forward<_A0>(__a0).*__f) { return std::__2::forward<_A0>(__a0).*__f; }

template <class _Fp, class _A0,
          class = __enable_if_bullet5<_Fp, _A0>>
inline __attribute__ ((__always_inline__))
auto
__invoke(_Fp&& __f, _A0&& __a0)
noexcept(noexcept(__a0 . get().*__f)) -> decltype(__a0 . get().*__f) { return __a0 . get().*__f; }

template <class _Fp, class _A0,
          class = __enable_if_bullet5<_Fp, _A0>>
inline __attribute__ ((__always_inline__))
constexpr auto
__invoke_constexpr(_Fp&& __f, _A0&& __a0)
noexcept(noexcept(__a0 . get().*__f)) -> decltype(__a0 . get().*__f) { return __a0 . get().*__f; }

template <class _Fp, class _A0,
          class = __enable_if_bullet6<_Fp, _A0>>
inline __attribute__ ((__always_inline__))
auto
__invoke(_Fp&& __f, _A0&& __a0)
noexcept(noexcept((*std::__2::forward<_A0>(__a0)).*__f)) -> decltype((*std::__2::forward<_A0>(__a0)).*__f) { return (*std::__2::forward<_A0>(__a0)).*__f; }

template <class _Fp, class _A0,
          class = __enable_if_bullet6<_Fp, _A0>>
inline __attribute__ ((__always_inline__))
constexpr auto
__invoke_constexpr(_Fp&& __f, _A0&& __a0)
noexcept(noexcept((*std::__2::forward<_A0>(__a0)).*__f)) -> decltype((*std::__2::forward<_A0>(__a0)).*__f) { return (*std::__2::forward<_A0>(__a0)).*__f; }

// bullet 7

template <class _Fp, class ..._Args>
inline __attribute__ ((__always_inline__))
auto
__invoke(_Fp&& __f, _Args&& ...__args)
noexcept(noexcept(std::__2::forward<_Fp>(__f)(std::__2::forward<_Args>(__args)...))) -> decltype(std::__2::forward<_Fp>(__f)(std::__2::forward<_Args>(__args)...)) { return std::__2::forward<_Fp>(__f)(std::__2::forward<_Args>(__args)...); }

template <class _Fp, class ..._Args>
inline __attribute__ ((__always_inline__))
constexpr auto
__invoke_constexpr(_Fp&& __f, _Args&& ...__args)
noexcept(noexcept(std::__2::forward<_Fp>(__f)(std::__2::forward<_Args>(__args)...))) -> decltype(std::__2::forward<_Fp>(__f)(std::__2::forward<_Args>(__args)...)) { return std::__2::forward<_Fp>(__f)(std::__2::forward<_Args>(__args)...); }


// __invokable

template <class _Ret, class _Fp, class ..._Args>
struct __invokable_r
    : private __check_complete<_Fp>
{
    using _Result = decltype(
        std::__2::__invoke(std::__2::declval<_Fp>(), std::__2::declval<_Args>()...));

    using type =
        typename conditional<
            !is_same<_Result, __nat>::value,
            typename conditional<
                is_void<_Ret>::value,
                true_type,
                is_convertible<_Result, _Ret>
            >::type,
            false_type
        >::type;
    static const bool value = type::value;
};

template <class _Fp, class ..._Args>
using __invokable = __invokable_r<void, _Fp, _Args...>;

template <bool _IsInvokable, bool _IsCVVoid, class _Ret, class _Fp, class ..._Args>
struct __nothrow_invokable_r_imp {
  static const bool value = false;
};

template <class _Ret, class _Fp, class ..._Args>
struct __nothrow_invokable_r_imp<true, false, _Ret, _Fp, _Args...>
{
    typedef __nothrow_invokable_r_imp _ThisT;

    template <class _Tp>
    static void __test_noexcept(_Tp) noexcept;

    static const bool value = noexcept(_ThisT::__test_noexcept<_Ret>(
        std::__2::__invoke(std::__2::declval<_Fp>(), std::__2::declval<_Args>()...)));
};

template <class _Ret, class _Fp, class ..._Args>
struct __nothrow_invokable_r_imp<true, true, _Ret, _Fp, _Args...>
{
    static const bool value = noexcept(
        std::__2::__invoke(std::__2::declval<_Fp>(), std::__2::declval<_Args>()...));
};

template <class _Ret, class _Fp, class ..._Args>
using __nothrow_invokable_r =
    __nothrow_invokable_r_imp<
            __invokable_r<_Ret, _Fp, _Args...>::value,
            is_void<_Ret>::value,
            _Ret, _Fp, _Args...
    >;

template <class _Fp, class ..._Args>
struct __invoke_of
    : public enable_if<
        __invokable<_Fp, _Args...>::value,
        typename __invokable_r<void, _Fp, _Args...>::_Result>
{
};

// result_of

template <class _Fp, class ..._Args>
class  result_of<_Fp(_Args...)>
    : public __invoke_of<_Fp, _Args...>
{
};

template <class _Tp> using result_of_t = typename result_of<_Tp>::type;



template <class _Tp> struct __is_swappable;
template <class _Tp> struct __is_nothrow_swappable;

template <class _Tp>
inline __attribute__ ((__always_inline__))
typename enable_if
<
    is_move_constructible<_Tp>::value &&
    is_move_assignable<_Tp>::value
>::type
swap(_Tp& __x, _Tp& __y) noexcept(is_nothrow_move_constructible<_Tp> ::value && is_nothrow_move_assignable<_Tp> ::value)
{
    _Tp __t(std::__2::move(__x));
    __x = std::__2::move(__y);
    __y = std::__2::move(__t);
}

template<class _Tp, size_t _Np>
inline __attribute__ ((__always_inline__))
typename enable_if<
    __is_swappable<_Tp>::value
>::type
swap(_Tp (&__a)[_Np], _Tp (&__b)[_Np]) noexcept(__is_nothrow_swappable<_Tp> ::value);

template <class _ForwardIterator1, class _ForwardIterator2>
inline __attribute__ ((__always_inline__))
void
iter_swap(_ForwardIterator1 __a, _ForwardIterator2 __b)
    //                                  _NOEXCEPT_(_NOEXCEPT_(swap(*__a, *__b)))
               noexcept(noexcept(swap(*std::__2::declval<_ForwardIterator1>(), *std::__2::declval<_ForwardIterator2>())))
{
    swap(*__a, *__b);
}

// __swappable

namespace __detail
{
// ALL generic swap overloads MUST already have a declaration available at this point.

template <class _Tp, class _Up = _Tp,
          bool _NotVoid = !is_void<_Tp>::value && !is_void<_Up>::value>
struct __swappable_with
{
    template <class _LHS, class _RHS>
    static decltype(swap(std::__2::declval<_LHS>(), std::__2::declval<_RHS>()))
    __test_swap(int);
    template <class, class>
    static __nat __test_swap(long);

    // Extra parens are needed for the C++03 definition of decltype.
    typedef decltype((__test_swap<_Tp, _Up>(0))) __swap1;
    typedef decltype((__test_swap<_Up, _Tp>(0))) __swap2;

    static const bool value = !is_same<__swap1, __nat>::value
                           && !is_same<__swap2, __nat>::value;
};

template <class _Tp, class _Up>
struct __swappable_with<_Tp, _Up,  false> : false_type {};

template <class _Tp, class _Up = _Tp, bool _Swappable = __swappable_with<_Tp, _Up>::value>
struct __nothrow_swappable_with {
  static const bool value =
      noexcept(swap(std::__2::declval<_Tp>(), std::__2::declval<_Up>()))
  &&  noexcept(swap(std::__2::declval<_Up>(), std::__2::declval<_Tp>()));
};

template <class _Tp, class _Up>
struct __nothrow_swappable_with<_Tp, _Up, false> : false_type {};

}  // __detail

template <class _Tp>
struct __is_swappable
    : public integral_constant<bool, __detail::__swappable_with<_Tp&>::value>
{
};

template <class _Tp>
struct __is_nothrow_swappable
    : public integral_constant<bool, __detail::__nothrow_swappable_with<_Tp&>::value>
{
};



template <class _Tp>
struct underlying_type
{
    typedef __underlying_type(_Tp) type;
};

template <class _Tp> using underlying_type_t = typename underlying_type<_Tp>::type;



template <class _Tp, bool = is_enum<_Tp>::value>
struct __sfinae_underlying_type
{
    typedef typename underlying_type<_Tp>::type type;
    typedef decltype(((type)1) + 0) __promoted_type;
};

template <class _Tp>
struct __sfinae_underlying_type<_Tp, false> {};

inline __attribute__ ((__always_inline__))
int __convert_to_integral(int __val) { return __val; }

inline __attribute__ ((__always_inline__))
unsigned __convert_to_integral(unsigned __val) { return __val; }

inline __attribute__ ((__always_inline__))
long __convert_to_integral(long __val) { return __val; }

inline __attribute__ ((__always_inline__))
unsigned long __convert_to_integral(unsigned long __val) { return __val; }

inline __attribute__ ((__always_inline__))
long long __convert_to_integral(long long __val) { return __val; }

inline __attribute__ ((__always_inline__))
unsigned long long __convert_to_integral(unsigned long long __val) {return __val; }


template <class _Tp>
inline __attribute__ ((__always_inline__))
typename __sfinae_underlying_type<_Tp>::__promoted_type
__convert_to_integral(_Tp __val) { return __val; }


template <class _Tp>
struct __has_operator_addressof_member_imp
{
    template <class _Up>
        static auto __test(int)
            -> typename __select_2nd<decltype(std::__2::declval<_Up>().operator&()), true_type>::type;
    template <class>
        static auto __test(long) -> false_type;

    static const bool value = decltype(__test<_Tp>(0))::value;
};

template <class _Tp>
struct __has_operator_addressof_free_imp
{
    template <class _Up>
        static auto __test(int)
            -> typename __select_2nd<decltype(operator&(std::__2::declval<_Up>())), true_type>::type;
    template <class>
        static auto __test(long) -> false_type;

    static const bool value = decltype(__test<_Tp>(0))::value;
};

template <class _Tp>
struct __has_operator_addressof
    : public integral_constant<bool, __has_operator_addressof_member_imp<_Tp>::value
                                  || __has_operator_addressof_free_imp<_Tp>::value>
{};



// These traits are used in __tree and __hash_table
struct __extract_key_fail_tag {};
struct __extract_key_self_tag {};
struct __extract_key_first_tag {};

template <class _ValTy, class _Key,
          class _RawValTy = typename __unconstref<_ValTy>::type>
struct __can_extract_key
    : conditional<is_same<_RawValTy, _Key>::value, __extract_key_self_tag,
                  __extract_key_fail_tag>::type {};

template <class _Pair, class _Key, class _First, class _Second>
struct __can_extract_key<_Pair, _Key, pair<_First, _Second>>
    : conditional<is_same<typename remove_const<_First>::type, _Key>::value,
                  __extract_key_first_tag, __extract_key_fail_tag>::type {};

// __can_extract_map_key uses true_type/false_type instead of the tags.
// It returns true if _Key != _ContainerValueTy (the container is a map not a set)
// and _ValTy == _Key.
template <class _ValTy, class _Key, class _ContainerValueTy,
          class _RawValTy = typename __unconstref<_ValTy>::type>
struct __can_extract_map_key
    : integral_constant<bool, is_same<_RawValTy, _Key>::value> {};

// This specialization returns __extract_key_fail_tag for non-map containers
// because _Key == _ContainerValueTy
template <class _ValTy, class _Key, class _RawValTy>
struct __can_extract_map_key<_ValTy, _Key, _Key, _RawValTy>
    : false_type {};


} }


// -*- C++ -*-
//===---------------------------- limits ----------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is dual licensed under the MIT and the University of Illinois Open
// Source Licenses. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//


/*
    limits synopsis

namespace std
{

template<class T>
class numeric_limits
{
public:
    static constexpr bool is_specialized = false;
    static constexpr T min() noexcept;
    static constexpr T max() noexcept;
    static constexpr T lowest() noexcept;

    static constexpr int  digits = 0;
    static constexpr int  digits10 = 0;
    static constexpr int  max_digits10 = 0;
    static constexpr bool is_signed = false;
    static constexpr bool is_integer = false;
    static constexpr bool is_exact = false;
    static constexpr int  radix = 0;
    static constexpr T epsilon() noexcept;
    static constexpr T round_error() noexcept;

    static constexpr int  min_exponent = 0;
    static constexpr int  min_exponent10 = 0;
    static constexpr int  max_exponent = 0;
    static constexpr int  max_exponent10 = 0;

    static constexpr bool has_infinity = false;
    static constexpr bool has_quiet_NaN = false;
    static constexpr bool has_signaling_NaN = false;
    static constexpr float_denorm_style has_denorm = denorm_absent;
    static constexpr bool has_denorm_loss = false;
    static constexpr T infinity() noexcept;
    static constexpr T quiet_NaN() noexcept;
    static constexpr T signaling_NaN() noexcept;
    static constexpr T denorm_min() noexcept;

    static constexpr bool is_iec559 = false;
    static constexpr bool is_bounded = false;
    static constexpr bool is_modulo = false;

    static constexpr bool traps = false;
    static constexpr bool tinyness_before = false;
    static constexpr float_round_style round_style = round_toward_zero;
};

enum __attribute__((packed)) float_round_style
{
    round_indeterminate       = -1,
    round_toward_zero         =  0,
    round_to_nearest          =  1,
    round_toward_infinity     =  2,
    round_toward_neg_infinity =  3
};

enum __attribute__((packed)) float_denorm_style
{
    denorm_indeterminate = -1,
    denorm_absent = 0,
    denorm_present = 1
};

template<> class numeric_limits<cv bool>;

template<> class numeric_limits<cv char>;
template<> class numeric_limits<cv signed char>;
template<> class numeric_limits<cv unsigned char>;
template<> class numeric_limits<cv wchar_t>;
template<> class numeric_limits<cv char16_t>;
template<> class numeric_limits<cv char32_t>;

template<> class numeric_limits<cv short>;
template<> class numeric_limits<cv int>;
template<> class numeric_limits<cv long>;
template<> class numeric_limits<cv long long>;
template<> class numeric_limits<cv unsigned short>;
template<> class numeric_limits<cv unsigned int>;
template<> class numeric_limits<cv unsigned long>;
template<> class numeric_limits<cv unsigned long long>;

template<> class numeric_limits<cv float>;
template<> class numeric_limits<cv double>;
template<> class numeric_limits<cv long double>;

}  // std

*/

// -*- C++ -*-
//===--------------------- support/ti/limits.h ----------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is dual licensed under the MIT and the University of Illinois Open
// Source Licenses. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//


/* -*- C++ -*- */
/*===--------------------------- complex.h --------------------------------===*/
/*                                                                            */
/*                     The LLVM Compiler Infrastructure                       */
/*                                                                            */
/* This file is dual licensed under the MIT and the University of Illinois Open
** Source Licenses. See LICENSE.TXT for details.
*/
/*===----------------------------------------------------------------------===*/


/*
    float.h synopsis

Macros:

    FLT_ROUNDS
    FLT_EVAL_METHOD     // C99
    FLT_RADIX

    FLT_MANT_DIG
    DBL_MANT_DIG
    LDBL_MANT_DIG

    DECIMAL_DIG         // C99

    FLT_DIG
    DBL_DIG
    LDBL_DIG

    FLT_MIN_EXP
    DBL_MIN_EXP
    LDBL_MIN_EXP

    FLT_MIN_10_EXP
    DBL_MIN_10_EXP
    LDBL_MIN_10_EXP

    FLT_MAX_EXP
    DBL_MAX_EXP
    LDBL_MAX_EXP

    FLT_MAX_10_EXP
    DBL_MAX_10_EXP
    LDBL_MAX_10_EXP

    FLT_MAX
    DBL_MAX
    LDBL_MAX

    FLT_EPSILON
    DBL_EPSILON
    LDBL_EPSILON

    FLT_MIN
    DBL_MIN
    LDBL_MIN

*/



/*****************************************************************************/
/* float.h                                                                   */
/*                                                                           */
/* Copyright (c) 1993 Texas Instruments Incorporated                         */
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

/********************************************************************/
/*    KEY:  FLT_     - APPLIES TO TYPE FLOAT                        */
/*          DBL_     - APPLIES TO TYPE DOUBLE                       */
/*          LDBL_    - APPLIES TO TYPE LONG DOUBLE                  */
/********************************************************************/


_Pragma("diag_push")
_Pragma("CHECK_MISRA(\"-20.1\")") /* standard headers must define standard names */










_Pragma("diag_pop")













_Pragma("push_macro(\"min\")") _Pragma("push_macro(\"max\")")
// -*- C++ -*-
//===------------------------ __undef_macros ------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//





namespace std { inline namespace __2 {

enum __attribute__((packed)) float_round_style
{
    round_indeterminate       = -1,
    round_toward_zero         =  0,
    round_to_nearest          =  1,
    round_toward_infinity     =  2,
    round_toward_neg_infinity =  3
};

enum __attribute__((packed)) float_denorm_style
{
    denorm_indeterminate = -1,
    denorm_absent = 0,
    denorm_present = 1
};

template <class _Tp, bool = is_arithmetic<_Tp>::value>
class __libcpp_numeric_limits
{
protected:
    typedef _Tp type;

    static constexpr const  bool is_specialized = false;
    __attribute__ ((__always_inline__)) static constexpr type min() noexcept {return type();}
    __attribute__ ((__always_inline__)) static constexpr type max() noexcept {return type();}
    __attribute__ ((__always_inline__)) static constexpr type lowest() noexcept {return type();}

    static constexpr const int  digits = 0;
    static constexpr const int  digits10 = 0;
    static constexpr const int  max_digits10 = 0;
    static constexpr const bool is_signed = false;
    static constexpr const bool is_integer = false;
    static constexpr const bool is_exact = false;
    static constexpr const int  radix = 0;
    __attribute__ ((__always_inline__)) static constexpr type epsilon() noexcept {return type();}
    __attribute__ ((__always_inline__)) static constexpr type round_error() noexcept {return type();}

    static constexpr const int  min_exponent = 0;
    static constexpr const int  min_exponent10 = 0;
    static constexpr const int  max_exponent = 0;
    static constexpr const int  max_exponent10 = 0;

    static constexpr const bool has_infinity = false;
    static constexpr const bool has_quiet_NaN = false;
    static constexpr const bool has_signaling_NaN = false;
    static constexpr const float_denorm_style has_denorm = denorm_absent;
    static constexpr const bool has_denorm_loss = false;
    __attribute__ ((__always_inline__)) static constexpr type infinity() noexcept {return type();}
    __attribute__ ((__always_inline__)) static constexpr type quiet_NaN() noexcept {return type();}
    __attribute__ ((__always_inline__)) static constexpr type signaling_NaN() noexcept {return type();}
    __attribute__ ((__always_inline__)) static constexpr type denorm_min() noexcept {return type();}

    static constexpr const bool is_iec559 = false;
    static constexpr const bool is_bounded = false;
    static constexpr const bool is_modulo = false;

    static constexpr const bool traps = false;
    static constexpr const bool tinyness_before = false;
    static constexpr const float_round_style round_style = round_toward_zero;
};

template <class _Tp, int __digits, bool _IsSigned>
struct __libcpp_compute_min
{
    static constexpr const _Tp value = _Tp(_Tp(1) << __digits);
};

template <class _Tp, int __digits>
struct __libcpp_compute_min<_Tp, __digits, false>
{
    static constexpr const _Tp value = _Tp(0);
};

template <class _Tp>
class __libcpp_numeric_limits<_Tp, true>
{
protected:
    typedef _Tp type;

    static constexpr const bool is_specialized = true;

    static constexpr const bool is_signed = type(-1) < type(0);
    static constexpr const int  digits = static_cast<int>(sizeof(type) * 8 - is_signed);
    static constexpr const int  digits10 = digits * 3 / 10;
    static constexpr const int  max_digits10 = 0;
    static constexpr const type __min = __libcpp_compute_min<type, digits, is_signed>::value;
    static constexpr const type __max = is_signed ? type(type(~0) ^ __min) : type(~0);
    __attribute__ ((__always_inline__)) static constexpr type min() noexcept {return __min;}
    __attribute__ ((__always_inline__)) static constexpr type max() noexcept {return __max;}
    __attribute__ ((__always_inline__)) static constexpr type lowest() noexcept {return min();}

    static constexpr const bool is_integer = true;
    static constexpr const bool is_exact = true;
    static constexpr const int  radix = 2;
    __attribute__ ((__always_inline__)) static constexpr type epsilon() noexcept {return type(0);}
    __attribute__ ((__always_inline__)) static constexpr type round_error() noexcept {return type(0);}

    static constexpr const int  min_exponent = 0;
    static constexpr const int  min_exponent10 = 0;
    static constexpr const int  max_exponent = 0;
    static constexpr const int  max_exponent10 = 0;

    static constexpr const bool has_infinity = false;
    static constexpr const bool has_quiet_NaN = false;
    static constexpr const bool has_signaling_NaN = false;
    static constexpr const float_denorm_style has_denorm = denorm_absent;
    static constexpr const bool has_denorm_loss = false;
    __attribute__ ((__always_inline__)) static constexpr type infinity() noexcept {return type(0);}
    __attribute__ ((__always_inline__)) static constexpr type quiet_NaN() noexcept {return type(0);}
    __attribute__ ((__always_inline__)) static constexpr type signaling_NaN() noexcept {return type(0);}
    __attribute__ ((__always_inline__)) static constexpr type denorm_min() noexcept {return type(0);}

    static constexpr const bool is_iec559 = false;
    static constexpr const bool is_bounded = true;
    static constexpr const bool is_modulo = !std::__2::is_signed<_Tp>::value;

    static constexpr const bool traps = false;
    static constexpr const bool tinyness_before = false;
    static constexpr const float_round_style round_style = round_toward_zero;
};

template <>
class __libcpp_numeric_limits<bool, true>
{
protected:
    typedef bool type;

    static constexpr const bool is_specialized = true;

    static constexpr const bool is_signed = false;
    static constexpr const int  digits = 1;
    static constexpr const int  digits10 = 0;
    static constexpr const int  max_digits10 = 0;
    static constexpr const type __min = false;
    static constexpr const type __max = true;
    __attribute__ ((__always_inline__)) static constexpr type min() noexcept {return __min;}
    __attribute__ ((__always_inline__)) static constexpr type max() noexcept {return __max;}
    __attribute__ ((__always_inline__)) static constexpr type lowest() noexcept {return min();}

    static constexpr const bool is_integer = true;
    static constexpr const bool is_exact = true;
    static constexpr const int  radix = 2;
    __attribute__ ((__always_inline__)) static constexpr type epsilon() noexcept {return type(0);}
    __attribute__ ((__always_inline__)) static constexpr type round_error() noexcept {return type(0);}

    static constexpr const int  min_exponent = 0;
    static constexpr const int  min_exponent10 = 0;
    static constexpr const int  max_exponent = 0;
    static constexpr const int  max_exponent10 = 0;

    static constexpr const bool has_infinity = false;
    static constexpr const bool has_quiet_NaN = false;
    static constexpr const bool has_signaling_NaN = false;
    static constexpr const float_denorm_style has_denorm = denorm_absent;
    static constexpr const bool has_denorm_loss = false;
    __attribute__ ((__always_inline__)) static constexpr type infinity() noexcept {return type(0);}
    __attribute__ ((__always_inline__)) static constexpr type quiet_NaN() noexcept {return type(0);}
    __attribute__ ((__always_inline__)) static constexpr type signaling_NaN() noexcept {return type(0);}
    __attribute__ ((__always_inline__)) static constexpr type denorm_min() noexcept {return type(0);}

    static constexpr const bool is_iec559 = false;
    static constexpr const bool is_bounded = true;
    static constexpr const bool is_modulo = false;

    static constexpr const bool traps = false;
    static constexpr const bool tinyness_before = false;
    static constexpr const float_round_style round_style = round_toward_zero;
};

template <>
class __libcpp_numeric_limits<float, true>
{
protected:
    typedef float type;

    static constexpr const bool is_specialized = true;

    static constexpr const bool is_signed = true;
    static constexpr const int  digits = 24;
    static constexpr const int  digits10 = 6;
    static constexpr const int  max_digits10 = 2+(digits * 30103l)/100000l;
    __attribute__ ((__always_inline__)) static constexpr type min() noexcept {return 1.175494351E-38F;}
    __attribute__ ((__always_inline__)) static constexpr type max() noexcept {return 3.402823466E+38F;}
    __attribute__ ((__always_inline__)) static constexpr type lowest() noexcept {return -max();}

    static constexpr const bool is_integer = false;
    static constexpr const bool is_exact = false;
    static constexpr const int  radix = 2;
    __attribute__ ((__always_inline__)) static constexpr type epsilon() noexcept {return 1.192092896E-07F;}
    __attribute__ ((__always_inline__)) static constexpr type round_error() noexcept {return 0.5F;}

    static constexpr const int  min_exponent = (-125);
    static constexpr const int  min_exponent10 = (-37);
    static constexpr const int  max_exponent = 128;
    static constexpr const int  max_exponent10 = 38;

    static constexpr const bool has_infinity = true;
    static constexpr const bool has_quiet_NaN = true;
    static constexpr const bool has_signaling_NaN = false;
    static constexpr const float_denorm_style has_denorm = denorm_absent;
    static constexpr const bool has_denorm_loss = false;
    __attribute__ ((__always_inline__)) static constexpr type infinity() noexcept {return __builtin_huge_valf();}
    __attribute__ ((__always_inline__)) static constexpr type quiet_NaN() noexcept {return __builtin_nanf("");}
    __attribute__ ((__always_inline__)) static constexpr type signaling_NaN() noexcept {return __builtin_nansf("");}
    __attribute__ ((__always_inline__)) static constexpr type denorm_min() noexcept {return has_denorm == denorm_absent ? min() : (0.0f);}

    static constexpr const bool is_iec559 = false; // Because has_signaling_NaN is false
    static constexpr const bool is_bounded = true;
    static constexpr const bool is_modulo = false;

    static constexpr const bool traps = false;
    static constexpr const bool tinyness_before = false;
    static constexpr const float_round_style round_style = round_to_nearest;
};

template <>
class __libcpp_numeric_limits<double, true>
{
protected:
    typedef double type;

    static constexpr const bool is_specialized = true;

    static constexpr const bool is_signed = true;
    static constexpr const int  digits = 53;
    static constexpr const int  digits10 = 15;
    static constexpr const int  max_digits10 = 2+(digits * 30103l)/100000l;
    __attribute__ ((__always_inline__)) static constexpr type min() noexcept {return 2.2250738585072014E-308;}
    __attribute__ ((__always_inline__)) static constexpr type max() noexcept {return 1.7976931348623157E+308;}
    __attribute__ ((__always_inline__)) static constexpr type lowest() noexcept {return -max();}

    static constexpr const bool is_integer = false;
    static constexpr const bool is_exact = false;
    static constexpr const int  radix = 2;
    __attribute__ ((__always_inline__)) static constexpr type epsilon() noexcept {return 2.2204460492503131E-16;}
    __attribute__ ((__always_inline__)) static constexpr type round_error() noexcept {return 0.5;}

    static constexpr const int  min_exponent = (-1021);
    static constexpr const int  min_exponent10 = (-307);
    static constexpr const int  max_exponent = 1024;
    static constexpr const int  max_exponent10 = 308;

    static constexpr const bool has_infinity = true;
    static constexpr const bool has_quiet_NaN = true;
    static constexpr const bool has_signaling_NaN = false;
    static constexpr const float_denorm_style has_denorm = denorm_absent;
    static constexpr const bool has_denorm_loss = false;
    __attribute__ ((__always_inline__)) static constexpr type infinity() noexcept {return __builtin_huge_val();}
    __attribute__ ((__always_inline__)) static constexpr type quiet_NaN() noexcept {return __builtin_nan("");}
    __attribute__ ((__always_inline__)) static constexpr type signaling_NaN() noexcept {return __builtin_nans("");}
    __attribute__ ((__always_inline__)) static constexpr type denorm_min() noexcept {return has_denorm == denorm_absent ? min() : (0.0);}

    static constexpr const bool is_iec559 = false; // Because has_signaling_NaN is false
    static constexpr const bool is_bounded = true;
    static constexpr const bool is_modulo = false;

    static constexpr const bool traps = false;
    static constexpr const bool tinyness_before = false;
    static constexpr const float_round_style round_style = round_to_nearest;
};

template <>
class __libcpp_numeric_limits<long double, true>
{
protected:
    typedef long double type;

    static constexpr const bool is_specialized = true;

    static constexpr const bool is_signed = true;
    static constexpr const int  digits = 53;
    static constexpr const int  digits10 = 15;
    static constexpr const int  max_digits10 = 2+(digits * 30103l)/100000l;
    __attribute__ ((__always_inline__)) static constexpr type min() noexcept {return 2.2250738585072014E-308L;}
    __attribute__ ((__always_inline__)) static constexpr type max() noexcept {return 1.7976931348623157E+308L;}
    __attribute__ ((__always_inline__)) static constexpr type lowest() noexcept {return -max();}

    static constexpr const bool is_integer = false;
    static constexpr const bool is_exact = false;
    static constexpr const int  radix = 2;
    __attribute__ ((__always_inline__)) static constexpr type epsilon() noexcept {return 2.2204460492503131E-16L;}
    __attribute__ ((__always_inline__)) static constexpr type round_error() noexcept {return 0.5;}

    static constexpr const int  min_exponent = (-1021);
    static constexpr const int  min_exponent10 = (-307);
    static constexpr const int  max_exponent = 1024;
    static constexpr const int  max_exponent10 = 308;

    static constexpr const bool has_infinity = true;
    static constexpr const bool has_quiet_NaN = true;
    static constexpr const bool has_signaling_NaN = false;
    static constexpr const float_denorm_style has_denorm = denorm_absent;
    static constexpr const bool has_denorm_loss = false;
    __attribute__ ((__always_inline__)) static constexpr type infinity() noexcept {return __builtin_huge_vall();}
    __attribute__ ((__always_inline__)) static constexpr type quiet_NaN() noexcept {return __builtin_nanl("");}
    __attribute__ ((__always_inline__)) static constexpr type signaling_NaN() noexcept {return __builtin_nansl("");}
    __attribute__ ((__always_inline__)) static constexpr type denorm_min() noexcept {return has_denorm == denorm_absent ? min() : (0.0l);}

    static constexpr const bool is_iec559 = false; // Because has_signaling_NaN is false
    static constexpr const bool is_bounded = true;
    static constexpr const bool is_modulo = false;

    static constexpr const bool traps = false;
    static constexpr const bool tinyness_before = false;
    static constexpr const float_round_style round_style = round_to_nearest;
};

template <class _Tp>
class  numeric_limits
    : private __libcpp_numeric_limits<typename remove_cv<_Tp>::type>
{
    typedef __libcpp_numeric_limits<typename remove_cv<_Tp>::type> __base;
    typedef typename __base::type type;
public:
    static constexpr const bool is_specialized = __base::is_specialized;
    __attribute__ ((__always_inline__)) static constexpr type min() noexcept {return __base::min();}
    __attribute__ ((__always_inline__)) static constexpr type max() noexcept {return __base::max();}
    __attribute__ ((__always_inline__)) static constexpr type lowest() noexcept {return __base::lowest();}

    static constexpr const int  digits = __base::digits;
    static constexpr const int  digits10 = __base::digits10;
    static constexpr const int  max_digits10 = __base::max_digits10;
    static constexpr const bool is_signed = __base::is_signed;
    static constexpr const bool is_integer = __base::is_integer;
    static constexpr const bool is_exact = __base::is_exact;
    static constexpr const int  radix = __base::radix;
    __attribute__ ((__always_inline__)) static constexpr type epsilon() noexcept {return __base::epsilon();}
    __attribute__ ((__always_inline__)) static constexpr type round_error() noexcept {return __base::round_error();}

    static constexpr const int  min_exponent = __base::min_exponent;
    static constexpr const int  min_exponent10 = __base::min_exponent10;
    static constexpr const int  max_exponent = __base::max_exponent;
    static constexpr const int  max_exponent10 = __base::max_exponent10;

    static constexpr const bool has_infinity = __base::has_infinity;
    static constexpr const bool has_quiet_NaN = __base::has_quiet_NaN;
    static constexpr const bool has_signaling_NaN = __base::has_signaling_NaN;
    static constexpr const float_denorm_style has_denorm = __base::has_denorm;
    static constexpr const bool has_denorm_loss = __base::has_denorm_loss;
    __attribute__ ((__always_inline__)) static constexpr type infinity() noexcept {return __base::infinity();}
    __attribute__ ((__always_inline__)) static constexpr type quiet_NaN() noexcept {return __base::quiet_NaN();}
    __attribute__ ((__always_inline__)) static constexpr type signaling_NaN() noexcept {return __base::signaling_NaN();}
    __attribute__ ((__always_inline__)) static constexpr type denorm_min() noexcept {return __base::denorm_min();}

    static constexpr const bool is_iec559 = __base::is_iec559;
    static constexpr const bool is_bounded = __base::is_bounded;
    static constexpr const bool is_modulo = __base::is_modulo;

    static constexpr const bool traps = __base::traps;
    static constexpr const bool tinyness_before = __base::tinyness_before;
    static constexpr const float_round_style round_style = __base::round_style;
};

template <class _Tp>
    constexpr const bool numeric_limits<_Tp>::is_specialized;
template <class _Tp>
    constexpr const int numeric_limits<_Tp>::digits;
template <class _Tp>
    constexpr const int numeric_limits<_Tp>::digits10;
template <class _Tp>
    constexpr const int numeric_limits<_Tp>::max_digits10;
template <class _Tp>
    constexpr const bool numeric_limits<_Tp>::is_signed;
template <class _Tp>
    constexpr const bool numeric_limits<_Tp>::is_integer;
template <class _Tp>
    constexpr const bool numeric_limits<_Tp>::is_exact;
template <class _Tp>
    constexpr const int numeric_limits<_Tp>::radix;
template <class _Tp>
    constexpr const int numeric_limits<_Tp>::min_exponent;
template <class _Tp>
    constexpr const int numeric_limits<_Tp>::min_exponent10;
template <class _Tp>
    constexpr const int numeric_limits<_Tp>::max_exponent;
template <class _Tp>
    constexpr const int numeric_limits<_Tp>::max_exponent10;
template <class _Tp>
    constexpr const bool numeric_limits<_Tp>::has_infinity;
template <class _Tp>
    constexpr const bool numeric_limits<_Tp>::has_quiet_NaN;
template <class _Tp>
    constexpr const bool numeric_limits<_Tp>::has_signaling_NaN;
template <class _Tp>
    constexpr const float_denorm_style numeric_limits<_Tp>::has_denorm;
template <class _Tp>
    constexpr const bool numeric_limits<_Tp>::has_denorm_loss;
template <class _Tp>
    constexpr const bool numeric_limits<_Tp>::is_iec559;
template <class _Tp>
    constexpr const bool numeric_limits<_Tp>::is_bounded;
template <class _Tp>
    constexpr const bool numeric_limits<_Tp>::is_modulo;
template <class _Tp>
    constexpr const bool numeric_limits<_Tp>::traps;
template <class _Tp>
    constexpr const bool numeric_limits<_Tp>::tinyness_before;
template <class _Tp>
    constexpr const float_round_style numeric_limits<_Tp>::round_style;

template <class _Tp>
class  numeric_limits<const _Tp>
    : private numeric_limits<_Tp>
{
    typedef numeric_limits<_Tp> __base;
    typedef _Tp type;
public:
    static constexpr const bool is_specialized = __base::is_specialized;
    __attribute__ ((__always_inline__)) static constexpr type min() noexcept {return __base::min();}
    __attribute__ ((__always_inline__)) static constexpr type max() noexcept {return __base::max();}
    __attribute__ ((__always_inline__)) static constexpr type lowest() noexcept {return __base::lowest();}

    static constexpr const int  digits = __base::digits;
    static constexpr const int  digits10 = __base::digits10;
    static constexpr const int  max_digits10 = __base::max_digits10;
    static constexpr const bool is_signed = __base::is_signed;
    static constexpr const bool is_integer = __base::is_integer;
    static constexpr const bool is_exact = __base::is_exact;
    static constexpr const int  radix = __base::radix;
    __attribute__ ((__always_inline__)) static constexpr type epsilon() noexcept {return __base::epsilon();}
    __attribute__ ((__always_inline__)) static constexpr type round_error() noexcept {return __base::round_error();}

    static constexpr const int  min_exponent = __base::min_exponent;
    static constexpr const int  min_exponent10 = __base::min_exponent10;
    static constexpr const int  max_exponent = __base::max_exponent;
    static constexpr const int  max_exponent10 = __base::max_exponent10;

    static constexpr const bool has_infinity = __base::has_infinity;
    static constexpr const bool has_quiet_NaN = __base::has_quiet_NaN;
    static constexpr const bool has_signaling_NaN = __base::has_signaling_NaN;
    static constexpr const float_denorm_style has_denorm = __base::has_denorm;
    static constexpr const bool has_denorm_loss = __base::has_denorm_loss;
    __attribute__ ((__always_inline__)) static constexpr type infinity() noexcept {return __base::infinity();}
    __attribute__ ((__always_inline__)) static constexpr type quiet_NaN() noexcept {return __base::quiet_NaN();}
    __attribute__ ((__always_inline__)) static constexpr type signaling_NaN() noexcept {return __base::signaling_NaN();}
    __attribute__ ((__always_inline__)) static constexpr type denorm_min() noexcept {return __base::denorm_min();}

    static constexpr const bool is_iec559 = __base::is_iec559;
    static constexpr const bool is_bounded = __base::is_bounded;
    static constexpr const bool is_modulo = __base::is_modulo;

    static constexpr const bool traps = __base::traps;
    static constexpr const bool tinyness_before = __base::tinyness_before;
    static constexpr const float_round_style round_style = __base::round_style;
};

template <class _Tp>
    constexpr const bool numeric_limits<const _Tp>::is_specialized;
template <class _Tp>
    constexpr const int numeric_limits<const _Tp>::digits;
template <class _Tp>
    constexpr const int numeric_limits<const _Tp>::digits10;
template <class _Tp>
    constexpr const int numeric_limits<const _Tp>::max_digits10;
template <class _Tp>
    constexpr const bool numeric_limits<const _Tp>::is_signed;
template <class _Tp>
    constexpr const bool numeric_limits<const _Tp>::is_integer;
template <class _Tp>
    constexpr const bool numeric_limits<const _Tp>::is_exact;
template <class _Tp>
    constexpr const int numeric_limits<const _Tp>::radix;
template <class _Tp>
    constexpr const int numeric_limits<const _Tp>::min_exponent;
template <class _Tp>
    constexpr const int numeric_limits<const _Tp>::min_exponent10;
template <class _Tp>
    constexpr const int numeric_limits<const _Tp>::max_exponent;
template <class _Tp>
    constexpr const int numeric_limits<const _Tp>::max_exponent10;
template <class _Tp>
    constexpr const bool numeric_limits<const _Tp>::has_infinity;
template <class _Tp>
    constexpr const bool numeric_limits<const _Tp>::has_quiet_NaN;
template <class _Tp>
    constexpr const bool numeric_limits<const _Tp>::has_signaling_NaN;
template <class _Tp>
    constexpr const float_denorm_style numeric_limits<const _Tp>::has_denorm;
template <class _Tp>
    constexpr const bool numeric_limits<const _Tp>::has_denorm_loss;
template <class _Tp>
    constexpr const bool numeric_limits<const _Tp>::is_iec559;
template <class _Tp>
    constexpr const bool numeric_limits<const _Tp>::is_bounded;
template <class _Tp>
    constexpr const bool numeric_limits<const _Tp>::is_modulo;
template <class _Tp>
    constexpr const bool numeric_limits<const _Tp>::traps;
template <class _Tp>
    constexpr const bool numeric_limits<const _Tp>::tinyness_before;
template <class _Tp>
    constexpr const float_round_style numeric_limits<const _Tp>::round_style;

template <class _Tp>
class  numeric_limits<volatile _Tp>
    : private numeric_limits<_Tp>
{
    typedef numeric_limits<_Tp> __base;
    typedef _Tp type;
public:
    static constexpr const bool is_specialized = __base::is_specialized;
    __attribute__ ((__always_inline__)) static constexpr type min() noexcept {return __base::min();}
    __attribute__ ((__always_inline__)) static constexpr type max() noexcept {return __base::max();}
    __attribute__ ((__always_inline__)) static constexpr type lowest() noexcept {return __base::lowest();}

    static constexpr const int  digits = __base::digits;
    static constexpr const int  digits10 = __base::digits10;
    static constexpr const int  max_digits10 = __base::max_digits10;
    static constexpr const bool is_signed = __base::is_signed;
    static constexpr const bool is_integer = __base::is_integer;
    static constexpr const bool is_exact = __base::is_exact;
    static constexpr const int  radix = __base::radix;
    __attribute__ ((__always_inline__)) static constexpr type epsilon() noexcept {return __base::epsilon();}
    __attribute__ ((__always_inline__)) static constexpr type round_error() noexcept {return __base::round_error();}

    static constexpr const int  min_exponent = __base::min_exponent;
    static constexpr const int  min_exponent10 = __base::min_exponent10;
    static constexpr const int  max_exponent = __base::max_exponent;
    static constexpr const int  max_exponent10 = __base::max_exponent10;

    static constexpr const bool has_infinity = __base::has_infinity;
    static constexpr const bool has_quiet_NaN = __base::has_quiet_NaN;
    static constexpr const bool has_signaling_NaN = __base::has_signaling_NaN;
    static constexpr const float_denorm_style has_denorm = __base::has_denorm;
    static constexpr const bool has_denorm_loss = __base::has_denorm_loss;
    __attribute__ ((__always_inline__)) static constexpr type infinity() noexcept {return __base::infinity();}
    __attribute__ ((__always_inline__)) static constexpr type quiet_NaN() noexcept {return __base::quiet_NaN();}
    __attribute__ ((__always_inline__)) static constexpr type signaling_NaN() noexcept {return __base::signaling_NaN();}
    __attribute__ ((__always_inline__)) static constexpr type denorm_min() noexcept {return __base::denorm_min();}

    static constexpr const bool is_iec559 = __base::is_iec559;
    static constexpr const bool is_bounded = __base::is_bounded;
    static constexpr const bool is_modulo = __base::is_modulo;

    static constexpr const bool traps = __base::traps;
    static constexpr const bool tinyness_before = __base::tinyness_before;
    static constexpr const float_round_style round_style = __base::round_style;
};

template <class _Tp>
    constexpr const bool numeric_limits<volatile _Tp>::is_specialized;
template <class _Tp>
    constexpr const int numeric_limits<volatile _Tp>::digits;
template <class _Tp>
    constexpr const int numeric_limits<volatile _Tp>::digits10;
template <class _Tp>
    constexpr const int numeric_limits<volatile _Tp>::max_digits10;
template <class _Tp>
    constexpr const bool numeric_limits<volatile _Tp>::is_signed;
template <class _Tp>
    constexpr const bool numeric_limits<volatile _Tp>::is_integer;
template <class _Tp>
    constexpr const bool numeric_limits<volatile _Tp>::is_exact;
template <class _Tp>
    constexpr const int numeric_limits<volatile _Tp>::radix;
template <class _Tp>
    constexpr const int numeric_limits<volatile _Tp>::min_exponent;
template <class _Tp>
    constexpr const int numeric_limits<volatile _Tp>::min_exponent10;
template <class _Tp>
    constexpr const int numeric_limits<volatile _Tp>::max_exponent;
template <class _Tp>
    constexpr const int numeric_limits<volatile _Tp>::max_exponent10;
template <class _Tp>
    constexpr const bool numeric_limits<volatile _Tp>::has_infinity;
template <class _Tp>
    constexpr const bool numeric_limits<volatile _Tp>::has_quiet_NaN;
template <class _Tp>
    constexpr const bool numeric_limits<volatile _Tp>::has_signaling_NaN;
template <class _Tp>
    constexpr const float_denorm_style numeric_limits<volatile _Tp>::has_denorm;
template <class _Tp>
    constexpr const bool numeric_limits<volatile _Tp>::has_denorm_loss;
template <class _Tp>
    constexpr const bool numeric_limits<volatile _Tp>::is_iec559;
template <class _Tp>
    constexpr const bool numeric_limits<volatile _Tp>::is_bounded;
template <class _Tp>
    constexpr const bool numeric_limits<volatile _Tp>::is_modulo;
template <class _Tp>
    constexpr const bool numeric_limits<volatile _Tp>::traps;
template <class _Tp>
    constexpr const bool numeric_limits<volatile _Tp>::tinyness_before;
template <class _Tp>
    constexpr const float_round_style numeric_limits<volatile _Tp>::round_style;

template <class _Tp>
class  numeric_limits<const volatile _Tp>
    : private numeric_limits<_Tp>
{
    typedef numeric_limits<_Tp> __base;
    typedef _Tp type;
public:
    static constexpr const bool is_specialized = __base::is_specialized;
    __attribute__ ((__always_inline__)) static constexpr type min() noexcept {return __base::min();}
    __attribute__ ((__always_inline__)) static constexpr type max() noexcept {return __base::max();}
    __attribute__ ((__always_inline__)) static constexpr type lowest() noexcept {return __base::lowest();}

    static constexpr const int  digits = __base::digits;
    static constexpr const int  digits10 = __base::digits10;
    static constexpr const int  max_digits10 = __base::max_digits10;
    static constexpr const bool is_signed = __base::is_signed;
    static constexpr const bool is_integer = __base::is_integer;
    static constexpr const bool is_exact = __base::is_exact;
    static constexpr const int  radix = __base::radix;
    __attribute__ ((__always_inline__)) static constexpr type epsilon() noexcept {return __base::epsilon();}
    __attribute__ ((__always_inline__)) static constexpr type round_error() noexcept {return __base::round_error();}

    static constexpr const int  min_exponent = __base::min_exponent;
    static constexpr const int  min_exponent10 = __base::min_exponent10;
    static constexpr const int  max_exponent = __base::max_exponent;
    static constexpr const int  max_exponent10 = __base::max_exponent10;

    static constexpr const bool has_infinity = __base::has_infinity;
    static constexpr const bool has_quiet_NaN = __base::has_quiet_NaN;
    static constexpr const bool has_signaling_NaN = __base::has_signaling_NaN;
    static constexpr const float_denorm_style has_denorm = __base::has_denorm;
    static constexpr const bool has_denorm_loss = __base::has_denorm_loss;
    __attribute__ ((__always_inline__)) static constexpr type infinity() noexcept {return __base::infinity();}
    __attribute__ ((__always_inline__)) static constexpr type quiet_NaN() noexcept {return __base::quiet_NaN();}
    __attribute__ ((__always_inline__)) static constexpr type signaling_NaN() noexcept {return __base::signaling_NaN();}
    __attribute__ ((__always_inline__)) static constexpr type denorm_min() noexcept {return __base::denorm_min();}

    static constexpr const bool is_iec559 = __base::is_iec559;
    static constexpr const bool is_bounded = __base::is_bounded;
    static constexpr const bool is_modulo = __base::is_modulo;

    static constexpr const bool traps = __base::traps;
    static constexpr const bool tinyness_before = __base::tinyness_before;
    static constexpr const float_round_style round_style = __base::round_style;
};

template <class _Tp>
    constexpr const bool numeric_limits<const volatile _Tp>::is_specialized;
template <class _Tp>
    constexpr const int numeric_limits<const volatile _Tp>::digits;
template <class _Tp>
    constexpr const int numeric_limits<const volatile _Tp>::digits10;
template <class _Tp>
    constexpr const int numeric_limits<const volatile _Tp>::max_digits10;
template <class _Tp>
    constexpr const bool numeric_limits<const volatile _Tp>::is_signed;
template <class _Tp>
    constexpr const bool numeric_limits<const volatile _Tp>::is_integer;
template <class _Tp>
    constexpr const bool numeric_limits<const volatile _Tp>::is_exact;
template <class _Tp>
    constexpr const int numeric_limits<const volatile _Tp>::radix;
template <class _Tp>
    constexpr const int numeric_limits<const volatile _Tp>::min_exponent;
template <class _Tp>
    constexpr const int numeric_limits<const volatile _Tp>::min_exponent10;
template <class _Tp>
    constexpr const int numeric_limits<const volatile _Tp>::max_exponent;
template <class _Tp>
    constexpr const int numeric_limits<const volatile _Tp>::max_exponent10;
template <class _Tp>
    constexpr const bool numeric_limits<const volatile _Tp>::has_infinity;
template <class _Tp>
    constexpr const bool numeric_limits<const volatile _Tp>::has_quiet_NaN;
template <class _Tp>
    constexpr const bool numeric_limits<const volatile _Tp>::has_signaling_NaN;
template <class _Tp>
    constexpr const float_denorm_style numeric_limits<const volatile _Tp>::has_denorm;
template <class _Tp>
    constexpr const bool numeric_limits<const volatile _Tp>::has_denorm_loss;
template <class _Tp>
    constexpr const bool numeric_limits<const volatile _Tp>::is_iec559;
template <class _Tp>
    constexpr const bool numeric_limits<const volatile _Tp>::is_bounded;
template <class _Tp>
    constexpr const bool numeric_limits<const volatile _Tp>::is_modulo;
template <class _Tp>
    constexpr const bool numeric_limits<const volatile _Tp>::traps;
template <class _Tp>
    constexpr const bool numeric_limits<const volatile _Tp>::tinyness_before;
template <class _Tp>
    constexpr const float_round_style numeric_limits<const volatile _Tp>::round_style;

} }

_Pragma("pop_macro(\"min\")") _Pragma("pop_macro(\"max\")")


namespace AdvancedMicrotech {

/**
 * Helper class to define a field within a frame. It provides with functionality to encode and decode the field within
 * a frame.
 *
 * @tparam FIELD_TYPE The type of the field. Can be any integral type. The decode will return this type, and the encode
 * will take it as an input parameter.
 * @tparam FRAME_TYPE The type of the frame. Can be any integral type. If it is a 16bit, 8bit value and so on.
 * @tparam mask The mask of where the bits with the information is within the frame.
 * @tparam position Is the right most position. The number of shifts to the right when decoding, and number of left
 *                  shifts when encoding.
 */
template<typename FIELD_TYPE, typename FRAME_TYPE, FRAME_TYPE mask, FRAME_TYPE position>
class Field {
  static_assert(std::is_integral<FRAME_TYPE>::value, "Frame type is not integral");
  static_assert(std::numeric_limits<FRAME_TYPE>::digits > position, "Position shift bigger than value coming");

public:
  /**
   * Method to decode a field within a frame.
   * @param value The raw value of the frame
   * @return The decoded value.
   */
  static constexpr FIELD_TYPE decode(const FRAME_TYPE value) {
    return static_cast<FIELD_TYPE>((value & mask) >> position);
  }

  /**
   * Method to encode the field into the frame to be send.
   *
   * @param value [output] The frame will be populated with the encoded value.
   * @param toEncode Value to encode from the field type
   * @return void
   */
  static constexpr void encode(FRAME_TYPE& value, FIELD_TYPE toEncode) {
    const FRAME_TYPE shiftedValue = static_cast<FRAME_TYPE>(toEncode) << POSITION;
    const FRAME_TYPE setVal = MASK & shiftedValue;
    const FRAME_TYPE resetVal = MASK & ~shiftedValue;
    value |= setVal;
    value &= ~resetVal;
  }

  static constexpr FRAME_TYPE MASK = mask;
  static constexpr FRAME_TYPE POSITION = position;
};



/**
 * Clears entire display and sets DDRAM address 0 in address counter.
 */
static constexpr uint8_t CLEAR_DISPLAY = 0x01;

/**
 * Sets DDRAM address 0 in address counter. Also returns display from being
 * shifted to original position. DDRAM contents remain unchanged.
 */
static constexpr uint8_t RETURN_HOME = 0x02;

/**
 * Class specifying fields of the EntryModeSet instruction.
 */
class EntryModeSet {
public:
    using RawType = uint8_t;

    enum class CursorMoveDirection {
        DECREMENT = 0,
        INCREMENT
    };

    static constexpr RawType BASE_VALUE = 0x04;

    static constexpr Field<bool, RawType, 0x01, 0> ACCOMPANIES_DISPLAY_SHIFT{};
    static constexpr Field<CursorMoveDirection, RawType, 0x02, 1> CURSOR_MOVE_DIRECTION{};

};

class DisplayControl {
public:
    using RawType = uint8_t;

    static constexpr RawType BASE_VALUE = 0x08;

    static constexpr Field<bool, RawType, 0x01, 0> BLINK_CURSOR{};
    static constexpr Field<bool, RawType, 0x02, 1> CURSOR_ON{};
    static constexpr Field<bool, RawType, 0x04, 2> DISPLAY_ON{};
};

class CursorOrDisplayShift {
public:
    using RawType = uint8_t;

    enum class ShiftSelection {
        CURSOR_MOVE = 0,
        DISPLAY_SHIFT,
    };

    enum class ShiftDirection {
        LEFT = 0,
        RIGHT,
    };
    static constexpr RawType BASE_VALUE = 0x10;

    static constexpr Field<ShiftDirection, RawType, 0x04, 2> SHIFT_DIRECTION{};
    static constexpr Field<ShiftSelection, RawType, 0x08, 3> SHIFT_SELECTION{};
};

class FunctionSet {
public:
    using RawType = uint8_t;

    enum class InterfaceDataLength : RawType {
        BITS_4 = 0,
        BITS_8,
    };

    enum class NumberOfLines : RawType{
        LINE_1 = 0,
        LINES_2
    };

    enum class CharachterFont : RawType{
        DOTS_5X8 = 0,
        DOTS_5X10 = 1,
    };

    static constexpr RawType BASE_VALUE = 0x20;

    static constexpr Field<CharachterFont, RawType, 0x04, 2> CHARACHTER_FONT{};
    static constexpr Field<NumberOfLines, RawType, 0x08, 3> NUMBER_OF_LINES{};
    static constexpr Field<InterfaceDataLength, RawType, 0x10, 4> INTERFACE_DATA_LENGTH{};
};

}


namespace AdvancedMicrotech {

template <typename RS,
          typename RW,
          typename E,
          typename BUS>
class LCD_T {
public:
  /**
   * Initialization of the LCD; set all pin directions, basic setup of the LCD, etc.
   */
  static constexpr void initialize() {
      displayControl = DisplayControl::BASE_VALUE;
      functionSet = FunctionSet::BASE_VALUE;

      // Initialize IOs
      RS::init();
      E::init();
      RW::init();
      BUS::init();

      // Create the function set frame to how we want to work:
      //  - Interface of 4 bits
      //  - 2 Lines
      //  - Character size 5x8 dots
      FunctionSet::INTERFACE_DATA_LENGTH.encode(functionSet, FunctionSet::InterfaceDataLength::BITS_4);
      FunctionSet::NUMBER_OF_LINES.encode(functionSet, FunctionSet::NumberOfLines::LINES_2);
      FunctionSet::CHARACHTER_FONT.encode(functionSet, FunctionSet::CharachterFont::DOTS_5X8);

      // Sets the function set to 4 bits.
      // According to datasheet, we first need to write the MSB of the functionSet frame once
      // and then write the whole frame again, so we set it first
      // to 4-bit operation.
      writeWordToBus(functionSet >> 4);
      sendInstruction(functionSet);
      clearDisplay();
  }
  /**
   * Enable (1) or disable (0) the display (i.e. hide all text)
   */
  static void enable(const bool enable) noexcept {
      DisplayControl::DISPLAY_ON.encode(displayControl, enable);
      sendInstruction(displayControl);
  }

  /**
   * Set the cursor to a certain x/y-position
   * (x,y) -> (0,0) is top left and (15, 1) is bottom right.
   */
  static void setCursorPosition(const uint8_t x, const uint8_t y) noexcept {
      static constexpr uint8_t SET_ADDRESS_BASE = 0x80;
      uint8_t address = SET_ADDRESS_BASE;
      if(y >= 1) {
          address |= 0x40;
      }
      if(x > 0x0F) {
          address |= 0x0F;
      } else {
          address |= x;
      }

      waitUntilNotBusy();
      setRegister(RegisterSelection::INSTRUCTION);
      writeByteToBus(address);
  }

  /**
   *  Show or hide the cursor
   *  @param show boolean to choose if the cursor should be shown. True to show, false to hide.
   */
  static void showCursor(const bool show) noexcept {
      DisplayControl::CURSOR_ON.encode(displayControl, show);
      sendInstruction(displayControl);
  }

  /**
   * Method to blink or not the cursor of the LCD.
   * @param blink boolean to choose if the cursor should blink. True to blink, false not to.
   */
  static void blinkCursor(const bool blink) noexcept {
      DisplayControl::BLINK_CURSOR.encode(displayControl, blink);
      sendInstruction(displayControl);
  }

  /**
   * Delete everything on the LCD
   */
  static void clearDisplay() noexcept {
      sendInstruction(CLEAR_DISPLAY);
  }

  /**
   * Put a single character on the display at the cursor's current position
   * @param character char to be written.
   */
  static void writeChar(const char character) noexcept {
      setRegister(RegisterSelection::DATA);
      writeByteToBus(character);
  }

  /**
   * Show a given string on the display. If the text is too long to display,
   * there is no line break, the string is being cut.
   * @param text string to be written
   */
  static void writeString(const char* text) noexcept {
      while(*text != 0x00) {
          writeChar(*text);
          text++;
      }
  }

  /**
   * Show a given number at the cursor's current position.
   * @param number
   */
  static void writeNumber(const int16_t number) noexcept {
      static constexpr int16_t SIGN_MASK = 0x8000;
      // Check if number is negative
      if(SIGN_MASK & number) {
          writeChar(0x2D); // Minus sign in ASCII

          // Flip every bit of it so its like a normal number
          // without the sign and add 1 because -1 is 0xFFFF, so after flipping
          // it is 0x0000 + 1 = 0x0001;
          number = ~number;
          number++;
      }
      int8_t i = 4;
      while(i >= 0) {
          writeDigitOfNumber(number, i);
          i--;
      }
  }

private:
  enum class RegisterSelection {
      INSTRUCTION = 0,
      DATA
  };
  enum class DataOperation {
      WRITE = 0,
      READ
  };

  static constexpr void setOperation(const DataOperation newOperation) noexcept {
      if(newOperation == DataOperation::READ) {
          readWrite.setState(Microtech::IOState::HIGH);
      } else {
          readWrite.setState(Microtech::IOState::LOW);
      }
  }
  static constexpr void setRegister(const RegisterSelection newSelection) noexcept {
      if(newSelection == RegisterSelection::DATA) {
          selectRegister.setState(Microtech::IOState::HIGH);
      } else {
          selectRegister.setState(Microtech::IOState::LOW);
      }
  }

  static constexpr void enableOperation(const bool enableOperation) noexcept {
      enableReadWrite.setState(enableOperation);
  }

  static void sendInstruction(const uint8_t instruction) noexcept {
      waitUntilNotBusy();
      setRegister(RegisterSelection::INSTRUCTION);
      writeByteToBus(instruction);
  }

  /**
   * Method to write word (4 bits) to the bus.
   * @param dataToWrite only the 4 LSB bits will be written
   */
  static void writeWordToBus(const uint8_t dataToWrite) noexcept {
     enableOperation(false);  // Disable write
     setOperation(DataOperation::WRITE);                 // Set to write mode
     dataBus.write(dataToWrite);                         // Put data to bus
     enableOperation(true); // Enable write

     //__delay_cycles(100);
     enableOperation(false);  // Disable write
  }
  /**
   * Method to write a byte (8 bits) to the bus.
   * @param dataToWrite byte to be written
   */
  static void writeByteToBus(const uint8_t dataToWrite) noexcept {
      const uint8_t dataMsb = (dataToWrite) >> 4;
      const uint8_t dataLsb = (0x0F & dataToWrite);
      writeWordToBus(dataMsb);
      writeWordToBus(dataLsb);
  }
  /**
   * Method to wait for the display until it is not busy anymore.
   * It is a blocking call. So once called it will only return when the display
   * is not busy anymore.
   */
  static void waitUntilNotBusy() noexcept {
      static constexpr uint8_t BUSY_MASK = 0x08;      // The mask to decode the busy bit
      setRegister(RegisterSelection::INSTRUCTION);
      setOperation(DataOperation::READ);              // Set to read mode
      dataBus.read();                                 // Perform a read, but main goal is to set the data bus as input.
      enableOperation(true);                          // Enable read
      //__delay_cycles(20);
      uint8_t dataVal = dataBus.read();               // Read value from Bus
      while(dataVal & BUSY_MASK) {
          __no_operation();
          dataVal = dataBus.read();                   // Read value from Bus
      }

      uint8_t addrCounter = (0xE & dataVal) << 4;
      enableOperation(false);      // Disable read
      //__delay_cycles(100);
      enableOperation(true);     // Enable read
      //__delay_cycles(100);
      addrCounter |= dataBus.read();                          // Get value from Bus
      enableOperation(false);      // Disable read
  }

  static void writeDigitOfNumber(int16_t number, uint8_t digit) {
      static constexpr int16_t POW10[] = {1, 10, 100, 1000, 10000};
      static constexpr int16_t BASE_ASCII = 0x30;
      if(digit > 4) {
          // Only support up to 4 digits. Early return
          return;
      }
      uint8_t digitToBeRemoved = 0;
      if(digit < 4) {
          digitToBeRemoved = digit + 1;
      } else {
          digitToBeRemoved = digit;
      }
      const int16_t originalNumber = number;
      const int16_t numberToProcess = number % POW10[digitToBeRemoved];
      if (originalNumber >= POW10[digit]) {
          const int16_t toBeWritten = numberToProcess / POW10[digit];
          writeChar(BASE_ASCII + toBeWritten);
      }
  }

  /**
   * Signal RS. Selects registers.
   *    False = Instruction register (for write)
   *            Busy flag: address counter (for read)
   *    True  = Data register (for write and read)
   */
  //const Microtech::OutputHandle selectRegister{Microtech::GPIOs::getOutputHandle<Microtech::IOPort::PORT_3, static_cast<uint8_t>(0)>()};
  //const Microtech::OutputHandle readWrite = Microtech::GPIOs::getOutputHandle<Microtech::IOPort::PORT_3, static_cast<uint8_t>(1)>();          ///< Signal R/W. Selects read or write. False = write; True = read
  //const Microtech::OutputHandle enableReadWrite = Microtech::GPIOs::getOutputHandle<Microtech::IOPort::PORT_3, static_cast<uint8_t>(2)>();     ///< Signal E. Enables/disables data read or write


 // Microtech::IoBus<Microtech::IOPort::PORT_2, 0x0F> dataBus; ///< Databus that connects the display to the microcontroller

  static uint8_t displayControl;           ///< Attribute that holds the current "state" of the display control.
  static uint8_t functionSet;              ///< Attribute that holds the current "state" of the function set.
};

}

// -*- C++ -*-
//===---------------------------- cstdio ----------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is dual licensed under the MIT and the University of Illinois Open
// Source Licenses. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//


/*
    cstdio synopsis

Macros:

    BUFSIZ
    EOF
    FILENAME_MAX
    FOPEN_MAX
    L_tmpnam
    NULL
    SEEK_CUR
    SEEK_END
    SEEK_SET
    TMP_MAX
    _IOFBF
    _IOLBF
    _IONBF
    stderr
    stdin
    stdout

namespace std
{

Types:

FILE
fpos_t
size_t

int remove(const char* filename);
int rename(const char* old, const char* new);
FILE* tmpfile(void);
char* tmpnam(char* s);
int fclose(FILE* stream);
int fflush(FILE* stream);
FILE* fopen(const char* restrict filename, const char* restrict mode);
FILE* freopen(const char* restrict filename, const char * restrict mode,
              FILE * restrict stream);
void setbuf(FILE* restrict stream, char* restrict buf);
int setvbuf(FILE* restrict stream, char* restrict buf, int mode, size_t size);
int fprintf(FILE* restrict stream, const char* restrict format, ...);
int fscanf(FILE* restrict stream, const char * restrict format, ...);
int printf(const char* restrict format, ...);
int scanf(const char* restrict format, ...);
int snprintf(char* restrict s, size_t n, const char* restrict format, ...);    // C99
int sprintf(char* restrict s, const char* restrict format, ...);
int sscanf(const char* restrict s, const char* restrict format, ...);
int vfprintf(FILE* restrict stream, const char* restrict format, va_list arg);
int vfscanf(FILE* restrict stream, const char* restrict format, va_list arg);  // C99
int vprintf(const char* restrict format, va_list arg);
int vscanf(const char* restrict format, va_list arg);                          // C99
int vsnprintf(char* restrict s, size_t n, const char* restrict format,         // C99
              va_list arg);
int vsprintf(char* restrict s, const char* restrict format, va_list arg);
int vsscanf(const char* restrict s, const char* restrict format, va_list arg); // C99
int fgetc(FILE* stream);
char* fgets(char* restrict s, int n, FILE* restrict stream);
int fputc(int c, FILE* stream);
int fputs(const char* restrict s, FILE* restrict stream);
int getc(FILE* stream);
int getchar(void);
char* gets(char* s);  // removed in C++14
int putc(int c, FILE* stream);
int putchar(int c);
int puts(const char* s);
int ungetc(int c, FILE* stream);
size_t fread(void* restrict ptr, size_t size, size_t nmemb,
             FILE* restrict stream);
size_t fwrite(const void* restrict ptr, size_t size, size_t nmemb,
              FILE* restrict stream);
int fgetpos(FILE* restrict stream, fpos_t* restrict pos);
int fseek(FILE* stream, long offset, int whence);
int fsetpos(FILE*stream, const fpos_t* pos);
long ftell(FILE* stream);
void rewind(FILE* stream);
void clearerr(FILE* stream);
int feof(FILE* stream);
int ferror(FILE* stream);
void perror(const char* s);

}  // std
*/

/* -*- C++ -*- */
/*===--------------------------- complex.h --------------------------------===*/
/*                                                                            */
/*                     The LLVM Compiler Infrastructure                       */
/*                                                                            */
/* This file is dual licensed under the MIT and the University of Illinois Open
** Source Licenses. See LICENSE.TXT for details.
*/
/*===----------------------------------------------------------------------===*/


/*
    stdio.h synopsis

Macros:

    BUFSIZ
    EOF
    FILENAME_MAX
    FOPEN_MAX
    L_tmpnam
    NULL
    SEEK_CUR
    SEEK_END
    SEEK_SET
    TMP_MAX
    _IOFBF
    _IOLBF
    _IONBF
    stderr
    stdin
    stdout

Types:

FILE
fpos_t
size_t

int remove(const char* filename);
int rename(const char* old, const char* new);
FILE* tmpfile(void);
char* tmpnam(char* s);
int fclose(FILE* stream);
int fflush(FILE* stream);
FILE* fopen(const char* restrict filename, const char* restrict mode);
FILE* freopen(const char* restrict filename, const char * restrict mode,
              FILE * restrict stream);
void setbuf(FILE* restrict stream, char* restrict buf);
int setvbuf(FILE* restrict stream, char* restrict buf, int mode, size_t size);
int fprintf(FILE* restrict stream, const char* restrict format, ...);
int fscanf(FILE* restrict stream, const char * restrict format, ...);
int printf(const char* restrict format, ...);
int scanf(const char* restrict format, ...);
int snprintf(char* restrict s, size_t n, const char* restrict format, ...);    // C99
int sprintf(char* restrict s, const char* restrict format, ...);
int sscanf(const char* restrict s, const char* restrict format, ...);
int vfprintf(FILE* restrict stream, const char* restrict format, va_list arg);
int vfscanf(FILE* restrict stream, const char* restrict format, va_list arg);  // C99
int vprintf(const char* restrict format, va_list arg);
int vscanf(const char* restrict format, va_list arg);                          // C99
int vsnprintf(char* restrict s, size_t n, const char* restrict format,         // C99
              va_list arg);
int vsprintf(char* restrict s, const char* restrict format, va_list arg);
int vsscanf(const char* restrict s, const char* restrict format, va_list arg); // C99
int fgetc(FILE* stream);
char* fgets(char* restrict s, int n, FILE* restrict stream);
int fputc(int c, FILE* stream);
int fputs(const char* restrict s, FILE* restrict stream);
int getc(FILE* stream);
int getchar(void);
char* gets(char* s);  // removed in C++14
int putc(int c, FILE* stream);
int putchar(int c);
int puts(const char* s);
int ungetc(int c, FILE* stream);
size_t fread(void* restrict ptr, size_t size, size_t nmemb,
             FILE* restrict stream);
size_t fwrite(const void* restrict ptr, size_t size, size_t nmemb,
              FILE* restrict stream);
int fgetpos(FILE* restrict stream, fpos_t* restrict pos);
int fseek(FILE* stream, long offset, int whence);
int fsetpos(FILE*stream, const fpos_t* pos);
long ftell(FILE* stream);
void rewind(FILE* stream);
void clearerr(FILE* stream);
int feof(FILE* stream);
int ferror(FILE* stream);
void perror(const char* s);
*/



/*****************************************************************************/
/* STDIO.H                                                                   */
/*                                                                           */
/* Copyright (c) 1993 Texas Instruments Incorporated                         */
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


/*****************************************************************************/
/* stdarg.h                                                                  */
/*                                                                           */
/* Copyright (c) 1996 Texas Instruments Incorporated                         */
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


#pragma diag_push
#pragma CHECK_MISRA("-20.1") /* standard headers must define standard names */
#pragma CHECK_MISRA("-20.2") /* standard headers must define standard names */
#pragma CHECK_MISRA("-19.7") /* function-like macros are OK */
#pragma CHECK_MISRA("-19.10") /* need types as macro arguments */


typedef __va_list va_list;


/*****************************************************************************/
/* On the MSP430, the stack grows down (towards 0x0), and arguments are      */
/* pushed in reverse order, so later arguments are at higher addresses.      */
/*****************************************************************************/

/*****************************************************************************/
/* VA_START(va_list ap, parmN)                                               */
/*                                                                           */
/*   Set "ap" to point to the address of the next argument past parmN.       */
/*   So add the size of parmN to the address of parmN.                       */
/*                                                                           */
/*   NOTES -must use the address of the level 1 symbol (via va_parmadr).     */
/*         -must account for "chars", which are widened to "ints".           */
/*                                                                           */
/*****************************************************************************/
/*****************************************************************************/
/* VA_ARG(va_list, type)                                                     */
/*                                                                           */
/*   Return next argument (currently pointed to by "va_list", and set the    */
/*   argument pointer to point to the next argument after current one.       */
/*                                                                           */
/*   Notes -must handle variables passed by reference (_va_argref())         */
/*         -must round up chars.                                             */
/*                                                                           */
/*****************************************************************************/



#pragma diag_pop



_Pragma("diag_push")
_Pragma("CHECK_MISRA(\"-20.2\")") /* reuse of standard macros/objects/funcs */
_Pragma("CHECK_MISRA(\"-20.1\")") /* redefining reserved identifiers */
_Pragma("CHECK_MISRA(\"-19.7\")") /* use function instead of function-like macro */
_Pragma("CHECK_MISRA(\"-19.4\")") /* macros should have only simple expansions */
_Pragma("CHECK_MISRA(\"-19.1\")") /* include should follow directive or comment */
_Pragma("CHECK_MISRA(\"-16.3\")") /* parameters without names */
_Pragma("CHECK_MISRA(\"-6.3\")")  /* use size/sign-specific typedefs */

/*---------------------------------------------------------------------------*/
/* Attributes are only available in relaxed ANSI mode.                       */
/*---------------------------------------------------------------------------*/

extern "C" {

/****************************************************************************/
/* TYPES THAT ANSI REQUIRES TO BE DEFINED                                   */
/****************************************************************************/

struct __sFILE {
      int fd;                    /* File descriptor */
      unsigned char* buf;        /* Pointer to start of buffer */
      unsigned char* pos;        /* Position in buffer */
      unsigned char* bufend;     /* Pointer to end of buffer */
      unsigned char* buff_stop;  /* Pointer to last read char in buffer */
      unsigned int   flags;      /* File status flags (see below) */
};

typedef struct __sFILE FILE;

typedef long fpos_t;

/* For C6x, long type can be 40-bits, so we use int to get 32-bit off_t. */
typedef long off_t;

/****************************************************************************/
/* DEVICE AND STREAM RELATED MACROS                                         */
/****************************************************************************/
/****************************************************************************/
/* MACROS THAT DEFINE AND USE FILE STATUS FLAGS                             */
/****************************************************************************/



/****************************************************************************/
/* MACROS THAT ANSI REQUIRES TO BE DEFINED                                  */
/****************************************************************************/








/******** END OF ANSI MACROS ************************************************/


/****************************************************************************/
/* DEVICE AND STREAM RELATED DATA STRUCTURES AND MACROS                     */
/****************************************************************************/

extern  FILE _ftable[10];
extern  char __TI_tmpnams[10][16];

/****************************************************************************/
/*   FUNCTION DEFINITIONS  - ANSI                                           */
/****************************************************************************/
/****************************************************************************/
/* OPERATIONS ON FILES                                                      */
/****************************************************************************/
extern  int     remove(const char *_file);
extern  int     rename(const char *_old, const char *_new);
extern  FILE   *tmpfile(void);
extern  char   *tmpnam(char *_s);

/****************************************************************************/
/* FILE ACCESS FUNCTIONS                                                    */
/****************************************************************************/
extern  int     fclose(FILE * __restrict _fp);
extern  FILE   *fopen(const char * __restrict _fname,
                                  const char * __restrict _mode);
extern  FILE   *freopen(const char * __restrict _fname,
                                    const char * __restrict _mode,
			            FILE * __restrict _fp);
extern  void    setbuf(FILE * __restrict _fp,
                                   char * __restrict _buf);
extern  int     setvbuf(FILE * __restrict _fp,
                                    char * __restrict _buf,
			            int _type, size_t _size);
extern  int     fflush(FILE *_fp);

/****************************************************************************/
/* FORMATTED INPUT/OUTPUT FUNCTIONS                                         */
/****************************************************************************/
extern  int fprintf(FILE * __restrict _fp,
                                const char * __restrict _format, ...)
               __attribute__((__format__ (__printf__, 2, 3)));
extern  int fscanf(FILE * __restrict _fp,
                               const char * __restrict _fmt, ...)
               __attribute__((__format__ (__scanf__, 2, 3)));
extern  int printf(const char * __restrict _format, ...)
               __attribute__((__format__ (__printf__, 1, 2)));
extern  int scanf(const char * __restrict _fmt, ...)
               __attribute__((__format__ (__scanf__, 1, 2)));
extern  int sprintf(char * __restrict _string,
                                const char * __restrict _format, ...)
               __attribute__((__format__ (__printf__, 2, 3)));
extern  int snprintf(char * __restrict _string, size_t _n,
				 const char * __restrict _format, ...)
               __attribute__((__format__ (__printf__, 3, 4)));
extern  int sscanf(const char * __restrict _str,
                               const char * __restrict _fmt, ...)
               __attribute__((__format__ (__scanf__, 2, 3)));
extern  int vfprintf(FILE * __restrict _fp,
                                 const char * __restrict _format, va_list _ap)
               __attribute__((__format__ (__printf__, 2, 0)));
extern  int vfscanf(FILE * __restrict _fp,
                                const char * __restrict _fmt, va_list _ap)
               __attribute__((__format__ (__scanf__, 2, 0)));
extern  int vprintf(const char * __restrict _format, va_list _ap)
               __attribute__((__format__ (__printf__, 1, 0)));
extern  int vscanf(const char * __restrict _format, va_list _ap)
               __attribute__((__format__ (__scanf__, 1, 0)));
extern  int vsprintf(char * __restrict _string,
                                 const char * __restrict _format, va_list _ap)
               __attribute__((__format__ (__printf__, 2, 0)));
extern  int vsnprintf(char * __restrict _string, size_t _n,
				  const char * __restrict _format, va_list _ap)
               __attribute__((__format__ (__printf__, 3, 0)));
extern  int vsscanf(const char * __restrict _str,
                                const char * __restrict _fmt, va_list _ap)
               __attribute__((__format__ (__scanf__, 2, 0)));
extern  int	asprintf(char **, const char *, ...)
               __attribute__((__format__ (__printf__, 2, 3)));
extern  int	vasprintf(char **, const char *, va_list)
               __attribute__((__format__ (__printf__, 2, 0)));

/****************************************************************************/
/* CHARACTER INPUT/OUTPUT FUNCTIONS                                         */
/****************************************************************************/
extern  int     fgetc(FILE *_fp);
extern  char   *fgets(char * __restrict _ptr, int _size,
				  FILE * __restrict _fp);
extern  int     fputc(int _c, FILE *_fp);
extern  int     fputs(const char * __restrict _ptr,
                                  FILE * __restrict _fp);
extern  int     getc(FILE *_p);
extern  int     getchar(void);
extern  char   *gets(char *_ptr);
extern  int     putc(int _x, FILE *_fp);
extern  int     putchar(int _x);
extern  int     puts(const char *_ptr);
extern  int     ungetc(int _c, FILE *_fp);

/****************************************************************************/
/* DIRECT INPUT/OUTPUT FUNCTIONS                                            */
/****************************************************************************/
extern  size_t  fread(void * __restrict _ptr,
                                  size_t _size, size_t _count,
				  FILE * __restrict _fp);
extern  size_t  fwrite(const void * __restrict _ptr,
                                   size_t _size, size_t _count,
                                   FILE * __restrict _fp);

/****************************************************************************/
/* FILE POSITIONING FUNCTIONS                                               */
/****************************************************************************/
extern  int     fgetpos(FILE * __restrict _fp,
                                    fpos_t * __restrict _pos);
extern  int     fseek(FILE *_fp, long _offset,
				  int _ptrname);
extern  int     fseeko(FILE *_fp, off_t _offset,
                                   int _ptrname);
extern  int     fsetpos(FILE * __restrict _fp,
                                    const fpos_t * __restrict _pos);
extern  long    ftell(FILE *_fp);
extern  off_t   ftello(FILE *_fp);
extern  void    rewind(FILE *_fp);

/****************************************************************************/
/* ERROR-HANDLING FUNCTIONS                                                 */
/****************************************************************************/
extern  void    clearerr(FILE *_fp);
extern  int     feof(FILE *_fp);
extern  int     ferror(FILE *_fp);
extern  void    perror(const char *_s);




} /* extern "C" */


/*----------------------------------------------------------------------------*/
/* If sys/cdefs.h is available, go ahead and include it. xlocale.h assumes    */
/* this file will have already included sys/cdefs.h.                          */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* If sys/_types.h is available, include it. xlocale.h assumes this file will */
/* have already provided a definition of __va_list.                           */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Include xlocale/_stdio.h if xlocale.h has already been included. This is   */
/* to conform with FreeBSD's xlocale implementation.                          */
/*----------------------------------------------------------------------------*/

_Pragma("diag_pop")



/* snprintf */





namespace std { inline namespace __2 {

using ::FILE;
using ::fpos_t;
using ::size_t;

using ::fclose;
using ::fflush;
using ::setbuf;
using ::setvbuf;
using ::fprintf;
using ::fscanf;
using ::snprintf;
using ::sprintf;
using ::sscanf;
using ::vfprintf;
using ::vfscanf;
using ::vsscanf;
using ::vsnprintf;
using ::vsprintf;
using ::fgetc;
using ::fgets;
using ::fputc;
using ::fputs;
using ::getc;
using ::putc;
using ::ungetc;
using ::fread;
using ::fwrite;
using ::fgetpos;
using ::fseek;
using ::fsetpos;
using ::ftell;
using ::rewind;
using ::clearerr;
using ::feof;
using ::ferror;
using ::perror;

using ::fopen;
using ::freopen;
using ::remove;
using ::rename;
using ::tmpfile;
using ::tmpnam;

using ::getchar;
using ::scanf;
using ::vscanf;

using ::printf;
using ::putchar;
using ::puts;
using ::vprintf;

} }

// -*- C++ -*-
//===--------------------------- cstdlib ----------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is dual licensed under the MIT and the University of Illinois Open
// Source Licenses. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//


/*
    cstdlib synopsis

Macros:

    EXIT_FAILURE
    EXIT_SUCCESS
    MB_CUR_MAX
    NULL
    RAND_MAX

namespace std
{

Types:

    size_t
    div_t
    ldiv_t
    lldiv_t                                                               // C99

double    atof (const char* nptr);
int       atoi (const char* nptr);
long      atol (const char* nptr);
long long atoll(const char* nptr);                                        // C99
double             strtod  (const char* restrict nptr, char** restrict endptr);
float              strtof  (const char* restrict nptr, char** restrict endptr); // C99
long double        strtold (const char* restrict nptr, char** restrict endptr); // C99
long               strtol  (const char* restrict nptr, char** restrict endptr, int base);
long long          strtoll (const char* restrict nptr, char** restrict endptr, int base); // C99
unsigned long      strtoul (const char* restrict nptr, char** restrict endptr, int base);
unsigned long long strtoull(const char* restrict nptr, char** restrict endptr, int base); // C99
int rand(void);
void srand(unsigned int seed);
void* calloc(size_t nmemb, size_t size);
void free(void* ptr);
void* malloc(size_t size);
void* realloc(void* ptr, size_t size);
void abort(void);
int atexit(void (*func)(void));
void exit(int status);
void _Exit(int status);
char* getenv(const char* name);
int system(const char* string);
void* bsearch(const void* key, const void* base, size_t nmemb, size_t size,
              int (*compar)(const void *, const void *));
void qsort(void* base, size_t nmemb, size_t size,
           int (*compar)(const void *, const void *));
int         abs(      int j);
long        abs(     long j);
long long   abs(long long j);                                             // C++0X
long       labs(     long j);
long long llabs(long long j);                                             // C99
div_t     div(      int numer,       int denom);
ldiv_t    div(     long numer,      long denom);
lldiv_t   div(long long numer, long long denom);                          // C++0X
ldiv_t   ldiv(     long numer,      long denom);
lldiv_t lldiv(long long numer, long long denom);                          // C99
int mblen(const char* s, size_t n);
int mbtowc(wchar_t* restrict pwc, const char* restrict s, size_t n);
int wctomb(char* s, wchar_t wchar);
size_t mbstowcs(wchar_t* restrict pwcs, const char* restrict s, size_t n);
size_t wcstombs(char* restrict s, const wchar_t* restrict pwcs, size_t n);
int at_quick_exit(void (*func)(void))                                     // C++11
void quick_exit(int status);                                              // C++11
void *aligned_alloc(size_t alignment, size_t size);                       // C11

}  // std

*/

/* -*- C++ -*- */
/*===--------------------------- complex.h --------------------------------===*/
/*                                                                            */
/*                     The LLVM Compiler Infrastructure                       */
/*                                                                            */
/* This file is dual licensed under the MIT and the University of Illinois Open
** Source Licenses. See LICENSE.TXT for details.
*/
/*===----------------------------------------------------------------------===*/


/*
    stdlib.h synopsis

Macros:

    EXIT_FAILURE
    EXIT_SUCCESS
    MB_CUR_MAX
    NULL
    RAND_MAX

Types:

    size_t
    div_t
    ldiv_t
    lldiv_t                                                               // C99

double    atof (const char* nptr);
int       atoi (const char* nptr);
long      atol (const char* nptr);
long long atoll(const char* nptr);                                        // C99
double             strtod  (const char* restrict nptr, char** restrict endptr);
float              strtof  (const char* restrict nptr, char** restrict endptr); // C99
long double        strtold (const char* restrict nptr, char** restrict endptr); // C99
long               strtol  (const char* restrict nptr, char** restrict endptr, int base);
long long          strtoll (const char* restrict nptr, char** restrict endptr, int base); // C99
unsigned long      strtoul (const char* restrict nptr, char** restrict endptr, int base);
unsigned long long strtoull(const char* restrict nptr, char** restrict endptr, int base); // C99
int rand(void);
void srand(unsigned int seed);
void* calloc(size_t nmemb, size_t size);
void free(void* ptr);
void* malloc(size_t size);
void* realloc(void* ptr, size_t size);
void abort(void);
int atexit(void (*func)(void));
void exit(int status);
void _Exit(int status);
char* getenv(const char* name);
int system(const char* string);
void* bsearch(const void* key, const void* base, size_t nmemb, size_t size,
              int (*compar)(const void *, const void *));
void qsort(void* base, size_t nmemb, size_t size,
           int (*compar)(const void *, const void *));
int         abs(      int j);
long        abs(     long j);
long long   abs(long long j);                                             // C++0X
long       labs(     long j);
long long llabs(long long j);                                             // C99
div_t     div(      int numer,       int denom);
ldiv_t    div(     long numer,      long denom);
lldiv_t   div(long long numer, long long denom);                          // C++0X
ldiv_t   ldiv(     long numer,      long denom);
lldiv_t lldiv(long long numer, long long denom);                          // C99
int mblen(const char* s, size_t n);
int mbtowc(wchar_t* restrict pwc, const char* restrict s, size_t n);
int wctomb(char* s, wchar_t wchar);
size_t mbstowcs(wchar_t* restrict pwcs, const char* restrict s, size_t n);
size_t wcstombs(char* restrict s, const wchar_t* restrict pwcs, size_t n);
int at_quick_exit(void (*func)(void))                                     // C++11
void quick_exit(int status);                                              // C++11
void *aligned_alloc(size_t alignment, size_t size);                       // C11

*/



/*****************************************************************************/
/* stdlib.h                                                                  */
/*                                                                           */
/* Copyright (c) 1993 Texas Instruments Incorporated                         */
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





/*---------------------------------------------------------------------------*/
/* Support for alloca is available in glibc stdlib.h by default.             */
/* We'll do the same for clang-based compilers since LLVM supports a         */
/* built-in for the alloca implementation.                                   */
/*---------------------------------------------------------------------------*/

_Pragma("diag_push")
_Pragma("CHECK_MISRA(\"-6.3\")") /* standard types required for standard headers */
_Pragma("CHECK_MISRA(\"-8.5\")") /* need to define inline function */
_Pragma("CHECK_MISRA(\"-19.1\")") /* #includes required for implementation */
_Pragma("CHECK_MISRA(\"-19.7\")") /* need function-like macros */
_Pragma("CHECK_MISRA(\"-20.1\")") /* standard headers must define standard names */
_Pragma("CHECK_MISRA(\"-20.2\")") /* standard headers must define standard names */

/*---------------------------------------------------------------------------*/
/* Attributes are only available in relaxed ANSI mode.                       */
/*---------------------------------------------------------------------------*/

extern "C" {

_Pragma("diag_push")
_Pragma("CHECK_MISRA(\"-5.7\")") /* keep names intact */

typedef struct { int quot, rem; } div_t;

typedef struct { long quot, rem; } ldiv_t;

typedef struct { long long quot, rem; } lldiv_t;

_Pragma("diag_pop")








/*---------------------------------------------------------------*/
/* NOTE - Normally, abs, labs, and fabs are expanded inline, so  */
/*        no formal definition is really required. However, ANSI */
/*        requires that they exist as separate functions, so     */
/*        they are supplied in the library.  The prototype is    */
/*        here mainly for documentation.                         */
/*---------------------------------------------------------------*/
_Pragma("diag_push")
_Pragma("CHECK_MISRA(\"-16.4\")") /* false positives due to builtin declarations */
      int       abs(int _val); 
      long      labs(long _val);
      long long llabs(long long _val);
_Pragma("diag_pop")

     int       atoi(const char *_st);
     long      atol(const char *_st);
     long long atoll(const char *_st);
     char     *ltoa(long val, char * buffer, int radix);
          extern  double    atof(const char *_st);

     long      strtol(const char * __restrict _st,
                                  char ** __restrict _endptr, int _base);
     unsigned long strtoul(const char * __restrict _st,
                                       char ** __restrict _endptr, int _base);
     long long strtoll(const char * __restrict _st,
                                   char ** __restrict _endptr, int _base);
     unsigned long long strtoull(const char * __restrict _st,
                                             char ** __restrict _endptr,
					     int _base);
     float     strtof(const char * __restrict _st,
                                  char ** __restrict _endptr);
     double    strtod(const char * __restrict _st,
                                  char ** __restrict _endptr);
     long double strtold(const char * __restrict _st,
                                     char ** __restrict _endptr);
    
     int    rand(void);
     void   srand(unsigned _seed);
    
     void  *calloc(size_t _num, size_t _size)
               __attribute__((malloc));
     void  *malloc(size_t _size)
               __attribute__((malloc));
     void  *realloc(void *_ptr, size_t _size);
     void   free(void *_ptr);
     void  *memalign(size_t _aln, size_t _size)
               __attribute__((malloc));
     void  *aligned_alloc(size_t _aln, size_t _size)
               __attribute__((malloc));

     void   __TI_heap_stats(void);
     void  *__TI_heap_check(void);
     size_t __TI_heap_total_available(void);
     size_t __TI_heap_largest_available(void);
    
    [[noreturn]]  void abort(void) noexcept;

    typedef void (*__TI_atexit_fn)(void);
     int    atexit(__TI_atexit_fn _func) noexcept;

    typedef int (*__TI_compar_fn)(const void *_a,const void *_b);
     void  *bsearch(const void *_key, const void *_base,
                                size_t _nmemb, size_t _size, 
                                __TI_compar_fn compar);
     void   qsort(void *_base, size_t _nmemb, size_t _size, 
                              __TI_compar_fn compar);

    [[noreturn]]  void exit(int _status);
    [[noreturn]]  void _Exit(int _status);

    [[noreturn]]  void quick_exit(int _status);
     int at_quick_exit(__TI_atexit_fn _func) noexcept;
    
     div_t  div(int _numer, int _denom);
     ldiv_t ldiv(long _numer, long _denom);
     lldiv_t lldiv(long long _numer, long long _denom);

     char  *getenv(const char *_string);
     int    system(const char *_name);

     int    mblen(const char *_s, size_t _n);
     size_t mbstowcs(wchar_t * __restrict _dest,
                                 const char * __restrict _src, size_t _n);
     int    mbtowc(wchar_t * __restrict _dest,
                               const char * __restrict _src, size_t _n);

     size_t wcstombs(char * __restrict _dest,
                                 const wchar_t * __restrict _src, size_t _n);
     int    wctomb(char *_s, wchar_t _wc);

} /* extern "C" */



/*****************************************************************************/
/* If we leave these active when in relaxed ANSI mode, we get infinite       */
/* recursion due to changes in type matching.  See comment in                */
/* ansi/sys_predef.c line 4377 on why we specifically check the              */
/* __TI_PROPRIETARY_STRICT_ANSI_MACRO macro here and its relation to strict  */
/* ANSI and relaxed ANSI parser modes.                                       */
/*****************************************************************************/

/* C2000-specific additions to header implemented with #include */


_Pragma("diag_pop")



_Pragma("diag_push")
_Pragma("CHECK_MISRA(\"-19.15\")") /* FreeBSD library requires code outside of the
                                 include guard */
_Pragma("CHECK_MISRA(\"-19.1\")")

/*----------------------------------------------------------------------------*/
/* If sys/cdefs.h is available, go ahead and include it. xlocale.h assumes    */
/* this file will have already included sys/cdefs.h.                          */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Include xlocale/_stdlib.h if xlocale.h has already been included. This     */
/* comes from FreeBSD's stdlib.h.                                             */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* The _TI_PROPRIETARY_PRAGMA macro exoands to a C99 _Pragma operator. */
/* The _Pragma statement is handled after the Pragma itself causing unexpected */
/* warnings due to the diagnostic state being popped. This is done to suppress */
/* unexpected 19.15 misra warnings.                                   */
/*----------------------------------------------------------------------------*/
#pragma diag_pop


extern "C++" {


/* MSVCRT already has the correct prototype in <stdlib.h> if __cplusplus is defined */
inline __attribute__ ((__always_inline__)) long      abs(     long __x) noexcept {return  labs(__x);}
inline __attribute__ ((__always_inline__)) long long abs(long long __x) noexcept {return llabs(__x);}

inline __attribute__ ((__always_inline__))  ldiv_t div(     long __x,      long __y) noexcept {return  ldiv(__x, __y);}
inline __attribute__ ((__always_inline__)) lldiv_t div(long long __x, long long __y) noexcept {return lldiv(__x, __y);}

}  /* extern "C++" */





namespace std { inline namespace __2 {

using ::size_t;
using ::div_t;
using ::ldiv_t;
using ::lldiv_t;
using ::atof;
using ::atoi;
using ::atol;
using ::atoll;
using ::strtod;
using ::strtof;
using ::strtold;
using ::strtol;
using ::strtoll;
using ::strtoul;
using ::strtoull;
using ::rand;
using ::srand;
using ::calloc;
using ::free;
using ::malloc;
using ::realloc;
using ::abort;
using ::atexit;
using ::exit;
using ::_Exit;
using ::getenv;
using ::system;
using ::bsearch;
using ::qsort;
using ::abs;
using ::labs;
using ::llabs;
using ::div;
using ::ldiv;
using ::lldiv;
using ::mblen;
using ::mbtowc;
using ::wctomb;
using ::mbstowcs;
using ::wcstombs;

} }


namespace AdvancedMicrotech {

/******************************************************************************
 * VARIABLES
 *****************************************************************************/
}
