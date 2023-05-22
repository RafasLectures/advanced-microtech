/******************************************************************************
 * @file                    adac.hpp
 * @author                  Rafael Andrioli Bauer
 * @date                    14.05.2023
 * @matriculation number    5163344
 * @e-mail contact          abauer.rafael@gmail.com
 * @brief                   <brief description>
 *
 * Here goes a detailed description if required.
 ******************************************************************************/

#ifndef EXERCISE_LIBS_ADAC_H_
#define EXERCISE_LIBS_ADAC_H_

/******************************************************************************
 * INCLUDES
 *****************************************************************************/

#include "i2c.hpp"

/******************************************************************************
 * CONSTANTS
 *****************************************************************************/



/******************************************************************************
 * VARIABLES
 *****************************************************************************/



/******************************************************************************
 * FUNCTION PROTOTYPES
 *****************************************************************************/

// All functions return 0 if everything went fine
// and anything but 0 if not.

// Initialize the ADC / DAC
unsigned char adac_init(void);

// Read all ADC-values and write it into the passed values-array.
// (Important: always pass an array of size four (at least).) (1 pt.)
unsigned char adac_read(unsigned char * values);

// Write a certain value to the DAC. (1 pt.)
unsigned char adac_write(unsigned char value);

#endif /* EXERCISE_LIBS_ADAC_H_ */
