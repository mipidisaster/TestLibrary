/**************************************************************************************************
 * @file        GPIO.h
 * @author      Thomas
 * @version     V0.1
 * @date        22 June 2018
 * @brief       Header file for the Generic GPIO structure handler
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Structure handler used to control the GPIO of multiple embedded devices, the initialisation
 * call for each will be slightly different, however all other functions will be universal.
 * Use of structure handler:
 *      Generate structure through "GPIOSetup", which will require the pinnumber and direction
 *      (output/input).
 *          STM devices will require the PORT address as well
 *
 *      For all function calls, a pointer to the structure to be controlled is required.
 *      To change the state of the pin either the "GPIOtoggle" or "GPIOset(GPIO_HIGH/GPIO_LOW)" for
 *      outputs.
 *      To read the state of the pin use "GPIOget".
 *************************************************************************************************/

#ifndef GPIO_H_
#define GPIO_H_

#include <stdint.h>
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
#include "stm32f1xx_hal.h"              // Include the HAL library

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
#include <wiringPi.h>                   // Include the wiringPi library

#else
//==================================================================================================
#error "Unrecognised target device"

#endif

// Types used within this class
typedef enum GPIO_VALUE { GPIO_LOW = 0, GPIO_HIGH = ~GPIO_LOW } _GPIOValue;
typedef enum GPIO_DIREC { GPIO_OUT = 0, GPIO_IN = ~GPIO_OUT} _GPIODirec;

// My structure used to control the GPIO hardware

typedef struct GPIO {
    // Structure entries which are generic to all embedded devices
    uint32_t        pinnumber;
    _GPIODirec      pindirection;

    // Embedded device specific entries
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    GPIO_TypeDef    *PortAddress;           // Store the Port Address of pin

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    _GPIOValue      pinvalue;               // Current value of pin

    GPIO(_GPIOValue pinvalue, uint32_t pinnumber, _GPIODirec pindirection);

#else
//==================================================================================================

#endif
}   _GPIO;

// Embedded devices specific functions
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
_GPIO GPIOSetup(GPIO_TypeDef *PortAddress, uint32_t pinnumber, _GPIODirec direction);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
_GPIO GPIOSetup(_GPIOValue pinvalue, uint32_t pinnumber, _GPIODirec pindirection);

#else
//==================================================================================================

#endif

// None specific functions
uint8_t GPIOset(_GPIO *handle, _GPIOValue value);  // Drive the GPIO output
uint8_t GPIOget(_GPIO *handle);                    // Read the GPIO output

uint8_t GPIOtoggle(_GPIO *handle);                // Toggle the GPIO output

#endif /* GPIO_H_ */
