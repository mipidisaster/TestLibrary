/**************************************************************************************************
 * @file        UARTDevice.h
 * @author      Thomas
 * @version     V0.1
 * @date        23 Jun 2018
 * @brief       Header file for the Generic UART structure handler
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Structure handler used to control the UART of multiple embedded devices, the initialisation
 * call for each will be slightly different, however all other functions will be universal.
 * Use of structure handler:
 *      Generate structure through "UARTDevSetup", which will require the desired buffer size, as
 *      well as:
 *          For STM32F devices, providing the address of the UART handler - form cubeMX
 *          For RaspberryPi, provide the location of the serial interface, and the desired baudrate
 *
 *      For all function calls, a pointer to the structure to be controlled is required.
 *      Depending upon how the programmer wants to use the UART device, will change which functions
 *      are utilised.
 *      For functions to wait for new data, or data to be transmitted use:
 *          "UARTDevPoleSigRead"    - Wait for new data to be read via UART
 *          "UARTDevPoleSigTransmt" - Wait for data to be transmitted via UART
 *          "UARTDevPoleTransmt"    - As above, however can transmit multiple data (array)
 *
 *      If interrupts are to be used, then will first need to be configured within the NVIC (which
 *      is not part of this handler), then the following can be used
 *          "UARTDevReceiveIT"      - To configure the interrupt handling for Receiving data
 *          "UARTDevTransmtIT"      - To configure the interrupt handling for Transmitting data
 *          "ARTDevSigTransmt_IT"   - Put data onto "Transmit" buffer to be set via UART (will
 *                                    enable the transmit buffer empty flag)
 *          "UARTDevSigRead_IT"     - Pulls any new data from the "Receive" buffer
 *          "UARTDevIRQHandle"      - Interrupt handler, to be called in Interrupt Routine
 *                                      -> How to do this is detailed in the source file for this
 *                                         function
 *************************************************************************************************/
#ifndef UARTDevice_H_
#define UARTDevice_H_

#include <stdint.h>
#include <malloc.h>
#include "ByteQueue/ByteQueue.h"        // Provide the template for the circular queue structure

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
#include "stm32f1xx_hal.h"              // Include the HAL library

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
#include <wiringSerial.h>               // Include the wiringPi UART/Serial library

#else
//==================================================================================================
#error "Unrecognised target device"

#endif

// Defines specific within this structure handler
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
// None

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
#define UARTD_ReceiveIntBit     0       // Define the bit position for enabling Receive interrupt
#define UARTD_TransmtIntBit     1       // Define the bit position for enabling Transmit interrupt

#define UARTD_EnabInter(reg, bit)  ((reg) |=  (0x01 << bit))    // Enable specified bit
#define UARTD_DisaInter(reg, bit)  ((reg) &= ~(0x01 << bit))    // Disable specified bit

#else
//==================================================================================================
// None

#endif

// Types used within this structure handler
typedef enum {
    UART_NoFault = 0,
    UART_DataError = 1,

    UART_Initialised = -1
} _UARTDevFlt;

typedef enum { UART_EnableIT = 0, UART_DisableIT = !UART_EnableIT } _UARTDevIT;

typedef struct UARTDev {
    // Structure entries which are generic to all embedded devices
    _UARTDevFlt     Flt;                // Fault state of the structure
    _ByteQueue      *Receive;           // Pointer to ByteQueue struct used for data "Receive"d via
                                        // UART
    _ByteQueue      *Transmit;          // Pointer to ByteQueue struct used for data to be
                                        // "Transmit"ted via UART

    // Embedded device specific entries
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    UART_HandleTypeDef  *UART_Handle;       // Store the UART handle

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    int                 UART_Handle;        // Stores the device to communicate too
    const char          *deviceloc;         // Store location file for UART device
    int                 baudrate;           // Store entered baudrate
    uint8_t             pseudo_interrupt;   // Pseudo interrupt register

#else
//==================================================================================================

#endif

}   _UARTDev;

// Embedded devices specific functions
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
_UARTDev UARTDevSetup(UART_HandleTypeDef *UART_Handle, uint32_t Buffersize);
    // Setup the UART struct, for STM32Fxx by providing the UART type define handle, as well as
    // a defined value for the depth of the UART buffers

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
_UARTDev UARTDevice(const char *deviceloc, int baud, uint32_t Buffersize);
    // Setup the UART struct, similar to the first version, however with the "Buffersize"
    // defined for the UART buffers.

#else
//==================================================================================================

#endif

// None specific functions
// 1> Following functions will WAIT for actions to complete before finishing
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
uint8_t UARTDevPoleSigRead(_UARTDev *handle);   // Will wait until there is new data to be read
void UARTDevPoleSigTransmt(_UARTDev *handle, uint8_t data); // Will wait until it can transfer data
                                                            // via UART

_UARTDevFlt UARTDevPoleTransmt(_UARTDev *handle, uint8_t *pData, uint16_t size);
    // Will wait until it can transfer data via UART, multiple entries

// 2> Interrupt functions
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void UARTDevSigTransmt_IT(_UARTDev *handle, uint8_t data);          // Add data to be transmitted
                                                                    // via UART
_BytQueState UARTDevSigRead_IT(_UARTDev *handle, uint8_t *pData);   // Take data from received
                                                                    // buffer if data is available
void UARTDevReceiveIT(_UARTDev *handle, _UARTDevIT intr);   // Configure the Receive Interrupt
void UARTDevTransmtIT(_UARTDev *handle, _UARTDevIT intr);   // Configure the Transmit Interrupt
void UARTDevIRQHandle(_UARTDev *handle);                    // Interrupt handler

#endif /* UART_UART_H_ */
