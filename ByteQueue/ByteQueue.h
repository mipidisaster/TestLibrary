/**************************************************************************************************
 * @file        ByteQueue.h
 * @author      Thomas
 * @version     V0.1
 * @date        22 Jun 2018
 * @brief       Header file for the Byte Queue structure handler
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Structure handler for a circular buffer (queue) to store data sources to be read at a later
 * time.
 * Use of structure handler:
 *      Generate structure through "ByteQuSetup", which will require the intended size of the
 *      queue.
 *
 *      For all function calls, a pointer to the structure to be controlled is required. *
 *      Then data can be added to the buffer via "ByteQuInWrite".
 *      ***NOTE***
 *          If the input pointer of data catches up to the output pointer (therefore queue is
 *          full), the write will force the output pointer to increment by 1, to ensure that the
 *          data within the queue is limited to only be the defined length old.
 *
 *      Read data from buffer via "ByteQuOutRead(<pointer>)", the read will only update the pointed
 *      data if there is new data within the queue (i.e. not empty). If empty then it will not
 *      update pointer, and return the state as Empty
 *
 *      "ByteQuFlush" can be used to clear the entire contents of the queue, and return all
 *      pointers back to the start of the buffer.
 *
 *      "ByteQuState" can be used to determine what the current state of the queue is, returning a
 *      enumerate type of "_BytQueState" defined below.
 *************************************************************************************************/
#ifndef BYTEQUEUE_H_
#define BYTEQUEUE_H_

#include <stdint.h>         // Include standard integer entries
#include <malloc.h>

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
// None

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// None

#else
//==================================================================================================
// None

#endif

// Defines specific within this structure handler
// None

// Types used within this structure handler
typedef enum {
    ByteQueue_Empty = 0,        // Indicates that the buffer is empty of any new data
    ByteQueue_NewData = 1,      // Indicates that the buffer contains new data to be read
    ByteQueue_FULL = 2          // Indicates that the buffer is full, and no new data can be added
} _BytQueState;

typedef struct BytQue {
    // Structure entries which are generic to all embedded devices
    uint32_t        input_pointer;      // Pointer to where the current input point is
    uint32_t        output_pointer;     // Pointer to where the current output point is

    uint32_t        length;             // Size of the buffer

    uint8_t         *pa;                // Points to the array (Buffer)
}   _ByteQueue;

// None specific functions
_ByteQueue  ByteQuSetup(uint32_t size);         // Constructor of the Queue structure, where
                                                // the size of the buffer is defined

void ByteQuFlush(_ByteQueue *handle);           // Clear the data within the buffer
_BytQueState ByteQuState(_ByteQueue *handle);   // Function to determine state of the buffer:
                                                // Full/NewData/Empty

void ByteQuInWrite(_ByteQueue *handle, uint8_t newdata);    // Add data onto the queue (if at limit
                                                            // of buffer, will override oldest
                                                            // pointer (maintaining defined size)
_BytQueState ByteQuOutRead(_ByteQueue *handle, uint8_t *readdata);  // Read next data entry in
                                                                    // queue (if data present)

// Device specific entries
// None

#endif /* BYTEQUEUE_H_ */
