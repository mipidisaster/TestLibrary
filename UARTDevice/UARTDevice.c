/**************************************************************************************************
 * @file        UART.c
 * @author      Thomas
 * @version     V0.1
 * @date        23 Jun 2018
 * @brief       Source file for the Generic UART structure handler
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <UARTDevice/UARTDevice.h>

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
_UARTDev UARTDevSetup(UART_HandleTypeDef *UART_Handle, uint32_t Buffersize) {
/**************************************************************************************************
 * Create a UART structure specific for the STM32F device
 *  Generate Receive/Transmit buffers are per the input defined size.
 *
 * As the STM32CubeMX already pre-generates the setting up and configuring of the desired UART
 * device, there is no need to define that within this function. Simply providing the handle is
 * required.
 *************************************************************************************************/
    _UARTDev handle;
    handle.UART_Handle  = UART_Handle;      // Copy data into structure
    handle.Flt          = UART_Initialised; // Initialise the fault to "initialised"

    handle.Receive      = (_ByteQueue *)malloc(1);      // Allocate memory for Receive Buffer
    *(handle.Receive)     = ByteQuSetup(Buffersize);    // Setup to desired size

    handle.Transmit     = (_ByteQueue *)malloc(1);      // Allocate memory for Transmit Buffer
    *(handle.Transmit)    = ByteQuSetup(Buffersize);    // Setup to desired size

    return (handle);     // Return generated UART
}

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
_UARTDev UARTDevice(const char *deviceloc, int baud, uint32_t Buffersize) {
/**************************************************************************************************
 * Create a UART structure specific for the Raspberry Pi
 *  The provided information is the folder location of the serial interface in text, along with the
 *  desired baudrate of the serial interface.
 *
 *  This will then open up the serial interface, and configure a "pseudo_interrutp" register, so
 *  as to provide the Raspberry Pi the same function use as other embedded devices.
 *  The Receive and Transmit buffers size will be as per input "BufferSize"
 *************************************************************************************************/
    _UARTDev handle;
    handle.deviceloc        = deviceloc;    // Capture the folder location of UART device
    handle.baudrate         = baud;         // Capture the desired baud rate
    handle.pseudo_interrupt = 0x00;         // pseudo interrupt register used to control the UART
                                            // interrupt for Raspberry Pi

    handle.Flt          = UART_Initialised; // Initialise the fault to "initialised"

    handle.Receive      = (_ByteQueue *)malloc(1);      // Allocate memory for Receive Buffer
    *(handle.Receive)     = ByteQuSetup(Buffersize);    // Setup to desired size

    handle.Transmit     = (_ByteQueue *)malloc(1);      // Allocate memory for Transmit Buffer
    *(handle->Transmit)    = ByteQuSetup(Buffersize);   // Setup to desired size

    handle.UART_Handle = serialOpen(handle.deviceloc, handle.baudrate);
            // Open the serial interface
}

#else
//==================================================================================================

#endif

uint8_t UARTDevPoleSigRead(_UARTDev *handle) {
/**************************************************************************************************
 * Will take a single byte of data from the UART peripheral, and return out of function.
 * Note that this will WAIT until there is data available to be read.
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    // Ensure that the UART interface has been enabled
    if ((handle->UART_Handle->Instance->CR1 & USART_CR1_UE) != USART_CR1_UE)
        __HAL_UART_ENABLE(handle->UART_Handle);     // Enable the UART interface if not enabled

        // Check to see if there is data to be read, done by checking the Read Data Register not
        // empty flag (RXNE), if this is TRUE then there is data to be read.
    while (__HAL_UART_GET_FLAG(handle->UART_Handle, UART_FLAG_RXNE) != SET) {}

    return ((uint8_t)handle->UART_Handle->Instance->DR);// Retrieve the read data, and pass out
                                                        // of function

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    int readbackdata = -1;      // Create a variable which will contain the read contents of the
                                // UART device
                                // Set this to -1, as this indicates that there is no data to be
                                // read, and need to loop until data is read
    while(readbackdata == -1) { // Loop until get data from UART
        readbackdata = serialGetchar(handle->UART_Handle);    // Read data from UART
            // Function will time out after 10s returning -1. As want to pole until new data is
            // available, keep looping until get anything but -1
    }

    return ((uint8_t) readbackdata);    // If get to this point data has been read from UART,
                                        // therefore return read value
#else
//==================================================================================================
    return(0);
#endif
}

void UARTDevPoleSigTransmt(_UARTDev *handle, uint8_t data) {
/**************************************************************************************************
 * Will take the provided data, and put onto the UART peripheral for transmission.
 * Note that this will WAIT until it is able to transmit the data.
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    // Ensure that the UART interface has been enabled
    if ((handle->UART_Handle->Instance->CR1 & USART_CR1_UE) != USART_CR1_UE)
        __HAL_UART_ENABLE(handle->UART_Handle);   // Enable the UART interface if not enabled

        // Check to see if the Transmit Data Register is empty (TXE), this will be set to TRUE
        // by the hardware to indicate that the contents of the TDR register have been setup for
        // transmission. Therefore new data can be added for transmission
    while (__HAL_UART_GET_FLAG(handle->UART_Handle, UART_FLAG_TXE) == RESET) {}

    handle->UART_Handle->Instance->DR = data;   // Now put the selected data onto the Data Register
                                                // (DR) for transmission.

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    serialPutchar(handle->UART_Handle, (unsigned char) data); // Send data via UART

#else
//==================================================================================================

#endif
}

_UARTDevFlt UARTDevPoleTransmt(_UARTDev *handle, uint8_t *pData, uint16_t size) {
/**************************************************************************************************
 * An extension of the Single Transmit function, this allows for an array of data to be set via
 * UART.
 * Again it will wait until it can transmit, and all data has been transmitted before exiting.
 * > However if there is no data, or the size is zero, it will return a fault.
 *************************************************************************************************/

    if (pData == 0 || size == 0)           // If no data has been requested to be set
        return (handle->Flt = UART_DataError);  // return error with DATA (also update fault state)

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    while (size > 0) {                      // Whilst there is data to be transferred
        UARTDevPoleSigTransmt(handle, *pData);  // Transmit the single point of data
        pData += sizeof(uint8_t);           // Increment pointer by the size of the data to be
                                            // transmitted
        size--;                             // Decrement the size
    }

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    char *newpa;                        // Create a character pointer
    uint32_t i;                         // Loop variable to go through contents of array

    newpa = (char *)malloc(size);       // Allocate memory for new temp array

    for (i = 0; i != size; i++) {       // Cycle through the size of the input array
        newpa[i] = *pData;              // Copy data into new array
        pData += sizeof(uint8_t);       // Increment the input array pointer
    }

    serialPuts(this->UART_Handle, newpa);   // Then send new data via UART
    free(newpa);                            // Free up temporary array location

#else
//==================================================================================================

#endif

    return (handle->Flt = UART_NoFault);// If have made it to this point, then there is no issues
                                        // also update fault state
}

void UARTDevSigTransmt_IT(_UARTDev *handle, uint8_t data) {
/**************************************************************************************************
 * INTERRUPTS:
 * The will add the input data onto the Transmit buffer, for it to be handled by the UART
 * interrupt. - The UART structure controller version of the interrupt handler is ".IRQHandle"
 *************************************************************************************************/
    ByteQuInWrite(handle->Transmit, data);      // Add data to the Transmit buffer
    UARTDevTransmtIT(handle, UART_EnableIT);    // Enable the transmit interrupt
}

_BytQueState UARTDevSigRead_IT(_UARTDev *handle, uint8_t *pData) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will read the latest read packet via UART, which will be stored within the "Received"
 * buffer. If there is no new data, it will return the "GenBuffer_Empty" value.
 * The UART structure controller version of the interrupt handler is ".IRQHandle"
 *************************************************************************************************/

    return(ByteQuOutRead(handle->Receive, pData));
}

void UARTDevReceiveIT(_UARTDev *handle, _UARTDevIT intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Receive interrupt event.
 *************************************************************************************************/
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    if (intr == UART_EnableIT) {
        __HAL_UART_ENABLE_IT(handle->UART_Handle, UART_IT_RXNE);    // Enable the RXNE interrupt
    }
    else {
        __HAL_UART_DISABLE_IT(handle->UART_Handle, UART_IT_RXNE);   // Disable the RXNE interrupt
    }

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    if (intr == UART_EnableIT) {
        UARTD_EnabInter(handle->pseudo_interrupt, UARTD_ReceiveIntBit);
        // Enable the pseudo Receive bit - via the "Pseudo interrupt" register
    }
    else {
        UARTD_DisaInter(handle->pseudo_interrupt, UARTD_ReceiveIntBit);
        // Disable the pseudo Receive bit - via the "Pseudo interrupt" register
    }
#else
//==================================================================================================

#endif
}

void UARTDevTransmtIT(_UARTDev *handle, _UARTDevIT intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Transmit interrupt event.
 *************************************************************************************************/
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    if (intr == UART_EnableIT) {
        __HAL_UART_ENABLE_IT(handle->UART_Handle, UART_IT_TXE);     // Enable the TXE interrupt
    }
    else {
        __HAL_UART_DISABLE_IT(handle->UART_Handle, UART_IT_TXE);    // Disable the TXE interrupt
    }

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    if (intr == UART_EnableIT) {
        UARTD_EnabInter(handle->pseudo_interrupt, UARTD_TransmtIntBit);
        // Enable the pseudo Transmit bit - via the "Pseudo interrupt" register
    }
    else {
        UARTD_DisaInter(handle->pseudo_interrupt, UARTD_TransmtIntBit);
        // Disable the pseudo Transmit bit - via the "Pseudo interrupt" register
    }
#else
//==================================================================================================

#endif
}

void UARTDevIRQHandle(_UARTDev *handle) {
/**************************************************************************************************
 * INTERRUPTS:
 * Interrupt Service Routine for the UART structure handler. Each of the supported devices needs to
 * call this function in different ways - therefore each implementation is mentioned within the
 * coded section.
 *
 * Function will then read the hardware status flags, and determine which interrupt has been
 * triggered:
 *      If receive interrupt enabled, then data from Data Register is stored onto the "Receive"
 *      buffer
 *
 *      If transmission interrupt enabled, then data from "Transmit" buffer is put onto the Data
 *      Register for transmission.
 *
 *      No other interrupts are currently supported.
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
/**************************************************************************************************
 * Example of call.
 *  As the main.h/main.c are included within the interrupt header and source file, the function
 *  call needs to be setup there.
 *  A global pointer to the UART structure needs to be setup, then within the main() routine, it
 *  needs to be configured to the desired settings (STM32CubeMX - handle linked to structure).
 *
 *  Then external function needs to call "UARTDevice_IRQHandler", and then be called within the
 *  interrupt function call:
 *
 *  main.c
 *      * Global handler
 *      _UARTDev UARTDevice;
 *
 *      main() {
 *      UARTDevice = UARTDevSetup(&huart1, 124);
 *      while() {};
 *      }
 *
 *      void UARTDevice_IRQHandler(void) {  UARTDevIRQHandle(&UARTDevice);  }
 *
 *  main.h
 *      void UARTDevice_IRQHandler(void);
 *************************************************************************************************/
    uint32_t isrflags   = READ_REG(handle->UART_Handle->Instance->SR);
        // Get the Interrupt flags

    uint32_t cr1its     = READ_REG(handle->UART_Handle->Instance->CR1);
        // Add status flags for Interrupts

    uint8_t tempdata = 0x00;    // Temporary variable to store data for UART

    // If the Receive Data Interrupt has been triggered AND is enabled as Interrupt then ...
    if(((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET)) {
        tempdata  = handle->UART_Handle->Instance->DR;      // Read data and put onto temp variable
        ByteQuInWrite(handle->Receive, (uint8_t) tempdata); // Add to Receive buffer
    }

    // If the Data Empty Interrupt has been triggered AND is enabled as Interrutp then...
    if(((isrflags & USART_SR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET)) {
        // If there is data to be transmitted, then take from buffer and transmit
        if (ByteQuOutRead(handle->Transmit, &tempdata) != ByteQueue_Empty) {
            handle->UART_Handle->Instance->DR = (uint8_t)tempdata;
        }
        else
            UARTDevTransmtIT(handle, UART_DisableIT);
    }

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
/**************************************************************************************************
 * Example of call.
 *  As setting up a real interrupt for Raspberry Pi involves hacking the kernel, which I am not
 *  doing, the route I have taken is to use threads - pthreads.
 *  What this involves is creating a separate stream (thread) which will just check the size of the
 *  UART peripheral buffer or if data has been requested to be sent (via pseudo interrupt
 *  register). Then the main thread can use the Read/Transmit functions as normal.
 *
 *  So setting up the pthread:
 *  The -pthread needs to be added to the G++ linker library, won't require the "-" within eclipse,
 *  however for "CMakeList", will need to be written as "-lpthread".
 *  Then within the main code add "include <pthread.h>
 *
 *  This will then allow you to create a new stream:
 *  main.c {
 *  pthread_t thread;   // Create thread handle
 *  if (pthread_create(&thread, NULL, &threadfunction, NULL) != 0) {
 *      std::cout << "Failed to create thread" << std::endl;
 *  }
 *
 * pthread_create will return 0 if it has created a new thread. In the example above "&thread" is
 * the thread handle created above, next entry ...no idea... 3rd entry is a pointer to the desired
 * function call, 4th can be used to provide values to the function - however I haven't tried this.
 *
 * void *threadfunction(void *value) {
 *  uint8_t UARTReturn;
 *  while (1) {
 *      delay(100);
 *      UARTDevIRQHandle(&UARTDevice);
 *  }
 *
 *  return 0;
 * }
 *
 * Similar to the STM32 a pointer to the UARTDevice will need to be made global to allow this
 * new thread to all the "UARTDevice_IRQHandler"
 *************************************************************************************************/
    int BufferContents = 0;             // Variable to store the amount of data in UART peripheral

    // Check to see if Receive Interrupt bit has been set.
    if (((handle->pseudo_interrupt & (0x01<<UARTD_ReceiveIntBit)) == (0x01<<UARTD_ReceiveIntBit)))
    {
        // If it has check to see if there is any data to be read
        BufferContents = serialDataAvail(handle->UART_Handle);  // Get the amount of data in UART

        while (BufferContents > 0) {
            ByteQuInWrite(handle->Receive, (uint8_t) serialGetchar(handle->UART_Handle))
            BufferContents--;
        }
    }

    uint8_t tempdata;

    if (((handle->pseudo_interrupt & (0x01<<UARTD_TransmtIntBit)) == (0x01<<UARTD_TransmtIntBit)))
    {

        while (ByteQuOutRead(handle->Transmit, &tempdata) != ByteQueue_Empty) {
            UARTDevPoleSigTransmt(handle, tempdata);
        }

        UARTDevTransmtIT(handle, UART_DisableIT);
    }
#else
//==================================================================================================

#endif
}
