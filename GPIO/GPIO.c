/**************************************************************************************************
 * @file        GPIO.c
 * @author      Thomas
 * @version     V0.1
 * @date        22 Jun 2018
 * @brief       Source file for the Generic GPIO structure handler
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include "GPIO/GPIO.h"

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
_GPIO GPIOSetup(GPIO_TypeDef *PortAddress, uint32_t pinnumber, _GPIODirec direction) {
/**************************************************************************************************
 * Create a GPIO structure specific for the STM32F device
 * Receives the PortAddress pointer, and pin number - all comes from the cubeMX output
 * Also requires the direction of the pin - INPUT/OUTPUT
 * Not setup of the port clock or pin, covered by the cubeMX outputs
 *************************************************************************************************/
    _GPIO handle;
    handle.pinnumber    = pinnumber;    // copy data into structure
    handle.PortAddress  = PortAddress;  //
    handle.pindirection = direction;    //

    return (handle);                    // Return generated handle
}
#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
_GPIO GPIOSetup(_GPIOValue pinvalue, uint32_t pinnumber, _GPIODirec pindirection) {
/**************************************************************************************************
 * Create a GPIO structure specific for the Raspberry Pi
 * Receives the initial pin state, pin number, and the direction - manually entered
 *************************************************************************************************/
    _GPIO handle;
    handle.pinnumber     = pinnumber;   // copy data into structure
    handle.pindirection  = pindirection;//
    handle.pinvalue      = pinvalue;    //

    if (handle.pindirection == GPIO_IN)         // If GPIO Mode is set for INPUT - GPIO_IN
        pinMode((int)handle.pinnumber, INPUT);  // Configure for INPUT
    else {                                      // If GPIO Mode is set for OUTPUT - GPIO_OUT
        pinMode((int)handle.pinnumber, OUTPUT); // Configure for OUTPUT
        GPIOset(&handle, handle.pinvalue);      // Set the pin for initial state
    }

    return (handle);                    // Return generated handle
}
#else
//==================================================================================================

#endif

uint8_t GPIOtoggle(_GPIO *handle) {
/**************************************************************************************************
 * Toggle the structure linked pin, ONLY if the structure has been declared as an OUTPUT.
 * Otherwise return error
 *************************************************************************************************/
    if (handle->pindirection != GPIO_OUT)   // Check direction of pin, if not equal to OUTPUT
        return -1;                          // return error '-1'

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    HAL_GPIO_TogglePin(handle->PortAddress, handle->pinnumber); // Use HAL function to toggle pin

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    if (handle->pinvalue == GPIO_LOW)   // If the Pin is set at LOW
        GPIOset(handle, GPIO_HIGH);     // Update to HIGH, function also updates the "pinvalue"
    else                                // If the Pin is set at HIGH
        GPIOset(handle, GPIO_LOW);      // Update to LOW, function also updates the "pinvalue"

#else
//==================================================================================================

#endif

    return 0;
}

uint8_t GPIOset(_GPIO *handle, _GPIOValue value) {
/**************************************************************************************************
 * Set the state of the output pin as per user demand "value", ONLY if the structure has been
 * declared as an OUTPUT
 * Otherwise return error
 *************************************************************************************************/
    if (handle->pindirection != GPIO_OUT)   // Check direction of pin, if not equal to OUTPUT
        return -1;                          // return error '-1'

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    if (value == GPIO_LOW)      // If demand is to set it LOW
        HAL_GPIO_WritePin(handle->PortAddress, handle->pinnumber, GPIO_PIN_RESET);
        // use HAL function to drive pin low
    else
        HAL_GPIO_WritePin(handle->PortAddress, handle->pinnumber, GPIO_PIN_SET);
        // use HAL function to drive pin High

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    if (value == GPIO_LOW)                          // If demand for Pin is set for LOW - GPIO_LOW
        digitalWrite((int)handle->pinnumber, LOW);  // Drive the pin LOW
    else
        digitalWrite((int)handle->pinnumber, HIGH); // Drive the pin HIGH

    handle->pinvalue      = value;                  // Update pin value

#else
//==================================================================================================

#endif

    return 0;
}

uint8_t GPIOget(_GPIO *handle) {
/**************************************************************************************************
 * Read the state of the input pin. Function will only work on INPUT pin, if pin is declared as an
 * OUTPUT it will return an error
 *************************************************************************************************/
    if (handle->pindirection != GPIO_IN)    // Check direction of pin, if not equal to INPUT
        return GPIO_LOW;                    // return LOW state

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    if (HAL_GPIO_ReadPin(handle->PortAddress, handle->pinnumber) == GPIO_PIN_RESET)
            // Check the state of the pin, and if it is RESET, then output LOW status
        return GPIO_LOW;
    else    // Otherwise output HIGH status
        return GPIO_HIGH;

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    return GPIO_LOW;
#else
//==================================================================================================

#endif
}
