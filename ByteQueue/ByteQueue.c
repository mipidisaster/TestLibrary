/**************************************************************************************************
 * @file        ByteQueue.c
 * @author      Thomas
 * @version     V0.1
 * @date        22 Jun 2018
 * @brief       Source file for the Byte Queue structure handler
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <ByteQueue/ByteQueue.h>


void ByteQuFlush(_ByteQueue *handle) {
/**************************************************************************************************
 * Function goes through the contents of the queue, and writes everything to "0", and then
 * returns the input/output pointers back to the start of the queue - ready for new data
 *************************************************************************************************/
    uint32_t i;                     // Variable used to loop through the entries within the queue

    for (i = 0; i != handle->length; i++) {     // Start looping through queue
        handle->pa[i] = 0;                      // Write the data back to "0"
    }

    handle->output_pointer  = 0;            // Initialise pointers back to the start of the queue
    handle->input_pointer   = 0;            // Initialise pointers back to the start of the queue
}

_ByteQueue  ByteQuSetup(uint32_t size) {
/**************************************************************************************************
 * Structure constructor, where the desired "size" of the queue is provided.
 *      It will generate a new array structure of the desired size, and link to the private poiner
 *      "pa"
 *
 * Once done it will call the "Flush" function to write contents, and setup pointers to the start
 * of the queue.
 *************************************************************************************************/
    _ByteQueue handle;

    handle.length = size;                   // Setup size of the queue as per input
    handle.pa = (uint8_t *)malloc(size);    // Allocate memory for desired size of queue


    ByteQuFlush(&handle);                   // Flush the data to default values

    return(handle);     // Return generated Queue
}

_BytQueState ByteQuState(_ByteQueue *handle) {
/**************************************************************************************************
 * Function to determine the state of the Queue - Empty/NewData/Full
 * It does this in 3 steps:
 *      1st     If the input pointer is equal to the output pointer. Then the queue is empty
 *              and data can be added to the queue
 *
 *      2nd     If the output pointer is 1 in front of the input pointer, then the queue is full
 *              (This does mean that the amount of data that can actually be written into the
 *              queue is 1 minus the defined size)
 *
 *      3rd     If none of the above are true, then there is new data within the queue which can
 *              be read
 *
 * Diagram:
 *                                     V Output pointer
 *             -------------------------------------------------------------------------------
 * Queue  ->  |    0    |    1    |    2    |    3    |    4    |    5    |    6    |    7    |
 *             -------------------------------------------------------------------------------
 *                           ^3        ^1        ^2  <- Input pointer
 *  Size is 8 deep
 *
 *      When the input pointer is at position 1 (equal to the Output pointer), then the queue
 *      is EMPTY, and new data can be added
 *
 *      When the input pointer is at position 2 (just infront of Output pointer + anywhere else)
 *      then there is NEWDATA which needs to be read by the Output
 *
 *      When the input pointer is at position 3 (just behind the Output pointer), then the queue
 *      is FULL. As data within entry [2] has NOT been read yet.
 *************************************************************************************************/
    if      (handle->output_pointer == handle->input_pointer)   // If the pointers are equal
        return (ByteQueue_Empty);                               // then queue is empty

    else if (handle->output_pointer == ((handle->input_pointer + 1) % handle->length))
                                                                // If output pointer is one behind
        return (ByteQueue_FULL);                                // the input, then queue is full

    else                                                        // If none of the above are true
        return (ByteQueue_NewData);                             // then there is data in the queue
                                                                // which needs to be read
}

void ByteQuInWrite(_ByteQueue *handle, uint8_t newdata) {
/**************************************************************************************************
 * Function will add data onto the queue.
 * Then increase the input pointer, and limit it to the defined size of the queue.
 * After this has been increased, if the queue is considered "Empty" (input and output pointers
 * are equal) then need to increment the output pointer as well.
 * So as to maintain the oldest point of data within the queue is limited to the size of the
 * queue.
 *************************************************************************************************/
    handle->pa[handle->input_pointer] = newdata;        // Add the input data into the queue
    handle->input_pointer = (handle->input_pointer + 1) % handle->length;
    // Increment the input pointer, then take the Modulus of this. This will then cause the
    // input_pointer to be circled round if it equal to the length.

    // If after the input pointer has been increased it is equal to the output pointer - therefore
    // is considered "Empty".
    // Then to force queue size to be maintained as per "length", need to increase the output
    // pointer (essentially ensuring that the oldest point in the queue is limited to "length"
    // old)
    if (ByteQuState(handle) == ByteQueue_Empty)         // Check the queue state, if equal to
                                                        // EMPTY, then
        handle->output_pointer = (handle->output_pointer + 1) % handle->length;
        // Increase the output pointer by 1, limited to size "length"
}

_BytQueState ByteQuOutRead(_ByteQueue *handle, uint8_t *readdata)
/**************************************************************************************************
 * Function will take data from the queue.
 * It will only provide an updated output if the queue contains data (i.e. is not empty), if it
 * is empty then it will attempt nothing and return the empty indication
 *
 * If there is data, again it will be linked to the pointed data. Then it will increase the output
 * pointer, and limit it to the defined size of the queue.
 *************************************************************************************************/
{
    _BytQueState returnentry = ByteQuState(handle); // Generate variable to store the current state
                                                    // of queue, and update with latest state

    if (!(returnentry == ByteQueue_Empty)) {// If the queue is not empty (therefore there is data
                                            // be read
        *readdata = handle->pa[handle->output_pointer];     // Update the output with the latest
                                                            // entry from queue
        handle->output_pointer = (handle->output_pointer + 1) % handle->length;
        // Increase the output pointer by 1, limited to size "length"
        return(returnentry);                // Return state of queue prior to read
    }
    else                                    // If state is equal to "Empty"
        return (returnentry);               // Return state of queue (which will be "Empty")
}
