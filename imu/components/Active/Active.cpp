/**
 * @file ActiveObject.cpp
 * @author Timothy Nguyen
 * @brief Active Object Framework using FreeRTOS
 * @version 0.1
 * @date 2021-08-08
 * 
 * Adapted from Herb Sutter's article: "Prefer Using Active Objects Instead of Naked Threads".
 */

#include "Active.hpp"

/**
 * @brief Process events one by one.
 */
void Active::EventLoopTask(void *args)
{
    Event evt;
    while( xQueueReceive(queue_handle, &evt, portMAX_DELAY) )
    {
        if(evt == *done)
        {
            
        }
        else
        {
            evt.Execute();
        }
    }
}
