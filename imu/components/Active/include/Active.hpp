/**
 * @file ActiveObject.hpp
 * @author Timothy Nguyen
 * @brief Active Object Framework using FreeRTOS
 * @version 0.1
 * @date 2021-08-08
 * 
 * Adapted from Herb Sutter's article: "Prefer Using Active Objects Instead of Naked Threads".
 */

#pragma once

#include "FreeRTOS/task.h"
#include "FreeRTOS/queue.h"

/**
 * @brief Helper class for active objects.
 * 
 * @param stack_size Stack size of event loop thread in bytes.
 * @param task_priority Task priority.
 */
class Active
{
public:
    Active(uint32_t stack_size, UBaseType_t task_priority) : done(nullptr)
    {
        xTaskCreatePinnedToCore(EventLoopTask,
                                "Event Loop Task",
                                stack_size,
                                NULL,
                                task_priority,
                                &task_handle,
                                APP_CPU_NUM);
        configASSERT(task_handle);
    }

    ~Active()
    {
        Post(done);
    }
    void Post(const Event * const evt){ xQueueSend(queue_handle, evt, 0); } // Post event to active object.
        

    class Event // Base type for events. 
    {
    public:
        virtual ~Event();
        virtual void Execute();
    };

private:
    TaskHandle_t task_handle;   // Event loop task.
    QueueHandle_t queue_handle; // Event queue.
    Event *done;                // Event to close thread.

    void EventLoopTask(void *args);
};
