# **ESP32 Projects**

- [Blinky](blinky/main/main.c)
- [FreeRTOS](FreeRTOS/main/main.c)
  - [Tasks](FreeRTOS/tasks/main/main.c)
  - [Mutexes and Counting Semaphores](FreeRTOS/mutex-and-counting/main/main.c)
  - [Task Notifications](FreeRTOS/task-notifications/main/main.c)
  - [Binary Semaphores](FreeRTOS/binary-semaphore/main/main.c)
  - [Queues](FreeRTOS/queues/main/main.c)
  - [Software Timer](FreeRTOS/software-timer/main/main.c)
  - [Event Groups](FreeRTOS/event-groups/main/main.c)
- [ESP32 High Resolution Timer](highres-timer/main/main.c)
- [Internal Heap Memory Allocation](sram/main/main.c)
- [Non-Volatile Storage](nvs/main/main.c)
- [Non-Volatile Storage of Structures](nvs-blob/main/main.c)
  
## **Notes**
* [Espressif's ESP-IDF framework](https://github.com/espressif/esp-idf) must be installed on user's host machine to build and flash the projects. 
* A [custom flash partition table](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/partition-tables.html) can be created for data storage and OTA applications. 
