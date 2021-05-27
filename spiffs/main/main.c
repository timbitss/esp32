/**
 * @file main.c
 * @author Timothy Nguyen 
 * @brief Create and read files within SPI Flash File System (SPIFFS)
 * @version 0.1
 * @date 2021-05-27
 * 
 * @note IDF must be configured to use custom partition table (custom-partitions.csv). 
 * @note spiffs_create_partition_image must be called from CMakeLists.txt in main folder
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_spiffs.h"
#include <stdlib.h>
#include <dirent.h>
#include <sys/unistd.h>
#include <sys/stat.h>

#define TAG "SPIFFS" // for ESP logging

#define BUFF_SIZE 300 // size of buffer to hold lines of characters including terminating null-character

/**
 * @brief Print contents of text file to monitor
 * 
 * @param full_path Full path of text file with SPIFFS prefix
 */
static void read_txt(const char* full_path)
{
    FILE* fp = fopen(full_path, "r");
        if(fp == NULL)
        {
            ESP_LOGE(TAG, "Could not open file in %s for reading", full_path);
        }
        else
        {
            char buf[BUFF_SIZE];
            while(fgets(buf, BUFF_SIZE, fp) != NULL) 
                printf("%s", buf);
            fclose(fp);
        }

    printf("\n"); 
}

/**
 * @brief Reads all files from directory stream and prints contents and metadata
 * 
 * @param file_prefix  file path prefix associated with file system 
 * 
 * @note When a C library function needs to open a file, 
 *       the virtual filesystem (VFS) ESP component searches for the 
 *       FS driver associated with that file path and forwards the call to that driver.
 *       All subsequent calls to C library functions for the returned FILE* stream will be forwarded to the FS driver.
 *       See https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/storage/vfs.html. 
 */
static void read_files(const char* directory)
{
    /* Open directory stream, read entries, print metadata and file contents */
    DIR* dirp = opendir(directory); 
    struct dirent* entryp;
    while((entryp = readdir(dirp)) != NULL) 
    {
        char full_path[BUFF_SIZE];
        sprintf(full_path, "%s/%s", directory, entryp->d_name);
        struct stat statsp;
        int err = stat(full_path, &statsp);
        if(err != 0)
            ESP_LOGE(TAG, "Could not obtain information from %s", full_path);
        else
            ESP_LOGI(TAG, "Full path of file: %s, File size in bytes: (%d)", full_path, (int)statsp.st_size);
        read_txt(full_path);
    }

    closedir(dirp);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing SPIFFS...");
    const esp_vfs_spiffs_conf_t spiffs_conf = 
    {
        .base_path = "/spiffs", // SPIFFS driver requires a path prefix 
        .partition_label = NULL, // finds first SPIFFS partition label
        .max_files = 5, // max files open at the same time
        .format_if_mount_failed = true
    }; 

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&spiffs_conf));
    ESP_LOGI(TAG, "SPIFFS mounted onto storage partition with path prefix /spiffs");

    /* Get some info */
    size_t total_bytes = 0, used_bytes = 0;
    ESP_ERROR_CHECK(esp_spiffs_info(NULL,  &total_bytes, &used_bytes));
    ESP_LOGI(TAG, "Size of file system: (%d)  Used bytes: (%d)", total_bytes, used_bytes);

    /* Create/overwrite file, then print text to file */
    FILE* fp = fopen("/spiffs/hi.txt", "w");
    if(fp == NULL)
        ESP_LOGE(TAG, "Could not open file for writing");
    else
    {
        fprintf(fp, "Writing text to hi.txt from ESP32.");
        fclose(fp);
    }

    ESP_LOGI(TAG, "Reading all text files within SPIFFS.");
    read_files(spiffs_conf.base_path); // read all files within SPIFFS "directory"

    // All done, unmount partition and disable SPIFFS
    esp_vfs_spiffs_unregister(NULL);
    ESP_LOGI(TAG, "SPIFFS unmounted");
}
