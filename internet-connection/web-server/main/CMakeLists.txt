# Register component into build system
idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       # list of components required by the public interface of this component
                       REQUIRES mbedtls 
                       # list of components required by source file 
                       PRIV_REQUIRES nvs_flash json esp_wifi connect server tmp102) 

# Create SPIFFS partition image in flash    
spiffs_create_partition_image(storage # name of partition
                              ../html # path to base directory
                              FLASH_IN_PROJECT)
