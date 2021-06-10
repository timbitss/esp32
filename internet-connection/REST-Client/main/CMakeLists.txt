# Register component into build system
idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       # list of components required by the public interface of this component
                       REQUIRES mbedtls 
                       # list of components required by source file 
                       PRIV_REQUIRES nvs_flash json esp_wifi rest connect) 