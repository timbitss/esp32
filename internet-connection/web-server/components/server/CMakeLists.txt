# Register component into build system
idf_component_register(SRCS "server.c"
                       INCLUDE_DIRS "include"
                       # list of components required by the public interface of this component
                       REQUIRES mbedtls esp_http_server 
                       # list of components required by source file 
                       PRIV_REQUIRES esp_http_server server tmp102 json spiffs) 


                       