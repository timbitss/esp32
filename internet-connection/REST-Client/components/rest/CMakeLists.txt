# Register component into build system
idf_component_register(SRCS "rest.c"
                       INCLUDE_DIRS "include"
                       # list of components required by the public interface of this component
                       REQUIRES mbedtls 
                       # list of components required by source file 
                       PRIV_REQUIRES esp_http_client) 