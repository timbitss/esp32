# Register component into build system
idf_component_register(SRCS "rest.c"
                       INCLUDE_DIRS "include"
                       REQUIRES mbedtls # list of components required by the public interface of this component
                       PRIV_REQUIRES esp_http_client json) # list of components required by source file 