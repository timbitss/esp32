# Register component into build system
idf_component_register(SRCS "MinIMU9.cpp"
                       INCLUDE_DIRS "include"
                       # list of components required by the public interface of this component
                       REQUIRES  RCFilter
                       # list of components required by source file 
                       PRIV_REQUIRES RCFilter)