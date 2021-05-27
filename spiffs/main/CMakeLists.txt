set(COMPONENT_SRCS "main.c" )
set(COMPONENT_ADD_INCLUDEDIRS ".")

register_component()

# Build a SPIFFS image from the contents of the 'txtfiles' directory
# that fits the partition named 'storage'. FLASH_IN_PROJECT indicates that
# the generated image should be flashed when the entire project is flashed to
# the target with 'idf.py -p PORT flash'.
spiffs_create_partition_image(storage ../txtfiles FLASH_IN_PROJECT)
