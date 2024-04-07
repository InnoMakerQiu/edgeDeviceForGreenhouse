# idf_component_register(SRCS "app_main.c" INCLUDE_DIRS ".")

file(GLOB SOURCES "./*.c")
idf_component_register(SRCS ${SOURCES} INCLUDE_DIRS ".")


