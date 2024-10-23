set(component_srcs "src/bmi160_esp32.c"  "include/bmi160.c")


idf_component_register(SRCS "${component_srcs}"
                       REQUIRES driver unity
                       INCLUDE_DIRS "include" "src")
