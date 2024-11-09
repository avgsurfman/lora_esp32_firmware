set(component_srcs "src/bmi160_esp32.c")

set(component_includes "include/bmi160.c")


idf_component_register(SRCS "${component_srcs}" "${component_includes}"
	               PRIV_REQUIRES i2c
                       INCLUDE_DIRS "include" "src")
