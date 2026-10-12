# ESP-IDF component manifest for M5Unit-THERMO.
# Upstream: https://github.com/m5stack/M5Unit-THERMO
#
# The public headers pull in both M5UnitUnified and M5Utility,
# because the periodic measurement data is held in m5::container::CircularBuffer.
# driver is only used by src/unit/unit_MLX90614.cpp (driver/gpio.h).
file(GLOB_RECURSE SRCS "src/*.cpp")
idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "src"
    REQUIRES M5UnitUnified M5Utility
    PRIV_REQUIRES driver
)
