
add_library(drivers_rf215)

target_sources(
    drivers_rf215

        PRIVATE
            src/rf215.c
            src/rf215_baseband.c
            src/rf215_regs.c
)

target_include_directories(
    drivers_rf215

        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

