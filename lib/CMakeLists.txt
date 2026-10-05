# SPDX-License-Identifier: MIT

# The core: no OS, no libc beyond <stdint.h>/<stddef.h>, no allocation, no
# float.
add_library(rf215_core)
add_library(kaonic::rf215 ALIAS rf215_core)

target_sources(
    rf215_core

        PRIVATE
            rf215_band.c
            rf215_dev.c
)

target_include_directories(
    rf215_core

        PUBLIC ${PROJECT_SOURCE_DIR}/include
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(rf215_core PUBLIC c_std_11)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(
        rf215_core

            PRIVATE
                -Wall -Wextra -Wpedantic -Wconversion -Wshadow
                -Wstrict-prototypes -Wmissing-prototypes
                $<$<BOOL:${RF215_WARNINGS_AS_ERRORS}>:-Werror>
    )
endif()

add_subdirectory(op)
add_subdirectory(phy)
