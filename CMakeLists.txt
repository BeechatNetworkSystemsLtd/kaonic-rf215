# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)

# The version is kept in the header, which the Kbuild port sees as well.
file(
    STRINGS include/rf215/rf215_version.h RF215_VERSION_DEFINES
    REGEX "^#define RF215_VERSION_(MAJOR|MINOR|PATCH) [0-9]+$"
)
string(REGEX REPLACE ".*MAJOR ([0-9]+).*MINOR ([0-9]+).*PATCH ([0-9]+).*" "\\1.\\2.\\3"
                     RF215_VERSION "${RF215_VERSION_DEFINES}")

project(kaonic-rf215 VERSION ${RF215_VERSION} LANGUAGES C)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(RF215_IS_TOP_LEVEL ON)
else()
    set(RF215_IS_TOP_LEVEL OFF)
endif()

option(RF215_BUILD_TESTS "Build the host tests of the core" ${RF215_IS_TOP_LEVEL})
option(RF215_WARNINGS_AS_ERRORS "Treat compiler warnings in the core as errors" OFF)

if(RF215_IS_TOP_LEVEL)
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_C_STANDARD_REQUIRED ON)
    set(CMAKE_C_EXTENSIONS OFF)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

# The core library. The Linux kernel module in port/linux-kernel/ compiles the
# same sources through Kbuild and is not built from here.
add_subdirectory(lib)

if(RF215_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
