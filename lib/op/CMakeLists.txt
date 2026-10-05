# SPDX-License-Identifier: MIT

target_sources(
    rf215_core

        PRIVATE
            op_configure.c
            op_iqif.c
            op_reset.c
            op_rx.c
            op_tx.c
)

target_include_directories(rf215_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
