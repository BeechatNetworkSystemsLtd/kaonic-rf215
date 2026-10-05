# SPDX-License-Identifier: MIT

# One table of recommended frontend settings per modulation.
target_sources(
    rf215_core

        PRIVATE
            rf215_phy_fsk.c
            rf215_phy_ofdm.c
            rf215_phy_qpsk.c
)
