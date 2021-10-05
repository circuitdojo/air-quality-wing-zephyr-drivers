#
# Copyright (c) 2021 Circuit Dojo LLC
#

# Subdirectories
add_subdirectory_ifdef(CONFIG_SGP40_CD sgp40)
add_subdirectory_ifdef(CONFIG_SHTC3 shtc3)
add_subdirectory_ifdef(CONFIG_HPMA115S0 hpma115s0)

# Common directory
zephyr_include_directories(sensirion-common)