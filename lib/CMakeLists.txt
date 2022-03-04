#
# Copyright (c) 2021 Circuit Dojo LLC
#

add_subdirectory(codec)

zephyr_library()
zephyr_library_sources(aqw.c)
zephyr_library_sources(${AQW_DIR}/ext/embedded-sgp/sgp40_voc_index/sensirion_voc_algorithm.c)