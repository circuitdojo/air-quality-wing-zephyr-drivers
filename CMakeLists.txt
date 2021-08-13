#
# Copyright (c) 2021 Circuit Dojo LLC
#

# Point to this project's root directory.
set(AQW_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "Air Quality Wing root directory")

# Subdirectories
add_subdirectory(drivers)
add_subdirectory_ifdef(CONFIG_AQW lib)

# Include headers
zephyr_include_directories(include)