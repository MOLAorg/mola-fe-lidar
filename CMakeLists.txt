# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2019, José Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Tell CMake we'll use C++ for use in its tests/flags
project(mola-fe-lidar-icp LANGUAGES CXX)

# -----------------------
# define lib:
file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

add_library(${PROJECT_NAME} SHARED
    ${LIB_SRCS}
    ${LIB_PUBLIC_HDRS}
    )

# Define common flags:
mola_set_target_build_options(${PROJECT_NAME})
mola_configure_library(${PROJECT_NAME})

# find dependencies:
find_package(MRPT 1.9.9 REQUIRED COMPONENTS slam graphs)

# Public lib Dependencies:
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    mola-kernel
    mrpt::graphs
    mrpt::slam
)

# ----------------------
# define app target:
add_executable(test-mola-fe-lidar-icp-segment-scan
    tests/test-mola-fe-lidar-icp-segment-scan.cpp)

# Define common flags:
mola_set_target_build_options(test-mola-fe-lidar-icp-segment-scan)
mola_configure_app(test-mola-fe-lidar-icp-segment-scan)

# find dependencies:
find_package(mrpt-tclap) # tclap wrapper, useful for Windows, etc.

# Dependencies:
target_link_libraries(test-mola-fe-lidar-icp-segment-scan
    mrpt::tclap
#    yaml-cpp
    ${PROJECT_NAME}
)

