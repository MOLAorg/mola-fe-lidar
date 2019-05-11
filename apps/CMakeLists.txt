# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2019, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# find dependencies:
find_package(MRPT 1.9.9 REQUIRED COMPONENTS gui tclap)

# ----------------------
# define app targets:
mola_add_executable(
	TARGET  mola-app-fe-lidar-3d-segment-scan
	SOURCES mola-app-fe-lidar-3d-segment-scan.cpp
	LINK_LIBRARIES
		mrpt::tclap
		mrpt::gui
		yaml-cpp
		${PROJECT_NAME}
)

mola_add_executable(
	TARGET  mola-app-fe-lidar-3d-align
	SOURCES mola-app-fe-lidar-3d-align.cpp
	LINK_LIBRARIES
		mrpt::tclap
		mrpt::gui
		yaml-cpp
		${PROJECT_NAME}
)
