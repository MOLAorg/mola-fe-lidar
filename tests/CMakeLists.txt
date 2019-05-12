# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2019, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

mola_add_executable(
	TARGET  mola-test-p2p2-olae
	SOURCES mola-test-p2p2-olae.cpp
	LINK_LIBRARIES
		p2p2
)
add_test(P2P2_OLAE mola-test-p2p2-olae)
