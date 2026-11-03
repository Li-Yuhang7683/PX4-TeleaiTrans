############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
############################################################################

# The normal module traversal can visit comp_attitude before ekf2. The
# EKF2 CMakeLists adds this directory again after ecl_EKF exists.
if(NOT TARGET ecl_EKF)
	return()
endif()

px4_add_module(
	MODULE modules__comp_attitude
	MAIN comp_attitude
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
		-fno-associative-math
	INCLUDES
		../ekf2
		../ekf2/EKF
		../ekf2/EKF/aid_sources
	STACK_MAIN
		4096
	STACK_MAX
		4096
	SRCS
		CompAttitude.cpp
		CompAttitude.hpp
		CompositeMath.hpp
	DEPENDS
		ecl_EKF
		px4_work_queue
	MODULE_CONFIG
		module.yaml
	)
