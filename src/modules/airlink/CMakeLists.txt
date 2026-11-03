############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE modules__airlink
	MAIN airlink
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		Airlink.cpp
		Airlink.hpp
		AirlinkImu.hpp
		AirlinkProtocol.hpp
		AirlinkSession.cpp
		AirlinkSession.hpp
	DEPENDS
		px4_work_queue
	)
