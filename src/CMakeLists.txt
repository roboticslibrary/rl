if(RL_BUILD_MATH)
	add_subdirectory(rl/math)
	list(APPEND TARGETS math)
endif()

if(RL_BUILD_UTIL)
	add_subdirectory(rl/util)
	list(APPEND TARGETS util)
endif()

if(RL_BUILD_XML)
	add_subdirectory(rl/xml)
	list(APPEND TARGETS xml)
endif()

if(RL_BUILD_KIN)
	add_subdirectory(rl/kin)
	list(APPEND TARGETS kin)
endif()

if(RL_BUILD_MDL)
	add_subdirectory(rl/mdl)
	list(APPEND TARGETS mdl)
endif()

if(RL_BUILD_HAL)
	add_subdirectory(rl/hal)
	list(APPEND TARGETS hal)
endif()

if(RL_BUILD_SG)
	add_subdirectory(rl/sg)
	list(APPEND TARGETS sg)
endif()

if(RL_BUILD_PLAN)
	add_subdirectory(rl/plan)
	list(APPEND TARGETS plan)
endif()

set(TARGETS ${TARGETS} PARENT_SCOPE)
