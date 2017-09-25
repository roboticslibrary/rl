if(BUILD_RL_MATH)
	add_subdirectory(rl/math)
	list(APPEND TARGETS math)
endif()

if(BUILD_RL_UTIL)
	add_subdirectory(rl/util)
	list(APPEND TARGETS util)
endif()

if(BUILD_RL_XML)
	add_subdirectory(rl/xml)
	list(APPEND TARGETS xml)
endif()

if(BUILD_RL_KIN)
	add_subdirectory(rl/kin)
	list(APPEND TARGETS kin)
endif()

if(BUILD_RL_MDL)
	add_subdirectory(rl/mdl)
	list(APPEND TARGETS mdl)
endif()

if(BUILD_RL_HAL)
	add_subdirectory(rl/hal)
	list(APPEND TARGETS hal)
endif()

if(BUILD_RL_SG)
	add_subdirectory(rl/sg)
	list(APPEND TARGETS sg)
endif()

if(BUILD_RL_PLAN)
	add_subdirectory(rl/plan)
	list(APPEND TARGETS plan)
endif()

set(TARGETS ${TARGETS} PARENT_SCOPE)
