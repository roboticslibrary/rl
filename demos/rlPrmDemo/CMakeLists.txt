find_package(Boost REQUIRED)

if(RL_BUILD_SG_BULLET OR RL_BUILD_SG_ODE OR RL_BUILD_SG_SOLID)
	add_executable(
		rlPrmDemo
		rlPrmDemo.cpp
		${rl_BINARY_DIR}/robotics-library.rc
	)
	
	target_link_libraries(
		rlPrmDemo
		mdl
		plan
		sg
		Boost::headers
	)
endif()
