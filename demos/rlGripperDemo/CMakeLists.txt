if(MSVC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.00.23918)
	find_package(Boost REQUIRED COMPONENTS thread)
endif()

add_executable(
	rlGripperDemo
	rlGripperDemo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

if(MSVC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.00.23918)
	target_compile_definitions(
		rlGripperDemo
		PUBLIC
		BOOST_ALL_NO_LIB
		BOOST_CHRONO_HEADER_ONLY
		BOOST_ERROR_CODE_HEADER_ONLY
		BOOST_SYSTEM_NO_DEPRECATED
	)
	
	target_include_directories(rlGripperDemo PUBLIC ${Boost_INCLUDE_DIR})
endif()

target_link_libraries(
	rlGripperDemo
	hal
	util
)

if(MSVC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.00.23918)
	target_link_libraries(rlGripperDemo ${Boost_LIBRARIES})
endif()

add_executable(
	rlRobotiqModelCDemo
	rlRobotiqModelCDemo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlRobotiqModelCDemo
	hal
)
