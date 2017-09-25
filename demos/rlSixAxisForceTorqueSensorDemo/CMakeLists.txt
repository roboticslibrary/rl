if(MSVC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.00.23918)
	find_package(Boost REQUIRED COMPONENTS thread)
endif()

add_executable(
	rlSixAxisForceTorqueSensorDemo
	rlSixAxisForceTorqueSensorDemo.cpp
)

if(MSVC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.00.23918)
	target_compile_definitions(
		rlSixAxisForceTorqueSensorDemo
		PUBLIC
		-DBOOST_ALL_NO_LIB
		-DBOOST_CHRONO_HEADER_ONLY
		-DBOOST_ERROR_CODE_HEADER_ONLY
		-DBOOST_SYSTEM_NO_DEPRECATED
	)
endif()

target_link_libraries(
	rlSixAxisForceTorqueSensorDemo
	hal
	util
)

if(MSVC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.00.23918)
	target_link_libraries(rlSixAxisForceTorqueSensorDemo ${Boost_LIBRARIES})
endif()
