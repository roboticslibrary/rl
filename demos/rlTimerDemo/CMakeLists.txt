project(rlTimerDemo)

if(MSVC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.00.23918)
	find_package(Boost REQUIRED COMPONENTS thread)
endif()

add_executable(
	rlTimerDemo
	rlTimerDemo.cpp
)

if(MSVC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.00.23918)
	target_compile_definitions(
		rlTimerDemo
		PUBLIC
		-DBOOST_ALL_NO_LIB
		-DBOOST_CHRONO_HEADER_ONLY
		-DBOOST_ERROR_CODE_HEADER_ONLY
		-DBOOST_SYSTEM_NO_DEPRECATED
	)
	
	target_include_directories(rlTimerDemo PUBLIC ${Boost_INCLUDE_DIR})
	
	target_link_libraries(rlTimerDemo ${Boost_LIBRARIES})
endif()
