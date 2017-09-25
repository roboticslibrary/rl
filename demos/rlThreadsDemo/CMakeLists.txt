if(MSVC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.00.23918)
	find_package(Boost REQUIRED COMPONENTS thread)
endif()

add_executable(
	rlThreadsDemo
	rlThreadsDemo.cpp
)

if(MSVC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.00.23918)
	target_compile_definitions(
		rlThreadsDemo
		PUBLIC
		-DBOOST_ALL_NO_LIB
		-DBOOST_CHRONO_HEADER_ONLY
		-DBOOST_ERROR_CODE_HEADER_ONLY
		-DBOOST_SYSTEM_NO_DEPRECATED
	)
	
	target_include_directories(rlThreadsDemo PUBLIC ${Boost_INCLUDE_DIR})
endif()

target_link_libraries(
	rlThreadsDemo
	util
)

if(MSVC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.00.23918)
	target_link_libraries(rlThreadsDemo ${Boost_LIBRARIES})
endif()
