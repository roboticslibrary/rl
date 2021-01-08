find_package(RTAI)
find_package(Threads REQUIRED)
find_package(Xenomai)

cmake_dependent_option(RL_BUILD_UTIL_RTAI "Build RTAI support" ON "RL_BUILD_UTIL;RTAI_FOUND" OFF)
cmake_dependent_option(RL_BUILD_UTIL_XENOMAI "Build Xenomai support" ON "RL_BUILD_UTIL;Xenomai_FOUND" OFF)

set(
	BASE_HDRS
	process.h
	thread.h
)
list(APPEND HDRS ${BASE_HDRS})

set(
	IO_HDRS
	io/BasicIosRestorer.h
	io/Hex.h
)
list(APPEND HDRS ${IO_HDRS})

if(RL_BUILD_UTIL_RTAI)
	set(
		RTAI_HDRS
		rtai/chrono.h
		rtai/thread.h
	)
	list(APPEND HDRS ${RTAI_HDRS})
endif()

if(RL_BUILD_UTIL_XENOMAI)
	set(
		XENOMAI_HDRS
		xenomai/chrono.h
		xenomai/condition_variable.h
		xenomai/mutex.h
		xenomai/thread.h
	)
	list(APPEND HDRS ${XENOMAI_HDRS})
endif()

if(NOT CMAKE_VERSION VERSION_LESS 3.19)
	add_library(util INTERFACE ${HDRS})
else()
	add_library(util INTERFACE)
	add_custom_target(util_sources SOURCES ${HDRS})
endif()

if(NOT CMAKE_VERSION VERSION_LESS 3.8)
	target_compile_features(util INTERFACE cxx_std_11)
endif()

target_link_libraries(util INTERFACE ${CMAKE_THREAD_LIBS_INIT})

target_include_directories(
	util
	INTERFACE
	$<BUILD_INTERFACE:${rl_BINARY_DIR}/src>
	$<BUILD_INTERFACE:${rl_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/rl-${PROJECT_VERSION}>
)

install(FILES ${BASE_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${PROJECT_VERSION}/rl/util COMPONENT development)
install(FILES ${IO_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${PROJECT_VERSION}/rl/util/io COMPONENT development)

if(RL_BUILD_UTIL_RTAI)
	target_include_directories(util INTERFACE ${RTAI_INCLUDE_DIRS})
	target_link_libraries(util INTERFACE ${RTAI_LIBRARIES})
	install(FILES ${RTAI_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${PROJECT_VERSION}/rl/util/rtai COMPONENT development)
endif()

if(RL_BUILD_UTIL_XENOMAI)
	target_compile_definitions(util INTERFACE ${Xenomai_DEFINITIONS})
	target_include_directories(util INTERFACE ${Xenomai_INCLUDE_DIRS})
	target_link_libraries(util INTERFACE ${Xenomai_LIBRARIES})
	install(FILES ${XENOMAI_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${PROJECT_VERSION}/rl/util/xenomai COMPONENT development)
endif()

if(NOT CMAKE_VERSION VERSION_LESS 3.19)
	set_target_properties(
		util
		PROPERTIES
		VERSION ${PROJECT_VERSION}
	)
endif()

install(
	TARGETS util
	EXPORT rl
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_SKIP
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
)
