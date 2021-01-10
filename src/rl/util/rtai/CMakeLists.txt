find_package(RTAI REQUIRED)

set(
	HDRS
	chrono.h
	thread.h
)

if(NOT CMAKE_VERSION VERSION_LESS 3.19)
	add_library(util_rtai INTERFACE ${HDRS})
else()
	add_library(util_rtai INTERFACE)
	add_custom_target(util_rtai_sources SOURCES ${HDRS})
endif()

if(NOT CMAKE_VERSION VERSION_LESS 3.8)
	target_compile_features(util_rtai INTERFACE cxx_std_11)
endif()

target_include_directories(
	util_rtai
	INTERFACE
	$<BUILD_INTERFACE:${rl_BINARY_DIR}/src>
	$<BUILD_INTERFACE:${rl_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/rl-${PROJECT_VERSION}>
	${RTAI_INCLUDE_DIRS}
)

target_link_libraries(util_rtai INTERFACE ${RTAI_LIBRARIES})

if(NOT CMAKE_VERSION VERSION_LESS 3.19)
	set_target_properties(
		util_rtai
		PROPERTIES
		VERSION ${PROJECT_VERSION}
	)
endif()

install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${PROJECT_VERSION}/rl/util/rtai COMPONENT development)

install(
	TARGETS util_rtai
	EXPORT rl
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_SKIP
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
)
