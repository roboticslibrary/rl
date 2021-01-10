find_package(Xenomai REQUIRED)

set(
	HDRS
	chrono.h
	condition_variable.h
	mutex.h
	thread.h
)

if(NOT CMAKE_VERSION VERSION_LESS 3.19)
	add_library(util_xenomai INTERFACE ${HDRS})
else()
	add_library(util_xenomai INTERFACE)
	add_custom_target(util_xenomai_sources SOURCES ${HDRS})
endif()

target_compile_definitions(util_xenomai INTERFACE ${Xenomai_DEFINITIONS})

if(NOT CMAKE_VERSION VERSION_LESS 3.8)
	target_compile_features(util_xenomai INTERFACE cxx_std_11)
endif()

target_include_directories(
	util_xenomai
	INTERFACE
	$<BUILD_INTERFACE:${rl_BINARY_DIR}/src>
	$<BUILD_INTERFACE:${rl_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/rl-${PROJECT_VERSION}>
	${XENOMAI_INCLUDE_DIRS}
)

target_link_libraries(util_xenomai INTERFACE ${XENOMAI_LIBRARIES})

if(NOT CMAKE_VERSION VERSION_LESS 3.19)
	set_target_properties(
		util_xenomai
		PROPERTIES
		VERSION ${PROJECT_VERSION}
	)
endif()

install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${PROJECT_VERSION}/rl/util/xenomai COMPONENT development)

install(
	TARGETS util_xenomai
	EXPORT rl
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_SKIP
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
)
