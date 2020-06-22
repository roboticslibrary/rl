set(
	HDRS
	algorithm.h
	iterator.h
	memory.h
)

if(NOT CMAKE_VERSION VERSION_LESS 3.0)
	add_library(std INTERFACE)
	add_custom_target(std_sources SOURCES ${HDRS})
else()
	add_library(std STATIC ${HDRS} dummy.cpp)
endif()

if(NOT CMAKE_VERSION VERSION_LESS 3.8)
	target_compile_features(std INTERFACE cxx_std_11)
endif()

target_include_directories(
	std
	INTERFACE
	$<BUILD_INTERFACE:${rl_BINARY_DIR}/src>
	$<BUILD_INTERFACE:${rl_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}>
)

install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}/rl/std COMPONENT development)

if(CMAKE_VERSION VERSION_LESS 3.0)
	set_target_properties(
		std
		PROPERTIES
		OUTPUT_NAME rlstd
		POSITION_INDEPENDENT_CODE ON
		VERSION ${VERSION}
	)
	
	if(MSVC)
		set_target_properties(
			std
			PROPERTIES
			DEBUG_POSTFIX d
		)
	endif()
endif()

install(
	TARGETS std
	EXPORT rl
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_SKIP
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
)
