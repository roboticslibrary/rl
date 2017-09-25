find_package(RTAI)
find_package(Threads REQUIRED)
find_package(Xenomai)

set(
	BASE_HDRS
	process.h
	thread.h
)
list(APPEND HDRS ${BASE_HDRS})

if(RTAI_FOUND)
	set(
		RTAI_HDRS
		rtai/chrono.h
		rtai/thread.h
	)
	list(APPEND HDRS ${RTAI_HDRS})
endif()

if(Xenomai_FOUND)
	set(
		XENOMAI_HDRS
		xenomai/chrono.h
		xenomai/condition_variable.h
		xenomai/mutex.h
		xenomai/thread.h
	)
	list(APPEND HDRS ${XENOMAI_HDRS})
endif()

if(NOT CMAKE_VERSION VERSION_LESS 3.0)
	add_library(util INTERFACE)
	add_custom_target(util_sources SOURCES ${HDRS})
else()
	add_library(util STATIC ${HDRS} dummy.cpp)
endif()

if(NOT CMAKE_VERSION VERSION_LESS 3.8)
	target_compile_features(util INTERFACE cxx_std_11)
endif()

if(NOT CMAKE_VERSION VERSION_LESS 3.0)
	target_link_libraries(util INTERFACE ${CMAKE_THREAD_LIBS_INIT})
else()
	target_link_libraries(util ${CMAKE_THREAD_LIBS_INIT})
endif()

target_include_directories(
	util
	INTERFACE
	$<BUILD_INTERFACE:${rl_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}>
)

install(FILES ${BASE_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}/rl/util COMPONENT development)

if(RTAI_FOUND)
	target_include_directories(util INTERFACE ${RTAI_INCLUDE_DIRS})
	if(NOT CMAKE_VERSION VERSION_LESS 3.0)
		target_link_libraries(util INTERFACE ${RTAI_LIBRARIES})
	else()
		target_link_libraries(util ${RTAI_LIBRARIES})
	endif()
	install(FILES ${RTAI_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}/rl/util/rtai COMPONENT development)
endif()

if(Xenomai_FOUND)
	target_compile_definitions(util INTERFACE ${Xenomai_DEFINITIONS})
	target_include_directories(util INTERFACE ${Xenomai_INCLUDE_DIRS})
	if(NOT CMAKE_VERSION VERSION_LESS 3.0)
		target_link_libraries(util INTERFACE ${Xenomai_LIBRARIES})
	else()
		target_link_libraries(util ${Xenomai_LIBRARIES})
	endif()
	install(FILES ${XENOMAI_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}/rl/util/xenomai COMPONENT development)
endif()

if(CMAKE_VERSION VERSION_LESS 3.0)
	set_target_properties(
		util
		PROPERTIES
		OUTPUT_NAME rlutil
		POSITION_INDEPENDENT_CODE ON
		VERSION ${VERSION}
	)
	
	if(WIN32)
		set_target_properties(
			util
			PROPERTIES
			DEBUG_POSTFIX d
		)
	endif()
endif()

install(
	TARGETS util
	EXPORT rl
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_SKIP
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
)
