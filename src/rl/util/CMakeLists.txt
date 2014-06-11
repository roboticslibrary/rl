project(rlutil)

find_package(Threads REQUIRED)

set(
	HDRS
	Mutex.h
	MutexLocker.h
	MutexTryLocker.h
	Semaphore.h
	Thread.h
	Timer.h
)

add_library(
	rlutil
	${HDRS}
	dummy.cpp
)

target_link_libraries(
	rlutil
	rlmath
	${CMAKE_THREAD_LIBS_INIT}
)

set_target_properties(
	rlutil
	PROPERTIES
	VERSION ${VERSION}
	DEBUG_POSTFIX d
)

install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl/util COMPONENT headers)

install(
	TARGETS rlutil
	EXPORT rl
	COMPONENT libraries
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
