project(rlkin)

find_package(Boost REQUIRED)

set(
	HDRS
	Element.h
	Exception.h
	Frame.h
	Joint.h
	Kinematics.h
	Link.h
	Prismatic.h
	Puma.h
	Revolute.h
	Rhino.h
	Transform.h
)
set(
	SRCS
	Element.cpp
	Exception.cpp
	Frame.cpp
	Joint.cpp
	Kinematics.cpp
	Link.cpp
	Prismatic.cpp
	Puma.cpp
	Revolute.cpp
	Rhino.cpp
	Transform.cpp
)

if(WIN32)
	add_library(
		rlkin
		STATIC
		${HDRS}
		${SRCS}
	)
else(WIN32)
	add_library(
		rlkin
		SHARED
		${HDRS}
		${SRCS}
	)
endif(WIN32)

target_include_directories(
	rlkin
	PUBLIC
	${Boost_INCLUDE_DIR}
)

target_link_libraries(
	rlkin
	rlmath
	rlxml
)

set_target_properties(
	rlkin
	PROPERTIES
	VERSION ${VERSION}
	DEBUG_POSTFIX d
)

install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl/kin COMPONENT headers)

install(
	TARGETS rlkin
	EXPORT rl
	COMPONENT libraries
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
