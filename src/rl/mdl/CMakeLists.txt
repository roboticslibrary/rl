project(rlmdl)

find_package(Boost REQUIRED)

set(
	HDRS
	Body.h
	Compound.h
	Cylindrical.h
	Dynamic.h
	Element.h
	Exception.h
	Factory.h
	Fixed.h
	Frame.h
	Helical.h
	Joint.h
	Kinematic.h
	Metric.h
	Model.h
	Prismatic.h
	Revolute.h
	Spherical.h
	Transform.h
	World.h
	XmlFactory.h
)
set(
	SRCS
	Body.cpp
	Compound.cpp
	Cylindrical.cpp
	Dynamic.cpp
	Element.cpp
	Exception.cpp
	Fixed.cpp
	Factory.cpp
	Frame.cpp
	Helical.cpp
	Joint.cpp
	Kinematic.cpp
	Metric.cpp
	Model.cpp
	Prismatic.cpp
	Revolute.cpp
	Spherical.cpp
	Transform.cpp
	World.cpp
	XmlFactory.cpp
)

if(WIN32)
	add_library(
		rlmdl
		STATIC
		${HDRS}
		${SRCS}
	)
else(WIN32)
	add_library(
		rlmdl
		SHARED
		${HDRS}
		${SRCS}
	)
endif(WIN32)

target_include_directories(
	rlmdl
	PUBLIC
	${Boost_INCLUDE_DIR}
)

target_link_libraries(
	rlmdl
	rlmath
	rlxml
)

set_target_properties(
	rlmdl
	PROPERTIES
	VERSION ${VERSION}
	DEBUG_POSTFIX d
)

install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl/mdl COMPONENT headers)

install(
	TARGETS rlmdl
	EXPORT rl
	COMPONENT libraries
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
