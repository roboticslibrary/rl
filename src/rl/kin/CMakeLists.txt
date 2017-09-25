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

add_library(
	kin
	${HDRS}
	${SRCS}
)

if(NOT CMAKE_VERSION VERSION_LESS 3.8)
	target_compile_features(kin PUBLIC cxx_std_11)
endif()

target_include_directories(
	kin
	PUBLIC
	$<BUILD_INTERFACE:${rl_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}>
	${Boost_INCLUDE_DIR}
)

target_link_libraries(
	kin
	math
	xml
)

set_target_properties(
	kin
	PROPERTIES
	OUTPUT_NAME rlkin
	POSITION_INDEPENDENT_CODE ON
	VERSION ${VERSION}
)

if(WIN32)
	if(BUILD_SHARED_LIBS)
		set_target_properties(
			kin
			PROPERTIES
			DEBUG_POSTFIX d
		)
	else()
		set_target_properties(
			kin
			PROPERTIES
			DEBUG_POSTFIX sd
			MINSIZEREL_POSTFIX s
			RELEASE_POSTFIX s
			RELWITHDEBINFO_POSTFIX s
		)
	endif()
endif()

install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}/rl/kin COMPONENT development)

install(
	TARGETS kin
	EXPORT rl
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_SKIP
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
)

if(BUILD_SHARED_LIBS)
	install(
		TARGETS kin
		EXPORT rl
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development NAMELINK_ONLY
	)
endif()

if(MSVC AND BUILD_SHARED_LIBS AND NOT CMAKE_VERSION VERSION_LESS 3.1)
	install(FILES $<TARGET_PDB_FILE:kin> DESTINATION ${CMAKE_INSTALL_BINDIR} CONFIGURATIONS Debug RelWithDebInfo COMPONENT debug)
endif()
